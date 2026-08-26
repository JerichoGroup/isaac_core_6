"""
RTP video streaming service.

Subscribe to a ROS 2 ``sensor_msgs/Image`` topic and push frames over RTP via a
GStreamer pipeline::

    appsrc ! videoconvert ! x264enc tune=zerolatency ! h264parse ! rtph264pay ! udpsink

The old repo (``ros_image_to_rtp_lib.py``) also side-channelled ``header.frame_id``
over a separate UDP port as a frame counter. Decision D17 established that
``header.frame_id`` is a coordinate frame name (the ``seq`` field was deliberately
removed in ROS 2), so the frame-id side channel is not replicated. Use
``header.stamp`` for frame correlation.

Dependencies: ``rclpy``, ``gi`` (PyGObject with GStreamer typelibs). Both are
unavailable inside Isaac's Python 3.12 interpreter and inside the test environment.
All imports of these packages are therefore LAZY -- performed inside the function
that needs them -- with a clear actionable error if missing. Importing this module
must always succeed.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

from isaac_core.config import SidecarServiceConfig
from isaac_core.sidecar.service import ServiceRegistry

logger = logging.getLogger(__name__)

# Encoding to GStreamer video format name mapping.
# Kept as a module-level constant so it can be tested as a pure function
# without importing GStreamer.
ENCODING_TO_GST_FORMAT: dict[str, str] = {
    "rgb8": "RGB",
    "bgr8": "BGR",
    "mono8": "GRAY8",
    "rgba8": "RGBA",
    "bgra8": "BGRA",
}


def caps_string(encoding: str, width: int, height: int, framerate: int = 30) -> str | None:
    """
    Build a GStreamer caps string for the given image encoding.

    Args:
        encoding: ROS image encoding name (e.g. ``"rgb8"``).
        width: Image width in pixels.
        height: Image height in pixels.
        framerate: Frames per second for the caps.

    Returns:
        The caps string, or ``None`` if the encoding is not supported.

    """
    fmt = ENCODING_TO_GST_FORMAT.get(encoding)
    if fmt is None:
        return None
    return f"video/x-raw,format={fmt},width={width},height={height},framerate={framerate}/1"


def _require_rclpy() -> Any:  # noqa: ANN401
    """
    Lazily import ``rclpy`` with an actionable error on failure.

    Returns:
        The ``rclpy`` module.

    Raises:
        ImportError: With an explanation of how to make rclpy available.

    """
    try:
        import rclpy  # noqa: PLC0415
    except ImportError as exc:
        msg = (
            "rclpy is not importable. The sidecar must run on system Python 3.10 with "
            "ROS 2 Humble sourced: `source /opt/ros/humble/setup.bash` then relaunch."
        )
        raise ImportError(msg) from exc
    return rclpy


def _require_gst() -> Any:  # noqa: ANN401
    """
    Lazily import GStreamer via ``gi`` with an actionable error on failure.

    Returns:
        The ``Gst`` module from ``gi.repository``.

    Raises:
        ImportError: With an explanation of how to make PyGObject/GStreamer available.

    """
    try:
        import gi  # noqa: PLC0415

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # noqa: PLC0415
    except (ImportError, ValueError) as exc:
        msg = (
            "PyGObject with GStreamer typelibs is not available. Install with:\n"
            "  apt install python3-gi python3-gi-cairo gir1.2-gstreamer-1.0 "
            "gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-base "
            "gstreamer1.0-plugins-good gstreamer1.0-plugins-bad "
            "gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools"
        )
        raise ImportError(msg) from exc
    return Gst


def _require_glib() -> Any:  # noqa: ANN401
    """
    Lazily import GLib via ``gi`` with an actionable error on failure.

    Returns:
        The ``GLib`` module from ``gi.repository``.

    Raises:
        ImportError: With an explanation of how to make PyGObject available.

    """
    try:
        import gi  # noqa: PLC0415

        gi.require_version("GLib", "2.0")
        from gi.repository import GLib  # noqa: PLC0415
    except (ImportError, ValueError) as exc:
        msg = (
            "PyGObject with GLib typelibs is not available. Install with:\n"
            "  apt install python3-gi python3-gi-cairo gir1.2-gstreamer-1.0"
        )
        raise ImportError(msg) from exc
    return GLib


class RtpVideoService:
    """
    Subscribe to a ROS 2 Image topic and stream over RTP via GStreamer.

    Config keys (in ``SidecarServiceConfig``, beyond the mandatory ``kind``):

    - ``topic`` (str): ROS 2 image topic to subscribe (required).
    - ``host`` (str): Destination host for RTP. Default ``"127.0.0.1"``.
    - ``video_port`` (int): UDP port for the RTP video stream. Default ``5004``.
    - ``framerate`` (int): Caps framerate numerator. Default ``30``.
    - ``bitrate`` (int): x264enc bitrate in kbit/s. Default ``10000``.
    """

    def __init__(self, config: SidecarServiceConfig) -> None:
        """
        Create an RTP video service from a sidecar service config.

        Args:
            config: The service configuration block. Must contain at least ``topic``.

        Raises:
            ValueError: If required configuration is missing.

        """
        extra: dict[str, Any] = config.model_extra or {}
        topic = extra.get("topic")
        if not topic:
            msg = "RTP service requires a 'topic' key in its config"
            raise ValueError(msg)

        self._topic: str = str(topic)
        self._host: str = str(extra.get("host", "127.0.0.1"))
        self._video_port: int = int(extra.get("video_port", 5004))
        self._framerate: int = int(extra.get("framerate", 30))
        self._bitrate: int = int(extra.get("bitrate", 10000))

        self._node: Any = None
        self._pipeline: Any = None
        self._appsrc: Any = None
        self._main_loop: Any = None
        self._gst_thread: threading.Thread | None = None
        self._spin_thread: threading.Thread | None = None
        self._last_caps: str | None = None
        self._healthy: bool = False

    @property
    def name(self) -> str:
        """Return the service name for logging."""
        return f"rtp:{self._topic}->{self._host}:{self._video_port}"

    def start(self) -> None:
        """
        Start the GStreamer pipeline and the ROS 2 subscription.

        Import ``rclpy`` and ``gi`` lazily. Raise ``ImportError`` with clear
        instructions if either is missing.
        """
        rclpy = _require_rclpy()
        gst = _require_gst()
        glib = _require_glib()

        gst.init(None)

        pipeline_desc = (
            "appsrc name=src is-live=true block=true format=time ! "
            "videoconvert ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} speed-preset=superfast ! "
            "h264parse ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self._host} port={self._video_port} sync=false async=false"
        )
        self._pipeline = gst.parse_launch(pipeline_desc)
        self._appsrc = self._pipeline.get_by_name("src")
        self._pipeline.set_state(gst.State.PLAYING)

        self._main_loop = glib.MainLoop()
        self._gst_thread = threading.Thread(
            target=self._main_loop.run,
            name="rtp-gst-loop",
            daemon=True,
        )
        self._gst_thread.start()

        if not rclpy.ok():
            rclpy.init()

        from rclpy.node import Node  # noqa: PLC0415
        from sensor_msgs.msg import Image  # noqa: PLC0415

        self._node = Node("isaac_core_rtp_streamer")
        self._node.create_subscription(Image, self._topic, self._image_callback, 10)

        self._spin_thread = threading.Thread(
            target=self._spin,
            name="rtp-ros-spin",
            daemon=True,
        )
        self._spin_thread.start()

        self._healthy = True
        logger.info("RTP streaming %s -> %s:%d", self._topic, self._host, self._video_port)

    def stop(self, timeout: float = 5.0) -> None:
        """
        Stop the ROS 2 subscription and GStreamer pipeline.

        Args:
            timeout: Maximum seconds to wait for threads to join.

        """
        self._healthy = False

        rclpy = _require_rclpy()
        gst = _require_gst()

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                logger.debug("rclpy.shutdown() raised (may already be shut down)")

        if self._spin_thread is not None:
            self._spin_thread.join(timeout=timeout)
            self._spin_thread = None

        if self._appsrc is not None:
            try:
                self._appsrc.emit("end-of-stream")
            except Exception:
                logger.debug("end-of-stream emit failed")

        if self._pipeline is not None:
            self._pipeline.set_state(gst.State.NULL)
            self._pipeline = None
            self._appsrc = None

        if self._main_loop is not None:
            try:
                self._main_loop.quit()
            except Exception:
                logger.debug("GLib MainLoop quit failed")
            self._main_loop = None

        if self._gst_thread is not None:
            self._gst_thread.join(timeout=timeout)
            self._gst_thread = None

        self._last_caps = None

    def is_healthy(self) -> bool:
        """Return whether the service is running and connected."""
        return self._healthy

    def _spin(self) -> None:
        """Spin the ROS 2 node until shutdown."""
        rclpy = _require_rclpy()
        try:
            rclpy.spin(self._node)
        except Exception:
            logger.debug("rclpy.spin exited")
        finally:
            self._healthy = False

    def _image_callback(self, msg: Any) -> None:  # noqa: ANN401
        """
        Push a ROS 2 Image into the GStreamer appsrc.

        Set caps only when the encoding or resolution changes, preserving the
        optimisation from the old implementation.
        """
        gst = _require_gst()

        width: int = msg.width
        height: int = msg.height
        encoding: str = msg.encoding

        new_caps_str = caps_string(encoding, width, height, self._framerate)
        if new_caps_str is None:
            logger.warning("Unsupported encoding %r; frame dropped", encoding)
            return

        if new_caps_str != self._last_caps:
            caps = gst.Caps.from_string(new_caps_str)
            self._appsrc.set_property("caps", caps)
            self._last_caps = new_caps_str

        raw = bytes(msg.data)
        buf = gst.Buffer.new_allocate(None, len(raw), None)
        buf.fill(0, raw)
        self._appsrc.emit("push-buffer", buf)


# Register with the service registry so the supervisor can instantiate by kind.
ServiceRegistry.register("rtp", RtpVideoService)
