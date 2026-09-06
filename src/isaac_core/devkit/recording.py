"""
Generic topic recorder replacing the four copy-paste capture classes.

Fixes defect #8: ``VideoCapture``, ``PoseCapture``, ``DistanceCapture`` and
``BboxCapture`` were ~95% identical, differing only in message type, topic and
serialiser. This module provides ONE generic :class:`TopicRecorder` parameterised
by those three things, plus thin factory functions that preserve the ergonomic
call sites.

**Import safety**: ``rclpy``, ``cv_bridge`` and ``cv2`` are NOT importable on a
machine without ROS 2. They are imported lazily inside the functions that actually
need them. ``import isaac_core.devkit.recording`` always succeeds -- this is a
tested property that keeps CI working.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
import logging
from pathlib import Path
import statistics
from typing import Any, Generic, TypeVar

from isaac_core.contracts import topics

logger = logging.getLogger(__name__)

# Type variable for the ROS message type.
MsgT = TypeVar("MsgT")

# Nanoseconds per second -- ROS 2 ``header.stamp`` is split into ``sec`` + ``nanosec``.
_NS_PER_S = 1_000_000_000

# Fallback frame rate (Hz) used only when timing cannot be measured (fewer than two
# frames, or every timestamp identical) and no override is supplied. A single-frame clip
# has no meaningful rate; one frame per second keeps the writer valid without guessing.
_FALLBACK_FPS = 1.0

# Minimum frames needed to measure an inter-frame interval (and hence a rate).
_MIN_FRAMES_FOR_RATE = 2


def _stamp_to_ns(stamp: Any) -> int:  # noqa: ANN401
    """
    Convert a ROS 2 ``builtin_interfaces/Time`` stamp to integer nanoseconds.

    Args:
        stamp: An object exposing ``sec`` and ``nanosec`` integer fields.

    Returns:
        The stamp as a single nanosecond count.

    """
    return int(stamp.sec) * _NS_PER_S + int(stamp.nanosec)


def measured_fps(stamps_ns: Sequence[int]) -> float:
    """
    Derive the true average frame rate (Hz) from per-frame timestamps.

    This is the heart of D22's "derive timing, never ask the user for an fps". The
    median inter-frame interval is used rather than the mean so a single dropped frame
    (a large gap) or a burst does not skew the rate; the median ignores outliers.
    Non-monotonic and duplicate timestamps contribute non-positive deltas which are
    discarded before the median is taken.

    Args:
        stamps_ns: Presentation timestamps in nanoseconds, in capture order.

    Returns:
        The measured frame rate in Hz, or :data:`_FALLBACK_FPS` when it cannot be
        measured (fewer than two frames, or no strictly positive interval exists).

    """
    if len(stamps_ns) < _MIN_FRAMES_FOR_RATE:
        return _FALLBACK_FPS
    deltas = [b - a for a, b in zip(stamps_ns, stamps_ns[1:]) if b - a > 0]
    if not deltas:
        return _FALLBACK_FPS
    median_ns = statistics.median(deltas)
    return _NS_PER_S / median_ns


def presentation_times_s(stamps_ns: Sequence[int]) -> list[float]:
    """
    Convert absolute timestamps to presentation times in seconds from the first frame.

    The first frame sits at ``0.0``; every later frame is offset by its real elapsed
    time. These are the exact per-frame times a variable-frame-rate muxer would need,
    written to the sidecar file so true timing survives even though the constant-rate
    writer below cannot express it directly.

    Args:
        stamps_ns: Presentation timestamps in nanoseconds, in capture order.

    Returns:
        Presentation times in seconds relative to the first frame. Non-monotonic
        inputs are clamped so the sequence never goes backwards.

    """
    if not stamps_ns:
        return []
    origin = stamps_ns[0]
    times: list[float] = []
    last = 0.0
    for ns in stamps_ns:
        t = (ns - origin) / _NS_PER_S
        # Clamp backwards jumps (non-monotonic input) so playback time never rewinds.
        last = max(t, last)
        times.append(last)
    return times


def _require_rclpy() -> Any:  # noqa: ANN401
    """
    Import and return rclpy, raising a clear error if unavailable.

    Returns:
        The rclpy module.

    Raises:
        ImportError: With an actionable message if rclpy is not importable.

    """
    try:
        import rclpy  # noqa: PLC0415
    except ImportError:
        msg = (
            "rclpy is not available. TopicRecorder requires a sourced ROS 2 "
            "environment. Run 'source /opt/ros/humble/setup.bash' (or your distro's "
            "equivalent) before using recording features."
        )
        raise ImportError(msg) from None
    return rclpy


class TopicRecorder(Generic[MsgT]):
    """
    Generic recorder for a single ROS 2 topic.

    Replaces the old copy-paste ``VideoCapture``/``PoseCapture``/``DistanceCapture``/
    ``BboxCapture`` classes with one parameterised implementation. The message type,
    topic, and serialisation strategy are injected at construction.

    Lifecycle::

        recorder = TopicRecorder(topic="/foo", msg_type=Foo, serialiser=my_ser)
        recorder.start()       # create the subscription
        recorder.spin()        # start background spinning
        ...
        recorder.stop()        # stop recording
        recorder.save_to(path) # persist collected data
        recorder.shutdown()    # tear down rclpy resources

    Or as a context manager::

        with TopicRecorder(...) as rec:
            rec.start()
            ...
            rec.stop()
            rec.save_to(path)

    """

    def __init__(
        self,
        topic: str,
        msg_type: type[MsgT],
        serialiser: Callable[[MsgT, int], Any],
        *,
        node_name: str | None = None,
        qos_depth: int = 10,
    ) -> None:
        """
        Initialise the recorder.

        Args:
            topic: ROS 2 topic to subscribe to.
            msg_type: The ROS message class.
            serialiser: Callable taking ``(message, frame_index)`` and returning a
                serialisable value to store.
            node_name: Name for the rclpy node. Defaults to a sanitised topic name.
            qos_depth: QoS history depth (KEEP_LAST).

        """
        self._topic = topic
        self._msg_type = msg_type
        self._serialiser = serialiser
        self._qos_depth = qos_depth
        self._node_name = node_name or ("recorder_" + topic.strip("/").replace("/", "_"))

        self._node: Any = None
        self._subscription: Any = None
        self._executor: Any = None
        self._spin_thread: Any = None
        self._recording: bool = False
        self._frame_index: int = 0
        self._frames: dict[int, Any] = {}

    @property
    def topic(self) -> str:
        """Return the subscribed topic."""
        return self._topic

    @property
    def frame_count(self) -> int:
        """Return the number of frames recorded so far."""
        return len(self._frames)

    @property
    def recording(self) -> bool:
        """Return whether recording is active."""
        return self._recording

    def start(self) -> None:
        """
        Create the rclpy node and subscription, and begin recording.

        Raises:
            ImportError: If rclpy is not available.

        """
        rclpy = _require_rclpy()

        from rclpy.qos import (  # noqa: PLC0415
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self._node_name)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self._qos_depth,
        )

        self._subscription = self._node.create_subscription(
            self._msg_type,
            self._topic,
            self._on_message,
            qos,
        )
        self._recording = True
        logger.info("started recording on %s", self._topic)

    def _on_message(self, msg: MsgT) -> None:
        """Handle an incoming message by serialising and storing it."""
        if not self._recording:
            return
        serialised = self._serialiser(msg, self._frame_index)
        self._frames[self._frame_index] = serialised
        self._frame_index += 1

    def stop(self) -> None:
        """Stop recording (messages still arrive but are not stored)."""
        self._recording = False
        logger.info("stopped recording on %s (%d frames)", self._topic, len(self._frames))

    def spin(self) -> None:
        """
        Start spinning the node in a background daemon thread.

        Raises:
            ImportError: If rclpy is not available.
            RuntimeError: If start() has not been called.

        """
        if self._node is None:
            msg = "call start() before spin()"
            raise RuntimeError(msg)

        _require_rclpy()

        import threading  # noqa: PLC0415

        from rclpy.executors import SingleThreadedExecutor  # noqa: PLC0415

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
            name=f"recorder-spin-{self._node_name}",
        )
        self._spin_thread.start()

    def save_to(self, path: str | Path) -> Path:
        """
        Persist the recorded frames using the serialiser's output format.

        Writes a pickle file keyed by frame index, matching the old capture classes'
        format for backward compatibility with existing analysis scripts.

        Args:
            path: Output file path.

        Returns:
            The resolved output path.

        """
        import pickle  # noqa: PLC0415

        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("wb") as fh:
            pickle.dump(self._frames, fh)
        logger.info("saved %d frames to %s", len(self._frames), out)
        return out

    def save_video(self, path: str | Path, *, fps_override: float | None = None) -> Path:
        """
        Write the recorded image frames to an mp4 at their true measured rate.

        This exists because 2023's capture asked the user for an fps and wrote a
        constant-rate mp4, but Isaac Sim renders at a variable ~30-50 fps, so every
        recording played partly too fast and partly too slow. Each frame here carries
        the real ``header.stamp`` captured at receive time (see :func:`video_recorder`),
        so the timing is derived rather than guessed -- honouring D22.

        Tradeoff, stated honestly: the only video library importable in this environment
        is OpenCV, whose ``VideoWriter`` emits **constant**-frame-rate output. True
        variable-frame-rate encoding would need PyAV or imageio-ffmpeg, which are not
        installed. So the mp4 is written at the *measured average* rate (the median
        inter-frame interval, robust to dropped frames), which already fixes the
        wrong-speed defect. To preserve the exact per-frame timing that a constant-rate
        container cannot express, a sidecar ``<name>.timestamps.txt`` is written next to
        the video with one presentation time (seconds, from the first frame) per line; a
        VFR remux with the system ``ffmpeg`` can consume it later without re-recording.

        Args:
            path: Output mp4 path. The sidecar timestamps file is derived from it.
            fps_override: Explicit constant rate to force. ``None`` (the default) derives
                the rate from the timestamps; any other value is used verbatim.

        Returns:
            The resolved output video path.

        Raises:
            RuntimeError: If no frames were recorded (nothing to write).

        """
        import cv2  # noqa: PLC0415

        frames = [self._frames[i] for i in sorted(self._frames)]
        if not frames:
            msg = "no frames recorded -- nothing to write"
            raise RuntimeError(msg)

        stamps_ns = [int(f["stamp_ns"]) for f in frames]
        fps = fps_override if fps_override is not None else measured_fps(stamps_ns)

        times = presentation_times_s(stamps_ns)
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        sidecar = out.with_suffix(out.suffix + ".timestamps.txt")
        sidecar.write_text("\n".join(f"{t:.9f}" for t in times) + "\n")

        first = frames[0]["image"]
        height, width = first.shape[0], first.shape[1]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out), fourcc, fps, (width, height))
        try:
            for f in frames:
                writer.write(f["image"])
        finally:
            writer.release()

        logger.info("wrote %d frames to %s at %.3f fps (measured)", len(frames), out, fps)
        return out

    def shutdown(self) -> None:
        """Tear down the rclpy node and executor."""
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=5.0)
            self._spin_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._subscription = None
        logger.info("shut down recorder for %s", self._topic)

    def __enter__(self) -> TopicRecorder[MsgT]:
        """Enter context manager."""
        return self

    def __exit__(self, *_args: object) -> None:
        """Exit context manager -- stop and shutdown."""
        self.stop()
        self.shutdown()


# --------------------------------------------------------------------------- #
# Factory functions -- preconfigured recorders for common topics.             #
# These preserve the ergonomic call sites while eliminating duplication.       #
# --------------------------------------------------------------------------- #


def video_recorder(
    topic: str = f"{topics.ROOT}/{topics.IMAGE_RGB}",
    *,
    node_name: str = "video_recorder",
) -> TopicRecorder[Any]:
    """
    Create a recorder for RGB image messages.

    The serialiser converts sensor_msgs/Image to a numpy array via cv_bridge and
    captures the message's ``header.stamp`` as integer nanoseconds. That timestamp is
    what lets :meth:`TopicRecorder.save_video` reconstruct the true frame rate instead
    of asking the user to guess one. Requires ``cv_bridge`` and ``cv2`` at usage time
    (not at import time).

    Args:
        topic: The image topic to subscribe to.
        node_name: Name for the rclpy node.

    Returns:
        A configured :class:`TopicRecorder`.

    """

    def _serialise_image(msg: Any, frame_index: int) -> dict[str, Any]:  # noqa: ANN401
        from cv_bridge import CvBridge  # noqa: PLC0415

        bridge = CvBridge()
        return {
            "frame": frame_index,
            "stamp_ns": _stamp_to_ns(msg.header.stamp),
            "image": bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"),
        }

    return TopicRecorder(
        topic=topic,
        msg_type=_lazy_image_type(),
        serialiser=_serialise_image,
        node_name=node_name,
    )


def pose_recorder(
    topic: str = f"{topics.ROOT}/{topics.GLOBAL_POSE}",
    *,
    node_name: str = "pose_recorder",
) -> TopicRecorder[Any]:
    """
    Create a recorder for GeoPoseStamped messages.

    Args:
        topic: The pose topic to subscribe to.
        node_name: Name for the rclpy node.

    Returns:
        A configured :class:`TopicRecorder`.

    """

    def _serialise_pose(msg: Any, frame_index: int) -> dict[str, Any]:  # noqa: ANN401
        pos = msg.pose.position
        orient = msg.pose.orientation
        return {
            "frame": frame_index,
            "lat": pos.latitude,
            "lon": pos.longitude,
            "alt": pos.altitude,
            "qx": orient.x,
            "qy": orient.y,
            "qz": orient.z,
            "qw": orient.w,
        }

    return TopicRecorder(
        topic=topic,
        msg_type=_lazy_geopose_type(),
        serialiser=_serialise_pose,
        node_name=node_name,
    )


def range_recorder(
    topic: str = f"{topics.ROOT}/{topics.DISTANCE_SENSOR}",
    *,
    node_name: str = "range_recorder",
) -> TopicRecorder[Any]:
    """
    Create a recorder for Range messages.

    Args:
        topic: The range sensor topic.
        node_name: Name for the rclpy node.

    Returns:
        A configured :class:`TopicRecorder`.

    """

    def _serialise_range(msg: Any, frame_index: int) -> dict[str, Any]:  # noqa: ANN401
        return {
            "frame": frame_index,
            "range_m": msg.range,
            "min_range_m": msg.min_range,
            "max_range_m": msg.max_range,
        }

    return TopicRecorder(
        topic=topic,
        msg_type=_lazy_range_type(),
        serialiser=_serialise_range,
        node_name=node_name,
    )


# Every per-detection array on isaac_core_ros2_msgs/FrameBboxes, in message declaration order.
_BBOX_ARRAY_FIELDS = (
    "target_name",
    "in_frame",
    "is_visible",
    "x1",
    "y1",
    "x2",
    "y2",
    "lat",
    "lon",
    "alt",
    "roll",
    "pitch",
    "yaw",
    "distance_x",
    "distance_y",
    "distance_z",
)


def _scalar(value: Any) -> Any:  # noqa: ANN401
    """
    Convert a numpy scalar from a ROS array field into a plain Python value.

    ROS array fields deserialise to numpy arrays, whose elements are numpy scalars that the
    json module cannot encode. Recording silently failing at write time is worse than a
    conversion here.

    Args:
        value: One element read out of a message array field.

    Returns:
        A JSON-encodable Python scalar.

    """
    item = getattr(value, "item", None)
    return item() if callable(item) else value


def bbox_recorder(
    topic: str = f"{topics.ROOT}/{topics.BBOX}",
    *,
    node_name: str = "bbox_recorder",
) -> TopicRecorder[Any]:
    """
    Create a recorder for bounding box messages.

    Args:
        topic: The bounding box topic.
        node_name: Name for the rclpy node.

    Returns:
        A configured :class:`TopicRecorder`.

    """

    def _serialise_bbox(msg: Any, frame_index: int) -> dict[str, Any]:  # noqa: ANN401
        # FrameBboxes carries parallel arrays rather than a Bbox[], so a detection is a slice
        # across every array. Recorded back as one dict per detection, which is what a reader
        # actually wants, and re-zips them here rather than making every consumer do it.
        count = len(msg.target_name)
        detections = [
            {name: _scalar(getattr(msg, name)[index]) for name in _BBOX_ARRAY_FIELDS} for index in range(count)
        ]
        return {"frame": frame_index, "bboxes": detections}

    return TopicRecorder(
        topic=topic,
        msg_type=_lazy_framebboxes_type(),
        serialiser=_serialise_bbox,
        node_name=node_name,
    )


# --------------------------------------------------------------------------- #
# Lazy ROS message type loaders -- import only at usage time.                 #
# --------------------------------------------------------------------------- #


def _lazy_image_type() -> Any:  # noqa: ANN401
    """Import and return sensor_msgs.msg.Image."""
    from sensor_msgs.msg import Image  # noqa: PLC0415

    return Image


def _lazy_geopose_type() -> Any:  # noqa: ANN401
    """Import and return geographic_msgs.msg.GeoPoseStamped."""
    from geographic_msgs.msg import GeoPoseStamped  # noqa: PLC0415

    return GeoPoseStamped


def _lazy_range_type() -> Any:  # noqa: ANN401
    """Import and return sensor_msgs.msg.Range."""
    from sensor_msgs.msg import Range  # noqa: PLC0415

    return Range


def _lazy_framebboxes_type() -> Any:  # noqa: ANN401
    """Import and return the FrameBboxes message type."""
    from isaac_core_ros2_msgs.msg import FrameBboxes  # noqa: PLC0415

    return FrameBboxes


__all__ = [
    "TopicRecorder",
    "bbox_recorder",
    "measured_fps",
    "pose_recorder",
    "presentation_times_s",
    "range_recorder",
    "video_recorder",
]
