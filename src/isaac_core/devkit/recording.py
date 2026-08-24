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

from collections.abc import Callable
import logging
from pathlib import Path
from typing import Any, Generic, TypeVar

from isaac_core.contracts import topics

logger = logging.getLogger(__name__)

# Type variable for the ROS message type.
MsgT = TypeVar("MsgT")


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

    The serialiser converts sensor_msgs/Image to a numpy array via cv_bridge.
    Requires ``cv_bridge`` and ``cv2`` at usage time (not at import time).

    Args:
        topic: The image topic to subscribe to.
        node_name: Name for the rclpy node.

    Returns:
        A configured :class:`TopicRecorder`.

    """

    def _serialise_image(msg: Any, frame_index: int) -> Any:  # noqa: ANN401
        from cv_bridge import CvBridge  # noqa: PLC0415

        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

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
        return {
            "frame": frame_index,
            "bboxes": [
                {
                    "target_name": b.target_name,
                    "in_frame": b.in_frame,
                    "is_visible": b.is_visible,
                    "x1": b.x1,
                    "y1": b.y1,
                    "x2": b.x2,
                    "y2": b.y2,
                }
                for b in msg.bboxes
            ],
        }

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
    from isaac_ros2_messages.msg import FrameBboxes  # noqa: PLC0415

    return FrameBboxes


__all__ = [
    "TopicRecorder",
    "bbox_recorder",
    "pose_recorder",
    "range_recorder",
    "video_recorder",
]
