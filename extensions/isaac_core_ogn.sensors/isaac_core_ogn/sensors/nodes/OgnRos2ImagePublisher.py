"""
OmniGraph node: write raw RGB from a render product and republish with rate limiting.

Combines an NVIDIA Replicator ROS 2 writer for raw RGB output with a subscriber/
republisher that rate-limits and stamps frames with a monotonic counter.

All state lives in internal_state, never on ``db`` (fixing defect #2 from the old repo).

rclpy lifetime: this node only creates and destroys its own rclpy node handle.
It never calls ``rclpy.shutdown()``.

Image publishing API (Isaac Sim 6 / Kit 110):
    Import path for BaseWriterNode changed from ``omni.isaac.core_nodes`` (dead in 4.5+)
    to ``isaacsim.core.nodes``.  Writer acquisition uses
    ``omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar`` + ``rep.writers.get``
    unchanged from the extscache ``omni.syntheticdata-0.6.15``.

Rate limiting:
    Isaac Sim 6 supports ``omni:sensor:tickRate`` on sensor prims for native multi-tick
    rendering (per NVIDIA's deprecation of frameSkipCount). However, our republisher sits
    OUTSIDE the render pipeline — it throttles an already-published ROS 2 stream — so the
    native tickRate does not supersede it. The hand-rolled gate is retained deliberately.
"""

import threading

import carb
from isaac_core_ogn.sensors.ogn.OgnRos2ImagePublisherDatabase import OgnRos2ImagePublisherDatabase
from isaacsim.core.nodes import BaseWriterNode
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import omni.usd
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | RIPB |"


class _Republisher(Node):
    """Rate-limiting image republisher with frame counter."""

    def __init__(self, raw_topic: str, repub_topic: str, queue_size: int, max_hz: float) -> None:
        if not rclpy.ok():
            try:
                rclpy.init()
            except RuntimeError:
                pass

        super().__init__("isaac_ros2_image_republisher")

        self.lock = threading.Lock()
        self.frame_counter: int = 0
        self.min_period: float = 1.0 / max(max_hz, 1.0)
        self.last_publish_time = self.get_clock().now()
        self.spinning = False

        self.publisher = self.create_publisher(Image, repub_topic, queue_size)
        self.subscriber = self.create_subscription(Image, raw_topic, self._on_image, queue_size)

        self._start_spin()

    def _on_image(self, msg: Image) -> None:
        with self.lock:
            now = self.get_clock().now()
            elapsed = (now - self.last_publish_time).nanoseconds / 1e9
            if elapsed < self.min_period:
                return
            self.last_publish_time = now
            msg.header.frame_id = str(self.frame_counter)
            self.frame_counter += 1
            self.publisher.publish(msg)

    def update_rate(self, hz: float) -> None:
        """Update the maximum publish rate."""
        if hz > 0:
            self.min_period = 1.0 / hz

    def _spin(self) -> None:
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)

    def _start_spin(self) -> None:
        if not self.spinning:
            threading.Thread(target=self._spin, daemon=True).start()
            self.spinning = True

    def shutdown_node(self) -> None:
        """Destroy this ROS 2 node. Does NOT call rclpy.shutdown()."""
        self.spinning = False
        try:
            super().destroy_node()
        except Exception as exc:
            carb.log_warn(f"{_LOG_PREFIX} Republisher destroy error: {exc}")


class _InternalState(BaseWriterNode):
    """Persistent per-node state for Ros2ImagePublisher."""

    def __init__(self) -> None:
        super().__init__(initialize=False)
        self.initialized: bool = False
        self.writer: rep.Writer | None = None
        self.republisher: _Republisher | None = None

    def setup_writer(self, render_product_path: str, queue_size: int, topic_name: str, context: int) -> bool:
        """
        Attach the Replicator ROS 2 RGB writer to the render product.

        Args:
            render_product_path: USD path to the render product prim.
            queue_size: ROS 2 publisher queue size.
            topic_name: Raw RGB topic name.
            context: Render context id.

        Returns:
            ``True`` on success, ``False`` if the path is invalid or writer fails.

        """
        stage = omni.usd.get_context().get_stage()
        if not render_product_path or not stage.GetPrimAtPath(render_product_path).IsValid():
            carb.log_error(f"{_LOG_PREFIX} Invalid render product path: {render_product_path}")
            return False

        try:
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            self.writer = rep.writers.get(rv + "ROS2PublishImage")
            if self.writer is None:
                carb.log_error(f"{_LOG_PREFIX} Writer '{rv}ROS2PublishImage' not found")
                return False

            self.writer.initialize(nodeNamespace="", queueSize=queue_size, topicName=topic_name, context=context)
            self.append_writer(self.writer)
            self.attach_writers(render_product_path)
            self.initialized = True
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Failed to initialize writer: {exc}")
            self.initialized = False
            return False
        else:
            return True

    def reset(self) -> None:
        """Release the writer and republisher."""
        if self.writer:
            self.custom_reset()
            self.writer = None
        if self.republisher is not None:
            self.republisher.shutdown_node()
            self.republisher = None
        self.initialized = False


class OgnRos2ImagePublisher:
    """OmniGraph node: raw RGB writer + rate-limited republisher."""

    @staticmethod
    def internal_state() -> _InternalState:
        """Return persistent per-node state."""
        return _InternalState()

    @staticmethod
    def compute(db: OgnRos2ImagePublisherDatabase) -> bool:
        """Set up writer on first call, manage republisher lifecycle."""
        state: _InternalState = db.internal_state

        if not db.inputs.enabled:
            if state.initialized:
                state.reset()
            return True

        if not state.initialized:
            success = state.setup_writer(
                render_product_path=str(db.inputs.render_product_path),
                queue_size=int(db.inputs.queue_size),
                topic_name=str(db.inputs.raw_topic),
                context=int(db.inputs.context),
            )
            if not success:
                return False

        if state.republisher is None:
            state.republisher = _Republisher(
                raw_topic=str(db.inputs.raw_topic),
                repub_topic=str(db.inputs.repub_topic),
                queue_size=int(db.inputs.queue_size),
                max_hz=float(db.inputs.publish_rate_hz),
            )
        else:
            state.republisher.update_rate(float(db.inputs.publish_rate_hz))

        return True

    @staticmethod
    def release(node: object) -> None:
        """Release writer and republisher resources — same object created in compute."""
        carb.log_info(f"{_LOG_PREFIX} Node release triggered")
        try:
            state = OgnRos2ImagePublisherDatabase.per_node_internal_state(node)
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Node release error: {exc}")
            return

        if state is not None:
            state.reset()
            carb.log_info(f"{_LOG_PREFIX} Node resources released")
