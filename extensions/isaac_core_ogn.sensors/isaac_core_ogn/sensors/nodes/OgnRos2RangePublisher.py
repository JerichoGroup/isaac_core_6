"""
OmniGraph node: publish sensor_msgs/Range to a ROS 2 topic.

Thin adapter: reads the range input, rate-limits, and publishes.

Rate limiting:
    Isaac Sim 6 supports ``omni:sensor:tickRate`` on sensor prims for native multi-tick
    rendering. That mechanism controls render-product output frequency, not arbitrary
    ROS 2 publish rates for non-camera data. The hand-rolled time gate is retained.

rclpy lifetime: this node only creates and destroys its own rclpy node handle.
It never calls ``rclpy.shutdown()``.
"""

import threading

import carb
from isaac_core_ogn.sensors.ogn.OgnRos2RangePublisherDatabase import OgnRos2RangePublisherDatabase
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | RRPB |"

# sensor_msgs/Range.INFRARED = 1
_RADIATION_TYPE_INFRARED = 1


class _InternalState:
    """Persistent per-node state for Ros2RangePublisher."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")

        self.ros_node = rclpy.create_node("range_publisher")
        try:
            self.ros_node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.lock = threading.Lock()
        self.spinning = False
        self.publisher = None
        self.last_publish_time = None
        self.publish_period: float = 0.02
        self.min_range: float = 0.2
        self.max_range: float = 180.0
        self.frame_id_counter: int = 0

    def configure(self, topic_name: str, hz: int, min_range: float, max_range: float) -> None:
        """
        Create the ROS 2 publisher and start spinning.

        Args:
            topic_name: Topic to publish on.
            hz: Publishing rate in Hz.
            min_range: Minimum valid range.
            max_range: Maximum valid range.

        """
        self.publish_period = 1.0 / max(hz, 1)
        self.min_range = min_range
        self.max_range = max_range
        self.publisher = self.ros_node.create_publisher(Range, topic_name, qos_profile_sensor_data)
        if not self.spinning:
            threading.Thread(target=self._spin, daemon=True).start()
            self.spinning = True

    def publish_range(self, range_value: float) -> None:
        """
        Publish the range if enough time has elapsed.

        Args:
            range_value: Current distance measurement in metres.

        """
        with self.lock:
            now = self.ros_node.get_clock().now()
            if self.last_publish_time is not None:
                elapsed = (now - self.last_publish_time).nanoseconds / 1e9
                if elapsed < self.publish_period:
                    return
            self.last_publish_time = now

            msg = Range()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = str(self.frame_id_counter)
            msg.radiation_type = _RADIATION_TYPE_INFRARED
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = range_value

            self.publisher.publish(msg)
            self.frame_id_counter += 1

    def _spin(self) -> None:
        executor = MultiThreadedExecutor()
        executor.add_node(self.ros_node)
        executor.spin()

    def destroy(self) -> None:
        """Destroy the ROS 2 node handle. Does NOT call rclpy.shutdown()."""
        try:
            self.ros_node.destroy_node()
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Failed to destroy ROS 2 node: {exc}")


class OgnRos2RangePublisher:
    """OmniGraph node: ROS 2 range publisher."""

    @staticmethod
    def internal_state() -> _InternalState:
        """Return persistent per-node state, ensuring rclpy is initialised."""
        if not rclpy.ok():
            try:
                rclpy.init()
            except RuntimeError as exc:
                carb.log_error(f"{_LOG_PREFIX} Failed to initialize rclpy: {exc}")

        return _InternalState()

    @staticmethod
    def compute(db: OgnRos2RangePublisherDatabase) -> bool:
        """Publish the range at the configured rate."""
        state: _InternalState = db.internal_state

        if state.publisher is None:
            state.configure(
                topic_name=str(db.inputs.topic_name),
                hz=int(db.inputs.publish_rate_hz),
                min_range=float(db.inputs.min_range),
                max_range=float(db.inputs.max_range),
            )

        state.publish_range(float(db.inputs.range_value))
        return True

    @staticmethod
    def release(node: object) -> None:
        """Destroy the ROS 2 node handle only — never call rclpy.shutdown()."""
        carb.log_info(f"{_LOG_PREFIX} Node release triggered")
        try:
            state = OgnRos2RangePublisherDatabase.per_node_internal_state(node)
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Node release error: {exc}")
            return

        if state is not None:
            state.destroy()
            carb.log_info(f"{_LOG_PREFIX} Node resources released")
