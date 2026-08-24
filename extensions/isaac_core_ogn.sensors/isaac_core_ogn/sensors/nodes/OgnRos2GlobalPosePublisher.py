"""
OmniGraph node: publish global pose to a ROS 2 GeoPoseStamped topic.

Thin adapter: reads position/orientation inputs, rate-limits, and publishes.

LEGACY PROTOCOL QUIRK (preserved deliberately):
    The GeoPoseStamped.pose.orientation quaternion fields are ABUSED to carry
    Euler angles in degrees: x=roll, y=pitch, z=yaw, w=1.0 (unused).  This is
    NOT a valid quaternion — it is a documented interface that downstream consumers
    (the dev kit, recording tools, and the RTP metadata channel) depend on.
    Changing it would silently break every consumer.  See KIRO.md §3 and §5 defect #4.

Rate limiting:
    Isaac Sim 6 supports ``omni:sensor:tickRate`` on sensor prims for native multi-tick
    rendering. That mechanism controls render-product output frequency, not arbitrary
    ROS 2 publish rates for non-camera data. The hand-rolled time gate is retained.

rclpy lifetime: this node only creates and destroys its own rclpy node handle.
It never calls ``rclpy.shutdown()``.
"""

import math
import threading

import carb
from geographic_msgs.msg import GeoPoseStamped
from isaac_core_ogn.sensors.ogn.OgnRos2GlobalPosePublisherDatabase import OgnRos2GlobalPosePublisherDatabase
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | RGPP |"


class _InternalState:
    """Persistent per-node state for Ros2GlobalPosePublisher."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")

        self.ros_node = rclpy.create_node("global_pose_publisher")
        try:
            self.ros_node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.lock = threading.Lock()
        self.spinning = False
        self.publisher = None
        self.last_publish_time = None
        self.publish_period: float = 0.1
        self.frame_id_counter: int = 0

    def configure(self, topic_name: str, hz: int) -> None:
        """
        Create the ROS 2 publisher and start spinning if not already done.

        Args:
            topic_name: Topic to publish on.
            hz: Publishing rate in Hz.

        """
        self.publish_period = 1.0 / max(hz, 1)
        self.publisher = self.ros_node.create_publisher(GeoPoseStamped, topic_name, qos_profile_sensor_data)
        if not self.spinning:
            threading.Thread(target=self._spin, daemon=True).start()
            self.spinning = True

    def publish_pose(self, position: tuple[float, ...], orientation: tuple[float, ...]) -> None:
        """
        Publish the pose if enough time has elapsed since the last publish.

        Args:
            position: ``(lat_deg, lon_deg, alt_m)``.
            orientation: ``(roll_r, pitch_r, yaw_r)`` in ENU radians.

        """
        with self.lock:
            now = self.ros_node.get_clock().now()
            if self.last_publish_time is not None:
                elapsed = (now - self.last_publish_time).nanoseconds / 1e9
                if elapsed < self.publish_period:
                    return
            self.last_publish_time = now

            msg = GeoPoseStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = str(self.frame_id_counter)

            msg.pose.position.latitude = float(position[0])
            msg.pose.position.longitude = float(position[1])
            msg.pose.position.altitude = float(position[2])

            # LEGACY PROTOCOL QUIRK: degrees stuffed into quaternion x/y/z, w=1.0.
            # This is NOT a valid quaternion. See module docstring for rationale.
            msg.pose.orientation.x = math.degrees(float(orientation[0]))
            msg.pose.orientation.y = math.degrees(float(orientation[1]))
            msg.pose.orientation.z = math.degrees(float(orientation[2]))
            msg.pose.orientation.w = 1.0

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


class OgnRos2GlobalPosePublisher:
    """OmniGraph node: global pose publisher over ROS 2."""

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
    def compute(db: OgnRos2GlobalPosePublisherDatabase) -> bool:
        """Publish the global pose at the configured rate."""
        state: _InternalState = db.internal_state

        if state.publisher is None:
            state.configure(str(db.inputs.topic_name), int(db.inputs.hz))

        state.publish_pose(tuple(db.inputs.global_position), tuple(db.inputs.global_orientation))
        return True

    @staticmethod
    def release(node: object) -> None:
        """Destroy the ROS 2 node handle only — never call rclpy.shutdown()."""
        carb.log_info(f"{_LOG_PREFIX} Node release triggered")
        try:
            state = OgnRos2GlobalPosePublisherDatabase.per_node_internal_state(node)
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Node release error: {exc}")
            return

        if state is not None:
            state.destroy()
            carb.log_info(f"{_LOG_PREFIX} Node resources released")
