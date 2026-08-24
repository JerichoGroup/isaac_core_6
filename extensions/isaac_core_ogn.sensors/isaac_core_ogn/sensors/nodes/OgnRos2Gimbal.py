"""
OmniGraph node: subscribe to a ROS 2 gimbal topic and output offsets in degrees.

Use start_* values until the first message arrives. The downstream math node
(GlobalPositionToLocalPosition) uses these degrees as gimbal offset inputs and
applies them via compose_rotation with the configured RotationFrame (D14).

rclpy lifetime: this node only creates and destroys its own rclpy node handle.
It never calls ``rclpy.shutdown()``.
"""

import threading

import carb
from isaac_core_ogn.sensors.ogn.OgnRos2GimbalDatabase import OgnRos2GimbalDatabase
from isaac_ros2_messages.msg import Gimbal
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | RG  |"


class _InternalState:
    """Persistent per-node state for Ros2Gimbal."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")

        self.ros_node = rclpy.create_node("ros2_gimbal_subscriber")
        try:
            self.ros_node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.lock = threading.Lock()
        self.spinning = False
        self.subscription = None
        self.received_first_msg = False
        self.latest_roll_deg: float = 0.0
        self.latest_pitch_deg: float = 0.0
        self.latest_yaw_deg: float = 0.0

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _gimbal_callback(self, msg: Gimbal) -> None:
        with self.lock:
            self.latest_roll_deg = msg.roll
            self.latest_pitch_deg = msg.pitch
            self.latest_yaw_deg = msg.yaw
            self.received_first_msg = True

    def subscribe(self, gimbal_topic: str) -> None:
        """
        Create the subscription if not already created and start spinning.

        Args:
            gimbal_topic: ROS 2 topic name for Gimbal messages.

        """
        if self.subscription is None:
            carb.log_info(f"{_LOG_PREFIX} Subscribing to gimbal topic: {gimbal_topic}")
            self.subscription = self.ros_node.create_subscription(
                Gimbal, gimbal_topic, self._gimbal_callback, self.qos_profile
            )

        if not self.spinning:
            threading.Thread(target=self._spin, daemon=True).start()
            self.spinning = True

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


class OgnRos2Gimbal:
    """OmniGraph node: ROS 2 gimbal subscriber outputting degrees."""

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
    def compute(db: OgnRos2GimbalDatabase) -> bool:
        """Subscribe and output gimbal angles (start_* before first message)."""
        state: _InternalState = db.internal_state
        state.subscribe(str(db.inputs.gimbal_topic))

        with state.lock:
            if not state.received_first_msg:
                db.outputs.roll_deg = float(db.inputs.start_roll_deg)
                db.outputs.pitch_deg = float(db.inputs.start_pitch_deg)
                db.outputs.yaw_deg = float(db.inputs.start_yaw_deg)
            else:
                db.outputs.roll_deg = state.latest_roll_deg
                db.outputs.pitch_deg = state.latest_pitch_deg
                db.outputs.yaw_deg = state.latest_yaw_deg

        return True

    @staticmethod
    def release(node: object) -> None:
        """Destroy the ROS 2 node handle only — never call rclpy.shutdown()."""
        carb.log_info(f"{_LOG_PREFIX} Node release triggered")
        try:
            state = OgnRos2GimbalDatabase.per_node_internal_state(node)
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Node release error: {exc}")
            return

        if state is not None:
            state.destroy()
            carb.log_info(f"{_LOG_PREFIX} Node resources released")
