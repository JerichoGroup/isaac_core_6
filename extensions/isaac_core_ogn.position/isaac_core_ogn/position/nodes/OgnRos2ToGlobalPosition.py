"""
OmniGraph node: subscribe to ROS 2 topics, output global position and ENU orientation.

Thin adapter: ROS 2 subscriptions deliver NavSatFix + PoseStamped from MAVROS, and
isaac_core.geo.quaternion_to_euler extracts Euler angles.

MAVROS already publishes orientation in ENU, so NO NED-to-ENU conversion is applied
on this path. This asymmetry with the UDP node is intentional and correct: the UDP
wire protocol carries NED (matching the aerospace convention), while MAVROS converts
to ENU internally before publishing PoseStamped.

rclpy lifetime: this node creates and destroys ONLY its own rclpy Node handle.
It never calls rclpy.shutdown() — that would tear the shared context out from under
other nodes running in the same process (old repo defect #3).
"""

import threading

import carb
from geometry_msgs.msg import PoseStamped
from isaac_core_ogn.position.ogn.OgnRos2ToGlobalPositionDatabase import OgnRos2ToGlobalPositionDatabase
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

from isaac_core.geo import quaternion_to_euler

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | RTGP |"


class _InternalState:
    """Persistent per-node state for Ros2ToGlobalPosition."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")

        self.ros_node: rclpy.node.Node = rclpy.create_node("ros2_global_position_subscriber")
        try:
            self.ros_node.declare_parameter("use_sim_time", True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.lock: threading.Lock = threading.Lock()
        self._executor: MultiThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._subscriptions_created: bool = False

        self.latest_lat: float = 0.0
        self.latest_lon: float = 0.0
        self.latest_alt: float = 0.0
        self.latest_qw: float = 1.0
        self.latest_qx: float = 0.0
        self.latest_qy: float = 0.0
        self.latest_qz: float = 0.0

        self._qos_profile: QoSProfile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _lla_callback(self, msg: NavSatFix) -> None:
        with self.lock:
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
            self.latest_alt = msg.altitude

    def _orientation_callback(self, msg: PoseStamped) -> None:
        with self.lock:
            q = msg.pose.orientation
            self.latest_qw = q.w
            self.latest_qx = q.x
            self.latest_qy = q.y
            self.latest_qz = q.z

    def create_subscriptions(self, lla_topic: str, orientation_topic: str) -> None:
        """
        Create subscriptions and start the executor if not already running.

        Args:
            lla_topic: ROS 2 topic name for NavSatFix messages.
            orientation_topic: ROS 2 topic name for PoseStamped messages.

        """
        if self._subscriptions_created:
            return

        carb.log_info(f"{_LOG_PREFIX} Subscribing to LLA: {lla_topic}, orientation: {orientation_topic}")
        self.ros_node.create_subscription(NavSatFix, lla_topic, self._lla_callback, self._qos_profile)
        self.ros_node.create_subscription(PoseStamped, orientation_topic, self._orientation_callback, self._qos_profile)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.ros_node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._subscriptions_created = True

    def destroy(self) -> None:
        """
        Shut down the executor and destroy the ROS 2 node handle.

        Does NOT call rclpy.shutdown() — only this node's own resources are freed.
        """
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None

        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

        try:
            self.ros_node.destroy_node()
        except rclpy.handle.InvalidHandle:
            pass


class OgnRos2ToGlobalPosition:
    """OmniGraph node: ROS 2 NavSatFix + PoseStamped to global position and ENU orientation."""

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
    def compute(db: OgnRos2ToGlobalPositionDatabase) -> bool:
        """
        Subscribe to topics and write latest position/orientation.

        MAVROS publishes orientation already in ENU (PoseStamped from
        /mavros/local_position/pose), so no NED-to-ENU frame conversion is
        applied here. The quaternion is decomposed to Euler angles directly.
        """
        state: _InternalState = db.internal_state
        state.create_subscriptions(str(db.inputs.lla_topic), str(db.inputs.orientation_topic))

        with state.lock:
            lat = state.latest_lat
            lon = state.latest_lon
            alt = state.latest_alt
            qw = state.latest_qw
            qx = state.latest_qx
            qy = state.latest_qy
            qz = state.latest_qz

        # MAVROS quaternion is already ENU — extract Euler directly, no frame swap.
        roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)

        db.outputs.global_position = [lat, lon, alt]
        db.outputs.global_orientation = [roll, pitch, yaw]
        return True

    @staticmethod
    def release(node: object) -> None:
        """Destroy the ROS 2 node handle and stop the executor — never call rclpy.shutdown()."""
        carb.log_info(f"{_LOG_PREFIX} Node release")
        try:
            state = OgnRos2ToGlobalPositionDatabase.per_node_internal_state(node)
        except RuntimeError:
            return

        if state is not None:
            state.destroy()
