"""Tests for the ROS 2 pose transport.

The point of this class is that a pose sender does not care which wire it uses: the same `PoseBot`
drives a UDP vehicle and a ROS one. So the tests that matter are the interface ones, which need no
ROS, plus a live publish when ROS 2 happens to be available.
"""

from __future__ import annotations

import importlib.util
import math
from typing import Final

import pytest

from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import PoseTransport, Ros2PoseTransport

_ROS_AVAILABLE: Final = importlib.util.find_spec("rclpy") is not None
_needs_ros = pytest.mark.skipif(not _ROS_AVAILABLE, reason="ROS 2 is not sourced")


def test_the_class_imports_without_ros_being_available() -> None:
    # This module is imported inside Isaac's interpreter, where rclpy cannot be imported at all. A
    # module-level `import rclpy` would therefore break the simulator, so the import is deferred to
    # construction and only a host-side caller ever pays for it.
    assert Ros2PoseTransport is not None


def test_it_satisfies_the_transport_protocol() -> None:
    # PoseTransport is runtime-checkable, so this is a real structural check rather than a comment.
    assert issubclass(Ros2PoseTransport, PoseTransport)


@pytest.mark.skipif(_ROS_AVAILABLE, reason="this checks the error when ROS is absent")
def test_without_ros_it_raises_an_error_naming_the_fix() -> None:
    with pytest.raises(ImportError, match="setup.bash"):
        Ros2PoseTransport()


@_needs_ros
def test_it_publishes_on_the_topics_the_ros_camera_layer_subscribes_to() -> None:
    # The layer resolves lla_topic and orientation_topic from the vehicle's mavros_namespace, so these
    # two names are the contract between sender and simulator.
    transport = Ros2PoseTransport(namespace="/test_mavros", node_name="isaac_core_transport_test")
    try:
        assert transport.namespace == "/test_mavros"
        published = {
            name
            for name, _type in transport._node.get_publisher_names_and_types_by_node(
                transport._node.get_name(),
                transport._node.get_namespace(),
            )
        }
        assert "/test_mavros/global_position/global" in published
        assert "/test_mavros/local_position/pose" in published
    finally:
        transport.close()


@_needs_ros
def test_a_trailing_slash_in_the_namespace_does_not_double_up() -> None:
    transport = Ros2PoseTransport(namespace="/test_mavros/", node_name="isaac_core_slash_test")
    try:
        assert transport.namespace == "/test_mavros"
    finally:
        transport.close()


@_needs_ros
def test_sending_after_close_raises_like_the_other_transports() -> None:
    transport = Ros2PoseTransport(namespace="/test_mavros", node_name="isaac_core_close_test")
    transport.close()
    assert transport.closed is True
    with pytest.raises(RuntimeError, match="closed"):
        transport.send(
            GeodeticPose(
                position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=1000.0),
                orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
            )
        )


@_needs_ros
def test_closing_twice_is_harmless() -> None:
    transport = Ros2PoseTransport(namespace="/test_mavros", node_name="isaac_core_double_close")
    transport.close()
    transport.close()
    assert transport.closed is True


@_needs_ros
def test_a_pose_is_published_as_position_and_attitude() -> None:
    # Subscribe to our own publisher and read both messages back, so the conversion is measured
    # rather than assumed: MAVROS publishes ENU, and this interface receives NED.
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix

    if not rclpy.ok():
        rclpy.init()
    listener = Node("isaac_core_transport_listener")
    fixes: list[NavSatFix] = []
    attitudes: list[PoseStamped] = []
    listener.create_subscription(NavSatFix, "/read_back/global_position/global", fixes.append, 10)
    listener.create_subscription(PoseStamped, "/read_back/local_position/pose", attitudes.append, 10)

    transport = Ros2PoseTransport(namespace="/read_back", node_name="isaac_core_transport_writer")
    try:
        pose = GeodeticPose(
            position=Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1234.0),
            orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=math.radians(90.0)),
        )
        deadline = 8.0
        waited = 0.0
        while (not fixes or not attitudes) and waited < deadline:
            transport.send(pose)
            rclpy.spin_once(listener, timeout_sec=0.1)
            waited += 0.1

        assert fixes, "no NavSatFix arrived"
        assert attitudes, "no PoseStamped arrived"
        assert fixes[-1].latitude == pytest.approx(32.22481)
        assert fixes[-1].longitude == pytest.approx(35.25621)
        assert fixes[-1].altitude == pytest.approx(1234.0)

        # A NED yaw of +90 is ENU yaw 0, so the published quaternion is the identity rotation.
        orientation = attitudes[-1].pose.orientation
        assert orientation.w == pytest.approx(1.0, abs=1e-6)
        assert orientation.z == pytest.approx(0.0, abs=1e-6)
    finally:
        transport.close()
        listener.destroy_node()
