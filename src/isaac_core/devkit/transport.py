"""Pose transport layer: ship encoded pose packets over the network at a controlled rate.

The vehicle module generates poses purely; this module adds the I/O. The split is
deliberate: vehicle logic stays testable without sockets, and the transport is
reusable with any pose source.

The ``pace()`` helper uses an accumulating deadline -- ``next_time += dt`` rather than
``sleep(dt)`` -- which prevents drift over long runs.
"""

from __future__ import annotations

from collections.abc import Iterator
import socket
import time
from typing import Protocol, runtime_checkable

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT
from isaac_core.contracts.pose import GeodeticPose
from isaac_core.geo.rotations import euler_to_quaternion, ned_to_enu
from isaac_core.protocol import encode


@runtime_checkable
class PoseTransport(Protocol):
    """Protocol for shipping a single encoded pose to a consumer.

    Implementations may send over UDP, record for tests, stream to a file, etc.
    """

    def send(self, pose: GeodeticPose) -> None:
        """Transmit one pose.

        Args:
            pose: The geodetic pose to ship. Must have NED orientation for UDP.

        """
        ...  # pragma: no cover

    def close(self) -> None:
        """Release any underlying resources (sockets, files, etc.)."""
        ...  # pragma: no cover


class UdpPoseTransport:
    """Send encoded 51-byte pose packets over a UDP socket.

    Uses :func:`isaac_core.protocol.encode` to produce the wire bytes, then sends
    them as a single datagram to the configured host and port.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_POSE_UDP_PORT) -> None:
        """Initialise the transport.

        Args:
            host: Destination host.
            port: Destination port.

        """
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._closed: bool = False

    @property
    def host(self) -> str:
        """Return the destination host."""
        return self._host

    @property
    def port(self) -> int:
        """Return the destination port."""
        return self._port

    def send(self, pose: GeodeticPose) -> None:
        """Encode and send a single pose datagram.

        Creates the socket lazily on first send.

        Args:
            pose: Geodetic pose with NED orientation.

        Raises:
            RuntimeError: If the transport has been closed.
            ValueError: If the pose orientation is not NED (from ``encode``).
            OSError: On socket failure.

        """
        # Closing used to only drop the socket, which send() then recreated: a transport used after
        # close silently carried on, and FakePoseTransport -- its drop-in replacement -- raised. A test
        # would catch a use-after-close that production would not.
        if self._closed:
            msg = "transport is closed"
            raise RuntimeError(msg)
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packet = encode(pose)
        self._sock.sendto(packet, (self._host, self._port))

    @property
    def closed(self) -> bool:
        """Return whether close() has been called, matching FakePoseTransport."""
        return self._closed

    def close(self) -> None:
        """Close the underlying UDP socket. Sending afterwards raises."""
        self._closed = True
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class Ros2PoseTransport:
    """Publish a pose the way MAVROS does, for a vehicle whose ``pose_source`` is ``"ros"``.

    Two messages per pose, because that is what the ROS camera layer subscribes to: a
    ``sensor_msgs/NavSatFix`` carrying the position and a ``geometry_msgs/PoseStamped`` carrying the
    attitude. MAVROS publishes ENU already, so the NED orientation this interface receives is
    converted once here -- the same single conversion the UDP path does on the way in.

    ``rclpy`` is imported on first use rather than at module import, because it cannot be imported
    inside Isaac's interpreter and this module is imported there. A host-side script gets a clear
    ImportError naming the fix; the simulator never touches this class.

    Args:
        namespace: MAVROS namespace the vehicle reads, matching ``vehicles.<id>.mavros_namespace``.
        node_name: ROS node name, so two senders in one process do not collide.

    """

    def __init__(self, namespace: str = "/mavros", node_name: str = "isaac_core_pose_sender") -> None:
        """Create the node and both publishers."""
        try:
            from geometry_msgs.msg import PoseStamped
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import NavSatFix
        except ImportError as exc:  # pragma: no cover - depends on the host having ROS 2
            msg = (
                "Ros2PoseTransport needs ROS 2 on the host: source /opt/ros/humble/setup.bash. "
                "It cannot run inside Isaac's interpreter, where rclpy is unimportable by design."
            )
            raise ImportError(msg) from exc

        self._rclpy = rclpy
        self._pose_stamped = PoseStamped
        self._nav_sat_fix = NavSatFix
        if not rclpy.ok():
            rclpy.init()
        self._node = Node(node_name)
        self._namespace = namespace.rstrip("/")
        self._fix_publisher = self._node.create_publisher(NavSatFix, f"{self._namespace}/global_position/global", 10)
        self._pose_publisher = self._node.create_publisher(PoseStamped, f"{self._namespace}/local_position/pose", 10)
        self._closed = False

    @property
    def namespace(self) -> str:
        """Return the MAVROS namespace being published under."""
        return self._namespace

    @property
    def closed(self) -> bool:
        """Return whether close() has been called."""
        return self._closed

    def send(self, pose: GeodeticPose) -> None:
        """Publish one position and one attitude message.

        Args:
            pose: The pose to publish. NED orientation, converted to ENU here.

        Raises:
            RuntimeError: If the transport has been closed.

        """
        if self._closed:
            msg = "transport is closed"
            raise RuntimeError(msg)

        stamp = self._node.get_clock().now().to_msg()

        fix = self._nav_sat_fix()
        fix.header.stamp = stamp
        fix.header.frame_id = "base_link"
        fix.latitude = pose.position.lat_deg
        fix.longitude = pose.position.lon_deg
        fix.altitude = pose.position.alt_m
        self._fix_publisher.publish(fix)

        enu = ned_to_enu(pose.orientation)
        w, x, y, z = euler_to_quaternion(enu.roll_r, enu.pitch_r, enu.yaw_r, RotationFrame.WORLD)
        attitude = self._pose_stamped()
        attitude.header.stamp = stamp
        attitude.header.frame_id = "map"
        attitude.pose.orientation.w = w
        attitude.pose.orientation.x = x
        attitude.pose.orientation.y = y
        attitude.pose.orientation.z = z
        self._pose_publisher.publish(attitude)

    def close(self) -> None:
        """Destroy the node. Sending afterwards raises."""
        if self._closed:
            return
        self._closed = True
        self._node.destroy_node()


class FakePoseTransport:
    """Record poses instead of sending them, for testing vehicle/mission logic.

    Acts as a drop-in for :class:`UdpPoseTransport` in test harnesses.
    """

    def __init__(self) -> None:
        """Initialise with an empty history."""
        self._history: list[GeodeticPose] = []
        self._closed: bool = False

    @property
    def history(self) -> list[GeodeticPose]:
        """Return the list of poses that were sent."""
        return self._history

    @property
    def closed(self) -> bool:
        """Return whether close() has been called."""
        return self._closed

    def send(self, pose: GeodeticPose) -> None:
        """Record a pose.

        Args:
            pose: The pose to record.

        Raises:
            RuntimeError: If the transport has been closed.

        """
        if self._closed:
            msg = "transport is closed"
            raise RuntimeError(msg)
        self._history.append(pose)

    def close(self) -> None:
        """Mark the transport as closed."""
        self._closed = True


def pace(
    poses: Iterator[GeodeticPose],
    transport: PoseTransport,
    rate_hz: float,
) -> int:
    """Send poses at a fixed real-time rate using an accumulating deadline.

    Uses ``time.perf_counter`` with ``next_time += dt`` to prevent drift. This is
    the technique that avoids compounding jitter and is
    worth preserving: each iteration targets an absolute deadline rather than
    sleeping a relative duration, so accumulated jitter cannot compound.

    Args:
        poses: An iterator yielding geodetic poses. Exhaustion ends the function.
        transport: Where to send each pose.
        rate_hz: Desired output rate in Hz (must be positive).

    Returns:
        The number of poses sent.

    Raises:
        ValueError: If ``rate_hz`` is not positive.

    """
    if rate_hz <= 0.0:
        msg = f"rate_hz must be positive, got {rate_hz}"
        raise ValueError(msg)

    dt = 1.0 / rate_hz
    count = 0
    next_time = time.perf_counter()

    for pose in poses:
        # Wait until the next deadline.
        now = time.perf_counter()
        sleep_duration = next_time - now
        if sleep_duration > 0:
            time.sleep(sleep_duration)

        transport.send(pose)
        count += 1
        next_time += dt

    return count


__all__ = [
    "Ros2PoseTransport",
    "FakePoseTransport",
    "PoseTransport",
    "UdpPoseTransport",
    "pace",
]
