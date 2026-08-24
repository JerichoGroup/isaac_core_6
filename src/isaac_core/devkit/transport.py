"""
Pose transport layer: ship encoded pose packets over the network at a controlled rate.

The vehicle module generates poses purely; this module adds the I/O. The split is
deliberate: vehicle logic stays testable without sockets, and the transport is
reusable with any pose source.

The ``pace()`` helper implements the accumulating-deadline real-time throttle that
the previous generation got right: ``next_time += dt`` rather than ``sleep(dt)``,
which prevents drift over long runs.
"""

from __future__ import annotations

from collections.abc import Iterator
import socket
import time
from typing import Protocol, runtime_checkable

from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT
from isaac_core.contracts.pose import GeodeticPose
from isaac_core.protocol import encode


@runtime_checkable
class PoseTransport(Protocol):
    """
    Protocol for shipping a single encoded pose to a consumer.

    Implementations may send over UDP, record for tests, stream to a file, etc.
    """

    def send(self, pose: GeodeticPose) -> None:
        """
        Transmit one pose.

        Args:
            pose: The geodetic pose to ship. Must have NED orientation for UDP.

        """
        ...  # pragma: no cover

    def close(self) -> None:
        """Release any underlying resources (sockets, files, etc.)."""
        ...  # pragma: no cover


class UdpPoseTransport:
    """
    Send encoded 51-byte pose packets over a UDP socket.

    Uses :func:`isaac_core.protocol.encode` to produce the wire bytes, then sends
    them as a single datagram to the configured host and port.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_POSE_UDP_PORT) -> None:
        """
        Initialise the transport.

        Args:
            host: Destination host.
            port: Destination port.

        """
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None

    @property
    def host(self) -> str:
        """Return the destination host."""
        return self._host

    @property
    def port(self) -> int:
        """Return the destination port."""
        return self._port

    def send(self, pose: GeodeticPose) -> None:
        """
        Encode and send a single pose datagram.

        Creates the socket lazily on first send.

        Args:
            pose: Geodetic pose with NED orientation.

        Raises:
            ValueError: If the pose orientation is not NED (from ``encode``).
            OSError: On socket failure.

        """
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packet = encode(pose)
        self._sock.sendto(packet, (self._host, self._port))

    def close(self) -> None:
        """Close the underlying UDP socket."""
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class FakePoseTransport:
    """
    Record poses instead of sending them, for testing vehicle/mission logic.

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
        """
        Record a pose.

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
    """
    Send poses at a fixed real-time rate using an accumulating deadline.

    Uses ``time.perf_counter`` with ``next_time += dt`` to prevent drift. This is
    the technique the previous generation got right in ``base_udp_sender`` and is
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
    "FakePoseTransport",
    "PoseTransport",
    "UdpPoseTransport",
    "pace",
]
