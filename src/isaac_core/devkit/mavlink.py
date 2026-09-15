"""Host-side MAVLink pose bridge.

MAVLink cannot be decoded inside Isaac Sim: ``pymavlink`` is not available in
Isaac's bundled Python 3.12, only on the host Python 3.10. So rather than teach the
simulator about MAVLink, this bridge runs on the host, reads a MAVLink stream, and
re-emits each pose as the repo's existing 51-byte UDP pose packet aimed at the
vehicle's pose port. The simulator therefore needs no MAVLink knowledge and no new
dependency -- it just receives the same packets it already understands.

Two MAVLink messages supply a pose. ``GLOBAL_POSITION_INT`` carries position and
``ATTITUDE`` carries orientation; they arrive independently and at different rates,
so the bridge holds the most recent of each and emits their combination. MAVLink
attitude is already NED radians, which is exactly what the wire format expects, so
no frame conversion is needed -- only the integer unit scaling on position.

``pymavlink`` is imported lazily inside :func:`_require_pymavlink` so that importing
this module (and running the test suite) never requires it, matching the ``rclpy``
handling in :mod:`isaac_core.devkit.recording`.
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import TYPE_CHECKING, Any

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import PoseTransport, UdpPoseTransport

if TYPE_CHECKING:
    from collections.abc import Callable

logger = logging.getLogger(__name__)

# MAVLink GLOBAL_POSITION_INT reports latitude and longitude as degrees scaled by 1e7
# (integer degE7); divide by this to recover decimal degrees.
DEG_E7_PER_DEG = 1e7

# MAVLink GLOBAL_POSITION_INT reports altitude as millimetres above mean sea level
# (integer); divide by this to recover metres, which is what the wire format carries.
MM_PER_M = 1e3

# Default paced output rate (Hz) at which combined poses are re-emitted to the sim.
DEFAULT_SEND_RATE_HZ = 30.0

# The two MAVLink message types the bridge consumes.
_POSITION_MSG = "GLOBAL_POSITION_INT"
_ATTITUDE_MSG = "ATTITUDE"


def _require_pymavlink() -> Any:
    """Import and return ``pymavlink.mavutil``, raising a clear error if unavailable.

    Returns:
        The ``pymavlink.mavutil`` module.

    Raises:
        ImportError: With an actionable message if pymavlink is not importable.

    """
    try:
        from pymavlink import mavutil
    except ImportError:
        msg = (
            "pymavlink is not available. MavlinkPoseBridge requires pymavlink, which "
            "runs on the host Python (not inside Isaac Sim). Install it with "
            "'pip install --user pymavlink'."
        )
        raise ImportError(msg) from None
    return mavutil


def connect(connection_string: str) -> Any:
    """Open a MAVLink connection, raising a clear error naming the endpoint on failure.

    Args:
        connection_string: A pymavlink connection string, e.g. ``udpin:0.0.0.0:14550``
            or ``tcp:127.0.0.1:5760``.

    Returns:
        The connected pymavlink connection object.

    Raises:
        ConnectionError: If the connection cannot be established, naming the string.

    """
    mavutil = _require_pymavlink()
    try:
        return mavutil.mavlink_connection(connection_string)
    except Exception as exc:
        msg = f"failed to open MAVLink connection {connection_string!r}: {exc}"
        raise ConnectionError(msg) from exc


class MavlinkPoseBridge:
    """Bridge a MAVLink pose stream to the simulator's UDP pose port.

    Holds the most recent ``GLOBAL_POSITION_INT`` and ``ATTITUDE`` messages, combines
    them into a :class:`GeodeticPose`, and ships that pose through a
    :class:`~isaac_core.devkit.transport.PoseTransport`. It never emits until at least
    one message of *each* type has arrived, so it can never place the aircraft at the
    all-zero pose (lat 0, lon 0 -- a real point in the Atlantic).

    Usage::

        bridge = MavlinkPoseBridge.connect("udpin:0.0.0.0:14550")
        bridge.run()          # blocks until stop() is called from another thread
    """

    def __init__(
        self,
        connection: Any,
        transport: PoseTransport,
        *,
        rate_hz: float = DEFAULT_SEND_RATE_HZ,
    ) -> None:
        """Initialise the bridge with an open connection and a transport.

        Args:
            connection: A duck-typed MAVLink connection exposing ``recv_match``.
            transport: Where to send each combined pose (typically a
                :class:`~isaac_core.devkit.transport.UdpPoseTransport`).
            rate_hz: Paced output rate in Hz for the send loop; must be positive.

        Raises:
            ValueError: If ``rate_hz`` is not positive.

        """
        if rate_hz <= 0.0:
            msg = f"rate_hz must be positive, got {rate_hz}"
            raise ValueError(msg)
        self._connection = connection
        self._transport = transport
        self._rate_hz = rate_hz
        self._position: Lla | None = None
        self._attitude: Rpy | None = None
        self._running = False

    @classmethod
    def connect(
        cls,
        connection_string: str,
        *,
        host: str = "127.0.0.1",
        port: int = DEFAULT_POSE_UDP_PORT,
        rate_hz: float = DEFAULT_SEND_RATE_HZ,
    ) -> MavlinkPoseBridge:
        """Build a bridge that reads ``connection_string`` and sends UDP to ``host:port``.

        Args:
            connection_string: A pymavlink connection string, e.g.
                ``udpin:0.0.0.0:14550``.
            host: Destination host for the simulator's pose port.
            port: Destination UDP pose port.
            rate_hz: Paced output rate in Hz.

        Returns:
            A ready-to-run :class:`MavlinkPoseBridge`.

        Raises:
            ImportError: If pymavlink is not installed.
            ConnectionError: If the endpoint cannot be opened.

        """
        connection = connect(connection_string)
        transport = UdpPoseTransport(host=host, port=port)
        return cls(connection, transport, rate_hz=rate_hz)

    @property
    def has_pose(self) -> bool:
        """Return whether both a position and an attitude have been received."""
        return self._position is not None and self._attitude is not None

    def _apply(self, message: Any) -> None:
        """Update the held state from one MAVLink message, ignoring unknown types.

        ``GLOBAL_POSITION_INT`` updates the held position (degE7 -> degrees, mm ->
        metres) and ``ATTITUDE`` updates the held orientation (already NED radians).
        Any other message type is ignored so an unexpected stream cannot crash the
        loop.

        Args:
            message: A decoded MAVLink message exposing ``get_type()`` and fields.

        """
        msg_type = message.get_type()
        if msg_type == _POSITION_MSG:
            self._position = Lla(
                lat_deg=message.lat / DEG_E7_PER_DEG,
                lon_deg=message.lon / DEG_E7_PER_DEG,
                alt_m=message.alt / MM_PER_M,
            )
        elif msg_type == _ATTITUDE_MSG:
            self._attitude = Rpy(
                roll_r=message.roll,
                pitch_r=message.pitch,
                yaw_r=message.yaw,
                frame=Frame.NED,
            )

    def current_pose(self) -> GeodeticPose | None:
        """Return the combined pose from the latest of each message, or ``None``.

        Returns ``None`` until both a position and an attitude have arrived, which is
        the guard that prevents ever emitting the all-zero Atlantic pose.

        Returns:
            The combined :class:`GeodeticPose`, or ``None`` if not yet complete.

        """
        if self._position is None or self._attitude is None:
            return None
        return GeodeticPose(position=self._position, orientation=self._attitude)

    def poll_once(self) -> GeodeticPose | None:
        """Drain all pending MAVLink messages, then return the current combined pose.

        Reads every message currently available (non-blocking) so the held state
        reflects the most recent of each type, then returns the combined pose.

        Returns:
            The combined pose, or ``None`` if a complete pose is not yet available.

        """
        while True:
            message = self._connection.recv_match(blocking=False)
            if message is None:
                break
            self._apply(message)
        return self.current_pose()

    def stop(self) -> None:
        """Signal the run loop to exit after its current iteration."""
        self._running = False

    def run(self, *, sleep: Callable[[float], None] | None = None) -> int:
        """Read MAVLink and re-emit poses at the configured rate until stopped.

        Uses an accumulating deadline (``next_time += dt``) to avoid drift, matching
        :func:`isaac_core.devkit.transport.pace`. A pose is sent only once a complete
        one is available; before that the loop keeps reading without emitting.

        Args:
            sleep: Injectable sleep function taking seconds, for testing. Defaults to
                :func:`time.sleep`.

        Returns:
            The number of poses sent before the loop stopped.

        """

        sleep_fn = sleep if sleep is not None else time.sleep
        dt = 1.0 / self._rate_hz
        sent = 0
        self._running = True
        next_time = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            wait = next_time - now
            if wait > 0:
                sleep_fn(wait)
            if not self._running:
                break
            pose = self.poll_once()
            if pose is not None:
                self._transport.send(pose)
                sent += 1
            next_time += dt
        return sent

    def close(self) -> None:
        """Close the transport and, if it supports it, the MAVLink connection."""
        self._transport.close()
        closer = getattr(self._connection, "close", None)
        if callable(closer):
            closer()


def main(argv: list[str] | None = None) -> int:
    """Run the MAVLink pose bridge from the command line.

    Args:
        argv: Argument vector for testing. Defaults to ``sys.argv[1:]``.

    Returns:
        A process exit code: ``0`` on clean stop, ``1`` on a handled error.

    """

    parser = argparse.ArgumentParser(
        prog="isaac-core-mavlink",
        description="Bridge a MAVLink pose stream to the Isaac Sim UDP pose port.",
    )
    parser.add_argument(
        "connection",
        help="pymavlink connection string, e.g. 'udpin:0.0.0.0:14550' or 'tcp:127.0.0.1:5760'",
    )
    parser.add_argument("--host", default="127.0.0.1", help="destination host for the pose port")
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_POSE_UDP_PORT,
        help="destination UDP pose port",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_SEND_RATE_HZ,
        help="paced output rate in Hz",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO)
    try:
        bridge = MavlinkPoseBridge.connect(
            args.connection,
            host=args.host,
            port=args.port,
            rate_hz=args.rate_hz,
        )
    except (ImportError, ConnectionError) as exc:
        logger.error("%s", exc)
        return 1

    logger.info(
        "bridging MAVLink %s -> udp %s:%d at %.1f Hz (waiting for first position and attitude)",
        args.connection,
        args.host,
        args.port,
        args.rate_hz,
    )
    try:
        sent = bridge.run()
    except KeyboardInterrupt:
        bridge.stop()
        sent = 0
    finally:
        bridge.close()
    logger.info("bridge stopped after sending %d poses", sent)
    return 0


__all__ = [
    "DEFAULT_SEND_RATE_HZ",
    "DEG_E7_PER_DEG",
    "MM_PER_M",
    "MavlinkPoseBridge",
    "connect",
    "main",
]
