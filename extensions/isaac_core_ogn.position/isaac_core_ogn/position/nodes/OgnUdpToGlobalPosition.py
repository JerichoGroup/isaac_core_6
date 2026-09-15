"""OmniGraph node: receive pose over UDP, output global position and ENU orientation.

Thin adapter over isaac_core.protocol (packet decoding) and isaac_core.geo
(NED-to-ENU conversion). The socket is the only I/O this module owns; all
parsing, validation and frame conversion are delegated to tested kernel code.
"""

import socket

import carb
from isaac_core_ogn.position.ogn.OgnUdpToGlobalPositionDatabase import OgnUdpToGlobalPositionDatabase

from isaac_core.contracts.packet import PACKET_SIZE
from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT
from isaac_core.geo import ned_to_enu
from isaac_core.protocol import HoldLastGoodDecoder

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | UTGP |"

# Bind address for the UDP socket (all interfaces).
_BIND_ADDRESS = "0.0.0.0"


class _InternalState:
    """Persistent per-node state for UdpToGlobalPosition."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")
        self.decoder: HoldLastGoodDecoder = HoldLastGoodDecoder()
        self.socket: socket.socket | None = None
        self.bound_port: int | None = None

    def ensure_socket(self, port: int) -> socket.socket:
        """Lazily create the UDP socket, binding to the requested port.

        If the port changes at runtime the previous socket is closed and a new
        one is bound.

        Args:
            port: UDP port to listen on.

        Returns:
            The bound, non-blocking socket.

        """
        if self.socket is None or self.bound_port != port:
            self.close()
            carb.log_info(f"{_LOG_PREFIX} Binding UDP socket to {_BIND_ADDRESS}:{port}")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((_BIND_ADDRESS, port))
            sock.setblocking(False)
            self.socket = sock
            self.bound_port = port
        return self.socket

    def close(self) -> None:
        """Close the socket if open."""
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError as exc:
                carb.log_warn(f"{_LOG_PREFIX} Error closing socket: {exc}")
            self.socket = None
            self.bound_port = None


class OgnUdpToGlobalPosition:
    """OmniGraph node: UDP pose packets to global position and ENU orientation."""

    @staticmethod
    def internal_state() -> _InternalState:
        """Return persistent per-node state."""
        return _InternalState()

    @staticmethod
    def compute(db: OgnUdpToGlobalPositionDatabase) -> bool:
        """Read UDP packets, decode, convert NED to ENU, write outputs.

        Drains all queued datagrams and keeps only the latest. On any decode
        failure the HoldLastGoodDecoder preserves the previous pose rather than
        snapping to zero. The cumulative failure count is exposed on an output
        so monitoring can detect sustained corruption without per-tick log spam.
        """
        port = int(db.inputs.udp_port) if db.inputs.udp_port else DEFAULT_POSE_UDP_PORT
        state: _InternalState = db.per_instance_state

        sock = state.ensure_socket(port)

        # Drain all queued packets, keep only the latest.
        raw_data: bytes | None = None
        try:
            while True:
                raw_data, _ = sock.recvfrom(PACKET_SIZE * 2)
        except BlockingIOError:
            pass

        if raw_data is not None:
            pose = state.decoder.decode(raw_data)
        else:
            pose = state.decoder.last_good

        if pose is not None:
            lla = pose.position
            # Wire protocol is NED; Isaac Sim requires ENU. Conversion delegated
            # to the kernel, which validates the frame tag and raises on misuse.
            enu_attitude = ned_to_enu(pose.orientation)
            db.outputs.global_position = [lla.lat_deg, lla.lon_deg, lla.alt_m]
            db.outputs.global_orientation = [enu_attitude.roll_r, enu_attitude.pitch_r, enu_attitude.yaw_r]

        db.outputs.decode_failure_count = state.decoder.failure_count
        return True

    @staticmethod
    def release(node: object) -> None:
        """Release socket resources on node teardown."""
        carb.log_info(f"{_LOG_PREFIX} Node release")
        try:
            state = OgnUdpToGlobalPositionDatabase.per_instance_internal_state(node)
        except RuntimeError:
            return

        if state is not None:
            state.close()
