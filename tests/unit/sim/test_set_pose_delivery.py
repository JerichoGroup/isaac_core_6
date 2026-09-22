"""`set_pose` must not lose its pose to a dropped datagram.

It sends through the same UDP path a real sender uses, which is right: writing the prim directly is
overwritten by the pose graph within a frame. But it sent the packet **once**, and a lone datagram is lost
if the receiver has not bound yet -- so the pose silently did not apply and `get_pose` read back zero.

Observed on a two-vehicle stage: `lead` held its commanded altitude while `wing`, given 560 m, read back
0.0. The same script had read wing back correctly on an earlier run, so it was intermittent. `PoseBot`
already streams its opening pose for exactly this reason, which is the precedent followed here.
"""

from __future__ import annotations

import socket
import threading
from typing import Any

import pytest

from isaac_core.config import load
from isaac_core.contracts.packet import PACKET_SIZE
from isaac_core.sim.planner import FeaturePlan
from isaac_core.sim.runtime import SET_POSE_DATAGRAMS, SET_POSE_GAP_S, SimulationRuntime


def _runtime(**overrides: Any) -> SimulationRuntime:
    """Return a runtime for the pure handler path, with no Isaac."""
    return SimulationRuntime(config=load(cli_overrides=dict(overrides) or None), plan=FeaturePlan())


def _listen(port: int, expected: int, timeout_s: float = 5.0) -> list[bytes]:
    """Collect datagrams on a port until the expected count arrives or time runs out.

    Args:
        port: Port to bind.
        expected: How many datagrams to wait for.
        timeout_s: Overall budget.

    Returns:
        The datagrams received.

    """
    received: list[bytes] = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(timeout_s)
    try:
        for _ in range(expected):
            try:
                received.append(sock.recv(4096))
            except TimeoutError:
                break
    finally:
        sock.close()
    return received


def test_more_than_one_datagram_is_sent() -> None:
    # The whole fix. One packet is enough only if nothing is dropped and the receiver is already bound.
    assert SET_POSE_DATAGRAMS > 1


def test_the_gap_is_short_enough_not_to_stall_a_caller() -> None:
    # `set_pose` is called in loops by scripts, so the repeat must not cost visible time.
    assert SET_POSE_GAP_S * SET_POSE_DATAGRAMS < 0.2


def test_the_pose_actually_arrives_on_the_vehicle_port() -> None:
    # Bind port 0 first to find a free one, then configure the vehicle onto it, so nothing is hardcoded.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    runtime = _runtime(**{"vehicles.drone_0.udp_port": str(port)})
    received: list[bytes] = []
    listener = threading.Thread(target=lambda: received.extend(_listen(port, SET_POSE_DATAGRAMS)), daemon=True)
    listener.start()
    # Let the listener bind before sending, which is the condition that used to be assumed.
    threading.Event().wait(0.3)

    result = runtime._handle_set_pose({"lat_deg": 32.22481, "lon_deg": 35.25621, "alt_m": 1000.0, "pitch_deg": -25.0})
    listener.join(timeout=5.0)

    assert result["udp_port"] == port
    assert len(received) == SET_POSE_DATAGRAMS, f"expected {SET_POSE_DATAGRAMS} datagrams, got {len(received)}"
    for packet in received:
        assert len(packet) == PACKET_SIZE, f"a datagram was not a {PACKET_SIZE}-byte pose packet"


def test_every_copy_is_the_same_packet() -> None:
    # Repeating must be idempotent: the last packet wins, so differing copies would make the final pose
    # depend on which ones survived.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    runtime = _runtime(**{"vehicles.drone_0.udp_port": str(port)})
    received: list[bytes] = []
    listener = threading.Thread(target=lambda: received.extend(_listen(port, SET_POSE_DATAGRAMS)), daemon=True)
    listener.start()
    threading.Event().wait(0.3)
    runtime._handle_set_pose({"lat_deg": 32.0, "lon_deg": 35.0, "alt_m": 900.0})
    listener.join(timeout=5.0)

    assert received, "nothing arrived"
    assert len(set(received)) == 1, "the copies differ, so which one survives changes the pose"


def test_a_missing_field_is_still_refused() -> None:
    from isaac_core.control.errors import InvalidParamsError

    with pytest.raises(InvalidParamsError, match="set_pose requires params.alt_m"):
        _runtime()._handle_set_pose({"lat_deg": 32.0, "lon_deg": 35.0})


def test_the_packet_decodes_to_what_was_asked_for() -> None:
    # End to end through the real encoder, so a change to either side shows up here.
    import math

    from isaac_core.protocol.pose_packet import decode

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    runtime = _runtime(**{"vehicles.drone_0.udp_port": str(port)})
    received: list[bytes] = []
    listener = threading.Thread(target=lambda: received.extend(_listen(port, 1)), daemon=True)
    listener.start()
    threading.Event().wait(0.3)
    runtime._handle_set_pose({"lat_deg": 32.22481, "lon_deg": 35.25621, "alt_m": 1234.5, "yaw_deg": 90.0})
    listener.join(timeout=5.0)

    assert received
    pose = decode(received[0])
    assert pose.position.lat_deg == pytest.approx(32.22481)
    assert pose.position.alt_m == pytest.approx(1234.5)
    assert pose.orientation.yaw_r == pytest.approx(math.radians(90.0))
