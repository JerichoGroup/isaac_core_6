"""Unit tests for the host-side MAVLink pose bridge.

These run with no network and no pymavlink installed: the MAVLink connection is a
duck-typed fake exposing ``recv_match`` and the transport is the repo's
:class:`~isaac_core.devkit.transport.FakePoseTransport`. The most important case is
the all-zero-pose guard -- nothing may be sent until both a position and an attitude
have arrived.
"""

from __future__ import annotations

import builtins
import math
from types import ModuleType

import pytest

from isaac_core.contracts.frames import Frame
from isaac_core.devkit.mavlink import (
    DEG_E7_PER_DEG,
    MM_PER_M,
    MavlinkPoseBridge,
    _require_pymavlink,
)
from isaac_core.devkit.transport import FakePoseTransport


class FakeMessage:
    """A duck-typed MAVLink message carrying a type tag and arbitrary fields."""

    def __init__(self, msg_type: str, **fields: float) -> None:
        """Store the message type and its named numeric fields."""
        self._msg_type = msg_type
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        """Return the MAVLink message type name."""
        return self._msg_type


class FakeConnection:
    """A duck-typed MAVLink connection yielding queued messages from ``recv_match``."""

    def __init__(self, messages: list[FakeMessage]) -> None:
        """Queue the messages to be returned, oldest first."""
        self._messages = list(messages)
        self.closed = False

    def recv_match(self, *, blocking: bool = False) -> FakeMessage | None:
        """Return the next queued message, or ``None`` when the queue is empty."""
        if self._messages:
            return self._messages.pop(0)
        return None

    def close(self) -> None:
        """Mark the connection closed."""
        self.closed = True


def _position(lat_e7: int, lon_e7: int, alt_mm: int) -> FakeMessage:
    """Build a fake GLOBAL_POSITION_INT message."""
    return FakeMessage("GLOBAL_POSITION_INT", lat=lat_e7, lon=lon_e7, alt=alt_mm)


def _attitude(roll_r: float, pitch_r: float, yaw_r: float) -> FakeMessage:
    """Build a fake ATTITUDE message."""
    return FakeMessage("ATTITUDE", roll=roll_r, pitch=pitch_r, yaw=yaw_r)


def test_unit_conversions_produce_correct_pose() -> None:
    """degE7 and millimetre fields convert to decimal degrees and metres."""
    conn = FakeConnection(
        [
            _position(lat_e7=322_248_100, lon_e7=352_562_100, alt_mm=1_500_000),
            _attitude(roll_r=0.1, pitch_r=-0.2, yaw_r=0.3),
        ]
    )
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    pose = bridge.poll_once()

    assert pose is not None
    assert pose.position.lat_deg == pytest.approx(32.22481)
    assert pose.position.lon_deg == pytest.approx(35.25621)
    assert pose.position.alt_m == pytest.approx(1500.0)
    assert pose.orientation.roll_r == pytest.approx(0.1)
    assert pose.orientation.pitch_r == pytest.approx(-0.2)
    assert pose.orientation.yaw_r == pytest.approx(0.3)
    assert pose.orientation.frame is Frame.NED


def test_conversion_factors_are_the_documented_values() -> None:
    """The named unit factors match the MAVLink field scalings."""
    assert DEG_E7_PER_DEG == 1e7
    assert MM_PER_M == 1e3


def test_no_pose_before_both_message_types_arrive() -> None:
    """Nothing is emitted with only a position, guarding the all-zero Atlantic pose."""
    conn = FakeConnection([_position(lat_e7=322_248_100, lon_e7=352_562_100, alt_mm=1_500_000)])
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    assert bridge.poll_once() is None
    assert bridge.has_pose is False


def test_no_pose_with_only_attitude() -> None:
    """Nothing is emitted with only an attitude either."""
    conn = FakeConnection([_attitude(roll_r=0.0, pitch_r=0.0, yaw_r=0.0)])
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    assert bridge.poll_once() is None
    assert bridge.has_pose is False


def test_run_sends_nothing_until_pose_complete() -> None:
    """The run loop emits no packet while the pose is incomplete."""
    conn = FakeConnection([_attitude(roll_r=0.0, pitch_r=0.0, yaw_r=0.0)])
    transport = FakePoseTransport()
    bridge = MavlinkPoseBridge(conn, transport, rate_hz=1000.0)

    def stop_after_first(_seconds: float) -> None:
        bridge.stop()

    sent = bridge.run(sleep=stop_after_first)

    assert sent == 0
    assert transport.history == []


def test_latest_of_each_message_is_used() -> None:
    """When several messages of a type arrive, the most recent one wins."""
    conn = FakeConnection(
        [
            _position(lat_e7=100_000_000, lon_e7=100_000_000, alt_mm=1_000),
            _attitude(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
            _position(lat_e7=322_248_100, lon_e7=352_562_100, alt_mm=2_000),
            _attitude(roll_r=0.5, pitch_r=0.4, yaw_r=0.3),
        ]
    )
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    pose = bridge.poll_once()

    assert pose is not None
    assert pose.position.lat_deg == pytest.approx(32.22481)
    assert pose.position.alt_m == pytest.approx(2.0)
    assert pose.orientation.roll_r == pytest.approx(0.5)


def test_different_rates_hold_last_position_across_attitude_updates() -> None:
    """A slow position stream is held while faster attitudes update the pose."""
    conn = FakeConnection([_position(lat_e7=322_248_100, lon_e7=352_562_100, alt_mm=1_000)])
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    # First attitude completes the pose.
    conn._messages.append(_attitude(roll_r=0.1, pitch_r=0.0, yaw_r=0.0))
    first = bridge.poll_once()
    # A later attitude arrives with no new position.
    conn._messages.append(_attitude(roll_r=0.9, pitch_r=0.0, yaw_r=0.0))
    second = bridge.poll_once()

    assert first is not None
    assert second is not None
    assert second.position.lat_deg == pytest.approx(32.22481)
    assert second.orientation.roll_r == pytest.approx(0.9)


def test_unknown_message_types_are_ignored() -> None:
    """An unrecognised message type does not crash the loop or corrupt state."""
    conn = FakeConnection(
        [
            FakeMessage("HEARTBEAT", custom_mode=0),
            _position(lat_e7=322_248_100, lon_e7=352_562_100, alt_mm=1_000),
            FakeMessage("SYS_STATUS", load=500),
            _attitude(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
        ]
    )
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    pose = bridge.poll_once()

    assert pose is not None
    assert pose.position.lat_deg == pytest.approx(32.22481)


def test_run_sends_paced_poses_and_stops() -> None:
    """The run loop emits a pose per iteration once complete, until stopped."""
    conn = FakeConnection(
        [
            _position(lat_e7=322_248_100, lon_e7=352_562_100, alt_mm=1_000),
            _attitude(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
        ]
    )
    transport = FakePoseTransport()
    bridge = MavlinkPoseBridge(conn, transport, rate_hz=1000.0)

    calls = {"n": 0}

    def stop_after_three(_seconds: float) -> None:
        calls["n"] += 1
        if calls["n"] >= 3:
            bridge.stop()

    sent = bridge.run(sleep=stop_after_three)

    assert sent == 3
    assert len(transport.history) == 3
    assert transport.history[0].position.lat_deg == pytest.approx(32.22481)


def test_negative_rate_is_rejected() -> None:
    """A non-positive rate is rejected at construction."""
    with pytest.raises(ValueError, match="rate_hz must be positive"):
        MavlinkPoseBridge(FakeConnection([]), FakePoseTransport(), rate_hz=0.0)


def test_close_closes_transport_and_connection() -> None:
    """close() releases both the transport and the connection."""
    conn = FakeConnection([])
    transport = FakePoseTransport()
    bridge = MavlinkPoseBridge(conn, transport)

    bridge.close()

    assert transport.closed is True
    assert conn.closed is True


def test_missing_pymavlink_raises_actionable_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """A missing pymavlink raises an ImportError naming the pip install command."""
    real_import = builtins.__import__

    def fake_import(name: str, *args: object, **kwargs: object) -> ModuleType:
        if name == "pymavlink" or name.startswith("pymavlink."):
            raise ImportError("no pymavlink")
        return real_import(name, *args, **kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(builtins, "__import__", fake_import)

    with pytest.raises(ImportError, match="pip install --user pymavlink"):
        _require_pymavlink()


def test_yaw_wraps_are_passed_through_unchanged() -> None:
    """ATTITUDE angles are already NED radians and pass through without conversion."""
    conn = FakeConnection(
        [
            _position(lat_e7=0, lon_e7=0, alt_mm=0),
            _attitude(roll_r=math.pi, pitch_r=-math.pi / 2, yaw_r=math.pi / 4),
        ]
    )
    bridge = MavlinkPoseBridge(conn, FakePoseTransport())

    pose = bridge.poll_once()

    assert pose is not None
    assert pose.orientation.yaw_r == pytest.approx(math.pi / 4)
    assert pose.orientation.frame is Frame.NED
