"""Tests for the pose sender controller, its CLI, and the packet reference table."""

from __future__ import annotations

import math
import socket
from typing import TYPE_CHECKING

import pytest

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.packet import PACKET_SIZE
from isaac_core.debug.pose_sender_gui import (
    PACKET_TABLE_LINES,
    PoseSenderController,
    build_parser,
    main,
)
from isaac_core.protocol.pose_packet import decode, encode

if TYPE_CHECKING:
    from collections.abc import Iterator


@pytest.fixture
def controller() -> Iterator[PoseSenderController]:
    """Yield a controller and release its socket afterwards."""
    instance = PoseSenderController()
    yield instance
    instance.close()


def test_defaults_sit_over_the_shipped_scene(controller: PoseSenderController) -> None:
    # These match the earth scene's Cesium georeference, so the camera starts over real
    # terrain instead of in empty space at (0, 0).
    assert controller.lat_deg == pytest.approx(32.22481)
    assert controller.lon_deg == pytest.approx(35.25621)


def test_degrees_are_converted_to_radians_exactly_once(controller: PoseSenderController) -> None:
    # The old GUI labelled these fields "degrees" while putting the same number on the
    # wire as radians. 90 is chosen because a skipped conversion (90.0) or a doubled one
    # (0.0274) is unmistakable next to the correct pi/2.
    controller.yaw_deg = 90.0
    pose = controller.build_pose()
    assert pose.orientation.yaw_r == pytest.approx(math.pi / 2)
    assert pose.orientation.frame is Frame.NED


def test_build_pose_round_trips_through_the_wire_codec(controller: PoseSenderController) -> None:
    controller.lat_deg = 31.7
    controller.lon_deg = 35.2
    controller.alt_m = 1234.5
    controller.roll_deg = 10.0
    controller.pitch_deg = -5.0
    controller.yaw_deg = 45.0

    raw = encode(controller.build_pose())
    assert len(raw) == PACKET_SIZE

    decoded = decode(raw)
    assert decoded.position.lat_deg == pytest.approx(31.7)
    assert decoded.position.alt_m == pytest.approx(1234.5)
    roll_deg, pitch_deg, yaw_deg = decoded.orientation.to_degrees()
    assert roll_deg == pytest.approx(10.0)
    assert pitch_deg == pytest.approx(-5.0)
    assert yaw_deg == pytest.approx(45.0)


@pytest.mark.parametrize(
    ("field", "step"),
    [("lat_deg", 0.001), ("lon_deg", 0.001), ("alt_m", 5.0), ("yaw_deg", 5.0)],
)
def test_nudge_moves_a_field_by_one_step(controller: PoseSenderController, field: str, step: float) -> None:
    before = getattr(controller, field)
    controller.nudge(field, 1)
    assert getattr(controller, field) == pytest.approx(before + step)
    controller.nudge(field, -1)
    assert getattr(controller, field) == pytest.approx(before)


def test_a_locked_field_does_not_move(controller: PoseSenderController) -> None:
    controller.locks["alt_m"] = True
    controller.nudge("alt_m", 1)
    assert controller.alt_m == pytest.approx(1000.0)


def test_nudging_an_unknown_field_is_ignored(controller: PoseSenderController) -> None:
    controller.nudge("not_a_field", 1)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("lat_deg", 120.0, 90.0),
        ("lat_deg", -120.0, -90.0),
        ("lon_deg", 200.0, 180.0),
        ("lon_deg", -200.0, -180.0),
    ],
)
def test_latitude_and_longitude_are_clamped(
    controller: PoseSenderController, field: str, value: float, expected: float
) -> None:
    # Nudging near a pole or the antimeridian must not produce a geodetically invalid
    # value that the encoder would then reject mid-flight.
    setattr(controller, field, value)
    pose = controller.build_pose()
    actual = pose.position.lat_deg if field == "lat_deg" else pose.position.lon_deg
    assert actual == pytest.approx(expected)


def test_rate_cannot_be_nudged_to_zero(controller: PoseSenderController) -> None:
    # A zero or negative rate would divide by zero in the send loop.
    controller.rate_hz = 0.5
    for _ in range(5):
        controller.nudge("rate_hz", -1)
    assert controller.rate_hz >= 0.1


def test_reset_restores_every_default(controller: PoseSenderController) -> None:
    controller.lat_deg = 1.0
    controller.alt_m = 9999.0
    controller.host = "10.0.0.1"
    controller.port = 40404
    controller.reset_to_defaults()
    assert controller.lat_deg == pytest.approx(32.22481)
    assert controller.alt_m == pytest.approx(1000.0)
    assert controller.host == "127.0.0.1"
    assert controller.port == 33333


def test_pause_suppresses_sending(controller: PoseSenderController) -> None:
    controller.paused = True
    assert controller.send_once() is False
    assert controller.log_entries == []


def test_send_once_delivers_a_decodable_packet(controller: PoseSenderController) -> None:
    # Bind port 0 and read back the assigned port: never hardcode one in a test.
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(2.0)
    try:
        controller.host, controller.port = receiver.getsockname()
        controller.alt_m = 750.0
        assert controller.send_once() is True
        raw, _ = receiver.recvfrom(PACKET_SIZE * 2)
    finally:
        receiver.close()

    assert len(raw) == PACKET_SIZE
    assert decode(raw).position.alt_m == pytest.approx(750.0)


def test_the_log_records_a_successful_send(controller: PoseSenderController) -> None:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    try:
        controller.host, controller.port = receiver.getsockname()
        controller.send_once()
    finally:
        receiver.close()
    assert len(controller.log_entries) == 1
    assert "alt=1000.00" in controller.log_entries[0]


def test_the_log_is_bounded_and_ends_with_the_newest_entry(controller: PoseSenderController) -> None:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    try:
        controller.host, controller.port = receiver.getsockname()
        for index in range(60):
            controller.alt_m = float(index)
            controller.send_once()
    finally:
        receiver.close()

    entries = controller.log_entries
    assert len(entries) == 50, "the ring buffer must not grow without bound"
    assert "alt=59.00" in entries[-1], "newest entry must be last"


def test_changing_the_target_rebinds_the_transport(controller: PoseSenderController) -> None:
    # The transport caches a socket; editing host or port in the GUI must take effect
    # rather than silently keeping sending to the old destination.
    first = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    first.bind(("127.0.0.1", 0))
    second.bind(("127.0.0.1", 0))
    second.settimeout(2.0)
    try:
        controller.host, controller.port = first.getsockname()
        controller.send_once()
        controller.host, controller.port = second.getsockname()
        controller.alt_m = 321.0
        controller.send_once()
        raw, _ = second.recvfrom(PACKET_SIZE * 2)
    finally:
        first.close()
        second.close()
    assert decode(raw).position.alt_m == pytest.approx(321.0)


def test_close_is_safe_to_call_twice(controller: PoseSenderController) -> None:
    controller.close()
    controller.close()


def test_the_packet_table_is_derived_from_the_contract() -> None:
    # The old GUI hardcoded a copy of this table, which then drifted from the real
    # format. Deriving it means a contract change shows up in the window.
    joined = "\n".join(PACKET_TABLE_LINES)
    assert f"Total: {PACKET_SIZE} bytes" in joined
    assert "RADIANS" in joined, "the table must say radians, matching the wire"
    assert "checksum" in joined


def test_help_exits_without_opening_a_window(capsys: pytest.CaptureFixture[str]) -> None:
    # main() used to ignore argv and call launch_gui() unconditionally, so --help opened
    # a tkinter window and blocked forever instead of printing usage.
    with pytest.raises(SystemExit) as excinfo:
        main(["--help"])
    assert excinfo.value.code == 0
    assert "usage:" in capsys.readouterr().out


def test_check_mode_validates_without_opening_a_window(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    launched: list[object] = []
    monkeypatch.setattr(
        "isaac_core.debug.pose_sender_gui.launch_gui",
        lambda controller=None: launched.append(controller),
    )
    assert main(["--check", "--alt-m", "1200"]) == 0
    assert not launched, "--check must not launch the GUI"
    out = capsys.readouterr().out
    assert "window not opened" in out
    assert "1200.0" in out


def test_cli_options_reach_the_controller(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: list[PoseSenderController | None] = []
    monkeypatch.setattr(
        "isaac_core.debug.pose_sender_gui.launch_gui",
        lambda controller=None: captured.append(controller),
    )
    main(["--host", "10.0.0.5", "--port", "40000", "--rate-hz", "50", "--alt-m", "777"])
    assert captured
    passed = captured[0]
    assert passed is not None
    assert passed.host == "10.0.0.5"
    assert passed.port == 40000
    assert passed.rate_hz == pytest.approx(50.0)
    assert passed.alt_m == pytest.approx(777.0)


def test_parser_defaults_match_the_controller_defaults() -> None:
    args = build_parser().parse_args([])
    fresh = PoseSenderController()
    try:
        assert args.host == fresh.host
        assert args.port == fresh.port
        assert args.lat_deg == pytest.approx(fresh.lat_deg)
    finally:
        fresh.close()
