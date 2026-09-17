"""Tests for the pose sender controller, its CLI, and the packet reference table."""

from __future__ import annotations

import math
from pathlib import Path
import socket
import subprocess
import tempfile
from typing import TYPE_CHECKING

import pytest

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.packet import PACKET_SIZE
from isaac_core.debug.pose_sender_gui import (
    PACKET_TABLE_LINES,
    PoseSenderController,
    SenderTabs,
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


# -- per-tab pose source ------------------------------------------------------ #


def test_a_tab_defaults_to_the_udp_wire() -> None:
    controller = PoseSenderController()
    assert controller.pose_source == "udp"
    assert controller.target_label == "UDP 127.0.0.1:33333"


def test_switching_to_ros_changes_the_target_without_touching_the_pose() -> None:
    # The source is the only thing that differs between tabs, which is what lets one window drive a
    # UDP vehicle and a ROS one side by side.
    controller = PoseSenderController()
    before = controller.build_pose()
    controller.set_pose_source("ros")
    assert controller.pose_source == "ros"
    assert controller.target_label == "ROS 2 /mavros"
    assert controller.build_pose() == before


def test_an_unsupported_pose_source_is_refused() -> None:
    # There are exactly two wires the simulator listens to; mavlink and replay are adapters onto UDP.
    controller = PoseSenderController()
    for bad in ("mavlink", "replay", "script", ""):
        with pytest.raises(ValueError, match="pose_source must be"):
            controller.set_pose_source(bad)


def test_switching_source_is_logged_so_the_window_shows_it() -> None:
    controller = PoseSenderController()
    controller.set_pose_source("ros")
    assert any("ROS 2" in entry for entry in controller.log_entries)


def test_switching_to_the_same_source_is_a_no_op() -> None:
    controller = PoseSenderController()
    controller.set_pose_source("udp")
    assert controller.log_entries == []


def test_the_target_key_changes_with_every_addressable_part() -> None:
    # The key is what triggers rebuilding the transport, so it has to move when the target does.
    controller = PoseSenderController()
    keys = {controller.target_key}
    controller.port = 33334
    keys.add(controller.target_key)
    controller.host = "192.0.2.9"
    keys.add(controller.target_key)
    controller.set_pose_source("ros")
    keys.add(controller.target_key)
    controller.ros_namespace = "/other"
    keys.add(controller.target_key)
    assert len(keys) == 5, f"a target change did not move the key: {keys}"


# -- copy as snippet ---------------------------------------------------------- #


def test_as_python_round_trips_through_the_devkit_signature() -> None:
    controller = PoseSenderController()
    controller.lat_deg = 32.5
    controller.alt_m = 1234.5
    snippet = controller.as_python()
    assert snippet.startswith("session.set_pose(")
    assert "lat_deg=32.500000" in snippet
    assert "alt_m=1234.50" in snippet


def test_as_toml_is_a_loadable_config_fragment() -> None:
    # Pasting it into a config must work, so it has to parse and validate.
    from isaac_core.config import load

    controller = PoseSenderController()
    controller.lat_deg = 31.5
    controller.lon_deg = 34.5
    controller.alt_m = 700.0
    with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as handle:
        handle.write(controller.as_toml())
        path = Path(handle.name)
    try:
        config = load(path=path)
        assert config.geo.enu_reference.lat_deg == pytest.approx(31.5)
        assert config.geo.enu_reference.alt_m == pytest.approx(700.0)
    finally:
        path.unlink(missing_ok=True)


# -- tabs --------------------------------------------------------------------- #


def test_a_window_starts_with_one_tab() -> None:
    tabs = SenderTabs()
    assert len(tabs) == 1
    assert tabs[0].name == "tab_1"


def test_each_new_tab_lands_on_the_port_the_simulator_will_listen_on() -> None:
    # The simulator allocates a vehicle's UDP port as base + vehicle index, so a two-vehicle swarm
    # needs no arithmetic from the user: tab 2 is already on 33334.
    tabs = SenderTabs()
    tabs.add()
    tabs.add()
    assert [controller.port for controller in tabs] == [33333, 33334, 33335]


def test_tabs_are_named_so_they_can_be_told_apart() -> None:
    tabs = SenderTabs()
    tabs.add()
    tabs.add()
    # Positional labels, not vehicle names: mixing the two produced a strip reading
    # "drone_0, vehicle_2, vehicle_3" where only the first came from the simulator.
    assert [controller.name for controller in tabs] == ["tab_1", "tab_2", "tab_3"]


def test_a_tab_can_be_created_on_the_ros_wire() -> None:
    tabs = SenderTabs()
    controller = tabs.add(pose_source="ros")
    assert controller.pose_source == "ros"
    assert controller.target_label.startswith("ROS 2")


def test_a_second_ros_tab_gets_its_own_namespace() -> None:
    # Two ROS tabs publishing into one namespace would give both vehicles both streams, and neither
    # operator could tell which was which.
    tabs = SenderTabs()
    first = tabs.add(pose_source="ros")
    second = tabs.add(pose_source="ros")
    assert first.ros_namespace != second.ros_namespace


def test_explicit_name_and_port_win_over_the_defaults() -> None:
    tabs = SenderTabs()
    controller = tabs.add(name="wing", port=40000)
    assert controller.name == "wing"
    assert controller.port == 40000


def test_removing_a_tab_drops_it() -> None:
    tabs = SenderTabs()
    tabs.add()
    tabs.remove(0)
    assert len(tabs) == 1


def test_the_last_tab_cannot_be_closed() -> None:
    # A window with no tabs has nothing to send and no way to get a tab back.
    tabs = SenderTabs()
    with pytest.raises(ValueError, match="last tab"):
        tabs.remove(0)


def test_closing_the_set_closes_every_transport() -> None:
    tabs = SenderTabs()
    tabs.add()
    tabs.close()
    # Closing twice must stay harmless, since the window closes on both the button and Ctrl-C.
    tabs.close()


# -- readback, counters and the stream URL ------------------------------------- #


def test_the_packet_counter_moves_with_every_send() -> None:
    # A tab that shows nothing is indistinguishable from a tab that is not sending.
    controller = PoseSenderController()
    assert controller.sent == 0
    controller.send_once()
    controller.send_once()
    assert controller.sent == 2


def test_a_paused_tab_sends_nothing() -> None:
    controller = PoseSenderController()
    controller.paused = True
    assert controller.send_once() is False
    assert controller.sent == 0


def test_the_log_names_the_real_target_not_just_a_socket() -> None:
    # It said "127.0.0.1:33333" even on a ROS tab, which is the wrong wire entirely.
    controller = PoseSenderController()
    controller.send_once()
    assert "UDP 127.0.0.1:33333" in controller.log_entries[-1]


def test_readback_reports_no_simulator_rather_than_raising() -> None:
    # It runs on a UI tick, so an unreachable simulator is normal and must never propagate.
    controller = PoseSenderController()
    controller.control_port = 1
    message = controller.readback()
    assert "no simulator" in message


def test_readback_vehicle_names_is_empty_without_a_simulator() -> None:
    controller = PoseSenderController()
    controller.control_port = 1
    assert controller.readback_vehicle_names() == ()


@pytest.mark.parametrize(
    ("port", "name", "namespaced", "expected"),
    [
        # One vehicle: the mount is bare, whatever the tab is called.
        (33333, "vehicle", False, "rtsp://127.0.0.1:8554/stream"),
        (33333, "lead", False, "rtsp://127.0.0.1:8554/stream"),
        # A swarm: every mount carries its vehicle name, including the first one. Guessing the bare
        # form for vehicle zero sent the button at /stream while the simulator served /lead/stream.
        (33333, "lead", True, "rtsp://127.0.0.1:8554/lead/stream"),
        (33334, "wing", True, "rtsp://127.0.0.1:8555/wing/stream"),
        (33335, "third", True, "rtsp://127.0.0.1:8556/third/stream"),
    ],
)
def test_the_stream_url_mirrors_how_the_simulator_allocates_streams(
    port: int, name: str, namespaced: bool, expected: str
) -> None:
    controller = PoseSenderController()
    controller.port = port
    controller.name = name
    controller.stream_namespaced = namespaced
    assert controller.rtsp_url == expected


# -- ramping ------------------------------------------------------------------- #


def test_ramping_walks_the_fields_to_the_target() -> None:
    # "Fly there" must move through the intermediate values, because a jump is not a flight and looks
    # wrong in a recording.
    from isaac_core.debug.pose_sender_gui import _ramp_controller

    controller = PoseSenderController()
    controller.alt_m = 1000.0
    _ramp_controller(controller, {"alt_m": 1200.0}, duration_s=0.05)
    assert controller.alt_m == pytest.approx(1200.0)


def test_ramping_ends_exactly_on_the_target() -> None:
    from isaac_core.debug.pose_sender_gui import _ramp_controller

    controller = PoseSenderController()
    _ramp_controller(controller, {"lat_deg": 33.0, "lon_deg": 36.0}, duration_s=0.05)
    assert controller.lat_deg == pytest.approx(33.0)
    assert controller.lon_deg == pytest.approx(36.0)


# -- the CLI's controller becomes tab one -------------------------------------- #


def test_a_supplied_controller_becomes_the_first_tab() -> None:
    # The CLI configures a controller from its flags before any window exists, so it must not end up
    # beside a default tab nobody asked for.
    tabs = SenderTabs()
    configured = PoseSenderController()
    configured.port = 45000
    configured.name = "from_cli"
    tabs.replace_first(configured)
    assert len(tabs) == 1
    assert tabs[0] is configured
    assert tabs[0].port == 45000


def test_a_supplied_controller_without_a_name_still_gets_one() -> None:
    tabs = SenderTabs()
    configured = PoseSenderController()
    configured.name = ""
    tabs.replace_first(configured)
    assert tabs[0].name == "tab_1"


# -- the stream URL is editable and the player is owned ------------------------ #


def test_the_stream_url_defaults_to_the_derived_guess() -> None:
    controller = PoseSenderController()
    assert controller.rtsp_url == controller.derived_rtsp_url


def test_a_typed_stream_url_wins() -> None:
    # Guessing cannot cover a second vehicle on a remote host or a hand-authored mount path, so there
    # has to be a way in.
    controller = PoseSenderController()
    controller.rtsp_url_override = "rtsp://10.0.0.5:8600/thermal"
    assert controller.rtsp_url == "rtsp://10.0.0.5:8600/thermal"


def test_clearing_the_override_returns_to_the_guess() -> None:
    controller = PoseSenderController()
    controller.rtsp_url_override = "rtsp://elsewhere/x"
    controller.rtsp_url_override = ""
    assert controller.rtsp_url == controller.derived_rtsp_url


def test_closing_a_tab_stops_the_players_it_opened() -> None:
    # A player started in its own session outlived the window, so closing the sender left an orphaned
    # stream on screen with nothing left to stop it.
    controller = PoseSenderController()
    player = subprocess.Popen(["sleep", "30"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        controller.track_player(player)
        assert player.poll() is None
        controller.close()
        assert player.poll() is not None, "the player outlived the tab"
    finally:
        if player.poll() is None:
            player.kill()
            player.wait(timeout=5)


def test_stopping_players_twice_is_harmless() -> None:
    controller = PoseSenderController()
    controller.stop_players()
    controller.stop_players()


# -- copy TOML keeps the angles ------------------------------------------------ #


def test_copy_toml_keeps_the_angles_as_a_comment() -> None:
    # There is no config key for a starting attitude, so dropping the angles silently loses half of
    # what the user was looking at when they pressed the button.
    controller = PoseSenderController()
    controller.roll_deg = 5.0
    controller.pitch_deg = -30.0
    controller.yaw_deg = 45.0
    text = controller.as_toml()
    assert "roll = 5.0" in text
    assert "pitch = -30.0" in text
    assert "yaw = 45.0" in text


def test_copy_toml_is_still_loadable_with_the_angles_present() -> None:
    from isaac_core.config import load

    controller = PoseSenderController()
    controller.pitch_deg = -30.0
    with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as handle:
        handle.write(controller.as_toml())
        path = Path(handle.name)
    try:
        config = load(path=path)
        assert config.geo.enu_reference.lat_deg == pytest.approx(controller.lat_deg)
    finally:
        path.unlink(missing_ok=True)


# -- adopting a vehicle does not rename the tab -------------------------------- #


def test_adopting_a_vehicle_sets_the_readback_target_not_the_label() -> None:
    # The tab strip used to read "drone_0, tab_2, tab_3", mixing a simulator name with placeholders.
    from isaac_core.debug.pose_sender_gui import _adopt_simulator_vehicle

    controller = PoseSenderController()
    controller.name = "tab_1"
    original = controller.name
    controller.readback_vehicle_names = lambda: ("lead", "wing")  # type: ignore[method-assign]
    _adopt_simulator_vehicle(controller, set())
    assert controller.vehicle == "lead"
    assert controller.name == original, "the tab label must stay positional"
