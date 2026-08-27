"""Tests for the terminal inspector, driven against a real control server."""

from __future__ import annotations

import socket
from typing import TYPE_CHECKING

import pytest

from isaac_core.control.server import ControlServer
from isaac_core.debug.inspector import main

if TYPE_CHECKING:
    from collections.abc import Iterator

# What the fake handlers report, so assertions can look for exact values.
_FAKE_TRANSLATE = [12.5, -34.25, 983.3]
_FAKE_ORIENT = [0.5, 0.5, -0.5, -0.5]


@pytest.fixture
def server() -> Iterator[ControlServer]:
    """Yield a started control server on an ephemeral port with fake handlers."""
    instance = ControlServer(bind_port=0)
    instance.register("ping", lambda _params: "pong")
    instance.register("get_state", lambda _params: {"playing": True, "frame": 42})
    instance.register("get_capabilities", lambda _params: {"features": ["camera_udp"]})
    instance.register("get_pose", lambda _params: {"translate": _FAKE_TRANSLATE, "orient": _FAKE_ORIENT})
    instance.register("get_config", lambda _params: {"sim": {"headless": True}})
    instance.start()
    yield instance
    instance.stop()


def _port_of(server: ControlServer) -> str:
    """Return the server's bound port as a string for argv."""
    assert server.port is not None
    return str(server.port)


def test_inspect_once_prints_state_capabilities_pose_and_config(
    server: ControlServer, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["--port", _port_of(server)]) == 0
    out = capsys.readouterr().out
    assert "42" in out, "state should be reported"
    assert "camera_udp" in out, "capabilities should be reported"
    assert "983.3" in out, "the live pose should be reported"
    assert "headless" in out, "config should be reported"


def test_polling_prints_one_row_per_sample(server: ControlServer, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--port", _port_of(server), "--poll", "0.01", "--count", "3"]) == 0
    out = capsys.readouterr().out
    # Three numbered rows, each carrying the translate value.
    assert out.count("983.300") == 3
    for sample in ("1", "2", "3"):
        assert f"\n{sample:>5}  " in out


def test_a_closed_port_reports_a_clear_actionable_error(capsys: pytest.CaptureFixture[str]) -> None:
    # Bind then close to obtain a port that is certainly free, rather than guessing one.
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    closed_port = probe.getsockname()[1]
    probe.close()

    assert main(["--port", str(closed_port)]) == 1
    err = capsys.readouterr().err
    assert str(closed_port) in err, "the error must name the port it tried"
    assert "127.0.0.1" in err, "the error must name the host it tried"
    assert "isaac-core run" in err, "the error should say how to start the simulator"


def test_a_failing_method_does_not_abort_the_whole_report(capsys: pytest.CaptureFixture[str]) -> None:
    # One broken handler must not hide the other sections: the inspector is a diagnostic
    # tool, so partial information is far more useful than a traceback.
    def _boom(_params: object) -> object:
        message = "no stage open"
        raise RuntimeError(message)

    instance = ControlServer(bind_port=0)
    instance.register("get_state", lambda _params: {"playing": False})
    instance.register("get_capabilities", lambda _params: {"features": []})
    instance.register("get_pose", _boom)
    instance.register("get_config", lambda _params: {"sim": {}})
    instance.start()
    try:
        assert instance.port is not None
        assert main(["--port", str(instance.port)]) == 0
    finally:
        instance.stop()

    out = capsys.readouterr().out
    assert "get_pose failed" in out
    assert "playing" in out, "the sections after the failure must still be printed"


def test_help_exits_cleanly(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as excinfo:
        main(["--help"])
    assert excinfo.value.code == 0
    assert "--poll" in capsys.readouterr().out


def test_inspect_prints_live_values_read_from_the_stage(capsys: pytest.CaptureFixture[str]) -> None:
    # Ofer's requirement: the inspector reports the REAL values read off the running stage
    # (via get_runtime_values), not values derived from config.
    instance = ControlServer(bind_port=0)
    instance.register("get_state", lambda _p: {"running": True})
    instance.register("get_capabilities", lambda _p: {"enabled": []})
    instance.register("get_pose", lambda _p: {"translate": [0, 0, 0], "orient": [1, 0, 0, 0]})
    instance.register("get_config", lambda _p: {"sim": {"scene": "earth"}})
    instance.register(
        "get_runtime_values",
        lambda _p: {
            "vehicle": "drone_0",
            "mount": "/World/Environment/drone_0",
            "udp_port": 33333,
            "rotation_frame": "world",
            "enu_reference": [32.22481, 35.25621, 516.7],
            "global_pose_topic": "/isaac_core/global_pose",
            "image_topic": "/isaac_core/image_rgb",
            "camera": {"focalLength": 22.7885, "horizontalAperture": 36.97, "verticalAperture": 20.8},
            "tilesets": {"/World/tilesets/Cesium_Tileset": "http://host/tileset.json"},
        },
    )
    instance.start()
    try:
        assert instance.port is not None
        assert main(["--port", str(instance.port)]) == 0
    finally:
        instance.stop()

    out = capsys.readouterr().out
    assert "Live values (read from the running stage)" in out
    assert "33333" in out, "the real udp_port read from the stage must be shown"
    assert "/World/Environment/drone_0" in out
    assert "22.7885" in out, "the real camera focalLength must be shown"
    assert "http://host/tileset.json" in out, "the real tileset URL must be shown"
    assert "no UDP/ROS pose received yet" in out, "idle pose should be explained"
