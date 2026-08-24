"""Tests for isaac_core.devkit.session: Sim facade and SimSession."""

from collections.abc import Generator

import pytest

from isaac_core.control.messages import Method
from isaac_core.control.server import ControlServer
from isaac_core.devkit.session import Sim

# -- fixtures --------------------------------------------------------------- #


@pytest.fixture()
def fake_server() -> Generator[ControlServer, None, None]:
    """Start a ControlServer on ephemeral port with fake handlers for all methods."""
    server = ControlServer(bind_port=0)

    # Register handlers for every method the session uses.
    server.register(Method.PING.value, lambda _p: "pong")
    server.register(Method.GET_STATE.value, lambda _p: {"vehicles": {"lead": {"pose_source": "udp"}}})
    server.register(Method.GET_CAPABILITIES.value, lambda _p: {"tilesets": True, "bboxes": False})
    server.register(Method.GET_CONFIG.value, lambda _p: {"sim": {"scene": "earth"}})
    server.register(Method.SET_CONFIG.value, lambda p: {"patched": p})
    server.register(
        Method.ENABLE_FEATURE.value, lambda p: {"enabled": p["feature_id"] if isinstance(p, dict) else None}
    )
    server.register(
        Method.DISABLE_FEATURE.value, lambda p: {"disabled": p["feature_id"] if isinstance(p, dict) else None}
    )
    server.register(Method.CAPTURE_FRAME.value, lambda p: {"path": p["path"] if isinstance(p, dict) else None})
    server.register(Method.PAUSE.value, lambda _p: "paused")
    server.register(Method.RESUME.value, lambda _p: "resumed")
    server.register(Method.STEP.value, lambda p: {"stepped": p["count"] if isinstance(p, dict) else 1})
    server.register(Method.RESET.value, lambda _p: "reset")

    server.start()
    yield server
    server.stop()


# -- Sim.attach ------------------------------------------------------------- #


def test_attach_connects_to_running_server(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None

    session = Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0)
    try:
        # Should be connected and functional.
        result = session.vehicles()
        assert "vehicles" in result
    finally:
        session.close()


def test_attach_requires_no_path(fake_server: ControlServer) -> None:
    # The key property fixing defect #7: no filesystem knowledge needed.
    port = fake_server.port
    assert port is not None

    # Only host and port -- no core_path, no repo path, no ISAACSIM_PATH.
    session = Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0)
    session.close()


def test_attach_timeout_when_no_server() -> None:
    # Use an ephemeral port that nothing is listening on.
    import socket  # noqa: PLC0415

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    # Port is now free but nothing is listening.

    with pytest.raises(TimeoutError):
        Sim.attach(host="127.0.0.1", port=port, timeout_s=0.3)


# -- Sim.launch ------------------------------------------------------------- #


def test_launch_raises_not_implemented() -> None:
    with pytest.raises(NotImplementedError, match="Isaac-side runtime"):
        Sim.launch()


# -- SimSession operations -------------------------------------------------- #


def test_session_pause(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        assert session.pause() == "paused"


def test_session_resume(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        assert session.resume() == "resumed"


def test_session_step(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        result = session.step(count=3)
        assert result == {"stepped": 3}


def test_session_reset(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        assert session.reset() == "reset"


def test_session_get_capabilities(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        caps = session.get_capabilities()
        assert caps["tilesets"] is True
        assert caps["bboxes"] is False


def test_session_config_get(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        cfg = session.config.get()
        assert cfg == {"sim": {"scene": "earth"}}


def test_session_config_patch(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        result = session.config.patch(headless=True)
        assert result == {"patched": {"headless": True}}


def test_session_features_enable(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        result = session.features.enable("thermal_cam")
        assert result == {"enabled": "thermal_cam"}


def test_session_features_disable(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        result = session.features.disable("thermal_cam")
        assert result == {"disabled": "thermal_cam"}


def test_session_capture_frame(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        result = session.capture_frame("output/frame_001.png")
        assert result == {"path": "output/frame_001.png"}


def test_session_vehicles(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        state = session.vehicles()
        assert state["vehicles"]["lead"]["pose_source"] == "udp"


# -- context manager -------------------------------------------------------- #


def test_session_context_manager(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        session.pause()
    # After exiting, client should be closed -- further calls should fail.
    with pytest.raises(ConnectionError):
        session.pause()


def test_session_works_without_context_manager(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    session = Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0)
    result = session.pause()
    assert result == "paused"
    session.close()


# -- error when server is gone ---------------------------------------------- #


def test_session_raises_when_server_gone() -> None:
    # Start a server, attach, then stop the server and try an operation.
    server = ControlServer(bind_port=0)
    server.register(Method.PING.value, lambda _p: "pong")
    server.register(Method.PAUSE.value, lambda _p: "paused")
    server.start()
    port = server.port
    assert port is not None

    session = Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0)
    # Verify it works first.
    assert session.pause() == "paused"

    # Kill the server.
    session.close()
    server.stop()

    # Re-attach should fail.
    with pytest.raises(TimeoutError):
        Sim.attach(host="127.0.0.1", port=port, timeout_s=0.3)
