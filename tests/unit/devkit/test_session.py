"""Tests for isaac_core.devkit.session: Sim facade and SimSession."""

from collections.abc import Generator
from pathlib import Path

import pytest

from isaac_core.control.messages import Method
from isaac_core.control.server import ControlServer
from isaac_core.devkit.session import Sim, SimSession

# -- fixtures --------------------------------------------------------------- #


@pytest.fixture()
def fake_server() -> Generator[ControlServer, None, None]:
    """Start a ControlServer on ephemeral port with fake handlers for all methods."""
    server = ControlServer(bind_port=0)

    # Register handlers for every method the session uses.
    server.register(Method.PING.value, lambda _p: "pong")
    server.register(Method.GET_STATE.value, lambda _p: {"vehicles": {"lead": {"pose_source": "udp"}}})
    server.register(Method.GET_POSE.value, lambda _p: {"vehicle": "drone_0", "translate": [0.0, 0.0, 983.3]})
    server.register(Method.GET_CAPABILITIES.value, lambda _p: {"tilesets": True, "bboxes": False})
    server.register(Method.GET_CONFIG.value, lambda _p: {"sim": {"scene": "earth"}})
    server.register(Method.SET_CONFIG.value, lambda p: {"patched": p})
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
        result = session.state()
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
    import socket

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    # Port is now free but nothing is listening.

    with pytest.raises(TimeoutError):
        Sim.attach(host="127.0.0.1", port=port, timeout_s=0.3)


# -- Sim.launch ------------------------------------------------------------- #


class _FakeProcess:
    """Stands in for a spawned simulator so no Isaac Sim is started."""

    def __init__(self) -> None:
        """Start alive, with nothing terminated or killed."""
        self.terminated = False
        self.killed = False
        self.waited = False
        self._alive = True

    def poll(self) -> int | None:
        """Return ``None`` while alive, mimicking ``subprocess.Popen``."""
        return None if self._alive else 0

    def terminate(self) -> None:
        """Record a polite shutdown request."""
        self.terminated = True
        self._alive = False

    def kill(self) -> None:
        """Record a forced shutdown."""
        self.killed = True
        self._alive = False

    def wait(self, timeout: float | None = None) -> int:
        """Record that the caller waited."""
        self.waited = True
        return 0


def test_launch_connects_and_returns_a_session(fake_server: ControlServer) -> None:
    # The launcher is injected so this exercises the real wiring -- config resolution,
    # command construction, readiness wait -- without starting Isaac Sim.
    port = fake_server.port
    assert port is not None
    captured: list[list[str]] = []
    process = _FakeProcess()

    def fake_launcher(command: list[str]) -> _FakeProcess:
        captured.append(command)
        return process

    session = Sim.launch(port=port, headless=True, timeout_s=5.0, launcher=fake_launcher)
    try:
        assert session.pause() == "paused", "the session must be usable"
    finally:
        session.close()

    assert captured, "the launcher was never called"
    command = captured[0]
    # Must invoke the module in Isaac's interpreter, never a path into this repo (D7).
    assert "-m" in command
    assert "isaac_core.sim" in command
    assert command[command.index("-m") + 1] == "isaac_core.sim"
    assert "--config" in command


def test_a_launched_session_stops_the_simulator_on_close(fake_server: ControlServer) -> None:
    # Otherwise a script that raises inside `with Sim.launch(...)` leaves Isaac running.
    port = fake_server.port
    assert port is not None
    process = _FakeProcess()
    session = Sim.launch(port=port, headless=True, timeout_s=5.0, launcher=lambda _cmd: process)
    session.close()
    assert process.terminated, "closing a launched session must stop the simulator"


def test_an_attached_session_leaves_the_simulator_running(fake_server: ControlServer) -> None:
    # attach() connects to somebody else's simulator and must not kill it.
    port = fake_server.port
    assert port is not None
    session = Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0)
    session.close()
    assert session._process is None


def test_launch_writes_a_resolved_config_the_simulator_can_read(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    captured: list[list[str]] = []

    def fake_launcher(command: list[str]) -> _FakeProcess:
        captured.append(command)
        return _FakeProcess()

    session = Sim.launch(port=port, headless=True, timeout_s=5.0, launcher=fake_launcher)
    session.close()

    config_path = Path(captured[0][captured[0].index("--config") + 1])
    assert config_path.is_file(), "the resolved config must exist on disk"
    text = config_path.read_text(encoding="utf-8")
    # The requested port has to reach the simulator, or it would open a different one.
    assert str(port) in text


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
        result = session.config.patch("gimbal.max_rate_deg_s", 10.0)
        # The wire form is params.key/params.value, which is why patch takes them positionally:
        # a **kwargs signature invited patch(gimbal_max_rate_deg_s=...), which the server rejects.
        assert result == {"patched": {"key": "gimbal.max_rate_deg_s", "value": 10.0}}


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
        state = session.state()
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


def test_session_get_pose(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        pose = session.get_pose()
    assert pose == {"vehicle": "drone_0", "translate": [0.0, 0.0, 983.3]}


def test_session_state(fake_server: ControlServer) -> None:
    port = fake_server.port
    assert port is not None
    with Sim.attach(host="127.0.0.1", port=port, timeout_s=5.0) as session:
        state = session.state()
    assert state == {"vehicles": {"lead": {"pose_source": "udp"}}}


def test_the_readme_agrees_with_which_members_are_properties() -> None:
    # The README listed `state` as a property while it was a method, so the documented
    # `session.state.get("ready")` raised AttributeError. Anything that crosses the wire is a method;
    # only the local objects are attributes.
    from pathlib import Path as _Path
    import re

    readme = (_Path(__file__).resolve().parents[3] / "README.md").read_text(encoding="utf-8")
    property_table = readme.split("| Property | What it is |")[1].split("\n\n")[0]
    documented = set(re.findall(r"\| `([a-z_]+)` \|", property_table))
    actual = {name for name in dir(SimSession) if isinstance(getattr(SimSession, name, None), property)}
    assert documented == actual, f"README property table {documented} does not match the class {actual}"
