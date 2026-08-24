"""Tests for the ControlClient: round-trip, error propagation, wait_until_ready."""

from collections.abc import Generator
import socket
import threading
import time

import pytest

from isaac_core.config.schema import ControlPlaneConfig
from isaac_core.control.client import ControlClient
from isaac_core.control.errors import AuthError, InternalError, MethodNotFoundError
from isaac_core.control.server import ControlServer

# -- fixtures --------------------------------------------------------------- #


@pytest.fixture()
def server_and_port() -> Generator[tuple[ControlServer, int], None, None]:
    """Start a server on loopback with an ephemeral port."""
    server = ControlServer(bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.register("echo", lambda params: params)

    def _add(params: dict[str, int] | list[int] | None) -> int:
        if isinstance(params, dict):
            return int(params["a"]) + int(params["b"])
        return 0

    server.register("add", _add)
    server.start()
    assert server.port is not None
    yield server, server.port
    server.stop()


# -- basic call round-trip -------------------------------------------------- #


def test_call_ping(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    client = ControlClient("127.0.0.1", port)
    client.connect()
    try:
        result = client.call("ping")
        assert result == "pong"
    finally:
        client.close()


def test_call_with_params(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    client = ControlClient("127.0.0.1", port)
    client.connect()
    try:
        result = client.call("add", {"a": 3, "b": 7})
        assert result == 10
    finally:
        client.close()


def test_call_echo(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    client = ControlClient("127.0.0.1", port)
    client.connect()
    try:
        result = client.call("echo", {"key": "value"})
        assert result == {"key": "value"}
    finally:
        client.close()


# -- error propagation ------------------------------------------------------ #


def test_method_not_found_raises(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    client = ControlClient("127.0.0.1", port)
    client.connect()
    try:
        with pytest.raises(MethodNotFoundError):
            client.call("nonexistent")
    finally:
        client.close()


def test_handler_exception_raises_internal_error(server_and_port: tuple[ControlServer, int]) -> None:
    server, port = server_and_port

    def _boom(_params: object) -> None:
        msg = "oops"
        raise RuntimeError(msg)

    server.register("boom", _boom)
    client = ControlClient("127.0.0.1", port)
    client.connect()
    try:
        with pytest.raises(InternalError, match="oops"):
            client.call("boom")
    finally:
        client.close()


# -- context manager -------------------------------------------------------- #


def test_context_manager(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    with ControlClient("127.0.0.1", port) as client:
        assert client.ping() == "pong"


# -- multiple calls on one connection --------------------------------------- #


def test_multiple_calls_same_connection(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    with ControlClient("127.0.0.1", port) as client:
        for i in range(10):
            result = client.call("add", {"a": i, "b": 1})
            assert result == i + 1


# -- call without connect raises -------------------------------------------- #


def test_call_without_connect_raises() -> None:
    client = ControlClient("127.0.0.1", 9999)
    with pytest.raises(ConnectionError, match="not connected"):
        client.call("ping")


# -- wait_until_ready ------------------------------------------------------- #


def test_wait_until_ready_succeeds_against_live_server(
    server_and_port: tuple[ControlServer, int],
) -> None:
    _server, port = server_and_port
    client = ControlClient("127.0.0.1", port)
    # Should return almost immediately since the server is already up.
    client.wait_until_ready(timeout_s=2.0)
    # After wait_until_ready, the client should be connected and usable.
    result = client.call("ping")
    assert result == "pong"
    client.close()


def test_wait_until_ready_times_out_against_dead_port() -> None:
    # Bind a port and close it immediately — nothing is listening.
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    dead_port = sock.getsockname()[1]
    sock.close()

    client = ControlClient("127.0.0.1", dead_port)
    with pytest.raises(TimeoutError, match="not ready"):
        client.wait_until_ready(timeout_s=0.5, poll_interval=0.05)


def test_wait_until_ready_detects_delayed_server_start() -> None:
    # Start the server after a short delay; wait_until_ready should find it.
    # First, find a free port.
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    delayed_server = ControlServer(bind_port=port)
    delayed_server.register("ping", lambda _params: "pong")

    def _delayed_start() -> None:
        time.sleep(0.3)
        delayed_server.start()

    t = threading.Thread(target=_delayed_start)
    t.start()

    try:
        client = ControlClient("127.0.0.1", port)
        client.wait_until_ready(timeout_s=3.0, poll_interval=0.05)
        result = client.call("ping")
        assert result == "pong"
        client.close()
    finally:
        t.join(timeout=2.0)
        delayed_server.stop()


# -- ping helper ------------------------------------------------------------ #


def test_ping_helper_method(server_and_port: tuple[ControlServer, int]) -> None:
    _server, port = server_and_port
    with ControlClient("127.0.0.1", port) as client:
        assert client.ping() == "pong"


# -- token-authenticated client --------------------------------------------- #


def test_client_with_token() -> None:
    config = ControlPlaneConfig(host="0.0.0.0", token="mytoken")
    server = ControlServer(config, bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.start()
    try:
        assert server.port is not None
        # Client with correct token succeeds.
        with ControlClient("127.0.0.1", server.port, token="mytoken") as client:
            assert client.ping() == "pong"
    finally:
        server.stop()


def test_client_without_token_on_authenticated_server() -> None:
    config = ControlPlaneConfig(host="0.0.0.0", token="mytoken")
    server = ControlServer(config, bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.start()
    try:
        assert server.port is not None
        with ControlClient("127.0.0.1", server.port) as client:
            with pytest.raises(AuthError):
                client.ping()
    finally:
        server.stop()


# -- clean shutdown --------------------------------------------------------- #


def test_clean_shutdown_no_leaked_threads() -> None:
    server = ControlServer(bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.start()
    assert server.port is not None

    # Use it once.
    with ControlClient("127.0.0.1", server.port) as client:
        client.ping()

    server.stop()
    time.sleep(0.1)
    thread_names = [t.name for t in threading.enumerate()]
    assert "isaac-core-control-server" not in thread_names
