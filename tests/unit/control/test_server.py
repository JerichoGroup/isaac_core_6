"""Tests for the ControlServer: dispatch, security, path confinement, lifecycle."""

from collections.abc import Generator
import json
from pathlib import Path
import socket
import threading
import time
from typing import Any

import pytest

from isaac_core.config.schema import ControlPlaneConfig
from isaac_core.control.errors import (
    AUTH_ERROR_CODE,
    INTERNAL_ERROR_CODE,
    InvalidParamsError,
    MethodNotFoundError,
)
from isaac_core.control.messages import (
    RpcRequest,
    decode_response,
    encode,
)
from isaac_core.control.server import ControlServer, confine_path

# -- helpers ---------------------------------------------------------------- #


def _send_recv(host: str, port: int, request: RpcRequest) -> dict[str, Any]:
    """Send a request and return the parsed response dict."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((host, port))
    try:
        sock.sendall(encode(request))
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
        result: dict[str, Any] = json.loads(buf.split(b"\n", 1)[0])
        return result
    finally:
        sock.close()


def _send_raw_recv(host: str, port: int, raw: bytes) -> dict[str, Any]:
    """Send raw bytes and return the parsed response dict."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((host, port))
    try:
        sock.sendall(raw)
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
        result: dict[str, Any] = json.loads(buf.split(b"\n", 1)[0])
        return result
    finally:
        sock.close()


@pytest.fixture()
def loopback_server() -> Generator[ControlServer, None, None]:
    """Start a server on loopback with an ephemeral port."""
    server = ControlServer(bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.register("echo", lambda params: params)
    server.start()
    yield server
    server.stop()


# -- request/response round-trip -------------------------------------------- #


def test_ping_returns_pong(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None
    req = RpcRequest(method="ping", id=1)
    resp = _send_recv("127.0.0.1", loopback_server.port, req)
    assert resp["result"] == "pong"
    assert resp["id"] == 1


def test_echo_returns_params(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None
    req = RpcRequest(method="echo", params={"x": 42}, id=2)
    resp = _send_recv("127.0.0.1", loopback_server.port, req)
    assert resp["result"] == {"x": 42}


# -- unknown method -> method-not-found ------------------------------------- #


def test_unknown_method_returns_method_not_found(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None
    req = RpcRequest(method="nonexistent", id=3)
    resp = _send_recv("127.0.0.1", loopback_server.port, req)
    assert resp["error"]["code"] == MethodNotFoundError.code


# -- malformed JSON -> parse error ------------------------------------------ #


def test_malformed_json_returns_parse_error(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None
    resp = _send_raw_recv("127.0.0.1", loopback_server.port, b"this is not json\n")
    assert resp["error"]["code"] == -32700


# -- handler exception -> internal error, server survives ------------------- #


def test_handler_exception_becomes_internal_error(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None

    def _raise(_params: object) -> None:
        msg = "kaboom"
        raise RuntimeError(msg)

    loopback_server.register("explode", _raise)
    req = RpcRequest(method="explode", id=4)
    resp = _send_recv("127.0.0.1", loopback_server.port, req)
    assert resp["error"]["code"] == INTERNAL_ERROR_CODE
    assert "kaboom" in resp["error"]["message"]


def test_server_still_serves_after_handler_exception(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None

    def _raise(_params: object) -> None:
        msg = "boom"
        raise RuntimeError(msg)

    loopback_server.register("explode2", _raise)
    # Trigger the exception.
    req_bad = RpcRequest(method="explode2", id=5)
    _send_recv("127.0.0.1", loopback_server.port, req_bad)

    # Server must still work.
    req_good = RpcRequest(method="ping", id=6)
    resp = _send_recv("127.0.0.1", loopback_server.port, req_good)
    assert resp["result"] == "pong"


# -- handler raising RpcError subclass passes through ----------------------- #


def test_handler_rpc_error_passes_through(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None

    def _invalid(_params: object) -> None:
        raise InvalidParamsError("x must be positive", data={"param": "x"})

    loopback_server.register("validate", _invalid)
    req = RpcRequest(method="validate", id=7)
    resp = _send_recv("127.0.0.1", loopback_server.port, req)
    assert resp["error"]["code"] == InvalidParamsError.code
    assert resp["error"]["data"] == {"param": "x"}


# -- token enforcement ------------------------------------------------------ #


def test_token_not_required_for_loopback(loopback_server: ControlServer) -> None:
    # loopback_server is bound to 127.0.0.1 — no token needed.
    assert loopback_server.port is not None
    req = RpcRequest(method="ping", id=8)  # no token
    resp = _send_recv("127.0.0.1", loopback_server.port, req)
    assert "error" not in resp or resp.get("error") is None


def test_token_required_for_non_loopback() -> None:
    # Bind to a non-loopback host config (0.0.0.0) which requires a token.
    config = ControlPlaneConfig(host="0.0.0.0", token="secret123")
    server = ControlServer(config, bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.start()
    try:
        assert server.port is not None
        # Request WITHOUT token should fail.
        req_no_token = RpcRequest(method="ping", id=9)
        resp = _send_recv("127.0.0.1", server.port, req_no_token)
        assert resp["error"]["code"] == AUTH_ERROR_CODE

        # Request WITH wrong token should fail.
        req_bad_token = RpcRequest(method="ping", id=10, token="wrong")
        resp = _send_recv("127.0.0.1", server.port, req_bad_token)
        assert resp["error"]["code"] == AUTH_ERROR_CODE

        # Request WITH correct token should succeed.
        req_good_token = RpcRequest(method="ping", id=11, token="secret123")
        resp = _send_recv("127.0.0.1", server.port, req_good_token)
        assert resp["result"] == "pong"
    finally:
        server.stop()


# -- path confinement ------------------------------------------------------- #


def test_confine_path_accepts_relative_under_root(tmp_path: Path) -> None:
    root = tmp_path / "output"
    root.mkdir()
    result = confine_path("frames/img_001.png", root)
    assert result == root / "frames" / "img_001.png"


def test_confine_path_rejects_etc_passwd(tmp_path: Path) -> None:
    root = tmp_path / "output"
    root.mkdir()
    with pytest.raises(PermissionError, match="outside output_root"):
        confine_path("/etc/passwd", root)


def test_confine_path_rejects_dot_dot_traversal(tmp_path: Path) -> None:
    root = tmp_path / "output"
    root.mkdir()
    with pytest.raises(PermissionError, match="outside output_root"):
        confine_path("../../etc/passwd", root)


def test_confine_path_rejects_absolute_escape(tmp_path: Path) -> None:
    root = tmp_path / "output"
    root.mkdir()
    with pytest.raises(PermissionError, match="outside output_root"):
        confine_path("/tmp/evil.txt", root)


def test_confine_path_rejects_symlink_escape(tmp_path: Path) -> None:
    # Create a symlink inside output_root that points outside.
    root = tmp_path / "output"
    root.mkdir()
    link = root / "escape"
    link.symlink_to("/etc")
    with pytest.raises(PermissionError, match="outside output_root"):
        confine_path("escape/passwd", root)


def test_confine_path_accepts_nested_valid_path(tmp_path: Path) -> None:
    root = tmp_path / "output"
    (root / "deep" / "dir").mkdir(parents=True)
    result = confine_path("deep/dir/file.png", root)
    assert result == root / "deep" / "dir" / "file.png"


# -- clean shutdown, no leaked threads -------------------------------------- #


def test_stop_leaves_no_server_thread_alive() -> None:
    server = ControlServer(bind_port=0)
    server.register("ping", lambda _params: "pong")
    server.start()
    server.stop()
    # Allow a moment for thread cleanup.
    time.sleep(0.1)
    thread_names = [t.name for t in threading.enumerate()]
    assert "isaac-core-control-server" not in thread_names


# -- concurrent clients ----------------------------------------------------- #


def test_concurrent_clients(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None
    results: list[str] = []
    errors: list[BaseException] = []

    def _worker(client_id: int) -> None:
        try:
            req = RpcRequest(method="ping", id=client_id)
            resp = _send_recv("127.0.0.1", loopback_server.port, req)  # type: ignore[arg-type]
            results.append(resp["result"])
        except BaseException as exc:
            errors.append(exc)

    threads = [threading.Thread(target=_worker, args=(i,)) for i in range(10)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=5.0)

    assert not errors
    assert len(results) == 10
    assert all(r == "pong" for r in results)


# -- multiple requests on a single connection ------------------------------- #


def test_multiple_requests_on_single_connection(loopback_server: ControlServer) -> None:
    assert loopback_server.port is not None
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect(("127.0.0.1", loopback_server.port))
    try:
        for i in range(5):
            req = RpcRequest(method="ping", id=100 + i)
            sock.sendall(encode(req))
            buf = b""
            while b"\n" not in buf:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
            line, _ = buf.split(b"\n", 1)
            resp = decode_response(line)
            assert resp.result == "pong"
            assert resp.id == 100 + i
    finally:
        sock.close()
