"""The client must not buffer a response without limit.

The server caps an incoming request line, because a peer that never sends a newline would otherwise make it
grow until the process dies. The client read the same way and had no cap, which is the same hazard pointed the
other way: a broken or hostile simulator could exhaust the memory of anything talking to it.

Low likelihood, and a two-line asymmetry in a place where the other direction was already handled.
"""

from __future__ import annotations

import socket
import threading

import pytest

from isaac_core.control.client import MAX_RESPONSE_BYTES, ControlClient


def test_the_cap_matches_the_servers() -> None:
    # Two different limits for the same hazard would be its own small bug.
    from isaac_core.control.server import MAX_REQUEST_BYTES

    assert MAX_RESPONSE_BYTES == MAX_REQUEST_BYTES


def test_a_response_without_a_newline_is_refused_rather_than_buffered() -> None:
    # The whole point: a peer that never terminates its line must not be able to grow the buffer forever.
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))  # port 0: never hardcode one in a test
    port = listener.getsockname()[1]
    listener.listen(1)

    stop = threading.Event()

    def flood() -> None:
        """Accept one connection and send newline-free bytes until told to stop."""
        try:
            conn, _ = listener.accept()
        except OSError:
            return
        payload = b"x" * 65536
        try:
            while not stop.is_set():
                conn.sendall(payload)
        except OSError:
            pass
        finally:
            conn.close()

    thread = threading.Thread(target=flood, daemon=True)
    thread.start()
    try:
        client = ControlClient(port=port, call_timeout_s=10.0)
        client.connect()
        try:
            with pytest.raises(ConnectionError, match="no newline"):
                client.call("ping")
        finally:
            client.close()
    finally:
        stop.set()
        thread.join(timeout=5.0)
        listener.close()


def test_a_normal_response_still_works() -> None:
    # The cap must not break the ordinary case, which is the risk with any limit added after the fact.
    import json

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    listener.listen(1)

    def reply() -> None:
        """Accept one connection and answer whatever id was asked for."""
        try:
            conn, _ = listener.accept()
        except OSError:
            return
        try:
            data = b""
            while b"\n" not in data:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                data += chunk
            request = json.loads(data.split(b"\n", 1)[0])
            conn.sendall(json.dumps({"jsonrpc": "2.0", "id": request["id"], "result": "pong"}).encode() + b"\n")
        except OSError:
            pass
        finally:
            conn.close()

    thread = threading.Thread(target=reply, daemon=True)
    thread.start()
    try:
        client = ControlClient(port=port, call_timeout_s=10.0)
        client.connect()
        try:
            assert client.call("ping") == "pong"
        finally:
            client.close()
    finally:
        thread.join(timeout=5.0)
        listener.close()
