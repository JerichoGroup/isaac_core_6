"""The call timeout is configurable, because 60 seconds is not always enough.

Methods that must run on the simulator's main thread -- `step`, `set_pose`, `capture_frame` -- wait for
that thread to reach them. A two-vehicle headless stage was measured holding it for longer than the
60-second default while the control plane kept answering `get_state` promptly, so calls failed on a
stage that was composed and healthy.

The timeout was hardcoded, so there was no way to raise it from a script. These tests exist because the
symptom is indistinguishable from a hang unless you know main-thread work is the cause.
"""

from __future__ import annotations

import socket
import threading

from isaac_core.control.client import DEFAULT_CALL_TIMEOUT_S, ControlClient


def test_the_default_is_unchanged() -> None:
    # Raising the default would hide real hangs behind a long wait, so the opt-in is deliberate.
    assert ControlClient().call_timeout_s == DEFAULT_CALL_TIMEOUT_S
    assert DEFAULT_CALL_TIMEOUT_S == 60.0


def test_a_custom_timeout_is_reported() -> None:
    assert ControlClient(call_timeout_s=240.0).call_timeout_s == 240.0


def test_the_custom_timeout_reaches_the_socket() -> None:
    # The bug this guards would be a stored-but-unused setting: the attribute reads back correctly while
    # the socket keeps the default, which no assertion on the property alone would catch.
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))  # port 0: never hardcode one in a test
    listener.listen(1)
    accepted: list[socket.socket] = []
    thread = threading.Thread(target=lambda: accepted.append(listener.accept()[0]), daemon=True)
    thread.start()
    try:
        client = ControlClient(port=listener.getsockname()[1], call_timeout_s=17.5)
        client.connect()
        assert client._sock is not None
        assert client._sock.gettimeout() == 17.5
        client.close()
    finally:
        thread.join(timeout=5.0)
        for sock in accepted:
            sock.close()
        listener.close()


def test_an_explicit_connect_timeout_still_wins() -> None:
    # `connect(timeout=...)` predates the constructor argument and is used by the readiness poll, so it
    # must keep overriding rather than being silently ignored.
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    accepted: list[socket.socket] = []
    thread = threading.Thread(target=lambda: accepted.append(listener.accept()[0]), daemon=True)
    thread.start()
    try:
        client = ControlClient(port=listener.getsockname()[1], call_timeout_s=240.0)
        client.connect(timeout=5.0)
        assert client._sock is not None
        assert client._sock.gettimeout() == 5.0
        client.close()
    finally:
        thread.join(timeout=5.0)
        for sock in accepted:
            sock.close()
        listener.close()


def test_the_devkit_exposes_it_on_both_entry_points() -> None:
    # A timeout settable on the client but not through `Sim` would be unreachable for the people who hit
    # this, since almost nobody constructs a ControlClient directly.
    import inspect

    from isaac_core.devkit import Sim

    for name in ("attach", "launch"):
        signature = inspect.signature(getattr(Sim, name))
        assert "call_timeout_s" in signature.parameters, f"Sim.{name} cannot set the call timeout"
        assert signature.parameters["call_timeout_s"].default == DEFAULT_CALL_TIMEOUT_S
