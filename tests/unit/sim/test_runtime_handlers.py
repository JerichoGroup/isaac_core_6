"""
Tests for the timeline control-plane handlers dispatching onto the step loop.

Regression: a timeline call issued from the control-server thread crashed inside Isaac's
throttling extension with ``RuntimeError: There is no current event loop in thread
'isaac-core-control-server'``. ``play``/``pause``/``update_app`` wrap ``omni.timeline``, whose
transitions fire Isaac extension callbacks that assume the main thread with a live asyncio event
loop. ``pause``, ``resume`` and ``step`` must therefore run their timeline work on the step-loop
thread via ``_on_main_thread``, never on the caller's thread.

These tests drive the handlers with a fake app that records which thread every call arrived on, so
they need no Isaac Sim.
"""

from __future__ import annotations

import queue
import sys
import threading
import time
import types
from typing import TYPE_CHECKING, Any

import pytest

from isaac_core.sim.runtime import SimulationRuntime

if TYPE_CHECKING:
    from collections.abc import Callable


@pytest.fixture(autouse=True)
def fake_timeline(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    Install a fake ``omni.timeline`` so handlers run without Isaac Sim.

    ``step`` calls ``forward_one_frame`` when the timeline is not playing, because
    ``update_app`` alone does not advance a paused timeline -- stepping while paused used to do
    nothing at all.
    """

    class _Timeline:
        def __init__(self) -> None:
            self.forwarded = 0
            self.played = 0

        def is_playing(self) -> bool:
            return False

        def forward_one_frame(self) -> None:
            self.forwarded += 1

        def play(self) -> None:
            self.played += 1

        def stop(self) -> None:
            pass

        def set_current_time(self, _value: float) -> None:
            pass

    module = types.ModuleType("omni.timeline")
    timeline = _Timeline()
    module.get_timeline_interface = lambda: timeline  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "omni.timeline", module)


class _FakeAppUtils:
    """A stand-in for ``isaacsim.core.experimental.utils.app`` that records call threads."""

    def __init__(self) -> None:
        """Initialise the per-method thread-id logs."""
        self.play_threads: list[int] = []
        self.pause_threads: list[int] = []
        self.update_threads: list[int] = []

    def play(self) -> None:
        """Record the thread a ``play`` call arrived on."""
        self.play_threads.append(threading.get_ident())

    def pause(self) -> None:
        """Record the thread a ``pause`` call arrived on."""
        self.pause_threads.append(threading.get_ident())

    def update_app(self) -> None:
        """Record the thread an ``update_app`` call arrived on."""
        self.update_threads.append(threading.get_ident())


class _HandlerRuntime(SimulationRuntime):
    """A runtime with the Isaac-dependent constructor bypassed, wired for handler tests."""

    def __init__(self, app_utils: Any) -> None:  # noqa: ANN401
        """
        Initialise only the fields the timeline handlers touch.

        Args:
            app_utils: The fake app-utils stand-in the handlers dispatch to.

        """
        from isaac_core.sim.runtime import _MainThreadTask  # noqa: PLC0415

        self._main_thread_tasks: queue.Queue[_MainThreadTask] = queue.Queue()
        self._loop_thread_id: int | None = None
        self._app_utils = app_utils


def _run_off_thread(call: "Callable[[], Any]", drain: "Callable[[], None]", loop_thread_id: int) -> Any:  # noqa: ANN401
    """
    Run ``call`` on a worker thread while draining the queue on this (the loop) thread.

    Args:
        call: The handler invocation to run off the loop thread.
        drain: The runtime's queue-draining method.
        loop_thread_id: The id assigned as the loop thread; asserted to be the current thread.

    Returns:
        Whatever ``call`` returned.

    """
    assert threading.get_ident() == loop_thread_id
    box: list[Any] = []
    error: list[BaseException] = []

    def worker() -> None:
        try:
            box.append(call())
        except BaseException as exc:  # noqa: BLE001
            error.append(exc)

    thread = threading.Thread(target=worker)
    thread.start()
    deadline = time.monotonic() + 5.0
    while not box and not error and time.monotonic() < deadline:
        drain()
        time.sleep(0.005)
    thread.join(timeout=5.0)
    if error:
        raise error[0]
    assert box, "the handler never completed"
    return box[0]


def test_resume_plays_on_the_loop_thread_never_the_caller() -> None:
    # The reported bug: resume from the control thread reached Isaac's throttling extension
    # _on_play and died with 'no current event loop'. play() must land on the loop thread.
    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)
    loop_id = threading.get_ident()
    runtime._loop_thread_id = loop_id

    result = _run_off_thread(lambda: runtime._handle_resume(None), runtime._drain_main_thread_tasks, loop_id)

    assert result == "resumed"
    assert app.play_threads == [loop_id]


def test_pause_pauses_on_the_loop_thread_never_the_caller() -> None:
    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)
    loop_id = threading.get_ident()
    runtime._loop_thread_id = loop_id

    result = _run_off_thread(lambda: runtime._handle_pause(None), runtime._drain_main_thread_tasks, loop_id)

    assert result == "paused"
    assert app.pause_threads == [loop_id]


def test_step_updates_on_the_loop_thread_never_the_caller() -> None:
    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)
    loop_id = threading.get_ident()
    runtime._loop_thread_id = loop_id

    result = _run_off_thread(lambda: runtime._handle_step(None), runtime._drain_main_thread_tasks, loop_id)

    assert result == "stepped"
    assert app.update_threads == [loop_id]


@pytest.mark.parametrize("count", [1, 3, 10])
def test_step_pumps_count_frames_on_the_loop_thread(count: int) -> None:
    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)
    loop_id = threading.get_ident()
    runtime._loop_thread_id = loop_id

    result = _run_off_thread(lambda: runtime._handle_step({"count": count}), runtime._drain_main_thread_tasks, loop_id)

    assert result == "stepped"
    assert len(app.update_threads) == count
    assert set(app.update_threads) == {loop_id}


def test_step_rejects_a_non_positive_count() -> None:
    from isaac_core.control import InvalidParamsError  # noqa: PLC0415

    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)
    runtime._loop_thread_id = threading.get_ident()

    with pytest.raises(InvalidParamsError):
        runtime._handle_step({"count": 0})
    assert app.update_threads == []


def test_resume_runs_inline_before_the_loop_starts() -> None:
    # Before the loop thread id is set there is nothing to hand off to, so the call runs
    # inline. Without this every handler would time out during startup.
    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)

    assert runtime._handle_resume(None) == "resumed"
    assert app.play_threads == [threading.get_ident()]


class _BoomAppUtils(_FakeAppUtils):
    """A fake whose ``play`` raises, to check errors propagate rather than hang."""

    def play(self) -> None:
        """Raise as if Isaac blew up on the timeline transition."""
        message = "no current event loop"
        raise RuntimeError(message)


def test_a_raising_timeline_call_propagates_to_the_caller() -> None:
    # A handler that raises inside the queued task must surface the error to the caller
    # rather than hanging it forever waiting on the task event.
    app = _BoomAppUtils()
    runtime = _HandlerRuntime(app)
    loop_id = threading.get_ident()
    runtime._loop_thread_id = loop_id

    with pytest.raises(RuntimeError, match="no current event loop"):
        _run_off_thread(lambda: runtime._handle_resume(None), runtime._drain_main_thread_tasks, loop_id)


def test_repeated_draining_does_not_lose_queued_work() -> None:
    # Draining an empty queue between real tasks must not drop later ones: every submitted
    # handler eventually runs.
    app = _FakeAppUtils()
    runtime = _HandlerRuntime(app)
    loop_id = threading.get_ident()
    runtime._loop_thread_id = loop_id

    results: list[Any] = []

    def submit(call: "Callable[[], Any]") -> threading.Thread:
        thread = threading.Thread(target=lambda: results.append(call()))
        thread.start()
        return thread

    threads = [
        submit(lambda: runtime._handle_pause(None)),
        submit(lambda: runtime._handle_resume(None)),
        submit(lambda: runtime._handle_step(None)),
    ]

    deadline = time.monotonic() + 5.0
    while len(results) < 3 and time.monotonic() < deadline:
        runtime._drain_main_thread_tasks()
        # An extra empty drain between real ones must not swallow pending work.
        runtime._drain_main_thread_tasks()
        time.sleep(0.005)
    for thread in threads:
        thread.join(timeout=5.0)

    assert sorted(results) == ["paused", "resumed", "stepped"]
    assert app.play_threads == [loop_id]
    assert app.pause_threads == [loop_id]
    assert app.update_threads == [loop_id]
