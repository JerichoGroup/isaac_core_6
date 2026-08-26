"""
Tests for the runtime's main-thread task dispatcher.

USD, Fabric and the Kit application are not thread-safe. Handlers that touch them must
hand the work to the step loop instead of running it on the control server's thread.
These tests drive the dispatcher directly, so they need no Isaac Sim.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Any

import pytest

from isaac_core.sim.runtime import MAIN_THREAD_TASK_TIMEOUT_S, SimulationRuntime

if TYPE_CHECKING:
    from collections.abc import Callable

if TYPE_CHECKING:
    pass


class _FakeRuntime(SimulationRuntime):
    """A runtime with the Isaac-dependent constructor bypassed."""

    def __init__(self) -> None:
        """Initialise only the fields the dispatcher touches."""
        import queue

        from isaac_core.sim.runtime import _MainThreadTask  # noqa: PLC0415

        self._main_thread_tasks: queue.Queue[_MainThreadTask] = queue.Queue()
        self._loop_thread_id: int | None = None


@pytest.fixture
def runtime() -> _FakeRuntime:
    """Return a runtime with just the dispatcher wired up."""
    return _FakeRuntime()


def test_runs_inline_before_the_loop_starts(runtime: _FakeRuntime) -> None:
    # With no loop yet there is nothing to hand off to, so the call must still happen.
    # Otherwise every handler would time out during startup.
    assert runtime._on_main_thread(lambda: "value") == "value"


def test_runs_inline_when_already_on_the_loop_thread(runtime: _FakeRuntime) -> None:
    # Queueing from the loop thread itself would deadlock: the thread would wait for
    # itself to drain the queue.
    runtime._loop_thread_id = threading.get_ident()
    assert runtime._on_main_thread(lambda: 7) == 7


def test_work_is_executed_by_the_draining_thread(runtime: _FakeRuntime) -> None:
    # The task must run on the thread that drains, not the thread that submits.
    runtime._loop_thread_id = threading.get_ident()
    ran_on: list[int] = []
    submitter_result: list[Any] = []

    def submit() -> None:
        submitter_result.append(runtime._on_main_thread(lambda: ran_on.append(threading.get_ident())))

    worker = threading.Thread(target=submit)
    worker.start()
    deadline = time.monotonic() + 5.0
    while not ran_on and time.monotonic() < deadline:
        runtime._drain_main_thread_tasks()
        time.sleep(0.01)
    worker.join(timeout=5.0)

    assert ran_on, "the task was never drained"
    assert ran_on[0] == threading.get_ident(), "task must run on the draining thread"


def test_the_result_reaches_the_submitting_thread(runtime: _FakeRuntime) -> None:
    runtime._loop_thread_id = threading.get_ident()
    received: list[Any] = []

    worker = threading.Thread(target=lambda: received.append(runtime._on_main_thread(lambda: {"translate": [1, 2, 3]})))
    worker.start()
    deadline = time.monotonic() + 5.0
    while not received and time.monotonic() < deadline:
        runtime._drain_main_thread_tasks()
        time.sleep(0.01)
    worker.join(timeout=5.0)

    assert received == [{"translate": [1, 2, 3]}]


def test_an_exception_is_re_raised_in_the_submitting_thread(runtime: _FakeRuntime) -> None:
    # A failing handler must surface as an error to its caller while leaving the step
    # loop turning.
    runtime._loop_thread_id = threading.get_ident()
    captured: list[BaseException] = []

    def submit() -> None:
        def boom() -> None:
            message = "no stage open"
            raise RuntimeError(message)

        try:
            runtime._on_main_thread(boom)
        except RuntimeError as exc:
            captured.append(exc)

    worker = threading.Thread(target=submit)
    worker.start()
    deadline = time.monotonic() + 5.0
    while not captured and time.monotonic() < deadline:
        runtime._drain_main_thread_tasks()
        time.sleep(0.01)
    worker.join(timeout=5.0)

    assert captured, "the exception never reached the caller"
    assert str(captured[0]) == "no stage open"


def test_draining_survives_a_failing_task(runtime: _FakeRuntime) -> None:
    runtime._loop_thread_id = threading.get_ident()
    results: list[Any] = []

    def submit(call: "Callable[[], Any]") -> threading.Thread:
        thread = threading.Thread(target=lambda: results.append(_swallow(runtime, call)))
        thread.start()
        return thread

    def _boom() -> None:
        message = "bad"
        raise ValueError(message)

    threads = [submit(_boom), submit(lambda: "fine")]
    deadline = time.monotonic() + 5.0
    while len(results) < 2 and time.monotonic() < deadline:
        runtime._drain_main_thread_tasks()
        time.sleep(0.01)
    for thread in threads:
        thread.join(timeout=5.0)

    # The good task still completes even though the other raised.
    assert "fine" in results


def _swallow(runtime: _FakeRuntime, call: "Callable[[], Any]") -> object:
    """Run a task and return its exception rather than propagating it."""
    try:
        return runtime._on_main_thread(call)
    except BaseException as exc:  # noqa: BLE001
        return exc


def test_a_wedged_loop_times_out_instead_of_hanging(runtime: _FakeRuntime, monkeypatch: pytest.MonkeyPatch) -> None:
    # Nothing ever drains here, which is what a wedged step loop looks like. A bounded
    # error is far better than a client that hangs forever.
    monkeypatch.setattr("isaac_core.sim.runtime.MAIN_THREAD_TASK_TIMEOUT_S", 0.05)
    runtime._loop_thread_id = threading.get_ident() + 1
    with pytest.raises(TimeoutError, match="did not run the task"):
        runtime._on_main_thread(lambda: "never")


def test_the_timeout_is_bounded_and_generous() -> None:
    # Long enough for a slow frame on a cold stage, short enough to surface a problem.
    assert 1.0 <= MAIN_THREAD_TASK_TIMEOUT_S <= 60.0
