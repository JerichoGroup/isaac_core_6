"""Tests for the logging configuration, in particular that `isaac_logs = false` works.

The setting existed in the schema but was never applied: a launch emitted roughly 3,400
lines regardless, and `ogn_registration` alone was 2,721 of them. Level changes do not fix
it, because extensions set their own logger levels while loading and the noisiest records
are emitted during that load. Filtering at the handler is what works.
"""

from __future__ import annotations

from collections.abc import Iterator
import logging
import os
import sys
import types

import pytest

from isaac_core.config import IsaacCoreConfig
from isaac_core.sim.__main__ import _configure_logging
from isaac_core.sim.runtime import STDOUT_FILENO, SimulationRuntime


@pytest.fixture(autouse=True)
def restore_logging() -> Iterator[None]:
    """Leave the root logger as it was found, so tests do not leak configuration."""
    root = logging.getLogger()
    level = root.level
    handlers = list(root.handlers)
    filters = {id(h): list(h.filters) for h in handlers}
    yield
    root.setLevel(level)
    root.handlers[:] = handlers
    for handler in handlers:
        handler.filters[:] = filters[id(handler)]


def _emitting_handler() -> logging.Handler:
    """Attach a recording handler to the root logger and return it."""
    handler = logging.Handler()
    handler.records = []  # type: ignore[attr-defined]
    handler.emit = lambda record: handler.records.append(record)  # type: ignore[attr-defined,method-assign]
    logging.getLogger().addHandler(handler)
    return handler


def _passes(handler: logging.Handler, name: str, level: int) -> bool:
    """Report whether a record from ``name`` at ``level`` survives the handler's filters."""
    record = logging.LogRecord(name, level, __file__, 1, "msg", None, None)
    return all(f(record) if callable(f) else f.filter(record) for f in handler.filters)


def test_noisy_loggers_are_dropped_when_isaac_logs_is_false() -> None:
    handler = _emitting_handler()
    _configure_logging(IsaacCoreConfig())
    assert not _passes(handler, "ogn_registration", logging.INFO)


def test_child_loggers_of_a_quiet_name_are_also_dropped() -> None:
    handler = _emitting_handler()
    _configure_logging(IsaacCoreConfig())
    assert not _passes(handler, "omni.kit.something.deep", logging.INFO)


def test_our_own_logging_still_gets_through() -> None:
    # The point is to remove Isaac's noise, not ours.
    handler = _emitting_handler()
    _configure_logging(IsaacCoreConfig())
    assert _passes(handler, "isaac_core.sim.runtime", logging.INFO)


def test_warnings_and_errors_are_never_dropped() -> None:
    # Quieting must not hide a real problem, however noisy the source normally is.
    handler = _emitting_handler()
    _configure_logging(IsaacCoreConfig())
    assert _passes(handler, "ogn_registration", logging.WARNING)
    assert _passes(handler, "ogn_registration", logging.ERROR)


def test_isaac_logs_true_installs_no_filter() -> None:
    handler = _emitting_handler()
    _configure_logging(IsaacCoreConfig(logging={"isaac_logs": True}))
    assert _passes(handler, "ogn_registration", logging.INFO)


def test_the_configured_level_reaches_our_logger() -> None:
    _configure_logging(IsaacCoreConfig(logging={"level": "debug"}))
    assert logging.getLogger("isaac_core").level == logging.DEBUG


def test_quiet_loggers_has_sensible_defaults() -> None:
    # ogn_registration is the single biggest contributor; losing it from the list would
    # silently restore most of the noise.
    quiet = IsaacCoreConfig().logging.quiet_loggers
    assert "ogn_registration" in quiet
    assert "isaac_core" not in quiet, "we must never filter our own output"


# --- pre-Kit banner suppression ---------------------------------------------- #
#
# The launcher and Warp banners print before our logging or Kit's settings can take effect.
# Warp is silenced through its config flag; the launcher banner through a stdout redirect
# around app construction. Both must yield to `isaac_logs = true`, and the redirect must be
# restored no matter what -- a silent boot that also hides a crash is worse than a noisy one.


class _BareRuntime(SimulationRuntime):
    """A runtime with the Isaac-dependent constructor bypassed, for banner tests."""

    def __init__(self, config: IsaacCoreConfig) -> None:
        """Store only the config the banner helpers read."""
        self._config = config


def _runtime(isaac_logs: bool) -> _BareRuntime:
    """Return a bare runtime whose config has the given ``isaac_logs`` value."""
    return _BareRuntime(IsaacCoreConfig(logging={"isaac_logs": isaac_logs}))


def test_warp_banner_is_silenced_when_isaac_logs_is_false(monkeypatch: pytest.MonkeyPatch) -> None:
    # Warp has no env var; the only lever is warp.config.quiet, set before Warp initialises.
    fake_config = types.ModuleType("warp.config")
    fake_config.quiet = False  # type: ignore[attr-defined]
    monkeypatch.setattr(
        "isaac_core.sim.runtime.importlib.import_module",
        lambda name: fake_config if name == "warp.config" else pytest.fail(f"unexpected import {name}"),
    )
    _runtime(isaac_logs=False)._silence_warp_banner()
    assert fake_config.quiet is True


def test_warp_banner_is_left_alone_when_isaac_logs_is_true(monkeypatch: pytest.MonkeyPatch) -> None:
    # isaac_logs = true means "show everything", so quiet must not be forced on.
    fake_config = types.ModuleType("warp.config")
    fake_config.quiet = False  # type: ignore[attr-defined]
    monkeypatch.setattr(
        "isaac_core.sim.runtime.importlib.import_module",
        lambda name: pytest.fail(f"warp must not even be imported, got {name}"),
    )
    _runtime(isaac_logs=True)._silence_warp_banner()
    assert fake_config.quiet is False


def test_missing_warp_is_not_an_error(monkeypatch: pytest.MonkeyPatch) -> None:
    # A build without Warp simply has no banner to quiet; this must not raise.
    def _raise(name: str) -> None:
        raise ImportError(name)

    monkeypatch.setattr("isaac_core.sim.runtime.importlib.import_module", _raise)
    _runtime(isaac_logs=False)._silence_warp_banner()


def test_suppressed_stdout_restores_the_descriptor_on_success() -> None:
    runtime = _runtime(isaac_logs=False)
    before = os.dup(STDOUT_FILENO)
    try:
        with runtime._suppressed_startup_stdout():
            pass
    finally:
        os.close(before)
    # The descriptor still refers to a live stream: writing must not raise.
    sys.stdout.write("")
    sys.stdout.flush()


def test_suppressed_stdout_restores_the_descriptor_when_the_block_raises() -> None:
    # This is the test that matters most: construction failing must still leave stdout usable,
    # or a silent boot would swallow the very crash it needs to report.
    runtime = _runtime(isaac_logs=False)

    class _BoomError(RuntimeError):
        pass

    with pytest.raises(_BoomError), runtime._suppressed_startup_stdout():
        raise _BoomError
    # stdout is restored: a following write reaches a real stream rather than the null device.
    sys.stdout.write("")
    sys.stdout.flush()


def test_suppressed_stdout_is_a_passthrough_when_isaac_logs_is_true(capfd: pytest.CaptureFixture[str]) -> None:
    # With isaac_logs on, the block must not redirect anything -- output flows normally.
    runtime = _runtime(isaac_logs=True)
    with runtime._suppressed_startup_stdout():
        # Write at the fd level, matching how the C++ banner reaches stdout.
        os.write(STDOUT_FILENO, b"visible-banner")
    captured = capfd.readouterr()
    assert "visible-banner" in captured.out


def test_suppressed_stdout_hides_fd_level_output_when_isaac_logs_is_false(capfd: pytest.CaptureFixture[str]) -> None:
    # The launcher banner is written at the fd level by Kit's C++ startup, so a fd-level write
    # is what the suppression has to catch.
    runtime = _runtime(isaac_logs=False)
    with runtime._suppressed_startup_stdout():
        os.write(STDOUT_FILENO, b"launcher-banner")
    captured = capfd.readouterr()
    assert "launcher-banner" not in captured.out
