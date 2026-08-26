"""
Tests for the logging configuration, in particular that `isaac_logs = false` works.

The setting existed in the schema but was never applied: a launch emitted roughly 3,400
lines regardless, and `ogn_registration` alone was 2,721 of them. Level changes do not fix
it, because extensions set their own logger levels while loading and the noisiest records
are emitted during that load. Filtering at the handler is what works.
"""

from __future__ import annotations

from collections.abc import Iterator
import logging

import pytest

from isaac_core.config import IsaacCoreConfig
from isaac_core.sim.__main__ import _configure_logging


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
