"""
Tests for the ``apply_stage_units`` composer helper.

The whole pipeline assumes 1 stage unit == 1 metre, so the helper must both write the
configured value and warn loudly when it is asked to deviate or when it conflicts with the
scene's own declaration. ``pxr.UsdGeom`` is faked so this runs without Isaac Sim, matching
the fake-stage approach used elsewhere in this directory.
"""

from __future__ import annotations

import contextlib
import logging
import types
from typing import Any

import pytest

from isaac_core.sim import composer
from isaac_core.sim.composer import apply_stage_units


class _FakeStage:
    """A stage that stores a single metres-per-unit value in metadata."""

    def __init__(self, meters_per_unit: float) -> None:
        """Store the scene's declared metres-per-unit."""
        self.meters_per_unit = meters_per_unit


@pytest.fixture(autouse=True)
def fake_usdgeom(monkeypatch: pytest.MonkeyPatch) -> None:
    """Install a fake ``pxr.UsdGeom`` that reads and writes the fake stage's metadata."""
    module = types.ModuleType("pxr.UsdGeom")

    def get(stage: _FakeStage) -> float:
        return stage.meters_per_unit

    def set_(stage: _FakeStage, value: float) -> None:
        stage.meters_per_unit = value

    module.GetStageMetersPerUnit = get  # type: ignore[attr-defined]
    module.SetStageMetersPerUnit = set_  # type: ignore[attr-defined]
    monkeypatch.setattr(composer, "_usdgeom", lambda: module)


@contextlib.contextmanager
def _captured_warnings() -> Any:  # noqa: ANN401
    """
    Capture warnings from the composer logger.

    ``caplog`` does not see this project's warnings, so attach a handler directly to the
    named logger and yield the raw records for the test to inspect.

    Returns:
        A list that fills with the composer's log records for the duration of the block.

    """
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.composer")
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    try:
        yield records
    finally:
        logger.removeHandler(handler)


def test_writes_the_configured_value_to_the_stage() -> None:
    stage = _FakeStage(meters_per_unit=1.0)
    assert apply_stage_units(stage, 1.0) is True
    assert stage.meters_per_unit == 1.0


def test_metric_default_does_not_warn() -> None:
    # 1.0 is the supported mode: no conflict and no rescale, so the helper stays quiet.
    stage = _FakeStage(meters_per_unit=1.0)
    with _captured_warnings() as records:
        apply_stage_units(stage, 1.0)
    warnings = [r.getMessage() for r in records if r.levelno >= logging.WARNING]
    assert warnings == []


def test_non_metric_value_warns_but_is_still_applied() -> None:
    # Anything other than 1.0 rescales the world relative to metre-based poses. It is applied
    # so the user is not silently overridden, but flagged as unsupported.
    stage = _FakeStage(meters_per_unit=0.01)
    with _captured_warnings() as records:
        result = apply_stage_units(stage, 0.01)
    assert result is True
    assert stage.meters_per_unit == 0.01
    warnings = [r.getMessage() for r in records if r.levelno >= logging.WARNING]
    assert any("assumes 1 unit == 1 metre" in message for message in warnings)


def test_scene_conflict_is_reported() -> None:
    # Scene says centimetres, config says metres: one of them is wrong about the numbers.
    stage = _FakeStage(meters_per_unit=0.01)
    with _captured_warnings() as records:
        apply_stage_units(stage, 1.0)
    warnings = [r.getMessage() for r in records if r.levelno >= logging.WARNING]
    assert any("scene declares metersPerUnit" in message for message in warnings)
    # Config wins: the stage ends up at the configured value.
    assert stage.meters_per_unit == 1.0


def test_matching_scene_and_config_does_not_report_a_conflict() -> None:
    # A scene already at the configured value must not be flagged as a conflict.
    stage = _FakeStage(meters_per_unit=1.0)
    with _captured_warnings() as records:
        apply_stage_units(stage, 1.0)
    warnings = [r.getMessage() for r in records if r.levelno >= logging.WARNING]
    assert not any("scene declares metersPerUnit" in message for message in warnings)


def test_tiny_float_drift_is_not_a_conflict() -> None:
    # A GUI-authored scene can land a hair off 1.0; drift inside tolerance is treated as equal.
    stage = _FakeStage(meters_per_unit=1.0 + 1e-12)
    with _captured_warnings() as records:
        apply_stage_units(stage, 1.0)
    warnings = [r.getMessage() for r in records if r.levelno >= logging.WARNING]
    assert warnings == []
