"""The resolved ENU reference must be reported, not only reported when it disagrees.

Only a mismatch was logged. A scene whose `/CesiumGeoreference` was missing, misspelled or authored at a
non-standard path therefore fell back to the config with **no log line at all**, and everything sat at a
plausible-looking but wrong offset with nothing in the output pointing at the cause.

Found by running a newly authored non-Cesium scene and noticing the log said nothing either way -- the
absence of a message was indistinguishable from a successful read.
"""

from __future__ import annotations

import contextlib
import logging
from typing import Any

import pytest

from isaac_core.config import IsaacCoreConfig
from isaac_core.sim.capabilities import FakeStageInspector
from isaac_core.sim.composer import _report_enu_reference
from isaac_core.sim.georeference import (
    CESIUM_GEOREFERENCE_PRIM,
    SOURCE_CONFIG,
    SOURCE_SCENE,
    ResolvedEnuReference,
    resolve_enu_reference,
)

# The shipped scene's reference point.
REFERENCE = {"lat_deg": 32.22481, "lon_deg": 35.25621, "alt_m": 516.7}


def _with_georeference() -> FakeStageInspector:
    """Return a stage that carries a georeference prim at the conventional path."""
    return FakeStageInspector(prims=frozenset({CESIUM_GEOREFERENCE_PRIM}))


@contextlib.contextmanager
def _captured() -> Any:
    """Capture the composer's log records.

    `caplog` does not see this project's loggers, which is already noted in `test_tileset_url.py`, so
    attach a handler to the named logger and collect the records directly.

    Returns:
        A list that fills with the composer's records for the duration of the block.

    """
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.composer")
    previous = logger.level
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    try:
        yield records
    finally:
        logger.removeHandler(handler)
        logger.setLevel(previous)


def _text(records: list[logging.LogRecord]) -> str:
    """Return the rendered text of captured records."""
    # getMessage() already interpolates the args; doing it again raises.
    return "\n".join(record.getMessage() for record in records)


def _resolved(source: str) -> ResolvedEnuReference:
    """Return a resolved reference with the shipped values and a given source."""
    config = IsaacCoreConfig(geo={"enu_reference": REFERENCE})
    return ResolvedEnuReference(config.geo.enu_reference, source)


def test_the_reference_is_logged_even_when_nothing_disagrees() -> None:
    # The whole point: a successful read must be visible, or its absence cannot be noticed.
    with _captured() as records:
        _report_enu_reference(_resolved(SOURCE_CONFIG), _with_georeference())
    text = _text(records)
    assert "ENU reference" in text, text
    assert "32.22481" in text
    assert "35.25621" in text
    assert "516.7" in text


def test_the_log_names_the_source() -> None:
    with _captured() as records:
        _report_enu_reference(_resolved(SOURCE_SCENE), _with_georeference())
    assert SOURCE_SCENE in _text(records)


def test_a_scene_without_a_georeference_says_so() -> None:
    # This is the case that used to be silent, and the one a newly authored scene hits.
    with _captured() as records:
        _report_enu_reference(_resolved(SOURCE_CONFIG), FakeStageInspector())
    text = _text(records)
    assert CESIUM_GEOREFERENCE_PRIM in text, text
    assert "no " in text.lower()


def test_a_scene_with_a_georeference_does_not_claim_it_is_missing() -> None:
    with _captured() as records:
        _report_enu_reference(_resolved(SOURCE_SCENE), _with_georeference())
    assert "has no" not in _text(records)


def test_the_config_wins_when_it_is_set_explicitly() -> None:
    # Guards the resolution rule itself, since the log above only reports what the rule decided.
    config = IsaacCoreConfig(geo={"enu_reference": REFERENCE})
    # Values that agree with the config: a large disagreement is a separate, already-tested error path,
    # and this test is about which source wins when both are present.
    scene: dict[str, Any] = {
        "cesium:georeferenceOrigin:latitude": REFERENCE["lat_deg"],
        "cesium:georeferenceOrigin:longitude": REFERENCE["lon_deg"],
        "cesium:georeferenceOrigin:height": REFERENCE["alt_m"],
    }
    resolved = resolve_enu_reference(
        config,
        FakeStageInspector(
            prims=frozenset({CESIUM_GEOREFERENCE_PRIM}),
            doubles={(CESIUM_GEOREFERENCE_PRIM, key): value for key, value in scene.items()},
        ),
    )
    assert resolved.source == SOURCE_CONFIG
    assert resolved.reference.lat_deg == pytest.approx(REFERENCE["lat_deg"])


def test_the_scene_wins_when_the_config_says_nothing() -> None:
    config = IsaacCoreConfig()
    scene: dict[str, Any] = {
        "cesium:georeferenceOrigin:latitude": 10.5,
        "cesium:georeferenceOrigin:longitude": 20.25,
        "cesium:georeferenceOrigin:height": 30.0,
    }
    resolved = resolve_enu_reference(
        config,
        FakeStageInspector(
            prims=frozenset({CESIUM_GEOREFERENCE_PRIM}),
            doubles={(CESIUM_GEOREFERENCE_PRIM, key): value for key, value in scene.items()},
        ),
    )
    assert resolved.source == SOURCE_SCENE
    assert resolved.reference.lat_deg == pytest.approx(10.5)


def test_a_missing_scene_georeference_falls_back_to_the_default() -> None:
    config = IsaacCoreConfig()
    resolved = resolve_enu_reference(config, FakeStageInspector())
    assert resolved.reference.lat_deg == pytest.approx(config.geo.enu_reference.lat_deg)
