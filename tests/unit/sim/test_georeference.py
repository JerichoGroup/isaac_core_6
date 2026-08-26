"""Tests for deriving the ENU reference from the scene's Cesium georeference."""

import pytest

from isaac_core.config import EnuReference, IsaacCoreConfig
from isaac_core.sim.capabilities import FakeStageInspector
from isaac_core.sim.georeference import (
    CESIUM_GEOREFERENCE_PRIM,
    HEIGHT_ATTRIBUTE,
    LATITUDE_ATTRIBUTE,
    LONGITUDE_ATTRIBUTE,
    SOURCE_CONFIG,
    SOURCE_DEFAULT,
    SOURCE_SCENE,
    describe_mismatch,
    read_scene_georeference,
    resolve_enu_reference,
)

# The origin authored in usd/scenes/earth.usda.
SCENE_LAT = 32.22481
SCENE_LON = 35.25621
SCENE_ALT = 516.7


def _stage_with_georeference(
    lat: float = SCENE_LAT,
    lon: float = SCENE_LON,
    alt: float = SCENE_ALT,
) -> FakeStageInspector:
    """Return an inspector for a stage carrying a complete Cesium georeference."""
    return FakeStageInspector(
        prims=frozenset({CESIUM_GEOREFERENCE_PRIM}),
        doubles={
            (CESIUM_GEOREFERENCE_PRIM, LATITUDE_ATTRIBUTE): lat,
            (CESIUM_GEOREFERENCE_PRIM, LONGITUDE_ATTRIBUTE): lon,
            (CESIUM_GEOREFERENCE_PRIM, HEIGHT_ATTRIBUTE): alt,
        },
    )


# --------------------------------------------------------------------------- #
# reading the scene
# --------------------------------------------------------------------------- #


def test_reads_a_complete_georeference() -> None:
    found = read_scene_georeference(_stage_with_georeference())
    assert found == EnuReference(lat_deg=SCENE_LAT, lon_deg=SCENE_LON, alt_m=SCENE_ALT)


def test_returns_none_when_the_prim_is_absent() -> None:
    assert read_scene_georeference(FakeStageInspector()) is None


@pytest.mark.parametrize("missing", [LATITUDE_ATTRIBUTE, LONGITUDE_ATTRIBUTE, HEIGHT_ATTRIBUTE])
def test_a_partial_georeference_counts_as_absent(missing: str) -> None:
    # Guessing a missing component would silently misplace the aircraft, which is worse
    # than falling back to a value someone chose.
    doubles = {
        (CESIUM_GEOREFERENCE_PRIM, LATITUDE_ATTRIBUTE): SCENE_LAT,
        (CESIUM_GEOREFERENCE_PRIM, LONGITUDE_ATTRIBUTE): SCENE_LON,
        (CESIUM_GEOREFERENCE_PRIM, HEIGHT_ATTRIBUTE): SCENE_ALT,
    }
    del doubles[(CESIUM_GEOREFERENCE_PRIM, missing)]
    inspector = FakeStageInspector(prims=frozenset({CESIUM_GEOREFERENCE_PRIM}), doubles=doubles)
    assert read_scene_georeference(inspector) is None


def test_reads_from_a_custom_prim_path() -> None:
    inspector = FakeStageInspector(
        prims=frozenset({"/World/Geo"}),
        doubles={
            ("/World/Geo", LATITUDE_ATTRIBUTE): 1.0,
            ("/World/Geo", LONGITUDE_ATTRIBUTE): 2.0,
            ("/World/Geo", HEIGHT_ATTRIBUTE): 3.0,
        },
    )
    found = read_scene_georeference(inspector, prim_path="/World/Geo")
    assert found == EnuReference(lat_deg=1.0, lon_deg=2.0, alt_m=3.0)


# --------------------------------------------------------------------------- #
# precedence
# --------------------------------------------------------------------------- #


def test_scene_wins_when_config_did_not_set_a_reference() -> None:
    # The whole point: nobody has to keep enu_reference in step with the scene.
    resolved = resolve_enu_reference(IsaacCoreConfig(), _stage_with_georeference(10.0, 20.0, 30.0))
    assert resolved.reference == EnuReference(lat_deg=10.0, lon_deg=20.0, alt_m=30.0)
    assert resolved.source == SOURCE_SCENE


def test_explicit_config_beats_the_scene() -> None:
    # An explicit value is an instruction — a stage with no Cesium, or a deliberate
    # offset while debugging.
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0}})
    resolved = resolve_enu_reference(config, _stage_with_georeference())
    assert resolved.reference == EnuReference(lat_deg=1.0, lon_deg=2.0, alt_m=3.0)
    assert resolved.source == SOURCE_CONFIG


def test_falls_back_to_the_schema_default_with_no_scene_georeference() -> None:
    resolved = resolve_enu_reference(IsaacCoreConfig(), FakeStageInspector())
    assert resolved.reference == IsaacCoreConfig().geo.enu_reference
    assert resolved.source == SOURCE_DEFAULT


def test_a_non_cesium_stage_still_resolves() -> None:
    # A warehouse scene with no Cesium at all must still fly.
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 0.0, "lon_deg": 0.0, "alt_m": 0.0}})
    resolved = resolve_enu_reference(config, FakeStageInspector())
    assert resolved.source == SOURCE_CONFIG


# --------------------------------------------------------------------------- #
# mismatch reporting
# --------------------------------------------------------------------------- #


def test_warns_when_an_explicit_reference_disagrees_with_the_scene() -> None:
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0}})
    inspector = _stage_with_georeference()
    warning = describe_mismatch(resolve_enu_reference(config, inspector), inspector)
    assert warning is not None
    assert "explicit value is being used" in warning
    assert str(SCENE_LAT) in warning


def test_no_warning_when_an_explicit_reference_matches_the_scene() -> None:
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": SCENE_LAT, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}})
    inspector = _stage_with_georeference()
    assert describe_mismatch(resolve_enu_reference(config, inspector), inspector) is None


def test_no_warning_when_the_scene_supplied_the_value() -> None:
    inspector = _stage_with_georeference()
    assert describe_mismatch(resolve_enu_reference(IsaacCoreConfig(), inspector), inspector) is None


def test_no_warning_when_the_stage_has_no_georeference() -> None:
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0}})
    inspector = FakeStageInspector()
    assert describe_mismatch(resolve_enu_reference(config, inspector), inspector) is None


def test_altitude_alone_does_not_trigger_a_mismatch_warning() -> None:
    # Altitude differing is routine — a camera reference above ground level is normal —
    # whereas a lat/lon difference means the wrong place on Earth.
    config = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT + 500}}
    )
    inspector = _stage_with_georeference()
    assert describe_mismatch(resolve_enu_reference(config, inspector), inspector) is None
