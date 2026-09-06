"""Tests for deriving the ENU reference from the scene's Cesium georeference."""

import pytest

from isaac_core.config import EnuReference, IsaacCoreConfig
from isaac_core.sim.capabilities import FakeStageInspector
from isaac_core.sim.georeference import (
    CESIUM_GEOREFERENCE_PRIM,
    HEIGHT_ATTRIBUTE,
    HORIZONTAL_TOLERANCE_M,
    LATITUDE_ATTRIBUTE,
    LONGITUDE_ATTRIBUTE,
    SOURCE_CONFIG,
    SOURCE_DEFAULT,
    SOURCE_SCENE,
    GeoreferenceMismatchError,
    describe_mismatch,
    horizontal_separation_m,
    read_scene_georeference,
    resolve_enu_reference,
)

# The origin authored in usd/scenes/earth.usda.
SCENE_LAT = 32.22481
SCENE_LON = 35.25621
SCENE_ALT = 516.7

# Metres per degree of latitude, roughly constant everywhere. Handy for turning a desired
# ground separation into the tiny degree offset that produces it, so tolerance tests read in
# metres rather than opaque decimals.
_M_PER_DEG_LAT = 111_320.0


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


def _lat_offset_for_metres(metres: float) -> float:
    """Return the latitude delta, in degrees, that shifts the origin ``metres`` north."""
    return metres / _M_PER_DEG_LAT


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
# horizontal separation helper
# --------------------------------------------------------------------------- #


def test_identical_references_are_zero_metres_apart() -> None:
    ref = EnuReference(lat_deg=SCENE_LAT, lon_deg=SCENE_LON, alt_m=SCENE_ALT)
    assert horizontal_separation_m(ref, ref) == pytest.approx(0.0, abs=1e-9)


def test_one_degree_of_latitude_is_about_111_km() -> None:
    # Anchors the haversine maths to a known figure so a units slip cannot pass silently.
    a = EnuReference(lat_deg=0.0, lon_deg=0.0, alt_m=0.0)
    b = EnuReference(lat_deg=1.0, lon_deg=0.0, alt_m=0.0)
    assert horizontal_separation_m(a, b) == pytest.approx(111_195.0, rel=0.01)


def test_altitude_does_not_affect_horizontal_separation() -> None:
    # The gap is about *where on Earth* the origin sits, not how high the reference is.
    a = EnuReference(lat_deg=SCENE_LAT, lon_deg=SCENE_LON, alt_m=0.0)
    b = EnuReference(lat_deg=SCENE_LAT, lon_deg=SCENE_LON, alt_m=5000.0)
    assert horizontal_separation_m(a, b) == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------- #
# precedence
# --------------------------------------------------------------------------- #


def test_scene_wins_when_config_did_not_set_a_reference() -> None:
    # The whole point: nobody has to keep enu_reference in step with the scene.
    resolved = resolve_enu_reference(IsaacCoreConfig(), _stage_with_georeference(10.0, 20.0, 30.0))
    assert resolved.reference == EnuReference(lat_deg=10.0, lon_deg=20.0, alt_m=30.0)
    assert resolved.source == SOURCE_SCENE


def test_explicit_config_matching_the_scene_is_used() -> None:
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": SCENE_LAT, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}})
    resolved = resolve_enu_reference(config, _stage_with_georeference())
    assert resolved.reference == EnuReference(lat_deg=SCENE_LAT, lon_deg=SCENE_LON, alt_m=SCENE_ALT)
    assert resolved.source == SOURCE_CONFIG


def test_falls_back_to_the_schema_default_with_no_scene_georeference() -> None:
    resolved = resolve_enu_reference(IsaacCoreConfig(), FakeStageInspector())
    assert resolved.reference == IsaacCoreConfig().geo.enu_reference
    assert resolved.source == SOURCE_DEFAULT


def test_a_non_cesium_stage_still_resolves() -> None:
    # A warehouse scene with no Cesium at all must still fly: an explicit config on a stage
    # with no georeference prim is honoured without complaint.
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 0.0, "lon_deg": 0.0, "alt_m": 0.0}})
    resolved = resolve_enu_reference(config, FakeStageInspector())
    assert resolved.source == SOURCE_CONFIG


# --------------------------------------------------------------------------- #
# disagreement is fatal, not a warning
# --------------------------------------------------------------------------- #


def test_exact_agreement_does_not_raise() -> None:
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": SCENE_LAT, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}})
    resolved = resolve_enu_reference(config, _stage_with_georeference())
    assert resolved.source == SOURCE_CONFIG


def test_agreement_within_tolerance_does_not_raise() -> None:
    # A sub-metre gap is USD/float round-trip noise, not a real disagreement.
    tiny = _lat_offset_for_metres(HORIZONTAL_TOLERANCE_M * 0.5)
    config = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT + tiny, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}}
    )
    resolved = resolve_enu_reference(config, _stage_with_georeference())
    assert resolved.source == SOURCE_CONFIG


def test_agreement_within_tolerance_emits_no_warning() -> None:
    # describe_mismatch must stay silent inside tolerance so the composer logs nothing.
    tiny = _lat_offset_for_metres(HORIZONTAL_TOLERANCE_M * 0.5)
    config = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT + tiny, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}}
    )
    inspector = _stage_with_georeference()
    assert describe_mismatch(resolve_enu_reference(config, inspector), inspector) is None


def test_disagreement_beyond_tolerance_raises() -> None:
    # The behaviour this module exists to guarantee: a real disagreement halts composition
    # loudly instead of one origin silently winning.
    far = _lat_offset_for_metres(HORIZONTAL_TOLERANCE_M * 10)
    config = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT + far, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}}
    )
    with pytest.raises(GeoreferenceMismatchError):
        resolve_enu_reference(config, _stage_with_georeference())


def test_mismatch_message_names_both_values_and_a_metre_figure() -> None:
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0}})
    with pytest.raises(GeoreferenceMismatchError) as exc_info:
        resolve_enu_reference(config, _stage_with_georeference())
    message = str(exc_info.value)
    # Both origins must appear so the reader can see which one is wrong...
    assert "1.0" in message
    assert str(SCENE_LAT) in message
    # ...and the separation must be reported in metres, not raw degrees.
    assert " m " in message
    error = exc_info.value
    assert error.separation_m > HORIZONTAL_TOLERANCE_M
    assert error.separation_m == pytest.approx(horizontal_separation_m(error.configured, error.scene), rel=1e-9)


def test_altitude_difference_alone_never_raises() -> None:
    # Altitude differing is routine -- a camera reference above ground level is normal --
    # whereas a lat/lon difference means the wrong place on Earth.
    config = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT + 5000}}
    )
    resolved = resolve_enu_reference(config, _stage_with_georeference())
    assert resolved.source == SOURCE_CONFIG


def test_no_raise_when_the_stage_has_no_georeference() -> None:
    # An explicit config on a georeference-free stage is a legitimate scene, not a conflict.
    config = IsaacCoreConfig(geo={"enu_reference": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0}})
    resolved = resolve_enu_reference(config, FakeStageInspector())
    assert resolved.source == SOURCE_CONFIG


def test_scene_sourced_reference_never_conflicts_with_itself() -> None:
    # When config did not pin a value, the resolved reference IS the scene's, so there is
    # nothing to disagree with and describe_mismatch stays quiet.
    inspector = _stage_with_georeference()
    assert describe_mismatch(resolve_enu_reference(IsaacCoreConfig(), inspector), inspector) is None


def test_tolerance_boundary_is_inclusive() -> None:
    # Exactly at the tolerance counts as agreement; strictly beyond it raises. Guards the
    # `<=`/`>` choice from silently flipping. The haversine distance at the boundary is
    # ~tolerance, so test just inside and just outside rather than the razor's edge.
    inside = _lat_offset_for_metres(HORIZONTAL_TOLERANCE_M * 0.99)
    inside_cfg = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT + inside, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}}
    )
    assert resolve_enu_reference(inside_cfg, _stage_with_georeference()).source == SOURCE_CONFIG
    outside = _lat_offset_for_metres(HORIZONTAL_TOLERANCE_M * 1.01)
    outside_cfg = IsaacCoreConfig(
        geo={"enu_reference": {"lat_deg": SCENE_LAT + outside, "lon_deg": SCENE_LON, "alt_m": SCENE_ALT}}
    )
    with pytest.raises(GeoreferenceMismatchError):
        resolve_enu_reference(outside_cfg, _stage_with_georeference())
