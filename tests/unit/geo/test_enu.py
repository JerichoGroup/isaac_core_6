"""Tests for the ENU coordinate converter."""

import math

import pytest

from isaac_core.contracts.pose import Lla
from isaac_core.geo.enu import EnuConverter

# Reference point used throughout (Nablus area, matching the 2023 default)
_REF_LAT = 32.22481
_REF_LON = 35.25621
_REF_ALT = 516.7


@pytest.fixture()
def converter() -> EnuConverter:
    return EnuConverter(_REF_LAT, _REF_LON, _REF_ALT)


def test_reference_point_maps_to_origin(converter: EnuConverter) -> None:
    # The reference point itself must produce exactly (0, 0, 0) in ENU.
    east, north, up = converter.lla_to_enu(Lla(_REF_LAT, _REF_LON, _REF_ALT))
    assert abs(east) < 1e-6
    assert abs(north) < 1e-6
    assert abs(up) < 1e-6


def test_lla_to_enu_round_trip_sub_millimetre(converter: EnuConverter) -> None:
    # An arbitrary nearby point should survive the round trip to sub-mm.
    original = Lla(lat_deg=32.225, lon_deg=35.257, alt_m=520.0)
    east, north, up = converter.lla_to_enu(original)
    recovered = converter.enu_to_lla(east, north, up)

    assert abs(recovered.lat_deg - original.lat_deg) < 1e-8  # ~1 mm
    assert abs(recovered.lon_deg - original.lon_deg) < 1e-8
    assert abs(recovered.alt_m - original.alt_m) < 1e-3


def test_lla_to_enu_round_trip_at_larger_distance(converter: EnuConverter) -> None:
    # 10 km offset -- still sub-centimetre accuracy expected.
    original = Lla(lat_deg=32.31, lon_deg=35.35, alt_m=600.0)
    east, north, up = converter.lla_to_enu(original)
    recovered = converter.enu_to_lla(east, north, up)

    assert abs(recovered.lat_deg - original.lat_deg) < 1e-7
    assert abs(recovered.lon_deg - original.lon_deg) < 1e-7
    assert abs(recovered.alt_m - original.alt_m) < 0.01


def test_eastward_offset_produces_positive_east(converter: EnuConverter) -> None:
    # Moving to a higher longitude (east) at the same latitude should give positive east.
    point = Lla(lat_deg=_REF_LAT, lon_deg=_REF_LON + 0.001, alt_m=_REF_ALT)
    east, north, up = converter.lla_to_enu(point)
    assert east > 0.0
    # North and up should be near zero
    assert abs(north) < 1.0
    assert abs(up) < 1.0


def test_northward_offset_produces_positive_north(converter: EnuConverter) -> None:
    # Moving to a higher latitude (north) at the same longitude should give positive north.
    point = Lla(lat_deg=_REF_LAT + 0.001, lon_deg=_REF_LON, alt_m=_REF_ALT)
    east, north, up = converter.lla_to_enu(point)
    assert north > 0.0
    assert abs(east) < 1.0


def test_upward_offset_produces_positive_up(converter: EnuConverter) -> None:
    # Increasing altitude should give positive up.
    point = Lla(lat_deg=_REF_LAT, lon_deg=_REF_LON, alt_m=_REF_ALT + 100.0)
    east, north, up = converter.lla_to_enu(point)
    assert up > 99.0  # Should be close to 100
    assert abs(east) < 1.0
    assert abs(north) < 1.0


def test_enu_to_lla_origin_returns_reference(converter: EnuConverter) -> None:
    # (0,0,0) in ENU must map back to the reference point.
    recovered = converter.enu_to_lla(0.0, 0.0, 0.0)
    assert abs(recovered.lat_deg - _REF_LAT) < 1e-10
    assert abs(recovered.lon_deg - _REF_LON) < 1e-10
    assert abs(recovered.alt_m - _REF_ALT) < 1e-6


def test_east_offset_magnitude_is_reasonable(converter: EnuConverter) -> None:
    # 1 degree of longitude at ~32° latitude is approximately 94 km.
    point = Lla(lat_deg=_REF_LAT, lon_deg=_REF_LON + 1.0, alt_m=_REF_ALT)
    east, _north, _up = converter.lla_to_enu(point)
    assert 90_000 < east < 100_000


def test_north_offset_magnitude_is_reasonable(converter: EnuConverter) -> None:
    # 1 degree of latitude is approximately 111 km.
    point = Lla(lat_deg=_REF_LAT + 1.0, lon_deg=_REF_LON, alt_m=_REF_ALT)
    _east, north, _up = converter.lla_to_enu(point)
    assert 110_000 < north < 112_000


def test_converter_is_independent_per_instance() -> None:
    # Two converters with different references must give different results.
    c1 = EnuConverter(32.0, 35.0, 0.0)
    c2 = EnuConverter(33.0, 36.0, 0.0)
    point = Lla(lat_deg=32.5, lon_deg=35.5, alt_m=100.0)
    e1, n1, u1 = c1.lla_to_enu(point)
    e2, n2, u2 = c2.lla_to_enu(point)
    # Must differ significantly
    assert abs(e1 - e2) > 1000.0 or abs(n1 - n2) > 1000.0


def test_negative_altitude_round_trips(converter: EnuConverter) -> None:
    # Below-ellipsoid points (Dead Sea) should round trip correctly.
    original = Lla(lat_deg=31.5, lon_deg=35.5, alt_m=-430.0)
    c = EnuConverter(31.5, 35.5, -430.0)
    east, north, up = c.lla_to_enu(original)
    assert abs(east) < 1e-6
    assert abs(north) < 1e-6
    assert abs(up) < 1e-6


def test_enu_symmetry_east_west(converter: EnuConverter) -> None:
    # Equidistant east and west offsets should give opposite east values.
    east_pt = Lla(lat_deg=_REF_LAT, lon_deg=_REF_LON + 0.01, alt_m=_REF_ALT)
    west_pt = Lla(lat_deg=_REF_LAT, lon_deg=_REF_LON - 0.01, alt_m=_REF_ALT)
    e_east, _, _ = converter.lla_to_enu(east_pt)
    e_west, _, _ = converter.lla_to_enu(west_pt)
    # Should be approximately equal in magnitude but opposite in sign.
    assert math.isclose(e_east, -e_west, rel_tol=1e-4)
