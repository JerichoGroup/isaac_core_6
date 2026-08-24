"""Tests for geodesic distance and metre-to-degree offset functions."""

import math

from isaac_core.contracts.pose import Lla
from isaac_core.geo.distance import EARTH_RADIUS_M, geodesic_distance_m, meters_to_latlon_offset

# --------------------------------------------------------------------------- #
# geodesic_distance_m
# --------------------------------------------------------------------------- #


def test_geodesic_distance_same_point_is_zero() -> None:
    p = Lla(32.22481, 35.25621, 516.7)
    assert geodesic_distance_m(p, p) == 0.0


def test_geodesic_distance_known_value() -> None:
    # Paris (48.8566, 2.3522) to London (51.5074, -0.1278): ~343.5 km by haversine
    paris = Lla(lat_deg=48.8566, lon_deg=2.3522, alt_m=0.0)
    london = Lla(lat_deg=51.5074, lon_deg=-0.1278, alt_m=0.0)
    dist = geodesic_distance_m(paris, london)
    # Haversine with R=6371000 gives ~343.56 km for these coordinates
    assert 343_000 < dist < 344_000


def test_geodesic_distance_is_symmetric() -> None:
    a = Lla(lat_deg=32.0, lon_deg=35.0, alt_m=0.0)
    b = Lla(lat_deg=33.0, lon_deg=36.0, alt_m=0.0)
    assert math.isclose(geodesic_distance_m(a, b), geodesic_distance_m(b, a))


def test_geodesic_distance_one_degree_latitude() -> None:
    # 1° of latitude ≈ 111.19 km (haversine with R=6371000)
    a = Lla(lat_deg=0.0, lon_deg=0.0, alt_m=0.0)
    b = Lla(lat_deg=1.0, lon_deg=0.0, alt_m=0.0)
    dist = geodesic_distance_m(a, b)
    expected = EARTH_RADIUS_M * math.radians(1.0)
    assert math.isclose(dist, expected, rel_tol=1e-10)


def test_geodesic_distance_antipodal_is_half_circumference() -> None:
    # Distance between (0,0) and (0,180) should be pi*R
    a = Lla(lat_deg=0.0, lon_deg=0.0, alt_m=0.0)
    b = Lla(lat_deg=0.0, lon_deg=180.0, alt_m=0.0)
    dist = geodesic_distance_m(a, b)
    expected = math.pi * EARTH_RADIUS_M
    assert math.isclose(dist, expected, rel_tol=1e-10)


def test_geodesic_distance_ignores_altitude() -> None:
    # Altitude should not affect the great-circle distance
    a = Lla(lat_deg=32.0, lon_deg=35.0, alt_m=0.0)
    b_low = Lla(lat_deg=33.0, lon_deg=35.0, alt_m=0.0)
    b_high = Lla(lat_deg=33.0, lon_deg=35.0, alt_m=10_000.0)
    assert math.isclose(geodesic_distance_m(a, b_low), geodesic_distance_m(a, b_high))


def test_geodesic_distance_small_offset() -> None:
    # ~100m offset northward from equator
    a = Lla(lat_deg=0.0, lon_deg=0.0, alt_m=0.0)
    dlat = 100.0 / EARTH_RADIUS_M  # radians for 100m
    b = Lla(lat_deg=math.degrees(dlat), lon_deg=0.0, alt_m=0.0)
    dist = geodesic_distance_m(a, b)
    assert math.isclose(dist, 100.0, rel_tol=1e-6)


# --------------------------------------------------------------------------- #
# meters_to_latlon_offset
# --------------------------------------------------------------------------- #


def test_meters_to_latlon_offset_zero_returns_zero() -> None:
    d_lat, d_lon = meters_to_latlon_offset(0.0, 0.0, 32.0)
    assert d_lat == 0.0
    assert d_lon == 0.0


def test_meters_to_latlon_offset_northward_increases_latitude() -> None:
    d_lat, d_lon = meters_to_latlon_offset(1000.0, 0.0, 32.0)
    assert d_lat > 0.0
    assert math.isclose(d_lon, 0.0)


def test_meters_to_latlon_offset_eastward_increases_longitude() -> None:
    d_lat, d_lon = meters_to_latlon_offset(0.0, 1000.0, 32.0)
    assert math.isclose(d_lat, 0.0)
    assert d_lon > 0.0


def test_meters_to_latlon_offset_latitude_value() -> None:
    # 111.19 km northward ≈ 1° latitude (at any latitude, for spherical model)
    north_m = EARTH_RADIUS_M * math.radians(1.0)
    d_lat, _ = meters_to_latlon_offset(north_m, 0.0, 0.0)
    assert math.isclose(d_lat, 1.0, rel_tol=1e-10)


def test_meters_to_latlon_offset_longitude_varies_with_latitude() -> None:
    # At the equator, 1° longitude ≈ 111.19 km
    # At 60° latitude, 1° longitude ≈ 55.6 km
    east_m = EARTH_RADIUS_M * math.radians(1.0)  # gives 1° at equator
    _, d_lon_equator = meters_to_latlon_offset(0.0, east_m, 0.0)
    _, d_lon_60 = meters_to_latlon_offset(0.0, east_m, 60.0)
    assert math.isclose(d_lon_equator, 1.0, rel_tol=1e-10)
    assert math.isclose(d_lon_60, 2.0, rel_tol=1e-6)  # cos(60°) = 0.5, so 2x


def test_meters_to_latlon_offset_negative_north() -> None:
    # Southward movement should decrease latitude
    d_lat, _ = meters_to_latlon_offset(-500.0, 0.0, 45.0)
    assert d_lat < 0.0


def test_meters_to_latlon_offset_negative_east() -> None:
    # Westward movement should decrease longitude
    _, d_lon = meters_to_latlon_offset(0.0, -500.0, 45.0)
    assert d_lon < 0.0


def test_meters_to_latlon_offset_round_trip_with_geodesic() -> None:
    # Convert 500m north + 300m east to lat/lon offset, then verify
    # the geodesic distance is consistent.
    ref_lat = 32.22481
    d_lat, d_lon = meters_to_latlon_offset(500.0, 300.0, ref_lat)

    ref = Lla(lat_deg=ref_lat, lon_deg=35.0, alt_m=0.0)
    target = Lla(lat_deg=ref_lat + d_lat, lon_deg=35.0 + d_lon, alt_m=0.0)
    dist = geodesic_distance_m(ref, target)
    # Expected: sqrt(500^2 + 300^2) ≈ 583.1 m
    expected = math.sqrt(500.0**2 + 300.0**2)
    assert math.isclose(dist, expected, rel_tol=0.001)  # 0.1% tolerance


def test_earth_radius_matches_2023() -> None:
    # Guard against accidental change of the constant
    assert EARTH_RADIUS_M == 6_371_000.0
