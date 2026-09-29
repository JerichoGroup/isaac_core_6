"""A flown path must face where it is actually going, in three dimensions.

Two defects found by watching a path being flown rather than by reading its endpoints:

*Pitch never moved.* `fly_path(face_travel=True)` promises to "point the airframe along the direction of
travel", and yaw did follow the ground track, but pitch was a fixed value. An aircraft descending 200 m
still pointed at the horizon, which reads as sliding rather than flying.

*The bearing was wrong.* Yaw came from `atan2(d_lon, d_lat)` on raw degrees. A degree of longitude is
shorter than a degree of latitude by `cos(latitude)`, so the east-west component was over-weighted -- about
18% at latitude 32, several degrees of bearing. Endpoint assertions cannot see this because the path still
arrives in the right place; only the heading along the way is wrong.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.pose import Lla
from isaac_core.vehicle import PathTrajectory

# A latitude where cos(lat) differs from 1 enough to matter, and the scene's own.
LAT = 32.22481
LON = 35.25621
RATE_HZ = 10.0


def _poses(waypoints: list[Lla], *, speed_mps: float = 60.0, **kwargs: float) -> list:
    """Return the poses a path yields."""
    return list(PathTrajectory(waypoints=tuple(waypoints), speed_mps=speed_mps, **kwargs).poses(rate_hz=RATE_HZ))


def _pitches(poses: list) -> list[float]:
    """Return pitch in degrees for each pose."""
    return [math.degrees(p.orientation.pitch_r) for p in poses]


# -- pitch follows the gradient -------------------------------------------------- #


def test_a_level_leg_holds_level_pitch() -> None:
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1000.0)])
    assert max(abs(v) for v in _pitches(poses)) < 1e-6


def test_a_climbing_leg_pitches_up() -> None:
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1300.0)])
    assert min(_pitches(poses)) > 1.0, "a climbing leg must pitch nose-up"


def test_a_descending_leg_pitches_down() -> None:
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1300.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1000.0)])
    assert max(_pitches(poses)) < -1.0, "a descending leg must pitch nose-down"


def test_the_pitch_matches_the_gradient() -> None:
    # 300 m of climb over a leg of known ground length: the angle is arctan of the gradient, not a guess.
    north_m = math.radians(0.004) * 6371000.0
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1300.0)])
    expected = math.degrees(math.atan2(300.0, north_m))
    assert _pitches(poses)[0] == pytest.approx(expected, abs=0.2)


def test_a_steeper_leg_pitches_more() -> None:
    shallow = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.008, lon_deg=LON, alt_m=1100.0)])
    steep = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.002, lon_deg=LON, alt_m=1100.0)])
    assert _pitches(steep)[0] > _pitches(shallow)[0]


def test_each_leg_gets_its_own_pitch() -> None:
    # The case that showed the bug: one path that descends then climbs.
    poses = _poses(
        [
            Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1300.0),
            Lla(lat_deg=LAT + 0.004, lon_deg=LON + 0.005, alt_m=1100.0),
            Lla(lat_deg=LAT, lon_deg=LON, alt_m=1200.0),
        ]
    )
    pitches = _pitches(poses)
    assert min(pitches) < -1.0, "the descending leg never pitched down"
    assert max(pitches) > 1.0, "the climbing leg never pitched up"


def test_pitch_r_is_an_offset_on_top_of_the_gradient() -> None:
    # Documented as an offset, so a deliberate nose-down bias must still work.
    plain = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1300.0)])
    biased = _poses(
        [Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1300.0)],
        pitch_r=math.radians(-10.0),
    )
    assert _pitches(biased)[0] == pytest.approx(_pitches(plain)[0] - 10.0, abs=1e-6)


# -- the bearing accounts for latitude ------------------------------------------- #


def test_due_north_is_zero_bearing() -> None:
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT + 0.004, lon_deg=LON, alt_m=1000.0)])
    assert math.degrees(poses[0].orientation.yaw_r) == pytest.approx(0.0, abs=1e-6)


def test_due_east_is_ninety_degrees() -> None:
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), Lla(lat_deg=LAT, lon_deg=LON + 0.004, alt_m=1000.0)])
    assert math.degrees(poses[0].orientation.yaw_r) == pytest.approx(90.0, abs=1e-6)


def test_equal_degree_deltas_are_not_forty_five_degrees_away_from_the_equator() -> None:
    # The whole point. Naively comparing raw degree deltas gives 45; the true bearing at latitude 32 is
    # about 40.3, because a degree of longitude is shorter there.
    delta = 0.01
    poses = _poses(
        [Lla(lat_deg=32.0, lon_deg=35.0, alt_m=1000.0), Lla(lat_deg=32.0 + delta, lon_deg=35.0 + delta, alt_m=1000.0)]
    )
    expected = math.degrees(math.atan2(math.radians(delta) * math.cos(math.radians(32.0)), math.radians(delta)))
    observed = math.degrees(poses[0].orientation.yaw_r)
    assert observed == pytest.approx(expected, abs=0.05)
    assert abs(observed - 45.0) > 3.0, "the bearing still ignores the cosine of latitude"


def test_the_bearing_error_grows_with_latitude() -> None:
    # A property check rather than a magic number: the further from the equator, the more the naive
    # calculation would have been wrong.
    delta = 0.01
    deviations = []
    for lat in (5.0, 35.0, 65.0):
        poses = _poses(
            [Lla(lat_deg=lat, lon_deg=0.0, alt_m=1000.0), Lla(lat_deg=lat + delta, lon_deg=delta, alt_m=1000.0)]
        )
        deviations.append(abs(math.degrees(poses[0].orientation.yaw_r) - 45.0))
    assert deviations == sorted(deviations), deviations


# -- the path still arrives ------------------------------------------------------ #


def test_the_path_still_ends_on_its_last_waypoint() -> None:
    # The facing fix must not disturb the arrival guarantee.
    end = Lla(lat_deg=LAT + 0.004, lon_deg=LON + 0.004, alt_m=1250.0)
    poses = _poses([Lla(lat_deg=LAT, lon_deg=LON, alt_m=1000.0), end])
    final = poses[-1].position
    assert final.lat_deg == pytest.approx(end.lat_deg, abs=1e-9)
    assert final.lon_deg == pytest.approx(end.lon_deg, abs=1e-9)
    assert final.alt_m == pytest.approx(end.alt_m, abs=1e-6)
