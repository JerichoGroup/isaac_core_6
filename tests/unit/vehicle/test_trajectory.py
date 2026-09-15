"""Tests for isaac_core.vehicle.trajectory."""

from collections.abc import Iterator
import math

import pytest

from isaac_core.contracts.angles import normalize_angle
from isaac_core.contracts.frames import Frame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.vehicle.trajectory import (
    HoldTrajectory,
    OrbitTrajectory,
    PathTrajectory,
    Trajectory,
)

# --------------------------------------------------------------------------- #
# HoldTrajectory
# --------------------------------------------------------------------------- #


def test_hold_satisfies_trajectory_protocol() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
    )
    hold = HoldTrajectory(pose=pose)
    assert isinstance(hold, Trajectory)


def test_hold_duration_is_infinite() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
    )
    hold = HoldTrajectory(pose=pose)
    assert hold.duration_s == math.inf


def test_hold_yields_same_pose_repeatedly() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.1, pitch_r=0.2, yaw_r=0.3, frame=Frame.NED),
    )
    hold = HoldTrajectory(pose=pose)
    it = hold.poses(rate_hz=10.0)
    assert isinstance(it, Iterator)
    for _ in range(100):
        p = next(it)
        assert p == pose


def test_hold_deterministic() -> None:
    # Same inputs produce identical sequences
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=1.0, frame=Frame.NED),
    )
    it1 = HoldTrajectory(pose=pose).poses(rate_hz=30.0)
    it2 = HoldTrajectory(pose=pose).poses(rate_hz=30.0)
    for _ in range(50):
        assert next(it1) == next(it2)


# --------------------------------------------------------------------------- #
# OrbitTrajectory
# --------------------------------------------------------------------------- #


def test_orbit_satisfies_trajectory_protocol() -> None:
    orb = OrbitTrajectory(
        center_lat_deg=32.0,
        center_lon_deg=35.0,
        radius_m=100.0,
        height_m=500.0,
        speed_mps=10.0,
        orbit_duration_s=62.83,
    )
    assert isinstance(orb, Trajectory)


def test_orbit_exact_sample_count() -> None:
    duration = 10.0
    rate = 30.0
    orb = OrbitTrajectory(
        center_lat_deg=32.0,
        center_lon_deg=35.0,
        radius_m=50.0,
        height_m=100.0,
        speed_mps=5.0,
        orbit_duration_s=duration,
    )
    poses = list(orb.poses(rate_hz=rate))
    expected = int(duration * rate)
    assert len(poses) == expected


def test_orbit_returns_to_start_after_full_revolution() -> None:
    # One complete revolution: circumference / speed = duration
    radius = 100.0
    circumference = 2.0 * math.pi * radius
    speed = 10.0
    duration = circumference / speed  # exactly one loop
    rate = 60.0

    orb = OrbitTrajectory(
        center_lat_deg=32.0,
        center_lon_deg=35.0,
        radius_m=radius,
        height_m=200.0,
        speed_mps=speed,
        orbit_duration_s=duration,
    )
    poses = list(orb.poses(rate_hz=rate))
    first = poses[0]
    last = poses[-1]

    # After a full circle the last sample should be very close to the first
    # Allow some tolerance due to discretisation
    assert math.isclose(first.position.lat_deg, last.position.lat_deg, abs_tol=1e-5)
    assert math.isclose(first.position.lon_deg, last.position.lon_deg, abs_tol=1e-5)


def test_orbit_yaw_is_tangent_to_path() -> None:
    # This test used to assert only `isfinite(yaw_r)`, which would pass with yaw hardcoded to 0.0 --
    # it named a behaviour and then checked nothing about it. Now it measures the actual bearing from
    # each sample to the next and asserts the reported yaw matches, which is what "tangent" means.
    radius = 200.0
    speed = 20.0
    duration = (2.0 * math.pi * radius) / speed
    rate = 30.0

    orbit = OrbitTrajectory(
        center_lat_deg=32.0,
        center_lon_deg=35.0,
        radius_m=radius,
        height_m=300.0,
        speed_mps=speed,
        orbit_duration_s=duration,
    )
    poses = list(orbit.poses(rate_hz=rate))
    assert len(poses) > 40

    # Sample away from the wrap-around at the very end, where the finite difference straddles the seam.
    for index in (len(poses) // 8, len(poses) // 4, len(poses) // 2, (3 * len(poses)) // 4):
        here, nxt = poses[index], poses[index + 1]
        assert here.orientation.frame is Frame.NED

        # Bearing of travel, in the same NED convention the trajectory reports: atan2(east, north).
        north = nxt.position.lat_deg - here.position.lat_deg
        east = (nxt.position.lon_deg - here.position.lon_deg) * math.cos(math.radians(here.position.lat_deg))
        travel_yaw_r = math.atan2(east, north)

        delta = normalize_angle(here.orientation.yaw_r - travel_yaw_r)
        assert abs(delta) < math.radians(2.0), (
            f"sample {index}: reported yaw {math.degrees(here.orientation.yaw_r):.2f} deg is not "
            f"tangent to the path bearing {math.degrees(travel_yaw_r):.2f} deg"
        )


def test_orbit_yaw_would_fail_if_it_were_constant() -> None:
    # A companion to the test above: yaw must actually sweep through the orbit, so a hardcoded or
    # frozen heading cannot pass.
    orbit = OrbitTrajectory(
        center_lat_deg=32.0,
        center_lon_deg=35.0,
        radius_m=200.0,
        height_m=300.0,
        speed_mps=20.0,
        orbit_duration_s=(2.0 * math.pi * 200.0) / 20.0,
    )
    yaws = [pose.orientation.yaw_r for pose in orbit.poses(rate_hz=10.0)]
    spread = max(yaws) - min(yaws)
    assert spread > math.radians(180.0), f"yaw only spanned {math.degrees(spread):.1f} deg over a full orbit"


def test_orbit_deterministic() -> None:
    kwargs = {
        "center_lat_deg": 32.0,
        "center_lon_deg": 35.0,
        "radius_m": 50.0,
        "height_m": 100.0,
        "speed_mps": 5.0,
        "orbit_duration_s": 5.0,
    }
    poses1 = list(OrbitTrajectory(**kwargs).poses(rate_hz=30.0))
    poses2 = list(OrbitTrajectory(**kwargs).poses(rate_hz=30.0))
    assert len(poses1) == len(poses2)
    for p1, p2 in zip(poses1, poses2):
        assert p1 == p2


def test_orbit_rejects_zero_radius() -> None:
    with pytest.raises(ValueError, match="radius_m"):
        OrbitTrajectory(
            center_lat_deg=32.0,
            center_lon_deg=35.0,
            radius_m=0.0,
            height_m=100.0,
            speed_mps=5.0,
            orbit_duration_s=5.0,
        )


def test_orbit_rejects_negative_speed() -> None:
    with pytest.raises(ValueError, match="speed_mps"):
        OrbitTrajectory(
            center_lat_deg=32.0,
            center_lon_deg=35.0,
            radius_m=50.0,
            height_m=100.0,
            speed_mps=-1.0,
            orbit_duration_s=5.0,
        )


# --------------------------------------------------------------------------- #
# PathTrajectory
# --------------------------------------------------------------------------- #


def test_path_satisfies_trajectory_protocol() -> None:
    wps = (Lla(32.0, 35.0, 100.0), Lla(32.001, 35.001, 100.0))
    path = PathTrajectory(waypoints=wps, speed_mps=5.0)
    assert isinstance(path, Trajectory)


def test_path_rejects_fewer_than_two_waypoints() -> None:
    with pytest.raises(ValueError, match="at least 2"):
        PathTrajectory(waypoints=(Lla(32.0, 35.0, 100.0),), speed_mps=5.0)


def test_path_rejects_empty_waypoints() -> None:
    with pytest.raises(ValueError, match="at least 2"):
        PathTrajectory(waypoints=(), speed_mps=5.0)


def test_path_rejects_non_positive_speed() -> None:
    wps = (Lla(32.0, 35.0, 100.0), Lla(32.001, 35.001, 100.0))
    with pytest.raises(ValueError, match="speed_mps"):
        PathTrajectory(waypoints=wps, speed_mps=0.0)


def test_path_exact_sample_count() -> None:
    wps = (Lla(32.0, 35.0, 100.0), Lla(32.001, 35.0, 100.0))
    path = PathTrajectory(waypoints=wps, speed_mps=5.0)
    rate = 30.0
    poses = list(path.poses(rate_hz=rate))
    expected = int(path.duration_s * rate)
    assert len(poses) == expected


def test_path_hits_waypoints() -> None:
    # Two-point path: first sample should be at (or very near) the start
    wp_start = Lla(32.0, 35.0, 100.0)
    wp_end = Lla(32.001, 35.0, 100.0)
    path = PathTrajectory(waypoints=(wp_start, wp_end), speed_mps=5.0)
    poses = list(path.poses(rate_hz=30.0))

    # First pose (step=0 → target_dist=0) should be exactly the start
    first = poses[0]
    assert math.isclose(first.position.lat_deg, wp_start.lat_deg, abs_tol=1e-9)
    assert math.isclose(first.position.lon_deg, wp_start.lon_deg, abs_tol=1e-9)


def test_path_constant_ground_speed() -> None:
    # Between consecutive poses, the distance should be approximately speed/rate
    wp_start = Lla(32.0, 35.0, 100.0)
    wp_end = Lla(32.01, 35.0, 100.0)
    speed = 10.0
    rate = 30.0
    path = PathTrajectory(waypoints=(wp_start, wp_end), speed_mps=speed)
    poses = list(path.poses(rate_hz=rate))

    expected_step_m = speed / rate  # distance per sample

    # Check speed is approximately constant across a span of samples
    from isaac_core.geo.distance import geodesic_distance_m

    deviations = []
    for i in range(1, min(len(poses), 20)):
        d = geodesic_distance_m(poses[i - 1].position, poses[i].position)
        deviations.append(abs(d - expected_step_m))
    # All steps should be within 1% of expected
    assert all(dev < expected_step_m * 0.01 for dev in deviations)


def test_path_deterministic() -> None:
    wps = (Lla(32.0, 35.0, 100.0), Lla(32.001, 35.001, 100.0), Lla(32.002, 35.0, 100.0))
    poses1 = list(PathTrajectory(waypoints=wps, speed_mps=5.0).poses(rate_hz=30.0))
    poses2 = list(PathTrajectory(waypoints=wps, speed_mps=5.0).poses(rate_hz=30.0))
    assert len(poses1) == len(poses2)
    for p1, p2 in zip(poses1, poses2):
        assert p1 == p2


def test_path_three_waypoints_passes_through_middle() -> None:
    # Ensure interpolation traverses the middle waypoint
    wp1 = Lla(32.0, 35.0, 100.0)
    wp2 = Lla(32.0005, 35.0, 100.0)  # halfway in latitude
    wp3 = Lla(32.001, 35.0, 100.0)
    path = PathTrajectory(waypoints=(wp1, wp2, wp3), speed_mps=5.0)
    poses = list(path.poses(rate_hz=30.0))

    # Find the pose closest to wp2's latitude
    mid_lats = [p.position.lat_deg for p in poses]
    closest_to_mid = min(mid_lats, key=lambda lat: abs(lat - wp2.lat_deg))
    assert math.isclose(closest_to_mid, wp2.lat_deg, abs_tol=1e-5)
