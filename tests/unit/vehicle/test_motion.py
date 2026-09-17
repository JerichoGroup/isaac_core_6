"""Tests for isaac_core.vehicle.motion."""

import ast
import inspect
import math
import pathlib

import numpy as np

from isaac_core.contracts.frames import RotationFrame
from isaac_core.geo.rotations import euler_to_matrix
from isaac_core.vehicle.limits import MotionLimits
import isaac_core.vehicle.motion as motion_module
from isaac_core.vehicle.motion import (
    move_forward_backward,
    move_right_left,
    move_to,
    move_up_down,
    steer,
    turn_pitch,
    turn_roll,
    turn_to_point,
    turn_yaw,
)
from isaac_core.vehicle.state import VehicleState


def _make_state(
    lat: float = 32.0,
    lon: float = 35.0,
    alt: float = 100.0,
    roll_r: float = 0.0,
    pitch_r: float = 0.0,
    yaw_r: float = 0.0,
) -> VehicleState:
    # from_angles, not VehicleState(rotation=euler_to_matrix(...)): the latter builds the matrix in
    # the library default frame while the state declares its own, which reads back the wrong angles
    # for any attitude with two non-zero components.
    return VehicleState.from_angles(lat, lon, alt, roll_r, pitch_r, yaw_r)


# --------------------------------------------------------------------------- #
# NO I/O: the central design property
# --------------------------------------------------------------------------- #


def test_no_io_imports_in_vehicle_package() -> None:
    # Scan all .py files in the vehicle package for forbidden imports
    vehicle_dir = pathlib.Path(__file__).resolve().parents[3] / "src" / "isaac_core" / "vehicle"
    forbidden = {"socket", "threading", "time"}
    for py_file in vehicle_dir.glob("*.py"):
        source = py_file.read_text()
        for mod in forbidden:
            # Check for 'import socket', 'import time', 'from time import', etc.
            for pattern in [f"import {mod}", f"from {mod}"]:
                assert pattern not in source, (
                    f"Forbidden import '{pattern}' found in {py_file.name}. "
                    f"The vehicle package must be pure and I/O-free."
                )


# --------------------------------------------------------------------------- #
# move_to
# --------------------------------------------------------------------------- #


def test_move_to_exact_sample_count() -> None:
    state = _make_state()
    poses = list(move_to(state, 32.001, 35.001, 200.0, duration_s=2.0, rate_hz=30.0))
    assert len(poses) == 60


def test_move_to_ends_at_target() -> None:
    state = _make_state()
    poses = list(move_to(state, 32.001, 35.001, 200.0, duration_s=2.0, rate_hz=30.0))
    last = poses[-1]
    assert math.isclose(last.position.lat_deg, 32.001, abs_tol=1e-9)
    assert math.isclose(last.position.lon_deg, 35.001, abs_tol=1e-9)
    assert math.isclose(last.position.alt_m, 200.0, abs_tol=1e-9)


def test_move_to_preserves_orientation() -> None:
    state = _make_state(roll_r=0.1, pitch_r=-0.2, yaw_r=0.5)
    poses = list(move_to(state, 32.001, 35.001, 200.0, duration_s=1.0, rate_hz=10.0))
    for p in poses:
        assert math.isclose(p.orientation.roll_r, 0.1, abs_tol=1e-10)
        assert math.isclose(p.orientation.pitch_r, -0.2, abs_tol=1e-10)
        assert math.isclose(p.orientation.yaw_r, 0.5, abs_tol=1e-10)


# --------------------------------------------------------------------------- #
# move_forward_backward
# --------------------------------------------------------------------------- #


def test_move_forward_exact_count() -> None:
    state = _make_state()
    poses = list(move_forward_backward(state, 50.0, duration_s=1.0, rate_hz=30.0))
    assert len(poses) == 30


def test_move_forward_heading_zero_goes_north() -> None:
    # heading=0 means north → latitude increases
    state = _make_state(yaw_r=0.0)
    poses = list(move_forward_backward(state, 100.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert last.position.lat_deg > state.lat_deg
    # Longitude should stay approximately the same
    assert math.isclose(last.position.lon_deg, state.lon_deg, abs_tol=1e-7)


def test_move_forward_heading_pi_goes_south() -> None:
    # heading ~ pi means south → latitude decreases
    state = _make_state(yaw_r=math.pi * 0.99)  # near pi to avoid exact pi singularity
    poses = list(move_forward_backward(state, 100.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert last.position.lat_deg < state.lat_deg


def test_move_forward_heading_pi_half_goes_east() -> None:
    # heading=pi/2 means east → longitude increases
    state = _make_state(yaw_r=math.pi / 2)
    poses = list(move_forward_backward(state, 100.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert last.position.lon_deg > state.lon_deg
    assert math.isclose(last.position.lat_deg, state.lat_deg, abs_tol=1e-6)


def test_move_backward_reverses_direction() -> None:
    state = _make_state(yaw_r=0.0)
    poses = list(move_forward_backward(state, -100.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    # heading=0 forward=north, backward=south
    assert last.position.lat_deg < state.lat_deg


def test_move_forward_multiple_headings_consistent() -> None:
    # At several headings, forward motion should move in the heading direction.
    # Tolerance is wider for diagonal headings because meters_to_latlon_offset
    # uses a spherical approximation that distorts at non-equatorial latitudes.
    for heading in [0.0, math.pi / 4, math.pi / 2, math.pi, -math.pi / 2, -math.pi / 4]:
        state = _make_state(yaw_r=heading)
        poses = list(move_forward_backward(state, 200.0, duration_s=1.0, rate_hz=10.0))
        last = poses[-1]
        # Compute the direction of displacement
        dlat = last.position.lat_deg - state.lat_deg
        dlon = last.position.lon_deg - state.lon_deg
        actual_heading = math.atan2(dlon, dlat)
        # Should be close to the commanded heading (allow wrapping)
        diff = abs(((actual_heading - heading) + math.pi) % (2 * math.pi) - math.pi)
        assert diff < 0.15, f"heading={heading}: expected ~{heading}, got {actual_heading}"


# --------------------------------------------------------------------------- #
# move_right_left
# --------------------------------------------------------------------------- #


def test_move_right_heading_zero_goes_east() -> None:
    # Right of north is east → longitude increases
    state = _make_state(yaw_r=0.0)
    poses = list(move_right_left(state, 100.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert last.position.lon_deg > state.lon_deg
    assert math.isclose(last.position.lat_deg, state.lat_deg, abs_tol=1e-6)


def test_move_left_heading_zero_goes_west() -> None:
    state = _make_state(yaw_r=0.0)
    poses = list(move_right_left(state, -100.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert last.position.lon_deg < state.lon_deg


# --------------------------------------------------------------------------- #
# move_up_down
# --------------------------------------------------------------------------- #


def test_move_up_increases_altitude() -> None:
    state = _make_state(alt=100.0)
    poses = list(move_up_down(state, 50.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert math.isclose(last.position.alt_m, 150.0, abs_tol=1e-9)


def test_move_down_decreases_altitude() -> None:
    state = _make_state(alt=100.0)
    poses = list(move_up_down(state, -30.0, duration_s=1.0, rate_hz=10.0))
    last = poses[-1]
    assert math.isclose(last.position.alt_m, 70.0, abs_tol=1e-9)


def test_move_up_position_unchanged() -> None:
    state = _make_state()
    poses = list(move_up_down(state, 50.0, duration_s=1.0, rate_hz=10.0))
    for p in poses:
        assert math.isclose(p.position.lat_deg, state.lat_deg)
        assert math.isclose(p.position.lon_deg, state.lon_deg)


# --------------------------------------------------------------------------- #
# turn_yaw — D14 payoff: WORLD vs BODY with roll
# --------------------------------------------------------------------------- #


def test_turn_yaw_world_frame_basic() -> None:
    state = _make_state(yaw_r=0.0)
    poses = list(turn_yaw(state, math.pi / 2, duration_s=1.0, rate_hz=30.0, frame=RotationFrame.WORLD))
    last = poses[-1]
    assert math.isclose(last.orientation.yaw_r, math.pi / 2, abs_tol=1e-6)


def test_turn_yaw_world_vs_body_differ_when_rolled() -> None:
    # D14: commanding +yaw on a rolled vehicle gives DIFFERENT results
    # under WORLD vs BODY frame. This proves the distinction matters.
    roll = math.pi / 4  # 45 degree roll
    state = _make_state(roll_r=roll, yaw_r=0.0)

    delta_yaw = math.pi / 4
    duration = 1.0
    rate = 30.0

    world_poses = list(turn_yaw(state, delta_yaw, duration, rate, frame=RotationFrame.WORLD))
    body_poses = list(turn_yaw(state, delta_yaw, duration, rate, frame=RotationFrame.BODY))

    world_last = world_poses[-1]
    body_last = body_poses[-1]

    # They must be different (the whole point of D14)
    world_rpy = (world_last.orientation.roll_r, world_last.orientation.pitch_r, world_last.orientation.yaw_r)
    body_rpy = (body_last.orientation.roll_r, body_last.orientation.pitch_r, body_last.orientation.yaw_r)

    # At least one angle must differ significantly
    diffs = [abs(a - b) for a, b in zip(world_rpy, body_rpy)]
    assert max(diffs) > 0.01, (
        f"WORLD and BODY frame yaw gave the same result on a rolled vehicle: " f"world={world_rpy}, body={body_rpy}"
    )


def test_turn_yaw_world_preserves_world_vertical() -> None:
    # Under WORLD frame, a yaw should be about the world vertical.
    # For a rolled vehicle, world-frame yaw changes heading without coupling
    # into the roll/pitch axes beyond what the decomposition introduces.
    state = _make_state(roll_r=0.0, pitch_r=0.0, yaw_r=0.0)
    poses = list(turn_yaw(state, math.pi / 2, duration_s=1.0, rate_hz=30.0, frame=RotationFrame.WORLD))
    last = poses[-1]
    # With no initial roll/pitch, WORLD yaw should purely change yaw
    assert math.isclose(last.orientation.roll_r, 0.0, abs_tol=1e-6)
    assert math.isclose(last.orientation.pitch_r, 0.0, abs_tol=1e-6)
    assert math.isclose(last.orientation.yaw_r, math.pi / 2, abs_tol=1e-6)


def test_turn_yaw_body_on_level_vehicle_equals_world() -> None:
    # With no roll/pitch, WORLD and BODY should give the same result
    state = _make_state(roll_r=0.0, pitch_r=0.0, yaw_r=0.0)
    delta = math.pi / 3

    world_poses = list(turn_yaw(state, delta, 1.0, 30.0, frame=RotationFrame.WORLD))
    body_poses = list(turn_yaw(state, delta, 1.0, 30.0, frame=RotationFrame.BODY))

    wl = world_poses[-1]
    bl = body_poses[-1]
    assert math.isclose(wl.orientation.yaw_r, bl.orientation.yaw_r, abs_tol=1e-6)
    assert math.isclose(wl.orientation.roll_r, bl.orientation.roll_r, abs_tol=1e-6)
    assert math.isclose(wl.orientation.pitch_r, bl.orientation.pitch_r, abs_tol=1e-6)


# --------------------------------------------------------------------------- #
# turn_roll and turn_pitch
# --------------------------------------------------------------------------- #


def test_turn_roll_changes_roll() -> None:
    state = _make_state()
    poses = list(turn_roll(state, math.pi / 6, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    assert math.isclose(last.orientation.roll_r, math.pi / 6, abs_tol=1e-6)


def test_turn_pitch_changes_pitch() -> None:
    state = _make_state()
    poses = list(turn_pitch(state, -math.pi / 6, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    assert math.isclose(last.orientation.pitch_r, -math.pi / 6, abs_tol=1e-6)


# --------------------------------------------------------------------------- #
# turn_to_point
# --------------------------------------------------------------------------- #


def test_turn_to_point_faces_target() -> None:
    # Target is due east: expected yaw ~ pi/2
    state = _make_state(lat=32.0, lon=35.0, alt=100.0)
    target_lon = 35.01  # east
    poses = list(turn_to_point(state, 32.0, target_lon, 100.0, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    # Yaw should be approximately pi/2 (east)
    assert math.isclose(last.orientation.yaw_r, math.pi / 2, abs_tol=0.01)


def test_turn_to_point_faces_north() -> None:
    state = _make_state(lat=32.0, lon=35.0, alt=100.0, yaw_r=math.pi / 2)
    target_lat = 32.01  # north
    poses = list(turn_to_point(state, target_lat, 35.0, 100.0, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    assert math.isclose(last.orientation.yaw_r, 0.0, abs_tol=0.01)


def test_turn_to_point_pitches_up_for_higher_target() -> None:
    state = _make_state(lat=32.0, lon=35.0, alt=100.0)
    poses = list(turn_to_point(state, 32.001, 35.0, 500.0, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    # Should pitch up (positive pitch for nose up)
    assert last.orientation.pitch_r > 0.0


def test_turn_to_point_heading_actually_points_at_target() -> None:
    # The heading after turn_to_point should make move_forward_backward
    # move toward the target. This is the real test of correctness.
    state = _make_state(lat=32.0, lon=35.0, alt=100.0, yaw_r=0.0)
    target_lat, target_lon = 32.001, 35.001

    # Turn to face the target
    poses = list(turn_to_point(state, target_lat, target_lon, 100.0, duration_s=1.0, rate_hz=30.0))
    final_yaw = poses[-1].orientation.yaw_r

    # Now move forward with that heading
    new_state = state.with_heading_r(final_yaw)
    move_poses = list(move_forward_backward(new_state, 50.0, duration_s=1.0, rate_hz=10.0))
    moved = move_poses[-1]

    # The moved position should be closer to the target than the original
    orig_dist = math.sqrt((target_lat - state.lat_deg) ** 2 + (target_lon - state.lon_deg) ** 2)
    new_dist = math.sqrt((target_lat - moved.position.lat_deg) ** 2 + (target_lon - moved.position.lon_deg) ** 2)
    assert new_dist < orig_dist


# --------------------------------------------------------------------------- #
# steer
# --------------------------------------------------------------------------- #


def test_steer_straight_zero_radius() -> None:
    # Zero radius = straight line along heading
    state = _make_state(yaw_r=0.0)
    poses = list(steer(state, turn_radius_m=0.0, speed_mps=10.0, duration_s=1.0, rate_hz=30.0))
    assert len(poses) == 30
    last = poses[-1]
    # Should have moved north
    assert last.position.lat_deg > state.lat_deg
    # Yaw unchanged
    assert math.isclose(last.orientation.yaw_r, 0.0, abs_tol=1e-6)


def test_steer_right_turns_yaw_positive() -> None:
    # Positive radius = right turn, yaw should increase
    state = _make_state(yaw_r=0.0)
    poses = list(steer(state, turn_radius_m=50.0, speed_mps=10.0, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    # speed/radius = 10/50 = 0.2 rad/s, 1 second = 0.2 rad ≈ 11.5 deg
    assert last.orientation.yaw_r > 0.0


def test_steer_left_turns_yaw_negative() -> None:
    state = _make_state(yaw_r=0.0)
    poses = list(steer(state, turn_radius_m=-50.0, speed_mps=10.0, duration_s=1.0, rate_hz=30.0))
    last = poses[-1]
    assert last.orientation.yaw_r < 0.0


def test_steer_exact_sample_count() -> None:
    state = _make_state()
    poses = list(steer(state, turn_radius_m=100.0, speed_mps=5.0, duration_s=2.0, rate_hz=30.0))
    assert len(poses) == 60


# --------------------------------------------------------------------------- #
# Limits integration
# --------------------------------------------------------------------------- #


def test_limits_clamp_forward_speed() -> None:
    state = _make_state(yaw_r=0.0)
    limits = MotionLimits(max_speed_mps=5.0)

    # Without limits: 100m in 1s = 100 m/s
    unlimited_poses = list(move_forward_backward(state, 100.0, 1.0, 10.0, limits=None))
    # With limits: clamped to 5 m/s → only moves 5m
    limited_poses = list(move_forward_backward(state, 100.0, 1.0, 10.0, limits=limits))

    unlimited_last = unlimited_poses[-1]
    limited_last = limited_poses[-1]
    # Limited should move less
    unlim_dist = abs(unlimited_last.position.lat_deg - state.lat_deg)
    lim_dist = abs(limited_last.position.lat_deg - state.lat_deg)
    assert lim_dist < unlim_dist


def test_none_limits_reproduce_unlimited_motion() -> None:
    # MotionLimits.unlimited() should give identical results to limits=None
    state = _make_state(yaw_r=math.pi / 4)
    poses_none = list(move_forward_backward(state, 50.0, 1.0, 10.0, limits=None))
    poses_unlim = list(move_forward_backward(state, 50.0, 1.0, 10.0, limits=MotionLimits.unlimited()))

    assert len(poses_none) == len(poses_unlim)
    for p1, p2 in zip(poses_none, poses_unlim):
        assert math.isclose(p1.position.lat_deg, p2.position.lat_deg, abs_tol=1e-12)
        assert math.isclose(p1.position.lon_deg, p2.position.lon_deg, abs_tol=1e-12)


def test_limits_clamp_climb_rate() -> None:
    state = _make_state(alt=100.0)
    limits = MotionLimits(max_climb_rate_mps=2.0)
    # 50m in 1s = 50 m/s climb rate, clamped to 2 → only moves 2m
    poses = list(move_up_down(state, 50.0, 1.0, 10.0, limits=limits))
    last = poses[-1]
    assert math.isclose(last.position.alt_m, 102.0, abs_tol=1e-9)


def test_limits_clamp_turn_rate() -> None:
    state = _make_state()
    # max turn rate = 10 deg/s, requesting pi rad (180 deg) in 1s = 180 deg/s
    limits = MotionLimits(max_turn_rate_deg_s=10.0)
    poses = list(turn_yaw(state, math.pi, 1.0, 30.0, limits=limits))
    last = poses[-1]
    # Should only achieve 10 degrees = ~0.1745 radians
    assert abs(last.orientation.yaw_r) < 0.2


# --------------------------------------------------------------------------- #
# Determinism
# --------------------------------------------------------------------------- #


def test_motion_deterministic() -> None:
    state = _make_state(roll_r=0.1, yaw_r=0.5)
    poses1 = list(steer(state, 50.0, 10.0, 2.0, 30.0))
    poses2 = list(steer(state, 50.0, 10.0, 2.0, 30.0))
    assert len(poses1) == len(poses2)
    for p1, p2 in zip(poses1, poses2):
        assert p1 == p2


# --------------------------------------------------------------------------- #
# State immutability under motion
# --------------------------------------------------------------------------- #


def test_move_to_does_not_mutate_original_state() -> None:
    mat = euler_to_matrix(0.0, 0.0, 0.0)
    state = VehicleState(lat_deg=32.0, lon_deg=35.0, alt_m=100.0, rotation=mat)
    original_lat = state.lat_deg
    original_rotation = state.rotation.copy()

    # Consume the full generator
    list(move_to(state, 33.0, 36.0, 200.0, 1.0, 10.0))

    # State must be unchanged
    assert state.lat_deg == original_lat
    assert np.allclose(state.rotation, original_rotation)


def test_move_to_honours_the_speed_limit_it_accepts() -> None:
    # Guards a real bug: move_to accepted `limits` and never referenced it, so a caller passing a
    # 1 m/s cap still got the full-distance move in the requested duration. Every sibling honours
    # limits, so ignoring it here meant the signature lied.
    state = VehicleState(lat_deg=32.0, lon_deg=35.0, alt_m=1000.0, rotation=np.eye(3))
    unlimited = list(move_to(state, 33.0, 35.0, 1000.0, duration_s=10.0, rate_hz=1.0))
    limited = list(
        move_to(
            state,
            33.0,
            35.0,
            1000.0,
            duration_s=10.0,
            rate_hz=1.0,
            limits=MotionLimits(max_speed_mps=1.0),
        )
    )
    assert len(limited) > len(unlimited), "the speed limit did not extend the move"
    assert limited[-1].position.lat_deg == unlimited[-1].position.lat_deg


def test_turn_to_point_honours_the_turn_rate_limit_it_accepts() -> None:
    # The same bug in the rotation path: turn_to_point took `limits` and ignored it, while
    # turn_yaw, turn_roll and turn_pitch all clamp.
    state = VehicleState(lat_deg=32.0, lon_deg=35.0, alt_m=1000.0, rotation=np.eye(3))
    unlimited = list(turn_to_point(state, 33.0, 36.0, 1000.0, duration_s=2.0, rate_hz=1.0))
    limited = list(
        turn_to_point(
            state,
            33.0,
            36.0,
            1000.0,
            duration_s=2.0,
            rate_hz=1.0,
            limits=MotionLimits(max_turn_rate_deg_s=1.0),
        )
    )
    assert len(limited) > len(unlimited), "the turn rate limit did not extend the turn"


def test_no_motion_function_accepts_limits_and_ignores_it() -> None:
    # The general invariant, so a new function cannot reintroduce the same lie.
    tree = ast.parse(inspect.getsource(motion_module))
    offenders = []
    for node in tree.body:
        if not isinstance(node, ast.FunctionDef):
            continue
        args = [arg.arg for arg in node.args.args] + [arg.arg for arg in node.args.kwonlyargs]
        if "limits" not in args:
            continue
        if "'limits'" not in ast.dump(ast.Module(body=node.body, type_ignores=[])):
            offenders.append(node.name)
    assert not offenders, f"these accept `limits` and never use it: {offenders}"
