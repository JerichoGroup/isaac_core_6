"""Tests for isaac_core.vehicle.state."""

import math

import numpy as np
import pytest

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.geo.rotations import euler_to_matrix
from isaac_core.vehicle.state import VehicleState

# --------------------------------------------------------------------------- #
# Construction and from_pose / to_pose round-trip
# --------------------------------------------------------------------------- #


def test_from_pose_basic() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
    )
    state = VehicleState.from_pose(pose)
    assert state.lat_deg == 32.0
    assert state.lon_deg == 35.0
    assert state.alt_m == 100.0


def test_from_pose_rejects_enu() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.ENU),
    )
    with pytest.raises(ValueError, match="NED"):
        VehicleState.from_pose(pose)


def test_to_pose_round_trip() -> None:
    # Construct with known Euler angles, convert out and back
    roll, pitch, yaw = 0.1, -0.2, 1.5
    pose_in = GeodeticPose(
        position=Lla(lat_deg=31.5, lon_deg=34.8, alt_m=200.0),
        orientation=Rpy(roll_r=roll, pitch_r=pitch, yaw_r=yaw, frame=Frame.NED),
    )
    state = VehicleState.from_pose(pose_in)
    pose_out = state.to_pose()

    assert math.isclose(pose_out.position.lat_deg, 31.5)
    assert math.isclose(pose_out.position.lon_deg, 34.8)
    assert math.isclose(pose_out.position.alt_m, 200.0)
    assert math.isclose(pose_out.orientation.roll_r, roll, abs_tol=1e-12)
    assert math.isclose(pose_out.orientation.pitch_r, pitch, abs_tol=1e-12)
    assert math.isclose(pose_out.orientation.yaw_r, yaw, abs_tol=1e-12)
    assert pose_out.frame is Frame.NED


# --------------------------------------------------------------------------- #
# Derived vectors and heading
# --------------------------------------------------------------------------- #


def test_heading_zero_when_yaw_is_zero() -> None:
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.0, 0.0, 0.0))
    assert math.isclose(state.heading_r, 0.0, abs_tol=1e-12)


def test_heading_pi_half() -> None:
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.0, 0.0, math.pi / 2))
    assert math.isclose(state.heading_r, math.pi / 2, abs_tol=1e-12)


def test_forward_vector_at_zero_heading() -> None:
    # At heading=0 (north), forward should be roughly [0, 1, 0] (body Y in world)
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.0, 0.0, 0.0))
    fwd = state.forward_vector
    assert math.isclose(fwd[0], 0.0, abs_tol=1e-12)
    assert math.isclose(fwd[1], 1.0, abs_tol=1e-12)
    assert math.isclose(fwd[2], 0.0, abs_tol=1e-12)


def test_forward_vector_at_90_deg_heading() -> None:
    # At heading=pi/2 (east), forward should be [sin(pi/2), cos(pi/2), 0] = [1, 0, 0]
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.0, 0.0, math.pi / 2))
    fwd = state.forward_vector
    assert math.isclose(fwd[0], 1.0, abs_tol=1e-12)
    assert math.isclose(fwd[1], 0.0, abs_tol=1e-12)
    assert math.isclose(fwd[2], 0.0, abs_tol=1e-12)


def test_right_vector_perpendicular_to_forward() -> None:
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.1, 0.2, 0.3))
    dot = float(np.dot(state.forward_vector, state.right_vector))
    assert math.isclose(dot, 0.0, abs_tol=1e-12)


def test_right_vector_at_zero_heading_points_east() -> None:
    # At heading=0 (north), right should be east = [1, 0, 0]
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.0, 0.0, 0.0))
    right = state.right_vector
    assert math.isclose(right[0], 1.0, abs_tol=1e-12)
    assert math.isclose(right[1], 0.0, abs_tol=1e-12)
    assert math.isclose(right[2], 0.0, abs_tol=1e-12)


def test_up_vector_vertical_for_level_vehicle() -> None:
    # A level vehicle should have up = [0, 0, 1]
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.0, 0.0, 0.3))
    assert math.isclose(float(state.up_vector[0]), 0.0, abs_tol=1e-12)
    assert math.isclose(float(state.up_vector[1]), 0.0, abs_tol=1e-12)
    assert math.isclose(float(state.up_vector[2]), 1.0, abs_tol=1e-12)


def test_up_vector_tilts_with_roll() -> None:
    # A rolled vehicle's up vector should tilt away from vertical
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(0.3, 0.0, 0.0))
    # Should not be purely vertical anymore
    assert not math.isclose(float(state.up_vector[2]), 1.0, abs_tol=1e-3)


# --------------------------------------------------------------------------- #
# Immutability: transitions return new states
# --------------------------------------------------------------------------- #


def test_with_position_does_not_mutate_original() -> None:
    original = VehicleState(lat_deg=10.0, lon_deg=20.0, alt_m=30.0, rotation=euler_to_matrix(0.0, 0.0, 0.0))
    new = original.with_position(11.0, 21.0, 31.0)
    # Original unchanged
    assert original.lat_deg == 10.0
    assert original.lon_deg == 20.0
    assert original.alt_m == 30.0
    # New has updated values
    assert new.lat_deg == 11.0
    assert new.lon_deg == 21.0
    assert new.alt_m == 31.0


def test_with_rotation_does_not_mutate_original() -> None:
    mat1 = euler_to_matrix(0.0, 0.0, 0.0)
    mat2 = euler_to_matrix(0.5, 0.0, 0.0)
    original = VehicleState(lat_deg=10.0, lon_deg=20.0, alt_m=30.0, rotation=mat1)
    new = original.with_rotation(mat2)
    # Original rotation unchanged
    assert np.allclose(original.rotation, mat1)
    assert np.allclose(new.rotation, mat2)


def test_frozen_state_rejects_attribute_mutation() -> None:
    state = VehicleState(lat_deg=10.0, lon_deg=20.0, alt_m=30.0, rotation=euler_to_matrix(0.0, 0.0, 0.0))
    with pytest.raises(AttributeError):
        state.lat_deg = 99.0  # type: ignore[misc]


# --------------------------------------------------------------------------- #
# with_heading_r preserves roll and pitch
# --------------------------------------------------------------------------- #


def test_with_heading_preserves_roll_and_pitch() -> None:
    roll, pitch = 0.3, -0.2
    state = VehicleState(lat_deg=0.0, lon_deg=0.0, alt_m=0.0, rotation=euler_to_matrix(roll, pitch, 0.5))
    new = state.with_heading_r(1.0)
    pose = new.to_pose()
    assert math.isclose(pose.orientation.roll_r, roll, abs_tol=1e-12)
    assert math.isclose(pose.orientation.pitch_r, pitch, abs_tol=1e-12)
    assert math.isclose(pose.orientation.yaw_r, 1.0, abs_tol=1e-12)
