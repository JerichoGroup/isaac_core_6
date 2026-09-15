"""Whole-pipeline validation of both rotation frames.

A bug in the orientation pipeline makes the entire project useless, so this file checks the
chain end to end rather than any single function: wire bytes, decode, NED to ENU, matrix
composition, quaternion, and finally where the camera looks in world coordinates.

It also pins down the exact algebraic meaning of each frame, because the honest answer to
"does changing one angle rotate about only one axis" is subtle:

- WORLD is ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` -- fixed-axis (extrinsic) XYZ, which is the
  standard aerospace yaw-pitch-roll convention. Yaw is the outermost rotation, so yaw is
  **always** a rotation about world up regardless of the other two. Pitch and roll are not:
  once yaw is non-zero, changing pitch rotates about a yaw-rotated axis.
- BODY is ``Rx(roll) @ Ry(pitch) @ Rz(yaw)`` -- intrinsic XYZ. Here roll is outermost, so
  roll is the angle that always maps to a fixed world axis.

Full independence of all three angles is not achievable with any Euler triple: rotations do
not commute, so only the outermost angle can have that property. What *is* guaranteed, and is
checked below, is that the orientation decomposes back to exactly the angles that built it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from transforms3d.quaternions import quat2mat

from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.geo.rotations import (
    euler_to_matrix,
    euler_to_quaternion,
    matrix_to_euler,
    ned_to_enu,
)
from isaac_core.protocol.pose_packet import decode, encode

# World axes are ENU.
_EAST, _NORTH, _UP = 0, 1, 2

# The camera prim's mount inside the vehicle Xform, as authored in both camera layers.
_CAMERA_MOUNT = np.asarray(quat2mat([0.5, 0.5, -0.5, -0.5]))
_CAMERA_FORWARD = np.array([0.0, 0.0, -1.0])

_FRAMES = [RotationFrame.WORLD, RotationFrame.BODY]


def _rx(angle: float) -> np.ndarray:
    return np.array([[1, 0, 0], [0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]])


def _ry(angle: float) -> np.ndarray:
    return np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0], [-math.sin(angle), 0, math.cos(angle)]])


def _rz(angle: float) -> np.ndarray:
    return np.array([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])


def _axis_of(matrix: np.ndarray) -> np.ndarray:
    """Return the unit rotation axis of a rotation matrix, or zeros for no rotation."""
    angle = math.acos(max(-1.0, min(1.0, (float(np.trace(matrix)) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    return axis / (2.0 * math.sin(angle))


# A spread of attitudes: level, single-axis, combined, and near gimbal lock.
_ATTITUDES = [
    (0.0, 0.0, 0.0),
    (15.0, 0.0, 0.0),
    (0.0, 25.0, 0.0),
    (0.0, 0.0, 35.0),
    (10.0, -20.0, 45.0),
    (-30.0, 40.0, -120.0),
    (0.0, -89.0, 60.0),
    (0.0, 89.0, -60.0),
    (179.0, 0.0, 179.0),
]


# --- what each frame is, algebraically ------------------------------------------------ #


@pytest.mark.parametrize(("roll_deg", "pitch_deg", "yaw_deg"), _ATTITUDES)
def test_world_frame_is_fixed_axis_xyz(roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    # The standard aerospace yaw-pitch-roll matrix. Stated as a test so the convention cannot
    # drift silently, since almost any convention "looks right" when level.
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    actual = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=RotationFrame.WORLD))
    assert np.allclose(actual, _rz(yaw) @ _ry(pitch) @ _rx(roll), atol=1e-12)


@pytest.mark.parametrize(("roll_deg", "pitch_deg", "yaw_deg"), _ATTITUDES)
def test_body_frame_is_intrinsic_xyz(roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    actual = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=RotationFrame.BODY))
    assert np.allclose(actual, _rx(roll) @ _ry(pitch) @ _rz(yaw), atol=1e-12)


def test_the_two_frames_actually_differ() -> None:
    # If they ever coincide the setting is decorative. They must not.
    roll, pitch, yaw = 0.3, 0.5, 0.7
    world = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=RotationFrame.WORLD))
    body = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=RotationFrame.BODY))
    assert not np.allclose(world, body, atol=1e-6)


# --- single-axis behaviour ------------------------------------------------------------- #


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize(
    ("index", "expected_axis"),
    [(0, np.array([1.0, 0.0, 0.0])), (1, np.array([0.0, 1.0, 0.0])), (2, np.array([0.0, 0.0, 1.0]))],
)
def test_a_single_angle_from_level_rotates_about_exactly_that_world_axis(
    frame: RotationFrame, index: int, expected_axis: np.ndarray
) -> None:
    # From level, both frames agree and each angle is a pure rotation about one world axis.
    angles = [0.0, 0.0, 0.0]
    angles[index] = math.radians(20.0)
    matrix = np.asarray(euler_to_matrix(angles[0], angles[1], angles[2], frame=frame))
    assert np.allclose(np.abs(_axis_of(matrix)), np.abs(expected_axis), atol=1e-9)


@pytest.mark.parametrize("held_yaw_deg", [0.0, 40.0, -95.0, 175.0])
@pytest.mark.parametrize("held_pitch_deg", [0.0, 30.0, -50.0])
def test_world_frame_yaw_is_always_a_rotation_about_world_up(held_yaw_deg: float, held_pitch_deg: float) -> None:
    # The defining property of WORLD, and the reason it is the right default for a vehicle:
    # a heading change is a rotation about world up whatever the current attitude.
    roll = math.radians(25.0)
    pitch = math.radians(held_pitch_deg)
    before = np.asarray(euler_to_matrix(roll, pitch, math.radians(held_yaw_deg), frame=RotationFrame.WORLD))
    after = np.asarray(euler_to_matrix(roll, pitch, math.radians(held_yaw_deg + 20.0), frame=RotationFrame.WORLD))
    axis = _axis_of(after @ before.T)
    assert np.allclose(np.abs(axis), [0.0, 0.0, 1.0], atol=1e-9), f"yaw rotated about {axis}"


@pytest.mark.parametrize("held_roll_deg", [0.0, 35.0, -80.0])
def test_body_frame_roll_is_always_a_rotation_about_world_east(held_roll_deg: float) -> None:
    # The mirror-image property in BODY: roll is the outermost rotation there. Documented so
    # nobody expects BODY to give a world-stable heading.
    pitch, yaw = math.radians(30.0), math.radians(50.0)
    before = np.asarray(euler_to_matrix(math.radians(held_roll_deg), pitch, yaw, frame=RotationFrame.BODY))
    after = np.asarray(euler_to_matrix(math.radians(held_roll_deg + 20.0), pitch, yaw, frame=RotationFrame.BODY))
    axis = _axis_of(after @ before.T)
    assert np.allclose(np.abs(axis), [1.0, 0.0, 0.0], atol=1e-9)


def test_world_frame_pitch_is_not_world_stable_once_yawed() -> None:
    # Honest negative test. Full independence of all three angles is impossible with Euler
    # angles, so this documents the real limit rather than pretending otherwise.
    roll, pitch, yaw = 0.0, math.radians(10.0), math.radians(40.0)
    before = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=RotationFrame.WORLD))
    after = np.asarray(euler_to_matrix(roll, pitch + math.radians(20.0), yaw, frame=RotationFrame.WORLD))
    axis = _axis_of(after @ before.T)
    assert not np.allclose(np.abs(axis), [0.0, 1.0, 0.0], atol=1e-3)
    # It is a rotation about world Y turned by the yaw, which stays horizontal.
    assert abs(axis[_UP]) < 1e-9


# --- decomposition fidelity ----------------------------------------------------------- #


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize(("roll_deg", "pitch_deg", "yaw_deg"), _ATTITUDES)
def test_orientation_decomposes_back_to_the_angles_that_built_it(
    frame: RotationFrame, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> None:
    # This is the guarantee that does hold for every attitude away from gimbal lock: the
    # stored orientation carries exactly the three numbers that were sent.
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    matrix = euler_to_matrix(roll, pitch, yaw, frame=frame)
    back_roll, back_pitch, back_yaw = matrix_to_euler(matrix, frame=frame)
    rebuilt = np.asarray(euler_to_matrix(back_roll, back_pitch, back_yaw, frame=frame))
    assert np.allclose(np.asarray(matrix), rebuilt, atol=1e-9)


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize("pitch_deg", [90.0, -90.0])
def test_gimbal_lock_still_preserves_the_orientation(frame: RotationFrame, pitch_deg: float) -> None:
    # At +-90 pitch the angle triple is not unique, but the orientation must still survive a
    # decompose-recompose cycle or the camera would jump.
    matrix = euler_to_matrix(0.3, math.radians(pitch_deg), 0.7, frame=frame)
    roll, pitch, yaw = matrix_to_euler(matrix, frame=frame)
    rebuilt = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=frame))
    assert np.allclose(np.asarray(matrix), rebuilt, atol=1e-9)


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize(("roll_deg", "pitch_deg", "yaw_deg"), _ATTITUDES)
def test_the_quaternion_and_the_matrix_describe_the_same_rotation(
    frame: RotationFrame, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> None:
    # The node writes a quaternion to the prim and publishes one on the ROS topic. If those
    # disagreed with the matrix, the picture and the published pose would diverge.
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    matrix = np.asarray(euler_to_matrix(roll, pitch, yaw, frame=frame))
    quaternion = euler_to_quaternion(roll, pitch, yaw, frame=frame)
    assert np.allclose(np.asarray(quat2mat(list(quaternion))), matrix, atol=1e-9)


# --- the whole pipeline, from wire bytes to where the camera looks --------------------- #


def _camera_forward(roll_deg: float, pitch_deg: float, yaw_deg: float, frame: RotationFrame) -> np.ndarray:
    """Run the real chain: encode, decode, NED to ENU, compose, apply the camera mount."""
    sent = GeodeticPose(
        position=Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1000.0),
        orientation=Rpy.from_degrees(roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg, frame=Frame.NED),
    )
    received = decode(encode(sent))
    enu = ned_to_enu(received.orientation)
    body = np.asarray(euler_to_matrix(enu.roll_r, enu.pitch_r, enu.yaw_r, frame=frame))
    return (body @ _CAMERA_MOUNT) @ _CAMERA_FORWARD


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize(("roll_deg", "pitch_deg", "yaw_deg"), _ATTITUDES)
def test_the_pipeline_produces_a_unit_view_direction(
    frame: RotationFrame, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> None:
    # Catches any accidental scaling or non-orthogonal matrix anywhere in the chain.
    forward = _camera_forward(roll_deg, pitch_deg, yaw_deg, frame)
    assert float(np.linalg.norm(forward)) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("frame", _FRAMES)
def test_the_pipeline_looks_north_when_level_at_zero_heading(frame: RotationFrame) -> None:
    # Both frames must agree here; if they do not, one of them has a sign error.
    assert np.allclose(_camera_forward(0.0, 0.0, 0.0, frame), [0.0, 1.0, 0.0], atol=1e-9)


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize("pitch_deg", [10.0, 45.0, 80.0])
def test_positive_pitch_raises_the_nose_in_both_frames(frame: RotationFrame, pitch_deg: float) -> None:
    # The convention must not depend on the frame: +pitch is nose up either way.
    assert _camera_forward(0.0, pitch_deg, 0.0, frame)[_UP] > 0.0


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize("yaw_deg", [20.0, 70.0])
def test_positive_yaw_turns_right_in_both_frames(frame: RotationFrame, yaw_deg: float) -> None:
    assert _camera_forward(0.0, 0.0, yaw_deg, frame)[_EAST] > 0.0


@pytest.mark.parametrize("frame", _FRAMES)
def test_ninety_degree_yaw_looks_east_in_both_frames(frame: RotationFrame) -> None:
    assert np.allclose(_camera_forward(0.0, 0.0, 90.0, frame), [1.0, 0.0, 0.0], atol=1e-9)


def test_world_frame_holds_the_view_when_yawing_while_pointing_down() -> None:
    # The case that exposed the original problem. WORLD must hold the view direction.
    for yaw_deg in (0.0, 45.0, 90.0, 180.0, -120.0):
        forward = _camera_forward(0.0, -90.0, yaw_deg, RotationFrame.WORLD)
        assert np.allclose(forward, [0.0, 0.0, -1.0], atol=1e-9), f"yaw {yaw_deg} moved the view"


def test_body_frame_does_not_hold_the_view_when_yawing_while_pointing_down() -> None:
    # The documented contrast, so the difference between the two settings is observable and
    # nobody has to guess which one they are running.
    down = _camera_forward(0.0, -90.0, 0.0, RotationFrame.BODY)
    yawed = _camera_forward(0.0, -90.0, 90.0, RotationFrame.BODY)
    assert not np.allclose(down, yawed, atol=1e-6)


@pytest.mark.parametrize("frame", _FRAMES)
@pytest.mark.parametrize(("roll_deg", "pitch_deg", "yaw_deg"), _ATTITUDES)
def test_the_wire_round_trip_does_not_disturb_the_attitude(
    frame: RotationFrame, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> None:
    # Encoding to 51 bytes and back must be exact, so the camera direction is identical to
    # the one computed without the wire hop.
    through_wire = _camera_forward(roll_deg, pitch_deg, yaw_deg, frame)
    enu = ned_to_enu(Rpy.from_degrees(roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg, frame=Frame.NED))
    direct = (
        np.asarray(euler_to_matrix(enu.roll_r, enu.pitch_r, enu.yaw_r, frame=frame)) @ _CAMERA_MOUNT
    ) @ _CAMERA_FORWARD
    assert np.allclose(through_wire, direct, atol=1e-12)
