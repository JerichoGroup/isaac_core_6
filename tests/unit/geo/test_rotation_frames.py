"""Tests for how the rotation frame changes the meaning of roll, pitch and yaw.

The behaviour these lock in is the one a pilot notices immediately: pitched straight down,
changing yaw should spin the view about its centre rather than swing where the nose points.
That only holds if yaw is applied about world up, which is what WORLD selects.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from isaac_core.contracts.frames import RotationFrame
from isaac_core.geo.rotations import euler_to_matrix, euler_to_quaternion, matrix_to_euler

# Body axes, per the project convention: +X nose, +Y left wing, +Z up.
_NOSE = np.array([1.0, 0.0, 0.0])
_UP = np.array([0.0, 0.0, 1.0])


def _nose_direction(roll_deg: float, pitch_deg: float, yaw_deg: float, frame: RotationFrame) -> np.ndarray:
    matrix = euler_to_matrix(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg), frame=frame)
    return np.asarray(matrix) @ _NOSE


def _up_direction(roll_deg: float, pitch_deg: float, yaw_deg: float, frame: RotationFrame) -> np.ndarray:
    matrix = euler_to_matrix(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg), frame=frame)
    return np.asarray(matrix) @ _UP


@pytest.mark.parametrize("yaw_deg", [0.0, 30.0, 45.0, 90.0, 180.0])
def test_world_frame_yaw_does_not_move_the_nose_when_pitched_vertical(yaw_deg: float) -> None:
    # The reported symptom. Pitched 90 degrees off level, yaw is a rotation about the view
    # axis, so the nose must stay put no matter what yaw is.
    baseline = _nose_direction(0.0, -90.0, 0.0, RotationFrame.WORLD)
    actual = _nose_direction(0.0, -90.0, yaw_deg, RotationFrame.WORLD)
    assert np.allclose(actual, baseline, atol=1e-9), f"nose moved: {actual} vs {baseline}"


def test_world_frame_yaw_still_rotates_the_view_about_its_centre() -> None:
    # It must not be a no-op either: the image should spin even though the nose holds.
    before = _up_direction(0.0, -90.0, 0.0, RotationFrame.WORLD)
    after = _up_direction(0.0, -90.0, 90.0, RotationFrame.WORLD)
    assert not np.allclose(after, before, atol=1e-6), "yaw had no effect at all"
    # A 90 degree spin about the view axis leaves the up vector perpendicular to where it was.
    assert abs(float(np.dot(before, after))) < 1e-9


def test_body_frame_yaw_does_move_the_nose_when_pitched_vertical() -> None:
    # The contrast, and the behaviour that made this look broken. Kept as a test because
    # BODY is still the right default for a gimbal bolted to the airframe.
    baseline = _nose_direction(0.0, -90.0, 0.0, RotationFrame.BODY)
    actual = _nose_direction(0.0, -90.0, 90.0, RotationFrame.BODY)
    assert not np.allclose(actual, baseline, atol=1e-6)


@pytest.mark.parametrize("yaw_deg", [0.0, 45.0, 120.0, -75.0])
def test_world_frame_yaw_keeps_the_nose_level_when_level(yaw_deg: float) -> None:
    # Straight and level, yaw is heading: the nose stays horizontal and swings in azimuth.
    nose = _nose_direction(0.0, 0.0, yaw_deg, RotationFrame.WORLD)
    assert nose[2] == pytest.approx(0.0, abs=1e-9), "heading change must not tilt the nose"


@pytest.mark.parametrize("frame", [RotationFrame.BODY, RotationFrame.WORLD])
@pytest.mark.parametrize(
    ("roll_deg", "pitch_deg", "yaw_deg"),
    [(0.0, 0.0, 0.0), (10.0, -25.0, 40.0), (-5.0, 60.0, -170.0), (45.0, 0.0, 90.0)],
)
def test_euler_round_trips_through_the_matrix(
    frame: RotationFrame, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> None:
    # Reading back with a different convention than was used to build is a silent corruption,
    # so the pair must agree.
    matrix = euler_to_matrix(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg), frame=frame)
    roll_r, pitch_r, yaw_r = matrix_to_euler(matrix, frame=frame)
    assert math.degrees(roll_r) == pytest.approx(roll_deg, abs=1e-6)
    assert math.degrees(pitch_r) == pytest.approx(pitch_deg, abs=1e-6)
    assert math.degrees(yaw_r) == pytest.approx(yaw_deg, abs=1e-6)


@pytest.mark.parametrize("frame", [RotationFrame.BODY, RotationFrame.WORLD])
def test_the_quaternion_agrees_with_the_matrix(frame: RotationFrame) -> None:
    # The node publishes a quaternion and writes a matrix-derived transform; a convention
    # mismatch between them would show as the camera and the ROS pose disagreeing.
    roll_r, pitch_r, yaw_r = math.radians(15.0), math.radians(-40.0), math.radians(70.0)
    matrix = np.asarray(euler_to_matrix(roll_r, pitch_r, yaw_r, frame=frame))
    w, x, y, z = euler_to_quaternion(roll_r, pitch_r, yaw_r, frame=frame)

    # Rebuild the matrix from the quaternion and compare how each maps the nose.
    from transforms3d.quaternions import quat2mat

    assert np.allclose(np.asarray(quat2mat([w, x, y, z])) @ _NOSE, matrix @ _NOSE, atol=1e-9)


def test_the_default_frame_is_unchanged() -> None:
    # BODY is the historical behaviour and matches the previous generation's `rxyz`.
    # Changing this default silently would alter every existing scene.
    explicit = euler_to_matrix(0.1, 0.2, 0.3, frame=RotationFrame.BODY)
    implicit = euler_to_matrix(0.1, 0.2, 0.3)
    assert np.allclose(np.asarray(explicit), np.asarray(implicit))
