"""Tests for what the camera actually does when you move a stick.

Everything else about orientation is a formula that can be "right" while the picture is
wrong. These tests assert the observable outcome instead: send a roll, pitch or yaw and check
where the camera ends up looking. They caught roll and pitch being swapped -- a pitch input
banked the image and a roll input tilted the nose -- which had survived a formula-level test
suite because the formula matched the previous generation exactly. The previous generation was
wrong too.

The chain modelled here is the real one: wire NED angles, `ned_to_enu`, `euler_to_matrix`,
then the camera prim's own 90 degree mount inside the vehicle Xform.
"""

from __future__ import annotations

import numpy as np
import pytest
from transforms3d.quaternions import quat2mat

from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import Rpy
from isaac_core.geo.rotations import euler_to_matrix, ned_to_enu

# The camera prim's orientation inside the vehicle Xform, as authored in the camera layers.
# USD stores quatd as (w, x, y, z).
_CAMERA_MOUNT = np.asarray(quat2mat([0.5, 0.5, -0.5, -0.5]))

# A USD camera looks along its own -Z, with +Y up and +X to the right in the image.
_CAMERA_FORWARD = np.array([0.0, 0.0, -1.0])
_CAMERA_UP = np.array([0.0, 1.0, 0.0])

# World axes are ENU: index 0 East, 1 North, 2 Up.
_EAST, _NORTH, _UP = 0, 1, 2


def _look(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Return the camera's world forward and up vectors for a wire attitude in NED degrees."""
    enu = ned_to_enu(Rpy.from_degrees(roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg, frame=Frame.NED))
    body = np.asarray(euler_to_matrix(enu.roll_r, enu.pitch_r, enu.yaw_r, frame=RotationFrame.WORLD))
    matrix = body @ _CAMERA_MOUNT
    return matrix @ _CAMERA_FORWARD, matrix @ _CAMERA_UP


def test_level_and_zero_heading_looks_north_and_upright() -> None:
    forward, up = _look(0.0, 0.0, 0.0)
    assert np.allclose(forward, [0.0, 1.0, 0.0], atol=1e-9), "zero yaw should look north"
    assert np.allclose(up, [0.0, 0.0, 1.0], atol=1e-9), "level flight should be upright"


@pytest.mark.parametrize("pitch_deg", [10.0, 30.0, 60.0])
def test_positive_pitch_raises_the_nose(pitch_deg: float) -> None:
    forward, _ = _look(0.0, pitch_deg, 0.0)
    assert forward[_UP] > 0.0, f"+pitch must look up, got {forward}"


@pytest.mark.parametrize("pitch_deg", [-10.0, -30.0, -60.0])
def test_negative_pitch_lowers_the_nose(pitch_deg: float) -> None:
    forward, _ = _look(0.0, pitch_deg, 0.0)
    assert forward[_UP] < 0.0, f"-pitch must look down, got {forward}"


def test_pitch_does_not_bank_the_image() -> None:
    # The reported bug: a pitch input rolled the picture instead of moving the nose.
    _, up = _look(0.0, 30.0, 0.0)
    assert abs(up[_EAST]) < 1e-9, f"pitch must not tilt the horizon, up={up}"


@pytest.mark.parametrize("roll_deg", [10.0, 30.0, 60.0])
def test_positive_roll_drops_the_right_wing(roll_deg: float) -> None:
    # Flying north, banking right tilts the aircraft's up vector toward the east.
    _, up = _look(roll_deg, 0.0, 0.0)
    assert up[_EAST] > 0.0, f"+roll must bank right, up={up}"


def test_roll_does_not_move_the_nose() -> None:
    # The other half of the reported bug: a roll input pitched the nose.
    forward, _ = _look(30.0, 0.0, 0.0)
    assert abs(forward[_UP]) < 1e-9, f"roll must not move the nose, forward={forward}"
    assert forward[_NORTH] == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("yaw_deg", [10.0, 45.0, 80.0])
def test_positive_yaw_turns_right(yaw_deg: float) -> None:
    # From a north heading, turning right swings the nose toward the east.
    forward, _ = _look(0.0, 0.0, yaw_deg)
    assert forward[_EAST] > 0.0, f"+yaw must turn right, forward={forward}"


@pytest.mark.parametrize("yaw_deg", [-10.0, -45.0, -80.0])
def test_negative_yaw_turns_left(yaw_deg: float) -> None:
    forward, _ = _look(0.0, 0.0, yaw_deg)
    assert forward[_EAST] < 0.0, f"-yaw must turn left, forward={forward}"


def test_yaw_keeps_the_nose_level() -> None:
    forward, up = _look(0.0, 0.0, 55.0)
    assert forward[_UP] == pytest.approx(0.0, abs=1e-9), "a heading change must not tilt the nose"
    assert np.allclose(up, [0.0, 0.0, 1.0], atol=1e-9), "a heading change must not bank the image"


def test_ninety_degree_yaw_looks_east() -> None:
    # Sanity anchor on the heading convention: NED yaw 90 is due east.
    forward, _ = _look(0.0, 0.0, 90.0)
    assert np.allclose(forward, [1.0, 0.0, 0.0], atol=1e-9), f"yaw 90 should look east, got {forward}"


def test_straight_down_is_reachable() -> None:
    # The case that exposed the frame problem: pitched fully down.
    forward, _ = _look(0.0, -90.0, 0.0)
    assert np.allclose(forward, [0.0, 0.0, -1.0], atol=1e-9), f"pitch -90 should look down, got {forward}"


@pytest.mark.parametrize("yaw_deg", [0.0, 45.0, 90.0, 180.0])
def test_yaw_while_pointing_down_only_spins_the_image(yaw_deg: float) -> None:
    # World-frame yaw holds heading independently of attitude, so pointing straight down the
    # view direction must not move no matter what yaw does.
    forward, _ = _look(0.0, -90.0, yaw_deg)
    assert np.allclose(forward, [0.0, 0.0, -1.0], atol=1e-9), f"yaw moved the nose: {forward}"
