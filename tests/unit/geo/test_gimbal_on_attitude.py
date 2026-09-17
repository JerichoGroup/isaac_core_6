"""Gimbal composition onto a airframe that is not level.

`test_gimbal_axes.py` pins the gimbal axes, but its helper hardcodes the airframe attitude to
``(0, 0, 0)``, so every assertion there describes a level aircraft. That is the same shape of gap that
let a rotation bug ship: one dimension exercised while the other is held at zero, and the two only
interact when both are non-zero.

Measured, the composition is correct — this file exists so it stays correct, and so the matrix row can
cite evidence for the claim it actually makes.

The gimbal is bolted to the airframe, so a gimbal angle is a rotation in the airframe's frame, not the
world's. That is what makes the rolled cases below look surprising and still be right: pitching a
camera that is mounted on a rolled aircraft tilts it partly sideways.
"""

from __future__ import annotations

import math
from typing import Final

import numpy as np
import pytest

from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import Lla, Rpy
from isaac_core.geo.enu import EnuConverter
from isaac_core.geo.pose_pipeline import compose_local_pose
from isaac_core.geo.rotations import ned_to_enu
from tests.unit.geo.test_gimbal_axes import CAMERA_LOCAL_QUAT, _quat_to_matrix

# The shipped scene's reference point, so these run in the same frame the simulator does.
_REFERENCE: Final = (32.22481, 35.25621, 516.7)

# Composition is exact; this covers quaternion round-tripping only.
_TOL_DEG: Final = 1e-6


def _look_direction(
    airframe_deg: tuple[float, float, float],
    gimbal_deg: tuple[float, float, float],
    frame: RotationFrame = RotationFrame.WORLD,
) -> np.ndarray:
    """Return the ENU direction the camera looks, for an airframe attitude and a gimbal offset.

    Args:
        airframe_deg: Airframe ``(roll, pitch, yaw)`` in degrees, NED, as it arrives on the wire.
        gimbal_deg: Gimbal ``(roll, pitch, yaw)`` offsets in degrees.
        frame: How the airframe attitude composes.

    Returns:
        A unit vector in ENU.

    """
    enu = ned_to_enu(
        Rpy(
            roll_r=math.radians(airframe_deg[0]),
            pitch_r=math.radians(airframe_deg[1]),
            yaw_r=math.radians(airframe_deg[2]),
            frame=Frame.NED,
        )
    )
    pose = compose_local_pose(
        converter=EnuConverter(*_REFERENCE),
        position=Lla(lat_deg=_REFERENCE[0], lon_deg=_REFERENCE[1], alt_m=_REFERENCE[2] + 1000.0),
        attitude_r=(enu.roll_r, enu.pitch_r, enu.yaw_r),
        rotation_frame=frame,
        gimbal_offset_deg=gimbal_deg,
    )
    qx, qy, qz, qw = pose.quaternion_xyzw
    xform = _quat_to_matrix(qw, qx, qy, qz)
    camera = _quat_to_matrix(*CAMERA_LOCAL_QUAT)
    look: np.ndarray = np.array([0.0, 0.0, -1.0]) @ camera @ xform
    return look


def _elevation_deg(look: np.ndarray) -> float:
    """Return how far above the horizon a look direction points, in degrees."""
    return math.degrees(math.asin(max(-1.0, min(1.0, float(look[2])))))


def _azimuth_deg(look: np.ndarray) -> float:
    """Return the compass-style azimuth of a look direction, in ENU degrees."""
    return math.degrees(math.atan2(float(look[1]), float(look[0])))


# -- pitch adds, which is the whole point of a gimbal --------------------------- #


@pytest.mark.parametrize(
    ("airframe_pitch_deg", "gimbal_pitch_deg", "expected_elevation_deg"),
    [
        (0.0, 0.0, 0.0),
        (0.0, -30.0, -30.0),
        (-30.0, 0.0, -30.0),
        (-30.0, -30.0, -60.0),
        (-20.0, -40.0, -60.0),
        (10.0, -40.0, -30.0),
    ],
)
def test_gimbal_pitch_adds_to_the_airframe_pitch(
    airframe_pitch_deg: float, gimbal_pitch_deg: float, expected_elevation_deg: float
) -> None:
    # The interesting rows are the ones where both are non-zero: only those exercise the composition,
    # and only those were missing.
    look = _look_direction((0.0, airframe_pitch_deg, 0.0), (0.0, gimbal_pitch_deg, 0.0))
    assert _elevation_deg(look) == pytest.approx(expected_elevation_deg, abs=_TOL_DEG)


def test_a_gimbal_pitch_does_not_change_where_a_pitched_aircraft_is_pointing() -> None:
    # Tilting the camera must not swing the heading, or a "look down" command would also turn.
    level = _look_direction((0.0, -30.0, 0.0), (0.0, 0.0, 0.0))
    tilted = _look_direction((0.0, -30.0, 0.0), (0.0, -30.0, 0.0))
    assert _azimuth_deg(tilted) == pytest.approx(_azimuth_deg(level), abs=_TOL_DEG)


# -- the gimbal follows the airframe's heading --------------------------------- #


@pytest.mark.parametrize("airframe_yaw_deg", [0.0, 45.0, 90.0, 180.0, -90.0])
def test_a_gimbal_tilt_holds_its_elevation_whatever_the_aircraft_heading(airframe_yaw_deg: float) -> None:
    # A bolted gimbal tilts in the airframe's frame, so its elevation cannot depend on the heading.
    look = _look_direction((0.0, 0.0, airframe_yaw_deg), (0.0, -30.0, 0.0))
    assert _elevation_deg(look) == pytest.approx(-30.0, abs=_TOL_DEG)


@pytest.mark.parametrize("airframe_yaw_deg", [0.0, 45.0, 90.0, -90.0])
def test_the_gimbal_looks_where_the_aircraft_is_heading(airframe_yaw_deg: float) -> None:
    # Yawing the aircraft must swing the view with it, by the same amount.
    reference = _azimuth_deg(_look_direction((0.0, 0.0, 0.0), (0.0, -20.0, 0.0)))
    swung = _azimuth_deg(_look_direction((0.0, 0.0, airframe_yaw_deg), (0.0, -20.0, 0.0)))
    # NED yaw increases clockwise while ENU azimuth increases anticlockwise, so the view swings the
    # other way by the same magnitude.
    assert math.cos(math.radians(swung - (reference - airframe_yaw_deg))) == pytest.approx(1.0, abs=1e-9)


# -- a rolled airframe tilts the gimbal's axes, which is correct --------------- #


def test_pitching_a_gimbal_on_a_rolled_aircraft_tilts_it_partly_sideways() -> None:
    # The surprising-but-correct case, and the reason this file explains the mount. A gimbal bolted to
    # an aircraft rolled 45 degrees does not have a horizontal pitch axis, so commanding 30 degrees of
    # pitch buys only sin(30)·cos(45) of elevation and spends the rest turning.
    look = _look_direction((45.0, 0.0, 0.0), (0.0, -30.0, 0.0))
    expected_elevation = math.degrees(math.asin(math.sin(math.radians(30.0)) * math.cos(math.radians(45.0))))
    assert _elevation_deg(look) == pytest.approx(-expected_elevation, abs=1e-4)
    assert _azimuth_deg(look) != pytest.approx(90.0, abs=1.0), "a rolled pitch must also swing the view"


def test_rolling_the_aircraft_alone_does_not_move_where_it_looks() -> None:
    # Roll rotates the image, it does not change the look direction. True of the airframe as much as
    # of the gimbal.
    level = _look_direction((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    rolled = _look_direction((45.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert _elevation_deg(rolled) == pytest.approx(_elevation_deg(level), abs=_TOL_DEG)
    assert _azimuth_deg(rolled) == pytest.approx(_azimuth_deg(level), abs=_TOL_DEG)


# -- all three axes non-zero at once ------------------------------------------- #


def test_a_full_attitude_with_a_full_gimbal_offset_stays_a_unit_direction() -> None:
    # The composition must not denormalise or produce NaN when nothing is zero, which is the case no
    # earlier test covered at all.
    look = _look_direction((15.0, -25.0, 135.0), (10.0, -20.0, 30.0))
    assert np.isfinite(look).all()
    assert float(np.linalg.norm(look)) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("frame", [RotationFrame.WORLD, RotationFrame.BODY])
def test_both_rotation_frames_compose_a_full_attitude_without_degenerating(frame: RotationFrame) -> None:
    look = _look_direction((15.0, -25.0, 135.0), (10.0, -20.0, 30.0), frame)
    assert np.isfinite(look).all()
    assert float(np.linalg.norm(look)) == pytest.approx(1.0, abs=1e-9)


def test_the_two_frames_differ_once_the_airframe_is_not_level() -> None:
    # If they agreed, the rotation_frame setting would be decoration. They agree on a level airframe,
    # which is exactly why a level-only test suite could not see the frame mattering.
    level_world = _look_direction((0.0, 0.0, 0.0), (0.0, -30.0, 0.0), RotationFrame.WORLD)
    level_body = _look_direction((0.0, 0.0, 0.0), (0.0, -30.0, 0.0), RotationFrame.BODY)
    assert _elevation_deg(level_world) == pytest.approx(_elevation_deg(level_body), abs=_TOL_DEG)

    tilted_world = _look_direction((20.0, -30.0, 60.0), (0.0, -30.0, 0.0), RotationFrame.WORLD)
    tilted_body = _look_direction((20.0, -30.0, 60.0), (0.0, -30.0, 0.0), RotationFrame.BODY)
    assert _elevation_deg(tilted_world) != pytest.approx(_elevation_deg(tilted_body), abs=0.1)
