"""Lock down the gimbal's observable behaviour: where the camera actually ends up pointing.

Asserts on the camera's world-space look direction rather than on Euler numbers or matrix
multiplication order. That distinction is the whole point: the composition *formula* looked correct
and its unit tests passed while a commanded pitch was coming out as roll on screen. Only following
the transform chain all the way to a direction vector exposed it.

Chain reproduced here, in the row-vector convention USD uses:

    look = (0, 0, -1) @ R_camera_local @ R_xform

where ``R_xform`` comes from the quaternion the position node publishes and ``R_camera_local`` is
the camera's authored orientation in the shipped layers.
"""

import math

import numpy as np
import pytest

from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import Lla, Rpy
from isaac_core.geo.enu import EnuConverter
from isaac_core.geo.pose_pipeline import compose_local_pose
from isaac_core.geo.rotations import (
    ned_to_enu,
)

# The camera's authored local orientation from the shipped camera layers: quatd (w, x, y, z).
CAMERA_LOCAL_QUAT = (0.5, 0.5, -0.5, -0.5)

# Tolerance in degrees. Generous because the chain runs through a quaternion round-trip.
TOL_DEG = 0.01


# Any reference works: the look direction does not depend on where the aircraft is.
_REFERENCE = (32.22481, 35.25621, 516.7)


def _quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Convert a (w, x, y, z) quaternion to a row-vector rotation matrix.

    Args:
        w: Real part.
        x: First imaginary component.
        y: Second imaginary component.
        z: Third imaginary component.

    Returns:
        A 3x3 rotation matrix for use as ``vector @ matrix``.

    """
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)],
            [2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)],
            [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _camera_axes(
    offset_roll_deg: float = 0.0,
    offset_pitch_deg: float = 0.0,
    offset_yaw_deg: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the camera's world look and up vectors for a gimbal offset, level north-bound flight.

    Drives the SAME kernel the OmniGraph node calls, rather than reimplementing it: this test used to
    carry a copy of the node's composition, so the node and the test could drift apart while both
    stayed green. The quaternion is read back in Isaac's storage order, exactly as the node writes it.

    Args:
        offset_roll_deg: Commanded gimbal roll in degrees.
        offset_pitch_deg: Commanded gimbal pitch in degrees.
        offset_yaw_deg: Commanded gimbal yaw in degrees.

    Returns:
        ``(look, up)`` unit vectors in ENU.

    """
    enu = ned_to_enu(Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED))
    pose = compose_local_pose(
        converter=EnuConverter(*_REFERENCE),
        position=Lla(lat_deg=_REFERENCE[0], lon_deg=_REFERENCE[1], alt_m=_REFERENCE[2] + 1000.0),
        attitude_r=(enu.roll_r, enu.pitch_r, enu.yaw_r),
        rotation_frame=RotationFrame.WORLD,
        gimbal_offset_deg=(offset_roll_deg, offset_pitch_deg, offset_yaw_deg),
    )
    # Isaac order is (x, y, z, w); _quat_to_matrix takes (w, x, y, z).
    qx, qy, qz, qw = pose.quaternion_xyzw
    xform = _quat_to_matrix(qw, qx, qy, qz)
    camera = _quat_to_matrix(*CAMERA_LOCAL_QUAT)

    look = np.array([0.0, 0.0, -1.0]) @ camera @ xform
    up = np.array([0.0, 1.0, 0.0]) @ camera @ xform
    return look, up


def _elevation_deg(look: np.ndarray) -> float:
    """Return the look direction's elevation above the horizon, in degrees."""
    return math.degrees(math.asin(float(np.clip(look[2], -1.0, 1.0))))


def _bearing_deg(look: np.ndarray) -> float:
    """Return the look direction's compass bearing in degrees, east of north."""
    return math.degrees(math.atan2(float(look[0]), float(look[1])))


def test_a_level_aircraft_looks_north_along_the_horizon() -> None:
    # Sanity anchor. An earlier attempt at this test used the body +X axis directly and had a
    # level aircraft looking SOUTH, which quietly invalidated every conclusion drawn from it.
    look, _ = _camera_axes()

    assert _elevation_deg(look) == pytest.approx(0.0, abs=TOL_DEG)
    assert _bearing_deg(look) == pytest.approx(0.0, abs=TOL_DEG)


@pytest.mark.parametrize("pitch_deg", [-45.0, -15.0, 15.0, 45.0])
def test_gimbal_pitch_moves_the_view_up_and_down_by_that_amount(pitch_deg: float) -> None:
    # The bug this guards: commanded pitch changed elevation by 0.00 degrees while commanded roll
    # changed it instead, because the offset was composed about fixed WORLD axes and the airframe
    # already carries a +90 degree ENU yaw that rotated pitch onto the roll axis.
    look, _ = _camera_axes(offset_pitch_deg=pitch_deg)

    assert _elevation_deg(look) == pytest.approx(pitch_deg, abs=TOL_DEG)
    assert _bearing_deg(look) == pytest.approx(0.0, abs=TOL_DEG)


@pytest.mark.parametrize("roll_deg", [-25.0, 25.0])
def test_gimbal_roll_tilts_the_image_without_moving_where_it_looks(roll_deg: float) -> None:
    """Keep roll out of the look direction: it rotates the image, nothing else."""
    look, up = _camera_axes(offset_roll_deg=roll_deg)

    assert _elevation_deg(look) == pytest.approx(0.0, abs=TOL_DEG)
    assert _bearing_deg(look) == pytest.approx(0.0, abs=TOL_DEG)
    # Image-up must have tilted, otherwise roll did nothing at all.
    assert abs(math.degrees(math.atan2(float(up[0]), float(up[2])))) == pytest.approx(abs(roll_deg), abs=TOL_DEG)


@pytest.mark.parametrize("yaw_deg", [30.0, 60.0, -45.0])
def test_gimbal_yaw_turns_the_view_right_for_positive_angles(yaw_deg: float) -> None:
    """Increase the compass bearing for a positive yaw, matching the documented convention."""
    look, _ = _camera_axes(offset_yaw_deg=yaw_deg)

    assert _bearing_deg(look) == pytest.approx(yaw_deg, abs=TOL_DEG)
    assert _elevation_deg(look) == pytest.approx(0.0, abs=TOL_DEG)


def test_pitch_and_roll_are_not_interchangeable() -> None:
    # Direct assertion of the reported symptom: these two commands must not produce the same
    # camera direction. Before the fix they effectively swapped.
    pitch_look, _ = _camera_axes(offset_pitch_deg=-25.0)
    roll_look, _ = _camera_axes(offset_roll_deg=-25.0)

    assert not np.allclose(pitch_look, roll_look, atol=1e-6)
    assert _elevation_deg(pitch_look) == pytest.approx(-25.0, abs=TOL_DEG)
    assert _elevation_deg(roll_look) == pytest.approx(0.0, abs=TOL_DEG)
