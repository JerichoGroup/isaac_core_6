"""Pose pipeline kernel: everything the camera-pose OmniGraph node needs to compute.

Extracted from the node so it is testable without Isaac Sim, a GPU, or a fake OmniGraph database.
The node is left as a thin adapter that reads inputs, calls :func:`compose_local_pose`, and writes
outputs.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.pose import Lla
from isaac_core.geo.enu import EnuConverter
from isaac_core.geo.rotations import (
    compose_rotation,
    euler_to_matrix,
    euler_to_quaternion,
    matrix_to_euler,
)

# A pose of exactly (0, 0, 0) means "nothing has arrived yet" rather than "the Gulf of Guinea".
# Treating it as real teleports the camera to the ENU origin on every pre-data tick.
_NO_FIX_TOLERANCE = 1e-12


@dataclass(frozen=True, slots=True)
class LocalPose:
    """A vehicle pose expressed in the stage's local ENU frame.

    Attributes:
        local_position: ``(east, north, up)`` in metres from the ENU reference.
        attitude_r: Composed ``(roll, pitch, yaw)`` in radians, airframe plus gimbal offset.
        quaternion_xyzw: The same attitude as a quaternion in Isaac's ``[x, y, z, w]`` order.

    """

    local_position: tuple[float, float, float]
    attitude_r: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def has_position_fix(position: tuple[float, float, float]) -> bool:
    """Report whether a global position carries real data.

    Args:
        position: ``(lat_deg, lon_deg, alt_m)`` as read off the wire.

    Returns:
        ``False`` when every component is zero, which is what an unconnected or pre-data input
        looks like.

    """
    return any(abs(value) > _NO_FIX_TOLERANCE for value in position)


def rotation_frame_from_text(value: str) -> RotationFrame:
    """Parse a rotation frame name, defaulting to ``BODY``.

    Args:
        value: Text from config or a node input, expected ``"body"`` or ``"world"``.

    Returns:
        The matching :class:`~isaac_core.contracts.frames.RotationFrame`.

    """
    if value.strip().lower() == "world":
        return RotationFrame.WORLD
    return RotationFrame.BODY


def quaternion_to_isaac_order(w: float, x: float, y: float, z: float) -> tuple[float, float, float, float]:
    """Reorder a ``(w, x, y, z)`` quaternion into Isaac's ``[x, y, z, w]`` storage order.

    OmniGraph ``quatd[4]`` attributes store IJKR, which the shipped OGN schemas document as
    ``x, y, z, w`` even though the GUI displays scalar-first. Getting this backwards is a 180 degree
    error, so it lives in one place.

    Args:
        w: Scalar component.
        x: X vector component.
        y: Y vector component.
        z: Z vector component.

    Returns:
        ``(x, y, z, w)``.

    """
    return (x, y, z, w)


def compose_local_pose(
    *,
    converter: EnuConverter,
    position: Lla,
    attitude_r: tuple[float, float, float],
    rotation_frame: RotationFrame,
    gimbal_offset_deg: tuple[float, float, float],
) -> LocalPose:
    """Turn a geodetic pose plus a gimbal offset into a local ENU pose.

    The rotation frame governs how the airframe's own roll/pitch/yaw compose, not merely how the
    offset lands on top: in ``WORLD`` it is aerospace yaw-pitch-roll so yaw holds heading whatever
    the attitude, while in ``BODY`` yaw is about the already-pitched axis.

    The gimbal offset is **always** body-relative and always carries the airframe's NED-to-ENU sign
    flips on pitch and yaw. Both matter, and getting either wrong makes a commanded pitch come out as
    roll: a level, north-heading aircraft already carries ENU yaw of +90 degrees, so composing the
    offset about fixed world axes rotates it onto the wrong axis entirely.

    Args:
        converter: Converter built for the stage's ENU reference.
        position: The vehicle's geodetic position.
        attitude_r: Airframe ``(roll, pitch, yaw)`` in radians, already in ENU.
        rotation_frame: How the airframe attitude composes.
        gimbal_offset_deg: Gimbal ``(roll, pitch, yaw)`` offsets in degrees.

    Returns:
        The composed :class:`LocalPose`.

    """
    east, north, up = converter.lla_to_enu(position)

    roll_r, pitch_r, yaw_r = attitude_r
    airframe = euler_to_matrix(roll_r, pitch_r, yaw_r, frame=rotation_frame)

    offset_roll_deg, offset_pitch_deg, offset_yaw_deg = gimbal_offset_deg
    offset = euler_to_matrix(
        math.radians(offset_roll_deg),
        -math.radians(offset_pitch_deg),
        -math.radians(offset_yaw_deg),
        frame=RotationFrame.BODY,
    )

    composed = compose_rotation(airframe, offset, RotationFrame.BODY)

    # Read back with the same convention, or the angles do not round-trip.
    composed_r = matrix_to_euler(composed, frame=rotation_frame)
    quaternion = euler_to_quaternion(*composed_r, frame=rotation_frame)

    return LocalPose(
        local_position=(east, north, up),
        attitude_r=composed_r,
        quaternion_xyzw=quaternion_to_isaac_order(*quaternion),
    )


__all__ = [
    "LocalPose",
    "compose_local_pose",
    "has_position_fix",
    "quaternion_to_isaac_order",
    "rotation_frame_from_text",
]
