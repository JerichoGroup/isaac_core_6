"""
Rotation utilities: frame conversion, Euler/matrix/quaternion wrappers, and SLERP.

All Euler angles use the intrinsic-XYZ convention (``transforms3d`` axes string
``rxyz``), imported as :data:`~isaac_core.contracts.frames.EULER_AXES`.

Quaternion convention throughout is ``(w, x, y, z)`` -- the same ordering that
``transforms3d`` uses natively and that Isaac Sim displays in its GUI.
"""

import math

import numpy as np
from numpy.typing import NDArray
from transforms3d.euler import euler2mat, euler2quat, mat2euler, quat2euler

from isaac_core.contracts.angles import normalize_angle
from isaac_core.contracts.frames import EULER_AXES, Frame, RotationFrame
from isaac_core.contracts.pose import Rpy

# Threshold above which slerp falls back to normalised linear interpolation.
# Matching the 2023 DOT_THRESHOLD constant from udp_bot.py.
SLERP_DOT_THRESHOLD: float = 0.9995


# Euler convention for world-frame composition: intrinsic ZYX, the standard
# aerospace yaw-pitch-roll order. Yaw is applied first, about world up, so heading
# stays independent of pitch and roll.
_WORLD_EULER_AXES: str = "rzyx"


def ned_to_enu(attitude: Rpy) -> Rpy:
    """
    Convert an attitude from NED to ENU.

    The mapping is::

        roll_enu  =  roll_ned
        pitch_enu = -pitch_ned
        yaw_enu   = -yaw_ned + pi/2   (normalised to [-pi, pi])

    Roll passes through, pitch flips sign, and yaw flips sign and rotates by 90 degrees to
    turn a compass heading into an ENU bearing.

    This deliberately does NOT match the previous generation, which swapped roll and pitch
    (``roll_enu = pitch_ned``). That swap made the two axes trade places at the camera: a
    pitch input banked the image and a roll input tilted the nose. Verified against the
    body-axis convention (+X nose, +Y left wing, +Z up) and the camera's own 90 degree
    mount: with this mapping ``+pitch`` raises the nose, ``+roll`` drops the right wing,
    and ``+yaw`` turns right. Anything calibrated against the old behaviour needs its signs
    revisited.

    Args:
        attitude: An :class:`~isaac_core.contracts.pose.Rpy` tagged with
            :attr:`~isaac_core.contracts.frames.Frame.NED`.

    Returns:
        A new :class:`~isaac_core.contracts.pose.Rpy` tagged with
        :attr:`~isaac_core.contracts.frames.Frame.ENU`.

    Raises:
        ValueError: If the input is already tagged ENU.

    """
    if attitude.frame is Frame.ENU:
        msg = "Attitude is already in ENU frame; cannot convert NED→ENU."
        raise ValueError(msg)

    roll_enu = attitude.roll_r
    pitch_enu = -attitude.pitch_r
    yaw_enu = normalize_angle(-attitude.yaw_r + math.pi / 2.0)

    return Rpy(roll_r=roll_enu, pitch_r=pitch_enu, yaw_r=yaw_enu, frame=Frame.ENU)


def enu_to_ned(attitude: Rpy) -> Rpy:
    """
    Convert an attitude from ENU to NED.

    This is the true inverse of :func:`ned_to_enu`::

        roll_ned  =  roll_enu
        pitch_ned = -pitch_enu
        yaw_ned   = -(yaw_enu - pi/2)   (normalised to [-pi, pi])

    Args:
        attitude: An :class:`~isaac_core.contracts.pose.Rpy` tagged with
            :attr:`~isaac_core.contracts.frames.Frame.ENU`.

    Returns:
        A new :class:`~isaac_core.contracts.pose.Rpy` tagged with
        :attr:`~isaac_core.contracts.frames.Frame.NED`.

    Raises:
        ValueError: If the input is already tagged NED.

    """
    if attitude.frame is Frame.NED:
        msg = "Attitude is already in NED frame; cannot convert ENU→NED."
        raise ValueError(msg)

    roll_ned = attitude.roll_r
    pitch_ned = -attitude.pitch_r
    yaw_ned = normalize_angle(-(attitude.yaw_r - math.pi / 2.0))

    return Rpy(roll_r=roll_ned, pitch_r=pitch_ned, yaw_r=yaw_ned, frame=Frame.NED)


def euler_to_matrix(
    roll_r: float,
    pitch_r: float,
    yaw_r: float,
    frame: RotationFrame = RotationFrame.BODY,
) -> NDArray[np.float64]:
    """
    Build a rotation matrix from roll, pitch and yaw in radians.

    The ``frame`` argument selects the composition order, which is what decides how the
    three angles interact -- not a cosmetic choice:

    - :attr:`~isaac_core.contracts.frames.RotationFrame.BODY` uses intrinsic XYZ
      (``rxyz``): roll, then pitch about the already-rolled axis, then yaw about the
      already-pitched axis. Yaw therefore behaves like a body-axis rotation, so an aircraft
      pitched 90 degrees down swings sideways as yaw changes.
    - :attr:`~isaac_core.contracts.frames.RotationFrame.WORLD` uses intrinsic ZYX
      (``rzyx``): yaw about world up first, then pitch, then roll. This is the standard
      aerospace yaw-pitch-roll convention, and it is what makes yaw hold heading
      independently of attitude -- pitched straight down, changing yaw spins the view about
      its centre instead of moving where the nose points.

    Args:
        roll_r: Roll in radians.
        pitch_r: Pitch in radians.
        yaw_r: Yaw in radians.
        frame: Composition order to use.

    Returns:
        A 3x3 rotation matrix.

    """
    if frame is RotationFrame.WORLD:
        # transforms3d takes the angles in axis order, so ZYX means (yaw, pitch, roll).
        return euler2mat(yaw_r, pitch_r, roll_r, axes=_WORLD_EULER_AXES)
    return euler2mat(roll_r, pitch_r, yaw_r, axes=EULER_AXES)


def matrix_to_euler(
    matrix: NDArray[np.float64],
    frame: RotationFrame = RotationFrame.BODY,
) -> tuple[float, float, float]:
    """
    Decompose a rotation matrix into roll, pitch and yaw in radians.

    Inverse of :func:`euler_to_matrix`, and must be called with the same ``frame`` or the
    angles will not round-trip.

    Args:
        matrix: A 3x3 rotation matrix.
        frame: Composition order the matrix was built with.

    Returns:
        ``(roll_r, pitch_r, yaw_r)`` in radians.

    """
    if frame is RotationFrame.WORLD:
        yaw_r, pitch_r, roll_r = mat2euler(matrix, axes=_WORLD_EULER_AXES)
        return (float(roll_r), float(pitch_r), float(yaw_r))
    roll_r, pitch_r, yaw_r = mat2euler(matrix, axes=EULER_AXES)
    return (float(roll_r), float(pitch_r), float(yaw_r))


def euler_to_quaternion(
    roll_r: float,
    pitch_r: float,
    yaw_r: float,
    frame: RotationFrame = RotationFrame.BODY,
) -> tuple[float, float, float, float]:
    """
    Convert roll, pitch and yaw in radians to a quaternion.

    Args:
        roll_r: Roll in radians.
        pitch_r: Pitch in radians.
        yaw_r: Yaw in radians.
        frame: Composition order; see :func:`euler_to_matrix`.

    Returns:
        ``(w, x, y, z)``.

    """
    if frame is RotationFrame.WORLD:
        w, x, y, z = euler2quat(yaw_r, pitch_r, roll_r, axes=_WORLD_EULER_AXES)
    else:
        w, x, y, z = euler2quat(roll_r, pitch_r, yaw_r, axes=EULER_AXES)
    return (float(w), float(x), float(y), float(z))


def quaternion_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Convert a quaternion to intrinsic-XYZ Euler angles.

    Quaternion input is ``(w, x, y, z)`` -- the native ``transforms3d``
    convention.

    Args:
        w: Scalar part.
        x: X component of the vector part.
        y: Y component of the vector part.
        z: Z component of the vector part.

    Returns:
        A tuple ``(roll, pitch, yaw)`` in radians.

    """
    roll, pitch, yaw = quat2euler((w, x, y, z), axes=EULER_AXES)
    return (float(roll), float(pitch), float(yaw))


def compose_rotation(
    current_matrix: NDArray[np.float64],
    delta_matrix: NDArray[np.float64],
    frame: RotationFrame,
) -> NDArray[np.float64]:
    """
    Compose a rotation delta onto a current orientation.

    This is the single place where decision D14 is expressed:

    - :attr:`~isaac_core.contracts.frames.RotationFrame.WORLD` (extrinsic):
      ``delta @ current``
    - :attr:`~isaac_core.contracts.frames.RotationFrame.BODY` (intrinsic):
      ``current @ delta``

    Args:
        current_matrix: The existing 3×3 rotation matrix.
        delta_matrix: The delta 3×3 rotation matrix to apply.
        frame: Whether the delta is applied in world or body frame.

    Returns:
        The resulting 3×3 rotation matrix.

    """
    if frame is RotationFrame.WORLD:
        result: NDArray[np.float64] = delta_matrix @ current_matrix
    else:
        result = current_matrix @ delta_matrix
    return result


def slerp(
    q_from: tuple[float, float, float, float],
    q_to: tuple[float, float, float, float],
    t: float,
) -> tuple[float, float, float, float]:
    """
    Spherical linear interpolation between two unit quaternions.

    Uses shortest-path negation: if the dot product is negative, one quaternion
    is flipped so interpolation traverses the short arc. When the quaternions are
    nearly identical (dot product > :data:`SLERP_DOT_THRESHOLD`), a normalised
    linear interpolation is used instead to avoid numerical instability.

    Quaternion convention is ``(w, x, y, z)``.

    Args:
        q_from: Start quaternion ``(w, x, y, z)``.
        q_to: End quaternion ``(w, x, y, z)``.
        t: Interpolation parameter in ``[0, 1]``.

    Returns:
        The interpolated quaternion ``(w, x, y, z)``, unit-normalised.

    """
    a = np.array(q_from, dtype=np.float64)
    b = np.array(q_to, dtype=np.float64)

    # Normalise inputs
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)

    dot = float(np.dot(a, b))

    # Shortest path: flip if negative hemisphere
    if dot < 0.0:
        b = -b
        dot = -dot

    # Near-identity fallback: normalised linear interpolation
    if dot > SLERP_DOT_THRESHOLD:
        result = a + t * (b - a)
        result = result / np.linalg.norm(result)
        return (float(result[0]), float(result[1]), float(result[2]), float(result[3]))

    # Standard SLERP
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)

    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0

    result = s0 * a + s1 * b
    result = result / np.linalg.norm(result)
    return (float(result[0]), float(result[1]), float(result[2]), float(result[3]))
