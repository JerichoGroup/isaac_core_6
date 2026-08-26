"""Tests for rotation utilities."""

import math

import numpy as np
from numpy.typing import NDArray
import pytest

from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import Rpy
from isaac_core.geo.rotations import (
    SLERP_DOT_THRESHOLD,
    compose_rotation,
    enu_to_ned,
    euler_to_matrix,
    euler_to_quaternion,
    matrix_to_euler,
    ned_to_enu,
    normalize_angle,
    quaternion_to_euler,
    slerp,
)

# --------------------------------------------------------------------------- #
# normalize_angle
# --------------------------------------------------------------------------- #


def test_normalize_angle_zero() -> None:
    assert normalize_angle(0.0) == 0.0


def test_normalize_angle_pi_maps_to_negative_pi() -> None:
    # math convention: pi normalises to -pi (the formula yields -pi for input pi)
    assert math.isclose(normalize_angle(math.pi), -math.pi)


def test_normalize_angle_negative_pi_stays() -> None:
    assert math.isclose(normalize_angle(-math.pi), -math.pi)


def test_normalize_angle_wraps_two_pi_to_zero() -> None:
    assert math.isclose(normalize_angle(2 * math.pi), 0.0, abs_tol=1e-15)


def test_normalize_angle_wraps_large_positive() -> None:
    result = normalize_angle(3 * math.pi)
    assert math.isclose(result, -math.pi, abs_tol=1e-12)


def test_normalize_angle_wraps_large_negative() -> None:
    result = normalize_angle(-3 * math.pi)
    assert math.isclose(result, -math.pi, abs_tol=1e-12)


@pytest.mark.parametrize("angle", [-math.pi / 2, 0.0, math.pi / 2, math.pi / 4])
def test_normalize_angle_values_already_in_range_unchanged(angle: float) -> None:
    assert math.isclose(normalize_angle(angle), angle)


# --------------------------------------------------------------------------- #
# ned_to_enu / enu_to_ned
# --------------------------------------------------------------------------- #


def test_ned_to_enu_mapping() -> None:
    # Roll passes through, pitch flips sign, yaw flips sign and rotates 90 degrees.
    # This deliberately differs from the previous generation, which swapped roll and pitch
    # and so made a pitch input bank the camera. See test_camera_axes.py for the behaviour
    # that pins this down.
    ned = Rpy(roll_r=0.1, pitch_r=0.2, yaw_r=0.3, frame=Frame.NED)
    enu = ned_to_enu(ned)
    assert enu.frame is Frame.ENU
    assert math.isclose(enu.roll_r, 0.1)
    assert math.isclose(enu.pitch_r, -0.2)
    assert math.isclose(enu.yaw_r, normalize_angle(-0.3 + math.pi / 2))


def test_enu_to_ned_is_the_exact_inverse() -> None:
    enu = Rpy(roll_r=0.1, pitch_r=-0.2, yaw_r=normalize_angle(-0.3 + math.pi / 2), frame=Frame.ENU)
    ned = enu_to_ned(enu)
    assert ned.frame is Frame.NED
    assert math.isclose(ned.roll_r, 0.1, abs_tol=1e-12)
    assert math.isclose(ned.pitch_r, 0.2, abs_tol=1e-12)
    assert math.isclose(ned.yaw_r, 0.3, abs_tol=1e-12)


def test_ned_to_enu_round_trip() -> None:
    ned = Rpy(roll_r=0.5, pitch_r=-0.3, yaw_r=1.2, frame=Frame.NED)
    recovered = enu_to_ned(ned_to_enu(ned))
    assert math.isclose(recovered.roll_r, ned.roll_r, abs_tol=1e-12)
    assert math.isclose(recovered.pitch_r, ned.pitch_r, abs_tol=1e-12)
    assert math.isclose(recovered.yaw_r, ned.yaw_r, abs_tol=1e-12)


def test_enu_to_ned_round_trip() -> None:
    enu = Rpy(roll_r=-0.7, pitch_r=0.4, yaw_r=-1.5, frame=Frame.ENU)
    recovered = ned_to_enu(enu_to_ned(enu))
    assert math.isclose(recovered.roll_r, enu.roll_r, abs_tol=1e-12)
    assert math.isclose(recovered.pitch_r, enu.pitch_r, abs_tol=1e-12)
    assert math.isclose(recovered.yaw_r, enu.yaw_r, abs_tol=1e-12)


def test_ned_to_enu_raises_if_already_enu() -> None:
    with pytest.raises(ValueError, match="already in ENU"):
        ned_to_enu(Rpy(0.0, 0.0, 0.0, frame=Frame.ENU))


def test_enu_to_ned_raises_if_already_ned() -> None:
    with pytest.raises(ValueError, match="already in NED"):
        enu_to_ned(Rpy(0.0, 0.0, 0.0, frame=Frame.NED))


def test_ned_to_enu_zero_attitude() -> None:
    # Zero NED should give yaw_enu = pi/2 (heading north in ENU = 90° from east)
    ned = Rpy(0.0, 0.0, 0.0, frame=Frame.NED)
    enu = ned_to_enu(ned)
    assert math.isclose(enu.roll_r, 0.0)
    assert math.isclose(enu.pitch_r, 0.0)
    assert math.isclose(enu.yaw_r, math.pi / 2)


def test_ned_to_enu_yaw_wraps_correctly() -> None:
    # yaw_ned = pi should give yaw_enu = -pi + pi/2 = -pi/2
    ned = Rpy(0.0, 0.0, math.pi, frame=Frame.NED)
    enu = ned_to_enu(ned)
    expected = normalize_angle(-math.pi + math.pi / 2)
    assert math.isclose(enu.yaw_r, expected, abs_tol=1e-12)


# --------------------------------------------------------------------------- #
# euler <-> matrix <-> quaternion
# --------------------------------------------------------------------------- #


def test_euler_to_matrix_identity() -> None:
    mat = euler_to_matrix(0.0, 0.0, 0.0)
    assert np.allclose(mat, np.eye(3))


def test_euler_to_matrix_and_back_round_trip() -> None:
    roll, pitch, yaw = 0.3, -0.2, 1.1
    mat = euler_to_matrix(roll, pitch, yaw)
    r2, p2, y2 = matrix_to_euler(mat)
    assert math.isclose(r2, roll, abs_tol=1e-12)
    assert math.isclose(p2, pitch, abs_tol=1e-12)
    assert math.isclose(y2, yaw, abs_tol=1e-12)


def test_euler_to_quaternion_identity() -> None:
    w, x, y, z = euler_to_quaternion(0.0, 0.0, 0.0)
    assert math.isclose(w, 1.0, abs_tol=1e-12)
    assert math.isclose(x, 0.0, abs_tol=1e-12)
    assert math.isclose(y, 0.0, abs_tol=1e-12)
    assert math.isclose(z, 0.0, abs_tol=1e-12)


def test_euler_quaternion_round_trip() -> None:
    roll, pitch, yaw = 0.5, -0.3, 2.0
    w, x, y, z = euler_to_quaternion(roll, pitch, yaw)
    r2, p2, y2 = quaternion_to_euler(w, x, y, z)
    assert math.isclose(r2, roll, abs_tol=1e-10)
    assert math.isclose(p2, pitch, abs_tol=1e-10)
    assert math.isclose(y2, yaw, abs_tol=1e-10)


def test_quaternion_is_unit_norm() -> None:
    w, x, y, z = euler_to_quaternion(1.0, -0.5, 0.8)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    assert math.isclose(norm, 1.0, abs_tol=1e-12)


# --------------------------------------------------------------------------- #
# compose_rotation -- D14 (the crux: prove order matters)
# --------------------------------------------------------------------------- #


def _rotation_matrices_differ(a: NDArray[np.float64], b: NDArray[np.float64]) -> bool:
    """Return True if two matrices differ meaningfully."""
    return not np.allclose(a, b, atol=1e-10)


def test_compose_rotation_world_vs_body_differ_for_noncommuting_case() -> None:
    # Pick two non-commuting rotations: a 90° roll and a 45° yaw.
    # If world == body, the test would fail, proving order matters.
    current = euler_to_matrix(math.pi / 2, 0.0, 0.0)  # 90° roll
    delta = euler_to_matrix(0.0, 0.0, math.pi / 4)  # 45° yaw

    world_result = compose_rotation(current, delta, RotationFrame.WORLD)
    body_result = compose_rotation(current, delta, RotationFrame.BODY)

    assert _rotation_matrices_differ(world_result, body_result)


def test_compose_rotation_world_is_extrinsic() -> None:
    # World: delta @ current. Verify numerically.
    current = euler_to_matrix(0.3, 0.0, 0.0)
    delta = euler_to_matrix(0.0, 0.0, 0.5)
    result = compose_rotation(current, delta, RotationFrame.WORLD)
    expected = delta @ current
    assert np.allclose(result, expected)


def test_compose_rotation_body_is_intrinsic() -> None:
    # Body: current @ delta. Verify numerically.
    current = euler_to_matrix(0.3, 0.0, 0.0)
    delta = euler_to_matrix(0.0, 0.0, 0.5)
    result = compose_rotation(current, delta, RotationFrame.BODY)
    expected = current @ delta
    assert np.allclose(result, expected)


def test_compose_rotation_identity_delta_is_noop() -> None:
    current = euler_to_matrix(0.5, -0.3, 1.0)
    identity = np.eye(3, dtype=np.float64)
    assert np.allclose(compose_rotation(current, identity, RotationFrame.WORLD), current)
    assert np.allclose(compose_rotation(current, identity, RotationFrame.BODY), current)


# --------------------------------------------------------------------------- #
# slerp
# --------------------------------------------------------------------------- #


def test_slerp_at_t0_returns_start() -> None:
    q_from = euler_to_quaternion(0.0, 0.0, 0.0)
    q_to = euler_to_quaternion(0.5, 0.3, -0.7)
    result = slerp(q_from, q_to, 0.0)
    for a, b in zip(result, q_from):
        assert math.isclose(a, b, abs_tol=1e-10)


def test_slerp_at_t1_returns_end() -> None:
    q_from = euler_to_quaternion(0.0, 0.0, 0.0)
    q_to = euler_to_quaternion(0.5, 0.3, -0.7)
    result = slerp(q_from, q_to, 1.0)
    for a, b in zip(result, q_to):
        assert math.isclose(a, b, abs_tol=1e-10)


def test_slerp_midpoint_is_halfway() -> None:
    # 90° yaw rotation: midpoint should be 45° yaw
    q_from = euler_to_quaternion(0.0, 0.0, 0.0)
    q_to = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    result = slerp(q_from, q_to, 0.5)
    roll, pitch, yaw = quaternion_to_euler(*result)
    assert math.isclose(roll, 0.0, abs_tol=1e-10)
    assert math.isclose(pitch, 0.0, abs_tol=1e-10)
    assert math.isclose(yaw, math.pi / 4, abs_tol=1e-6)


def test_slerp_shortest_path_negation() -> None:
    # q and -q represent the same rotation; slerp should still work.
    q_from = euler_to_quaternion(0.0, 0.0, 0.0)
    q_to = euler_to_quaternion(0.0, 0.0, 0.1)
    # Negate q_to; slerp should still produce the same result as non-negated.
    q_to_neg = (-q_to[0], -q_to[1], -q_to[2], -q_to[3])
    result_normal = slerp(q_from, q_to, 0.5)
    result_negated = slerp(q_from, q_to_neg, 0.5)
    for a, b in zip(result_normal, result_negated):
        assert math.isclose(a, b, abs_tol=1e-10)


def test_slerp_near_identity_uses_lerp_fallback() -> None:
    # Two quaternions differing by less than the threshold use linear fallback.
    # Create a tiny rotation that will produce dot > SLERP_DOT_THRESHOLD.
    q_from = euler_to_quaternion(0.0, 0.0, 0.0)
    # A rotation of ~0.01 radians gives dot ≈ cos(0.005) ≈ 0.9999875 > 0.9995
    q_to = euler_to_quaternion(0.0, 0.0, 0.01)
    dot = sum(a * b for a, b in zip(q_from, q_to))
    assert dot > SLERP_DOT_THRESHOLD  # Confirm we're in the fallback path

    result = slerp(q_from, q_to, 0.5)
    # Should be approximately half the rotation
    _roll, _pitch, yaw = quaternion_to_euler(*result)
    assert math.isclose(yaw, 0.005, abs_tol=1e-4)


def test_slerp_result_is_unit_quaternion() -> None:
    q_from = euler_to_quaternion(0.3, -0.2, 1.0)
    q_to = euler_to_quaternion(-0.5, 0.4, -1.5)
    result = slerp(q_from, q_to, 0.37)
    norm = math.sqrt(sum(c * c for c in result))
    assert math.isclose(norm, 1.0, abs_tol=1e-12)


def test_slerp_same_quaternion_returns_itself() -> None:
    q = euler_to_quaternion(0.3, -0.2, 1.0)
    result = slerp(q, q, 0.5)
    for a, b in zip(result, q):
        assert math.isclose(a, b, abs_tol=1e-10)
