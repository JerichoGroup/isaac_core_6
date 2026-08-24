"""Tests for isaac_core.vehicle.limits."""

import math

import pytest

from isaac_core.vehicle.limits import MotionLimits

# --------------------------------------------------------------------------- #
# Construction and validation
# --------------------------------------------------------------------------- #


def test_unlimited_factory_all_none() -> None:
    lim = MotionLimits.unlimited()
    assert lim.max_speed_mps is None
    assert lim.max_accel_mps2 is None
    assert lim.max_turn_rate_deg_s is None
    assert lim.max_climb_rate_mps is None


def test_rejects_zero_speed() -> None:
    with pytest.raises(ValueError, match="max_speed_mps"):
        MotionLimits(max_speed_mps=0.0)


def test_rejects_negative_turn_rate() -> None:
    with pytest.raises(ValueError, match="max_turn_rate_deg_s"):
        MotionLimits(max_turn_rate_deg_s=-1.0)


def test_rejects_negative_accel() -> None:
    with pytest.raises(ValueError, match="max_accel_mps2"):
        MotionLimits(max_accel_mps2=-5.0)


def test_rejects_negative_climb_rate() -> None:
    with pytest.raises(ValueError, match="max_climb_rate_mps"):
        MotionLimits(max_climb_rate_mps=-2.0)


def test_positive_values_accepted() -> None:
    lim = MotionLimits(max_speed_mps=10.0, max_accel_mps2=3.0, max_turn_rate_deg_s=45.0, max_climb_rate_mps=5.0)
    assert lim.max_speed_mps == 10.0


# --------------------------------------------------------------------------- #
# clamp_speed
# --------------------------------------------------------------------------- #


def test_clamp_speed_unlimited_passthrough() -> None:
    lim = MotionLimits.unlimited()
    assert lim.clamp_speed(999.0) == 999.0


def test_clamp_speed_limits_positive() -> None:
    lim = MotionLimits(max_speed_mps=5.0)
    assert lim.clamp_speed(10.0) == 5.0


def test_clamp_speed_preserves_sign_negative() -> None:
    # Negative speed (backward) should be clamped in magnitude but keep sign
    lim = MotionLimits(max_speed_mps=5.0)
    assert lim.clamp_speed(-10.0) == -5.0


def test_clamp_speed_below_limit_unchanged() -> None:
    lim = MotionLimits(max_speed_mps=10.0)
    assert lim.clamp_speed(3.0) == 3.0


# --------------------------------------------------------------------------- #
# clamp_turn_rate_r
# --------------------------------------------------------------------------- #


def test_clamp_turn_rate_unlimited_passthrough() -> None:
    lim = MotionLimits.unlimited()
    assert lim.clamp_turn_rate_r(99.0) == 99.0


def test_clamp_turn_rate_limits_magnitude() -> None:
    lim = MotionLimits(max_turn_rate_deg_s=90.0)
    max_rad_s = math.radians(90.0)
    # Give it 180 deg/s worth of radians
    assert math.isclose(lim.clamp_turn_rate_r(math.radians(180.0)), max_rad_s)


def test_clamp_turn_rate_preserves_negative_sign() -> None:
    lim = MotionLimits(max_turn_rate_deg_s=90.0)
    max_rad_s = math.radians(90.0)
    assert math.isclose(lim.clamp_turn_rate_r(-math.radians(180.0)), -max_rad_s)


# --------------------------------------------------------------------------- #
# clamp_climb_rate
# --------------------------------------------------------------------------- #


def test_clamp_climb_rate_unlimited_passthrough() -> None:
    lim = MotionLimits.unlimited()
    assert lim.clamp_climb_rate(50.0) == 50.0


def test_clamp_climb_rate_caps_ascending() -> None:
    lim = MotionLimits(max_climb_rate_mps=3.0)
    assert lim.clamp_climb_rate(10.0) == 3.0


def test_clamp_climb_rate_caps_descending() -> None:
    lim = MotionLimits(max_climb_rate_mps=3.0)
    assert lim.clamp_climb_rate(-10.0) == -3.0


# --------------------------------------------------------------------------- #
# effective_speed (speed + acceleration limiting)
# --------------------------------------------------------------------------- #


def test_effective_speed_unlimited_returns_desired() -> None:
    lim = MotionLimits.unlimited()
    assert lim.effective_speed(desired_mps=20.0, dt=0.1, current_speed_mps=0.0) == 20.0


def test_effective_speed_accel_limited() -> None:
    # max_accel = 5 m/s², dt = 0.1s → max change = 0.5 m/s per step
    lim = MotionLimits(max_accel_mps2=5.0)
    result = lim.effective_speed(desired_mps=10.0, dt=0.1, current_speed_mps=0.0)
    assert math.isclose(result, 0.5)


def test_effective_speed_both_limits_speed_cap_dominates() -> None:
    # max_speed = 2 m/s, accel would allow more
    lim = MotionLimits(max_speed_mps=2.0, max_accel_mps2=100.0)
    result = lim.effective_speed(desired_mps=50.0, dt=1.0, current_speed_mps=0.0)
    assert math.isclose(result, 2.0)


def test_effective_speed_deceleration_limited() -> None:
    # Decelerating from 10 to 0, max_accel limits the decel
    lim = MotionLimits(max_accel_mps2=5.0)
    result = lim.effective_speed(desired_mps=0.0, dt=0.1, current_speed_mps=10.0)
    # Max change = 0.5, so 10 - 0.5 = 9.5
    assert math.isclose(result, 9.5)


# --------------------------------------------------------------------------- #
# Frozen immutability
# --------------------------------------------------------------------------- #


def test_frozen_rejects_mutation() -> None:
    lim = MotionLimits(max_speed_mps=10.0)
    with pytest.raises(AttributeError):
        lim.max_speed_mps = 20.0  # type: ignore[misc]
