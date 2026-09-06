"""
Tests for gimbal angle state, slew-rate limiting and mechanical-stop clamping.

The behaviours locked in here are the ones that go subtly wrong: crossing the +/-pi seam by
the long way instead of the short one, a rate of zero freezing the gimbal instead of meaning
"no limit", and a rate limit overshooting the target on the final step.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.gimbal import GimbalAngles, clamp_angles, slew_towards


def test_from_degrees_and_to_degrees_round_trip() -> None:
    angles = GimbalAngles.from_degrees(10.0, -25.0, 90.0)
    roll_deg, pitch_deg, yaw_deg = angles.to_degrees()
    assert roll_deg == pytest.approx(10.0)
    assert pitch_deg == pytest.approx(-25.0)
    assert yaw_deg == pytest.approx(90.0)


def test_from_degrees_stores_radians() -> None:
    angles = GimbalAngles.from_degrees(0.0, 0.0, 180.0)
    assert angles.yaw_r == pytest.approx(math.pi)


def test_as_tuple_is_radians_in_order() -> None:
    angles = GimbalAngles(0.1, 0.2, 0.3)
    assert angles.as_tuple() == (0.1, 0.2, 0.3)


def test_normalized_wraps_every_axis_to_pi_band() -> None:
    angles = GimbalAngles(3.0 * math.pi, -3.0 * math.pi, 2.5 * math.pi)
    normalized = angles.normalized()
    for value in normalized.as_tuple():
        assert -math.pi <= value <= math.pi
    # 3*pi and -3*pi both land on the -pi end of the half-open [-pi, pi) wrap.
    assert normalized.roll_r == pytest.approx(-math.pi)
    assert normalized.yaw_r == pytest.approx(0.5 * math.pi)


def test_gimbal_angles_is_frozen() -> None:
    angles = GimbalAngles(0.0, 0.0, 0.0)
    with pytest.raises(AttributeError):
        angles.roll_r = 1.0  # type: ignore[misc]


# shortest-path: 179 -> -179 degrees is a +2 degree move across the seam, not -358 degrees.
def test_slew_takes_shortest_path_across_the_pi_seam() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 179.0)
    target = GimbalAngles.from_degrees(0.0, 0.0, -179.0)
    # One degree per second for one second: cannot reach, so it must step +1 degree to -180/180,
    # never regress toward 0 the long way.
    result = slew_towards(current, target, max_rate_r_s=math.radians(1.0), dt_s=1.0)
    assert math.degrees(result.yaw_r) == pytest.approx(180.0) or math.degrees(result.yaw_r) == pytest.approx(-180.0)


# shortest-path: with enough rate the short arc lands directly on the target, not the long way.
def test_slew_shortest_path_reaches_target_when_rate_allows() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 179.0)
    target = GimbalAngles.from_degrees(0.0, 0.0, -179.0)
    result = slew_towards(current, target, max_rate_r_s=math.radians(10.0), dt_s=1.0)
    assert math.degrees(result.yaw_r) == pytest.approx(-179.0)


# zero-means-unlimited: a rate of 0 snaps to the target, matching the config default of 0.
def test_slew_zero_rate_snaps_to_target() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 0.0)
    target = GimbalAngles.from_degrees(30.0, -45.0, 120.0)
    result = slew_towards(current, target, max_rate_r_s=0.0, dt_s=0.016)
    assert result.to_degrees() == pytest.approx((30.0, -45.0, 120.0))


# zero-means-unlimited: a negative rate is treated the same as zero -- unlimited.
def test_slew_negative_rate_snaps_to_target() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 0.0)
    target = GimbalAngles.from_degrees(10.0, 20.0, 30.0)
    result = slew_towards(current, target, max_rate_r_s=-5.0, dt_s=1.0)
    assert result.to_degrees() == pytest.approx((10.0, 20.0, 30.0))


@pytest.mark.parametrize("dt_s", [0.0, -0.5, -1.0])
def test_slew_non_positive_dt_returns_current(dt_s: float) -> None:
    current = GimbalAngles.from_degrees(5.0, 5.0, 5.0)
    target = GimbalAngles.from_degrees(90.0, 90.0, 90.0)
    result = slew_towards(current, target, max_rate_r_s=math.radians(10.0), dt_s=dt_s)
    assert result.to_degrees() == pytest.approx((5.0, 5.0, 5.0))


# no-overshoot: when the remaining delta is under one step, land exactly on the target.
def test_slew_does_not_overshoot_target() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 0.0)
    target = GimbalAngles.from_degrees(0.0, 0.0, 1.0)
    # Rate would permit 10 degrees this tick, but the target is only 1 degree away.
    result = slew_towards(current, target, max_rate_r_s=math.radians(10.0), dt_s=1.0)
    assert math.degrees(result.yaw_r) == pytest.approx(1.0)


def test_slew_moves_by_at_most_one_step_when_far() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 0.0)
    target = GimbalAngles.from_degrees(0.0, 0.0, 90.0)
    result = slew_towards(current, target, max_rate_r_s=math.radians(30.0), dt_s=1.0)
    assert math.degrees(result.yaw_r) == pytest.approx(30.0)


def test_slew_limits_each_axis_independently() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 0.0)
    # Roll is close, pitch is far: roll should land, pitch should be capped at one step.
    target = GimbalAngles.from_degrees(2.0, 90.0, 0.0)
    result = slew_towards(current, target, max_rate_r_s=math.radians(5.0), dt_s=1.0)
    assert math.degrees(result.roll_r) == pytest.approx(2.0)
    assert math.degrees(result.pitch_r) == pytest.approx(5.0)


def test_slew_negative_direction_steps_toward_target() -> None:
    current = GimbalAngles.from_degrees(0.0, 0.0, 90.0)
    target = GimbalAngles.from_degrees(0.0, 0.0, 0.0)
    result = slew_towards(current, target, max_rate_r_s=math.radians(30.0), dt_s=1.0)
    assert math.degrees(result.yaw_r) == pytest.approx(60.0)


def test_slew_output_is_normalised() -> None:
    current = GimbalAngles(0.0, 0.0, 3.0)
    target = GimbalAngles(0.0, 0.0, 3.2)
    result = slew_towards(current, target, max_rate_r_s=0.0, dt_s=1.0)
    # 3.2 rad wraps below pi.
    assert -math.pi <= result.yaw_r <= math.pi


def test_clamp_applies_per_axis_bounds() -> None:
    angles = GimbalAngles.from_degrees(50.0, -80.0, 200.0)
    limits = (
        (math.radians(-30.0), math.radians(30.0)),
        (math.radians(-45.0), math.radians(45.0)),
        (None, None),
    )
    result = clamp_angles(angles, limits)
    assert math.degrees(result.roll_r) == pytest.approx(30.0)
    assert math.degrees(result.pitch_r) == pytest.approx(-45.0)


def test_clamp_none_bound_is_unconstrained() -> None:
    angles = GimbalAngles.from_degrees(89.0, 0.0, 0.0)
    limits: tuple[tuple[float | None, float | None], ...] = (
        (None, math.radians(45.0)),
        (None, None),
        (None, None),
    )
    result = clamp_angles(angles, limits)  # type: ignore[arg-type]
    # Upper bound clamps, absent lower bound does not.
    assert math.degrees(result.roll_r) == pytest.approx(45.0)


def test_clamp_leaves_in_range_values_untouched() -> None:
    angles = GimbalAngles.from_degrees(10.0, -10.0, 20.0)
    limits = (
        (math.radians(-30.0), math.radians(30.0)),
        (math.radians(-30.0), math.radians(30.0)),
        (math.radians(-30.0), math.radians(30.0)),
    )
    result = clamp_angles(angles, limits)
    assert result.to_degrees() == pytest.approx((10.0, -10.0, 20.0))


def test_clamp_output_is_normalised() -> None:
    angles = GimbalAngles(0.0, 0.0, 0.0)
    limits = ((None, None), (None, None), (4.0, 5.0))
    result = clamp_angles(angles, limits)
    # Lower bound 4.0 rad wraps below pi after clamping.
    assert -math.pi <= result.yaw_r <= math.pi


def test_clamp_rejects_inverted_bounds() -> None:
    angles = GimbalAngles(0.0, 0.0, 0.0)
    limits = ((math.radians(30.0), math.radians(-30.0)), (None, None), (None, None))
    with pytest.raises(ValueError, match="max must not be below min"):
        clamp_angles(angles, limits)
