"""Tests for the ROS 2 timestamp conversion."""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.stamp import (
    NANOSECONDS_PER_SECOND,
    SEC_MAX,
    SEC_MIN,
    seconds_to_ros_stamp,
)


@pytest.mark.parametrize(
    ("seconds", "expected"),
    [
        (0.0, (0, 0)),
        (1.0, (1, 0)),
        (1.5, (1, 500_000_000)),
        (0.25, (0, 250_000_000)),
        (123.456789, (123, 456_789_000)),
        (3600.0, (3600, 0)),
    ],
)
def test_common_values_split_correctly(seconds: float, expected: tuple[int, int]) -> None:
    assert seconds_to_ros_stamp(seconds) == expected


def test_the_remainder_is_nanoseconds_not_milliseconds() -> None:
    # A wrong scale factor is the likeliest bug here and would still look plausible.
    # Half a second is 5e8 nanoseconds, not 500 or 5e5.
    _, nanosec = seconds_to_ros_stamp(10.5)
    assert nanosec == 500_000_000


@pytest.mark.parametrize(
    ("seconds", "expected"),
    [
        (-0.25, (-1, 750_000_000)),
        (-1.0, (-1, 0)),
        (-1.5, (-2, 500_000_000)),
    ],
)
def test_negative_times_use_floor_semantics(seconds: float, expected: tuple[int, int]) -> None:
    # builtin_interfaces/Time cannot hold a negative nanosec, so the remainder must stay
    # non-negative and the seconds must floor rather than truncate toward zero.
    assert seconds_to_ros_stamp(seconds) == expected


def test_rounding_up_to_a_full_second_carries_into_sec() -> None:
    # This value rounds to exactly 1e9 nanoseconds. Without the carry the result would be
    # (1, 1000000000), which ROS treats as malformed.
    sec, nanosec = seconds_to_ros_stamp(1.9999999999)
    assert nanosec < NANOSECONDS_PER_SECOND
    assert (sec, nanosec) == (2, 0)


@pytest.mark.parametrize(
    "seconds",
    [0.0, 0.5, 1.9999999999, 123.456789, -0.25, -1.5, 999.999999999],
)
def test_nanosec_is_always_a_valid_unsigned_remainder(seconds: float) -> None:
    _, nanosec = seconds_to_ros_stamp(seconds)
    assert 0 <= nanosec < NANOSECONDS_PER_SECOND


@pytest.mark.parametrize("seconds", [math.nan, math.inf, -math.inf])
def test_non_finite_input_is_rejected(seconds: float) -> None:
    # An unconnected or uninitialised double arriving as NaN must not become a timestamp.
    with pytest.raises(ValueError, match="finite"):
        seconds_to_ros_stamp(seconds)


@pytest.mark.parametrize("seconds", [float(SEC_MAX) + 1.0, float(SEC_MIN) - 2.0])
def test_values_outside_the_int32_sec_field_are_rejected(seconds: float) -> None:
    # Silently wrapping a timestamp is worse than refusing to emit one.
    with pytest.raises(ValueError, match="int32"):
        seconds_to_ros_stamp(seconds)


def test_the_conversion_is_monotonic() -> None:
    # Ordering is the main thing a consumer needs from a stamp, so it must survive the
    # split into two integers.
    samples = [0.0, 0.001, 0.5, 0.999999999, 1.0, 1.000000001, 2.5, 100.25]
    totals = [sec * NANOSECONDS_PER_SECOND + nanosec for sec, nanosec in map(seconds_to_ros_stamp, samples)]
    assert totals == sorted(totals)
    assert len(set(totals)) == len(totals), "distinct times must not collapse to one stamp"


def test_round_trip_recovers_the_input() -> None:
    for seconds in (0.0, 1.5, 123.456789, 3600.25):
        sec, nanosec = seconds_to_ros_stamp(seconds)
        assert sec + nanosec / NANOSECONDS_PER_SECOND == pytest.approx(seconds)
