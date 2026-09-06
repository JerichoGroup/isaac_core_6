"""
Tests for range-sensor value semantics.

The boundary cases are the whole point: `sensor_msgs/Range` distinguishes "nothing detected"
from "something at max range", and a rangefinder that silently clamps loses that distinction.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.rangefinder import (
    NO_DETECTION,
    TOO_CLOSE,
    is_valid_reading,
    resolve_range,
)

# A typical rated band, matching the kind of values the 2023 config used.
_MIN = 0.2
_MAX = 100.0


@pytest.mark.parametrize("distance", [0.2, 1.0, 50.0, 99.999, 100.0])
def test_a_hit_inside_the_rated_band_is_reported_as_is(distance: float) -> None:
    assert resolve_range(True, distance, _MIN, _MAX) == pytest.approx(distance)


def test_no_hit_reports_no_detection() -> None:
    # ROS convention: +inf means "the beam went out and found nothing".
    assert resolve_range(False, 0.0, _MIN, _MAX) == NO_DETECTION
    assert math.isinf(resolve_range(False, 0.0, _MIN, _MAX))


def test_a_hit_beyond_max_range_reports_no_detection() -> None:
    # Not clamped to max: clamping would make "nothing there" indistinguishable from
    # "something exactly at max range", which is the distinction Range exists to preserve.
    assert resolve_range(True, 150.0, _MIN, _MAX) == NO_DETECTION


def test_a_hit_closer_than_min_range_reports_too_close() -> None:
    # Real hardware cannot measure inside its minimum; ROS convention is -inf.
    assert resolve_range(True, 0.05, _MIN, _MAX) == TOO_CLOSE
    assert resolve_range(True, 0.05, _MIN, _MAX) < 0


def test_the_band_edges_are_inclusive() -> None:
    assert resolve_range(True, _MIN, _MIN, _MAX) == pytest.approx(_MIN)
    assert resolve_range(True, _MAX, _MIN, _MAX) == pytest.approx(_MAX)


def test_a_non_finite_distance_is_treated_as_no_detection() -> None:
    # A physics query can hand back inf/nan; that must not propagate as a measurement.
    assert resolve_range(True, math.inf, _MIN, _MAX) == NO_DETECTION
    assert resolve_range(True, math.nan, _MIN, _MAX) == NO_DETECTION


@pytest.mark.parametrize(
    ("low", "high"),
    [(0.0, 0.0), (5.0, 5.0), (10.0, 1.0), (-1.0, 10.0)],
)
def test_a_nonsensical_rated_band_is_rejected(low: float, high: float) -> None:
    # Every reading would be meaningless, so fail loudly at the source rather than emitting
    # garbage ranges all run.
    with pytest.raises(ValueError, match="range|negative"):
        resolve_range(True, 1.0, low, high)


@pytest.mark.parametrize("bound", [math.inf, math.nan])
def test_non_finite_bounds_are_rejected(bound: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        resolve_range(True, 1.0, _MIN, bound)


def test_is_valid_reading_separates_measurements_from_sentinels() -> None:
    assert is_valid_reading(12.5)
    assert not is_valid_reading(NO_DETECTION)
    assert not is_valid_reading(TOO_CLOSE)
