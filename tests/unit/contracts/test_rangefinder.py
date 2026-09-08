"""Tests for the rangefinder boundary semantics."""

import math

import pytest

from isaac_core.contracts.rangefinder import (
    clamp_to_band,
    is_saturated,
    is_valid_reading,
    resolve_range,
)

MIN_M = 0.2
MAX_M = 5000.0


def test_a_hit_inside_the_band_is_reported_verbatim() -> None:
    """Return the measured distance unchanged when it lies inside the rated band."""
    assert resolve_range(hit=True, distance_m=482.5, min_range_m=MIN_M, max_range_m=MAX_M) == 482.5


def test_no_hit_reports_the_rated_maximum_not_infinity() -> None:
    # Deliberately NOT +inf, which is the ROS convention. float("inf") cannot be cast to int, so a
    # consumer doing int(msg.range) crashed exactly when the sensor saw nothing -- the common case
    # for a downward rangefinder in level flight.
    result = resolve_range(hit=False, distance_m=0.0, min_range_m=MIN_M, max_range_m=MAX_M)

    assert result == MAX_M
    assert math.isfinite(result)
    assert int(result) == 5000


def test_a_hit_beyond_the_maximum_saturates_at_the_maximum() -> None:
    """Clamp a too-distant hit to the rated maximum."""
    assert resolve_range(hit=True, distance_m=9001.0, min_range_m=MIN_M, max_range_m=MAX_M) == MAX_M


def test_a_hit_nearer_than_the_minimum_saturates_at_the_minimum() -> None:
    """Clamp a too-close hit to the rated minimum rather than reporting -inf."""
    result = resolve_range(hit=True, distance_m=0.05, min_range_m=MIN_M, max_range_m=MAX_M)

    assert result == MIN_M
    assert math.isfinite(result)


def test_a_non_finite_distance_is_treated_as_nothing_seen() -> None:
    # A misbehaving query returning inf or nan must not propagate that to the wire, which is the
    # whole point of clamping.
    for bad in (math.inf, -math.inf, math.nan):
        assert resolve_range(hit=True, distance_m=bad, min_range_m=MIN_M, max_range_m=MAX_M) == MAX_M


@pytest.mark.parametrize(
    ("min_m", "max_m"),
    [(0.2, 0.2), (5.0, 1.0), (-1.0, 10.0), (0.0, math.inf), (math.nan, 10.0)],
)
def test_an_impossible_band_is_rejected(min_m: float, max_m: float) -> None:
    """Reject a band in which no reading could be meaningful."""
    with pytest.raises(ValueError, match="range|min_range|max_range"):
        resolve_range(hit=True, distance_m=1.0, min_range_m=min_m, max_range_m=max_m)


def test_every_result_is_finite_across_the_whole_input_space() -> None:
    # The guarantee consumers depend on: whatever the sensor does, the published number can be
    # cast to int without raising.
    for hit in (True, False):
        for distance in (-10.0, 0.0, 0.1, MIN_M, 1.0, MAX_M, 9001.0, math.inf, math.nan):
            result = resolve_range(hit=hit, distance_m=distance, min_range_m=MIN_M, max_range_m=MAX_M)
            assert math.isfinite(result)
            assert MIN_M <= result <= MAX_M
            int(result)


@pytest.mark.parametrize(
    ("value", "expected"),
    [(MIN_M, True), (MAX_M, True), (0.1, True), (9001.0, True), (100.0, False), (MIN_M + 0.01, False)],
)
def test_saturation_is_detectable(value: float, expected: bool) -> None:
    """Report a reading sitting on either band edge as saturated."""
    assert is_saturated(value, MIN_M, MAX_M) is expected


def test_a_mid_band_reading_is_valid_and_an_edge_reading_is_not() -> None:
    """Treat only strictly-inside readings as measurements."""
    assert is_valid_reading(1000.0, MIN_M, MAX_M)
    assert not is_valid_reading(MAX_M, MIN_M, MAX_M)
    assert not is_valid_reading(MIN_M, MIN_M, MAX_M)


def test_clamp_is_independent_of_hit_handling() -> None:
    """Clamp a value into the band without reference to whether a ray hit."""
    assert clamp_to_band(-5.0, MIN_M, MAX_M) == MIN_M
    assert clamp_to_band(6000.0, MIN_M, MAX_M) == MAX_M
    assert clamp_to_band(12.5, MIN_M, MAX_M) == 12.5
