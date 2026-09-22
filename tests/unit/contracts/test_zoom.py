"""Zoom maths.

The property that matters most is that level is linear in *field of view*, not focal length. Halfway
between a wide and a long lens should look halfway zoomed; interpolating focal length instead lands almost
fully zoomed in, which is the mistake this file exists to prevent.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.zoom import (
    ZoomRange,
    clamp_level,
    focal_for_hfov_deg,
    hfov_deg_for_focal,
    slew_hfov_deg,
)

# The shipped camera's aperture, so the numbers here are the ones a default run would produce.
APERTURE_MM = 36.83

# A 10x range, wide enough that linear-in-FOV and linear-in-focal disagree obviously.
WIDE_FOCAL_MM = 20.0
LONG_FOCAL_MM = 200.0


def _range() -> ZoomRange:
    """Return the reference zoom range."""
    return ZoomRange(
        focal_min_mm=WIDE_FOCAL_MM,
        focal_max_mm=LONG_FOCAL_MM,
        horizontal_aperture_mm=APERTURE_MM,
    )


# -- the lens equation, both directions ----------------------------------------- #


def test_the_field_of_view_follows_the_lens_equation() -> None:
    # Hand-computed: 2*atan(36.83 / (2*20)) = 85.2746 degrees.
    assert hfov_deg_for_focal(20.0, 36.83) == pytest.approx(85.2746, abs=0.001)


def test_a_longer_lens_sees_less() -> None:
    assert hfov_deg_for_focal(200.0, APERTURE_MM) < hfov_deg_for_focal(20.0, APERTURE_MM)


def test_focal_and_field_of_view_round_trip() -> None:
    for focal in (12.0, 20.0, 50.0, 135.0, 400.0):
        hfov = hfov_deg_for_focal(focal, APERTURE_MM)
        assert focal_for_hfov_deg(hfov, APERTURE_MM) == pytest.approx(focal, rel=1e-9)


@pytest.mark.parametrize("focal", [0.0, -1.0])
def test_a_nonpositive_focal_length_is_rejected(focal: float) -> None:
    with pytest.raises(ValueError, match="focal_length_mm must be positive"):
        hfov_deg_for_focal(focal, APERTURE_MM)


@pytest.mark.parametrize("aperture", [0.0, -5.0])
def test_a_nonpositive_aperture_is_rejected(aperture: float) -> None:
    with pytest.raises(ValueError, match="horizontal_aperture_mm must be positive"):
        hfov_deg_for_focal(20.0, aperture)


@pytest.mark.parametrize("hfov", [0.0, -10.0, 180.0, 200.0])
def test_an_impossible_field_of_view_is_rejected(hfov: float) -> None:
    # The tangent blows up approaching a half turn, so this would return a nonsense focal length.
    with pytest.raises(ValueError, match="hfov_deg must be in"):
        focal_for_hfov_deg(hfov, APERTURE_MM)


# -- the range and its ends ----------------------------------------------------- #


def test_level_zero_is_the_widest_view() -> None:
    zoom = _range()
    assert zoom.focal_mm_at(0.0) == pytest.approx(WIDE_FOCAL_MM, rel=1e-9)
    assert zoom.hfov_deg_at(0.0) == pytest.approx(zoom.widest_hfov_deg, rel=1e-9)


def test_level_one_is_the_narrowest_view() -> None:
    zoom = _range()
    assert zoom.focal_mm_at(1.0) == pytest.approx(LONG_FOCAL_MM, rel=1e-9)
    assert zoom.hfov_deg_at(1.0) == pytest.approx(zoom.narrowest_hfov_deg, rel=1e-9)


def test_zooming_in_narrows_the_view_monotonically() -> None:
    zoom = _range()
    views = [zoom.hfov_deg_at(level / 10.0) for level in range(11)]
    assert views == sorted(views, reverse=True), views


def test_halfway_is_halfway_in_field_of_view_not_focal_length() -> None:
    # The whole point. Linear in FOV puts the midpoint near 45 degrees; linear in focal length would put
    # it at 110 mm, which is about 19 degrees -- nearly fully zoomed in, and not what "halfway" means.
    zoom = _range()
    midpoint_hfov = zoom.hfov_deg_at(0.5)
    expected = (zoom.widest_hfov_deg + zoom.narrowest_hfov_deg) / 2.0
    assert midpoint_hfov == pytest.approx(expected, rel=1e-9)

    linear_in_focal_hfov = hfov_deg_for_focal((WIDE_FOCAL_MM + LONG_FOCAL_MM) / 2.0, APERTURE_MM)
    assert (
        midpoint_hfov > linear_in_focal_hfov + 20.0
    ), "the midpoint is indistinguishable from interpolating focal length, so the choice is not implemented"


def test_a_level_outside_the_range_is_clamped_rather_than_rejected() -> None:
    # A zoom control that errored at the end of its own travel would be surprising.
    zoom = _range()
    assert zoom.focal_mm_at(-5.0) == pytest.approx(WIDE_FOCAL_MM, rel=1e-9)
    assert zoom.focal_mm_at(9.0) == pytest.approx(LONG_FOCAL_MM, rel=1e-9)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_a_nonfinite_level_is_rejected(bad: float) -> None:
    # Clamping NaN would silently produce NaN, which writes a broken focal length to the prim.
    with pytest.raises(ValueError, match="must be finite"):
        clamp_level(bad)


def test_level_and_focal_length_are_inverses() -> None:
    zoom = _range()
    for level in (0.0, 0.15, 0.5, 0.83, 1.0):
        focal = zoom.focal_mm_at(level)
        assert zoom.level_for_focal_mm(focal) == pytest.approx(level, abs=1e-9)


def test_a_focal_length_beyond_the_range_maps_to_an_end() -> None:
    zoom = _range()
    assert zoom.level_for_focal_mm(1.0) == pytest.approx(0.0)
    assert zoom.level_for_focal_mm(5000.0) == pytest.approx(1.0)


def test_a_focal_length_is_clamped_into_the_range() -> None:
    zoom = _range()
    assert zoom.clamp_focal_mm(1.0) == pytest.approx(WIDE_FOCAL_MM)
    assert zoom.clamp_focal_mm(5000.0) == pytest.approx(LONG_FOCAL_MM)
    assert zoom.clamp_focal_mm(50.0) == pytest.approx(50.0)


def test_a_fixed_lens_is_a_valid_range() -> None:
    # min == max is a prime lens, not an error, and asking for its level must not divide by zero.
    zoom = ZoomRange(focal_min_mm=50.0, focal_max_mm=50.0, horizontal_aperture_mm=APERTURE_MM)
    assert zoom.focal_mm_at(0.0) == pytest.approx(50.0)
    assert zoom.focal_mm_at(1.0) == pytest.approx(50.0)
    assert zoom.level_for_focal_mm(50.0) == pytest.approx(0.0)


def test_a_reversed_range_is_rejected_naming_both_keys() -> None:
    with pytest.raises(ValueError, match="focal_length_min_mm.*must not exceed.*focal_length_max_mm"):
        ZoomRange(focal_min_mm=200.0, focal_max_mm=20.0, horizontal_aperture_mm=APERTURE_MM)


@pytest.mark.parametrize(
    ("kwargs", "name"),
    [
        ({"focal_min_mm": 0.0, "focal_max_mm": 50.0}, "focal_min_mm"),
        ({"focal_min_mm": 20.0, "focal_max_mm": -1.0}, "focal_max_mm"),
    ],
)
def test_a_nonpositive_bound_is_rejected(kwargs: dict[str, float], name: str) -> None:
    with pytest.raises(ValueError, match=f"{name} must be positive"):
        ZoomRange(horizontal_aperture_mm=APERTURE_MM, **kwargs)


def test_a_nonpositive_aperture_in_the_range_is_rejected() -> None:
    with pytest.raises(ValueError, match="horizontal_aperture_mm must be positive"):
        ZoomRange(focal_min_mm=20.0, focal_max_mm=200.0, horizontal_aperture_mm=0.0)


# -- rate limiting -------------------------------------------------------------- #


def test_no_rate_limit_snaps_to_the_target() -> None:
    # Matches how the gimbal reads its own limit: unset means instant, not frozen.
    assert slew_hfov_deg(80.0, 20.0, 0.0, 0.1) == pytest.approx(20.0)
    assert slew_hfov_deg(80.0, 20.0, -1.0, 0.1) == pytest.approx(20.0)


def test_a_rate_limit_makes_the_zoom_a_move_rather_than_a_jump() -> None:
    assert slew_hfov_deg(80.0, 20.0, 30.0, 0.1) == pytest.approx(77.0)


def test_the_limit_applies_in_both_directions() -> None:
    assert slew_hfov_deg(20.0, 80.0, 30.0, 0.1) == pytest.approx(23.0)


def test_the_target_is_not_overshot() -> None:
    # Overshooting then correcting would show as a visible wobble at the end of every zoom.
    assert slew_hfov_deg(20.5, 20.0, 30.0, 1.0) == pytest.approx(20.0)
    assert slew_hfov_deg(19.5, 20.0, 30.0, 1.0) == pytest.approx(20.0)


def test_a_nonpositive_interval_holds_position() -> None:
    assert slew_hfov_deg(50.0, 20.0, 30.0, 0.0) == pytest.approx(50.0)
    assert slew_hfov_deg(50.0, 20.0, 30.0, -0.1) == pytest.approx(50.0)


def test_a_limited_zoom_arrives_and_stays() -> None:
    # Walk it like the loop does, then confirm it settles instead of oscillating.
    current = 85.0
    for _ in range(400):
        current = slew_hfov_deg(current, 10.0, 30.0, 1.0 / 60.0)
    assert current == pytest.approx(10.0)
    assert slew_hfov_deg(current, 10.0, 30.0, 1.0 / 60.0) == pytest.approx(10.0)
