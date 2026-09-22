"""The zoom control-plane handlers: validation, clamping and what they report.

`contracts/test_zoom.py` covers the maths. This covers the surface a caller touches: what happens when the
camera has no zoom travel, when both or neither parameter is given, and whether the numbers reported back
agree with each other. A `set_zoom` that returned a level and a focal length describing different lenses
would look correct in either number alone.
"""

from __future__ import annotations

from typing import Any

import pytest

from isaac_core.config import load
from isaac_core.control.errors import InvalidParamsError
from isaac_core.sim.planner import FeaturePlan
from isaac_core.sim.runtime import SimulationRuntime

# A 10x range, and the aperture the shipped camera resolves to.
WIDE_MM = 20.0
LONG_MM = 200.0


def _runtime(**overrides: Any) -> SimulationRuntime:
    """Return a runtime whose config has the given overrides, without launching Isaac.

    Args:
        **overrides: Dotted config keys.

    Returns:
        An unstarted runtime, which is enough for the pure handler paths.

    """
    config = load(cli_overrides=dict(overrides))
    # An empty plan is enough: the zoom handlers read config and stage, never the plan.
    return SimulationRuntime(config=config, plan=FeaturePlan())


def _zoomable() -> SimulationRuntime:
    """Return a runtime whose camera has zoom travel."""
    return _runtime(
        **{
            "vehicles.drone_0.camera.focal_length_min_mm": WIDE_MM,
            "vehicles.drone_0.camera.focal_length_max_mm": LONG_MM,
        }
    )


# -- a camera with no travel ----------------------------------------------------- #


def test_a_camera_without_travel_refuses_and_names_the_keys() -> None:
    # Inventing a range would silently zoom a camera the user never configured for it.
    runtime = _runtime()
    with pytest.raises(RuntimeError, match="does not zoom.*focal_length_min_mm"):
        runtime._handle_set_zoom({"level": 0.5})


def test_only_one_bound_configured_still_refuses() -> None:
    runtime = _runtime(**{"vehicles.drone_0.camera.focal_length_min_mm": WIDE_MM})
    with pytest.raises(RuntimeError, match="does not zoom"):
        runtime._handle_set_zoom({"level": 0.5})


def test_get_zoom_on_a_fixed_camera_refuses_too() -> None:
    runtime = _runtime()
    with pytest.raises(RuntimeError, match="does not zoom"):
        runtime._handle_get_zoom(None)


# -- parameter validation -------------------------------------------------------- #


def test_neither_parameter_is_rejected() -> None:
    with pytest.raises(InvalidParamsError, match="exactly one of level or focal_mm"):
        _zoomable()._handle_set_zoom({})


def test_both_parameters_are_rejected() -> None:
    # Accepting both would mean silently ignoring one, and the caller could not tell which.
    with pytest.raises(InvalidParamsError, match="exactly one of level or focal_mm"):
        _zoomable()._handle_set_zoom({"level": 0.5, "focal_mm": 50.0})


def test_explicit_nulls_count_as_absent() -> None:
    # The devkit omits a parameter by not sending it, but a hand-written client may send null.
    with pytest.raises(InvalidParamsError, match="exactly one of"):
        _zoomable()._handle_set_zoom({"level": None, "focal_mm": None})


# -- what set_zoom reports ------------------------------------------------------- #


def test_level_zero_reports_the_widest_lens() -> None:
    reported = _zoomable()._handle_set_zoom({"level": 0.0})
    assert reported["focal_length_mm"] == pytest.approx(WIDE_MM, rel=1e-6)
    assert reported["level"] == pytest.approx(0.0)


def test_level_one_reports_the_longest_lens() -> None:
    reported = _zoomable()._handle_set_zoom({"level": 1.0})
    assert reported["focal_length_mm"] == pytest.approx(LONG_MM, rel=1e-6)
    assert reported["level"] == pytest.approx(1.0)


def test_the_midpoint_is_halfway_in_field_of_view() -> None:
    runtime = _zoomable()
    wide = runtime._handle_set_zoom({"level": 0.0})["hfov_deg"]
    long_end = runtime._handle_set_zoom({"level": 1.0})["hfov_deg"]
    middle = runtime._handle_set_zoom({"level": 0.5})["hfov_deg"]
    assert middle == pytest.approx((wide + long_end) / 2.0, rel=1e-6)


def test_the_three_reported_numbers_describe_the_same_lens() -> None:
    # A level, a focal length and a field of view that disagree would each look right alone.
    from isaac_core.contracts.zoom import hfov_deg_for_focal
    from isaac_core.sim.configurator import resolve_horizontal_aperture

    runtime = _zoomable()
    reported = runtime._handle_set_zoom({"level": 0.37})
    aperture = resolve_horizontal_aperture(runtime._config, "drone_0")
    assert hfov_deg_for_focal(reported["focal_length_mm"], aperture) == pytest.approx(reported["hfov_deg"], rel=1e-5)


def test_an_exact_focal_length_reports_the_level_it_lands_on() -> None:
    reported = _zoomable()._handle_set_zoom({"focal_mm": 50.0})
    assert reported["focal_length_mm"] == pytest.approx(50.0, rel=1e-6)
    assert 0.0 < reported["level"] < 1.0


@pytest.mark.parametrize(("level", "expected_mm"), [(-3.0, WIDE_MM), (7.0, LONG_MM)])
def test_a_level_outside_the_range_is_clamped(level: float, expected_mm: float) -> None:
    reported = _zoomable()._handle_set_zoom({"level": level})
    assert reported["focal_length_mm"] == pytest.approx(expected_mm, rel=1e-6)


@pytest.mark.parametrize(("focal", "expected_mm"), [(1.0, WIDE_MM), (9999.0, LONG_MM)])
def test_a_focal_length_outside_the_range_is_clamped(focal: float, expected_mm: float) -> None:
    reported = _zoomable()._handle_set_zoom({"focal_mm": focal})
    assert reported["focal_length_mm"] == pytest.approx(expected_mm, rel=1e-6)


# -- what get_zoom reports ------------------------------------------------------- #


def test_get_zoom_before_any_command_reports_the_composed_camera() -> None:
    # Assuming an end of the range would misreport a camera that was never zoomed.
    runtime = _zoomable()
    state = runtime._handle_get_zoom(None)
    configured = runtime._config.vehicles["drone_0"].camera.focal_length_mm
    assert state["focal_length_mm"] == pytest.approx(configured, rel=1e-6)
    assert state["moving"] is False


def test_get_zoom_reports_the_configured_travel() -> None:
    state = _zoomable()._handle_get_zoom(None)
    assert state["focal_length_min_mm"] == pytest.approx(WIDE_MM)
    assert state["focal_length_max_mm"] == pytest.approx(LONG_MM)


def test_a_commanded_zoom_reports_itself_as_moving() -> None:
    runtime = _zoomable()
    runtime._handle_set_zoom({"level": 1.0})
    assert runtime._handle_get_zoom(None)["moving"] is True
