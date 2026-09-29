"""The per-frame zoom loop: the part that actually moves the lens.

`contracts/test_zoom.py` covers the maths and `sim/test_zoom_handlers.py` covers what `set_zoom` reports.
Neither runs `_step_zoom`, which is the code that converts a field of view into a focal length, writes
`focalLength` to the camera prim, and decides when the move is finished. So the whole write path had no
automated test: it was verified once by hand against a live prim and never again.

Worse, `tests/unit/config/test_every_key_is_wired.py` excused `camera.zoom_max_rate_deg_s` from its
anti-inertness guard by citing `contracts/test_zoom.py` -- a file with no reference to the runtime at all.
The exemption was satisfied by a citation rather than by coverage, which is the same failure as a feature
matrix row citing evidence that does not exercise it.

The runtime is driven directly with a fake stage, so these run without Isaac.
"""

from __future__ import annotations

import sys
import types
from typing import Any

import pytest

from isaac_core.config import load
from isaac_core.contracts.zoom import hfov_deg_for_focal
from isaac_core.sim.configurator import resolve_horizontal_aperture
from isaac_core.sim.planner import FeaturePlan
from isaac_core.sim.runtime import SimulationRuntime

WIDE_MM = 20.0
LONG_MM = 200.0
CAMERA_PRIM = "/World/Environment/drone_0/Xform/main_camera_01"


class FakeAttribute:
    """A USD attribute that records what was written to it."""

    def __init__(self, name: str) -> None:
        """Remember the attribute's name and start with no value."""
        self.name = name
        self.value: float | None = None
        self.writes: list[float] = []

    def IsValid(self) -> bool:
        """Report the attribute as present."""
        return True

    def Set(self, value: float) -> None:
        """Record a write."""
        self.value = float(value)
        self.writes.append(float(value))


class FakePrim:
    """A USD prim exposing recording attributes."""

    def __init__(self, path: str) -> None:
        """Create a prim with no attributes yet."""
        self.path = path
        self.attributes: dict[str, FakeAttribute] = {}

    def IsValid(self) -> bool:
        """Report the prim as present."""
        return True

    def GetAttribute(self, name: str) -> FakeAttribute:
        """Return a recording attribute, creating it on first ask."""
        return self.attributes.setdefault(name, FakeAttribute(name))


class FakeStage:
    """Just enough stage for the zoom loop: prim lookup by path."""

    def __init__(self) -> None:
        """Start with a single camera prim."""
        self.prims: dict[str, FakePrim] = {CAMERA_PRIM: FakePrim(CAMERA_PRIM)}

    def GetPrimAtPath(self, path: Any) -> FakePrim:
        """Return the prim at a path, creating a valid stand-in if absent."""
        key = str(path)
        return self.prims.setdefault(key, FakePrim(key))


@pytest.fixture(autouse=True)
def _stub_pxr_sdf() -> Any:
    """Provide the one piece of `pxr` the attribute write needs.

    `_write_float_attribute` resolves a prim through `pxr.Sdf.Path`, which exists only inside Isaac. The
    write path is otherwise plain Python, so stubbing that single symbol lets the loop under test run here
    rather than leaving it untestable outside a simulator.

    Yields:
        None; the stub is removed afterwards.

    """
    created = "pxr" not in sys.modules
    pxr = sys.modules.get("pxr") or types.ModuleType("pxr")
    sdf = types.ModuleType("pxr.Sdf")
    sdf.Path = str  # type: ignore[attr-defined]
    previous_sdf = getattr(pxr, "Sdf", None)
    pxr.Sdf = sdf  # type: ignore[attr-defined]
    sys.modules["pxr"] = pxr
    sys.modules["pxr.Sdf"] = sdf
    try:
        yield
    finally:
        sys.modules.pop("pxr.Sdf", None)
        if created:
            sys.modules.pop("pxr", None)
        elif previous_sdf is not None:
            pxr.Sdf = previous_sdf  # type: ignore[attr-defined]


def _runtime(rate_deg_s: float = 0.0, focal_mm: float = 50.0) -> SimulationRuntime:
    """Return a runtime wired to a fake stage, with zoom travel configured.

    Args:
        rate_deg_s: The camera's zoom slew limit. Zero means snap.
        focal_mm: The camera's composed focal length, where an uncommanded zoom sits.

    Returns:
        A runtime whose `_stage()` and camera prim resolution are faked.

    """
    config = load(
        cli_overrides={
            "vehicles.drone_0.camera.focal_length_min_mm": str(WIDE_MM),
            "vehicles.drone_0.camera.focal_length_max_mm": str(LONG_MM),
            "vehicles.drone_0.camera.zoom_max_rate_deg_s": str(rate_deg_s),
            "vehicles.drone_0.camera.focal_length_mm": str(focal_mm),
        }
    )
    runtime = SimulationRuntime(config=config, plan=FeaturePlan())
    stage = FakeStage()
    runtime._stage = lambda: stage  # type: ignore[method-assign]
    # Resolving the camera prim needs a real UsdGeom traversal, which is Isaac-only.
    runtime._camera_prim_path = lambda vehicle_id: CAMERA_PRIM  # type: ignore[method-assign]
    runtime._fake_stage = stage  # type: ignore[attr-defined]
    return runtime


def _written(runtime: SimulationRuntime) -> list[float]:
    """Return every focalLength value the loop wrote, in order."""
    stage: FakeStage = runtime._fake_stage  # type: ignore[attr-defined]
    return stage.prims[CAMERA_PRIM].attributes.get("focalLength", FakeAttribute("focalLength")).writes


def _aperture(runtime: SimulationRuntime) -> float:
    """Return the horizontal aperture the runtime resolves for the camera."""
    return resolve_horizontal_aperture(runtime._config, "drone_0")


# -- the loop does nothing until asked ------------------------------------------- #


def test_an_uncommanded_zoom_writes_nothing() -> None:
    # A run that never zooms must leave the composed focal length exactly as authored.
    runtime = _runtime()
    for _ in range(10):
        runtime._step_zoom()
    assert _written(runtime) == []


# -- a snap zoom arrives in one frame ------------------------------------------- #


def test_an_unlimited_zoom_writes_the_target_focal_length() -> None:
    # The write path itself: level 1.0 must put 200 mm on the prim, not merely report it.
    runtime = _runtime(rate_deg_s=0.0)
    runtime._handle_set_zoom({"level": 1.0})
    runtime._step_zoom()
    writes = _written(runtime)
    assert writes, "the loop wrote no focalLength at all"
    assert writes[-1] == pytest.approx(LONG_MM, rel=1e-6)


def test_level_zero_writes_the_widest_lens() -> None:
    runtime = _runtime(rate_deg_s=0.0)
    runtime._handle_set_zoom({"level": 0.0})
    runtime._step_zoom()
    assert _written(runtime)[-1] == pytest.approx(WIDE_MM, rel=1e-6)


def test_the_midpoint_write_is_halfway_in_field_of_view() -> None:
    # Guards the choice the whole feature rests on, measured at the prim rather than in the reply: a
    # focal-length interpolation would write about 110 mm instead.
    runtime = _runtime(rate_deg_s=0.0)
    runtime._handle_set_zoom({"level": 0.5})
    runtime._step_zoom()
    aperture = _aperture(runtime)
    written_hfov = hfov_deg_for_focal(_written(runtime)[-1], aperture)
    widest = hfov_deg_for_focal(WIDE_MM, aperture)
    narrowest = hfov_deg_for_focal(LONG_MM, aperture)
    assert written_hfov == pytest.approx((widest + narrowest) / 2.0, rel=1e-6)
    assert _written(runtime)[-1] < 60.0, "this looks like focal-length interpolation, not field of view"


def test_an_arrived_zoom_stops_being_a_target() -> None:
    # Otherwise the loop rewrites the same value every frame for the rest of the run.
    runtime = _runtime(rate_deg_s=0.0)
    runtime._handle_set_zoom({"level": 1.0})
    runtime._step_zoom()
    assert runtime._handle_get_zoom(None)["moving"] is False
    count = len(_written(runtime))
    for _ in range(5):
        runtime._step_zoom()
    assert len(_written(runtime)) == count, "the loop kept writing after arriving"


# -- a rate-limited zoom is a move, not a jump ---------------------------------- #


def test_a_rate_limited_zoom_takes_several_frames() -> None:
    # The behaviour `zoom_max_rate_deg_s` exists for, and the reason the config guard's exemption was
    # wrong: nothing exercised this before.
    runtime = _runtime(rate_deg_s=30.0)
    runtime._handle_set_zoom({"level": 1.0})
    runtime._step_zoom()
    first = _written(runtime)[-1]
    assert first < LONG_MM, "the lens snapped to the target despite a rate limit"
    assert first > WIDE_MM, "the lens did not move at all"


def test_a_rate_limited_zoom_moves_monotonically_toward_the_target() -> None:
    # A path assertion, not an endpoint one: this is where an interpolation bug would hide.
    runtime = _runtime(rate_deg_s=30.0)
    runtime._handle_set_zoom({"level": 1.0})
    for _ in range(400):
        runtime._step_zoom()
    writes = _written(runtime)
    assert len(writes) > 3, f"expected a multi-frame ramp, got {len(writes)} writes"
    assert writes == sorted(writes), "the focal length did not advance monotonically"
    assert writes[-1] == pytest.approx(LONG_MM, rel=1e-4), "the ramp never reached the target"


def test_a_rate_limited_zoom_settles_and_stops() -> None:
    runtime = _runtime(rate_deg_s=30.0)
    runtime._handle_set_zoom({"level": 1.0})
    for _ in range(400):
        runtime._step_zoom()
    assert runtime._handle_get_zoom(None)["moving"] is False
    settled = len(_written(runtime))
    for _ in range(10):
        runtime._step_zoom()
    assert len(_written(runtime)) == settled, "the loop never stopped writing"


def test_zooming_out_ramps_the_other_way() -> None:
    runtime = _runtime(rate_deg_s=30.0, focal_mm=LONG_MM)
    runtime._handle_set_zoom({"level": 0.0})
    for _ in range(400):
        runtime._step_zoom()
    writes = _written(runtime)
    assert writes == sorted(writes, reverse=True), "zooming out did not shorten the lens monotonically"
    assert writes[-1] == pytest.approx(WIDE_MM, rel=1e-4)


# -- what it writes, and where --------------------------------------------------- #


def test_only_focal_length_is_written() -> None:
    # The aperture must be held: moving it would change the field of view by moving the sensor, and would
    # invalidate every intrinsic derived from it.
    runtime = _runtime(rate_deg_s=0.0)
    runtime._handle_set_zoom({"level": 0.5})
    runtime._step_zoom()
    stage: FakeStage = runtime._fake_stage  # type: ignore[attr-defined]
    touched = {name for name, attr in stage.prims[CAMERA_PRIM].attributes.items() if attr.writes}
    assert touched == {"focalLength"}, f"the loop also wrote {touched - {'focalLength'}}"


def test_a_camera_without_travel_is_dropped_rather_than_looping() -> None:
    # A target left in place for a camera that cannot zoom would be retried every frame forever.
    runtime = _runtime()
    runtime._zoom_targets["drone_0"] = 45.0
    runtime._config.vehicles["drone_0"].camera.__dict__["focal_length_min_mm"] = None
    runtime._step_zoom()
    assert "drone_0" not in runtime._zoom_targets
