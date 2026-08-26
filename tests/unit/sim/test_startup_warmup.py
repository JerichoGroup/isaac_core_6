"""
Tests for the startup warm-up that fixes the intermittent Kit segfault.

Opening or creating a stage immediately after enabling ``isaacsim.ros2.bridge`` crashed
roughly half of all launches on this install, inside Kit's parallel graph executor. Pumping
frames before any stage operation fixed it: 0 crashes in 27 runs against 5 in 10 before.

These tests lock in the ordering with a fake app module, so they need no Isaac Sim. The
ordering is invisible at runtime until it regresses, at which point the symptom is a
one-in-two crash that looks like flaky hardware.
"""

from __future__ import annotations

import pytest

from isaac_core.sim.runtime import (
    EXTENSION_WARMUP_FRAMES,
    STAGE_SETTLE_FRAMES,
    SimulationRuntime,
)


class _FakeAppUtils:
    """Records update calls so a test can assert how many frames were pumped."""

    def __init__(self) -> None:
        """Start with no recorded calls."""
        self.updates = 0
        self.events: list[str] = []

    def update_app(self) -> None:
        """Count one frame."""
        self.updates += 1
        self.events.append("update")

    def enable_extension(self, name: str) -> bool:
        """Record an extension being enabled and report success."""
        self.events.append(f"enable:{name}")
        return True


class _BareRuntime(SimulationRuntime):
    """A runtime with the Isaac-dependent constructor bypassed."""

    def __init__(self) -> None:
        """Initialise nothing; each test sets only what it needs."""


@pytest.fixture
def runtime() -> _BareRuntime:
    """Return a runtime with no Isaac state."""
    return _BareRuntime()


def test_warmup_frame_counts_are_meaningful() -> None:
    # Zero would silently disable the fix while leaving the call site looking correct.
    assert EXTENSION_WARMUP_FRAMES > 0
    assert STAGE_SETTLE_FRAMES > 0
    # Startup cost stays trivial; these are frames, not seconds.
    assert EXTENSION_WARMUP_FRAMES <= 600
    assert STAGE_SETTLE_FRAMES <= 600


def test_pump_advances_exactly_the_requested_frames(runtime: _BareRuntime) -> None:
    app_utils = _FakeAppUtils()
    runtime._pump(app_utils, 7)
    assert app_utils.updates == 7


def test_pump_of_zero_frames_does_nothing(runtime: _BareRuntime) -> None:
    app_utils = _FakeAppUtils()
    runtime._pump(app_utils, 0)
    assert app_utils.updates == 0


def test_extensions_are_followed_by_a_warmup(monkeypatch: pytest.MonkeyPatch) -> None:
    # The whole point: no stage operation may happen until frames have been pumped.
    from isaac_core.config import IsaacCoreConfig

    app_utils = _FakeAppUtils()
    runtime = _BareRuntime()
    runtime._config = IsaacCoreConfig()

    def fake_import(name: str) -> "_FakeAppUtils":
        assert name == "isaacsim.core.experimental.utils.app"
        return app_utils

    monkeypatch.setattr("isaac_core.sim.runtime.importlib.import_module", fake_import)
    runtime._enable_required_extensions()

    enables = [e for e in app_utils.events if e.startswith("enable:")]
    assert enables, "no extensions were enabled"

    # Every enable must come before every update, and the updates must be the warm-up.
    assert "update" in app_utils.events, "no warm-up frames were pumped after enabling extensions"
    first_update = app_utils.events.index("update")
    last_enable = max(index for index, event in enumerate(app_utils.events) if event.startswith("enable:"))
    assert last_enable < first_update, "warm-up must come after all extensions are enabled"
    assert app_utils.updates == EXTENSION_WARMUP_FRAMES


def test_the_bridge_is_among_the_default_extensions() -> None:
    # The bridge is what needs the warm-up, and it is required for ROS publishing. If it
    # ever leaves the defaults, this test should be revisited rather than deleted.
    from isaac_core.config import IsaacCoreConfig

    assert "isaacsim.ros2.bridge" in IsaacCoreConfig().sim.extensions
