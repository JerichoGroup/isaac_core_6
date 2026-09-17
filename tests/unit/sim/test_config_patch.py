"""Tests for the runtime config patch allowlist."""

import pytest

from isaac_core.sim.runtime import PATCHABLE_CONFIG_KEYS


def test_the_allowlist_only_contains_keys_with_runtime_readers() -> None:
    # The allowlist is the whole safety mechanism. A key applied once at composition time must
    # never appear here: patching it would update the config object while the stage kept the old
    # value, which is a silent lie rather than a feature.
    assert PATCHABLE_CONFIG_KEYS == frozenset({"gimbal.max_rate_deg_s"})


@pytest.mark.parametrize(
    "key",
    [
        "sim.headless",
        "sim.scene",
        "vehicles.drone_0.camera.width",
        "cesium.tileset_server_url",
        "geo.enu_reference",
    ],
)
def test_compose_time_keys_are_not_patchable(key: str) -> None:
    """Keep composition-time settings out of the patchable set."""
    assert key not in PATCHABLE_CONFIG_KEYS


def test_get_config_reports_a_patched_value_rather_than_the_startup_one() -> None:
    # Config models are frozen, so a patch is recorded beside them. Dumping the model alone meant
    # patching a key and reading it back returned the old number while the new one was in force.
    from isaac_core.config import IsaacCoreConfig
    from isaac_core.sim.runtime import SimulationRuntime

    runtime = SimulationRuntime.__new__(SimulationRuntime)
    runtime._config = IsaacCoreConfig()
    runtime._config_overrides = {}
    vehicle = runtime._config.first_vehicle_id

    unpatched = runtime._handle_get_config(None)
    assert unpatched["vehicles"][vehicle]["gimbal"]["max_rate_deg_s"] is None

    runtime._config_overrides["gimbal.max_rate_deg_s"] = 10.0
    patched = runtime._handle_get_config(None)
    assert patched["vehicles"][vehicle]["gimbal"]["max_rate_deg_s"] == 10.0
    assert runtime._gimbal_max_rate_deg_s() == 10.0, "the reported value must be the one in force"
