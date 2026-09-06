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
        "vehicles.drone_0.cameras.eo.width",
        "cesium.tileset_server_url",
        "geo.enu_reference",
    ],
)
def test_compose_time_keys_are_not_patchable(key: str) -> None:
    """Keep composition-time settings out of the patchable set."""
    assert key not in PATCHABLE_CONFIG_KEYS
