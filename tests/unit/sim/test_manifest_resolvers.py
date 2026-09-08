"""Guard that every resolver a shipped manifest asks for actually exists."""

import re

import pytest

from isaac_core.assets import LAYERS_DIR
from isaac_core.config import load
from isaac_core.sim.configurator import ConfigKeyError, _resolve_runtime_value
from isaac_core.sim.georeference import ResolvedEnuReference

# Every distinct `resolve = "name"` across the shipped layer manifests.
REQUESTED = sorted(
    {
        match
        for path in LAYERS_DIR.glob("*/layer.toml")
        for match in re.findall(r'resolve\s*=\s*"([a-z_]+)"', path.read_text(encoding="utf-8"))
    }
)


def test_some_resolvers_are_requested() -> None:
    """Fail if the scan finds nothing, which would make the guard below vacuous."""
    assert REQUESTED, f"no resolve names found under {LAYERS_DIR}"


@pytest.mark.parametrize("name", REQUESTED)
def test_every_requested_resolver_exists(name: str) -> None:
    # This is the guard that was missing. A manifest referencing a resolver that does not exist
    # raises ConfigKeyError at COMPOSE time, i.e. only when Isaac Sim is actually running -- so it
    # sailed through the whole unit suite and broke every scenario on the user's machine instead.
    # Renaming or forgetting to register a resolver is now caught here, with no Isaac needed.
    config = load()
    enu = ResolvedEnuReference(reference=config.geo.enu_reference, source="config")
    vehicle_id = next(iter(config.vehicles))
    camera_id = next(iter(config.vehicles[vehicle_id].cameras))
    try:
        _resolve_runtime_value(
            name,
            config=config,
            enu_reference=enu,
            camera_prim="/World/Environment/drone_0/Xform/main_camera_01",
            vehicle_id=vehicle_id,
            camera_id=camera_id,
        )
    except ConfigKeyError as exc:  # pragma: no cover - only on a real regression
        pytest.fail(f"manifest asks for resolver {name!r} but it is not registered: {exc}")
