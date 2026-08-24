"""
Contract tests binding ``config/default.toml`` to the schema.

``config/default.toml`` is the team's reference for the whole configuration
surface, so it has to stay a *valid* config. Without this test it could drift into
documenting keys the schema rejects, or values the schema would refuse, and nobody
would find out until a run failed.
"""

from pathlib import Path
import sys
from typing import Any

from isaac_core.config import IsaacCoreConfig

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = REPO_ROOT / "config" / "default.toml"


def _load_default_toml() -> dict[str, Any]:
    """Parse the shipped default config file."""
    with DEFAULT_CONFIG.open("rb") as handle:
        data: dict[str, Any] = tomllib.load(handle)
    return data


def test_default_config_file_exists() -> None:
    assert DEFAULT_CONFIG.is_file(), f"missing {DEFAULT_CONFIG}"


def test_default_config_is_valid_toml() -> None:
    assert isinstance(_load_default_toml(), dict)


def test_default_config_validates_against_the_schema() -> None:
    # The load must not raise: every key documented in default.toml has to exist
    # in the schema, and every value has to satisfy its constraints.
    config = IsaacCoreConfig(**_load_default_toml())
    assert isinstance(config, IsaacCoreConfig)


def test_default_config_describes_a_working_single_vehicle_setup() -> None:
    config = IsaacCoreConfig(**_load_default_toml())
    assert config.is_single_vehicle
    assert list(config.vehicles) == ["drone_0"]
    assert list(config.vehicles["drone_0"].cameras) == ["eo"]
    # Single vehicle, single camera collapses to the flat legacy topic names.
    assert config.topic_resolver("drone_0", "eo").resolve("image_rgb") == "/isaac_core/image_rgb"


def test_default_config_ships_no_hardcoded_tile_server() -> None:
    # The previous generation committed an internal IP to source. Never again.
    config = IsaacCoreConfig(**_load_default_toml())
    assert config.cesium.tileset_server_url is None


def test_default_config_control_plane_is_loopback_only() -> None:
    config = IsaacCoreConfig(**_load_default_toml())
    assert config.sim.control_plane.host == "127.0.0.1"
    assert config.sim.control_plane.token == ""


def test_default_config_contains_no_routable_ip_literals() -> None:
    # Guards against a deployment-specific address creeping back into defaults.
    text = DEFAULT_CONFIG.read_text(encoding="utf-8")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#") or "=" not in stripped:
            continue
        assert (
            "10." not in stripped.split("=", 1)[1] or "127.0.0.1" in stripped
        ), f"possible hardcoded address in default.toml: {stripped!r}"


def test_documented_layer_sections_use_the_uniform_layers_namespace() -> None:
    # D13: every layer's settings live under [layers.<id>], which is what lets a
    # third-party layer extend the config with no change to isaac_core.
    raw = _load_default_toml()
    assert "layers" in raw
    for layer_id, settings in raw["layers"].items():
        assert isinstance(layer_id, str)
        assert isinstance(settings, dict)
