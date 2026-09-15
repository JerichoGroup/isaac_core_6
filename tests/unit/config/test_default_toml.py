"""Contract tests binding ``config/default.toml`` to the schema.

``config/default.toml`` is the team's reference for the whole configuration
surface, so it has to stay a *valid* config. Without this test it could drift into
documenting keys the schema rejects, or values the schema would refuse, and nobody
would find out until a run failed.
"""

from pathlib import Path
import sys
from typing import Any, Final

from isaac_core.config import IsaacCoreConfig, load as load_config

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
    # D13: a layer's own settings live under [layers.<id>], which is what lets a third-party layer
    # extend the config with no change to isaac_core. This test used to REQUIRE example sections to
    # be present, which kept three misleading ones alive: nothing in src reads config.layers, and
    # [layers.distance_sensor] max_range_m = 180.0 contradicted the real per-vehicle key
    # (5000.0), whose docstring explains that a 180 m ray never reaches the ground. So the shape is
    # asserted, not the presence -- an empty namespace is correct while no shipped layer uses it.
    raw = _load_default_toml()
    for layer_id, settings in raw.get("layers", {}).items():
        assert isinstance(layer_id, str)
        assert isinstance(settings, dict)


def test_no_documented_layer_section_names_a_layer_that_does_not_ship() -> None:
    # [layers.sat] documented a layer that never existed, and [layers.bbox_publisher] used an id
    # that is not the shipped layer's id (it is "bbox"). Both read as configurable features.
    shipped = {p.name for p in (REPO_ROOT / "src" / "isaac_core" / "assets" / "layers").iterdir() if p.is_dir()}
    documented = set(_load_default_toml().get("layers", {}))
    unknown = documented - shipped
    assert not unknown, f"config/default.toml documents [layers.*] ids that do not ship: {sorted(unknown)}"


# Keys where config/default.toml deliberately differs from the schema default, with the reason.
# default.toml is a *working starting config*, not a dump of the schema, so a few values are set
# to something usable rather than to the bare default.
_INTENTIONAL_DIVERGENCES: Final[dict[str, str]] = {
    "features.enabled": "the shipped file enables a usable set; the schema default is empty",
    "vehicles.drone_0.gimbal.max_rate_deg_s": "the shipped file demonstrates a rate limit; the schema default is unlimited",
}


def _flatten(data: dict[str, object], prefix: str = "") -> dict[str, object]:
    """Flatten a nested config dump into dotted keys."""
    flat: dict[str, object] = {}
    for key, value in data.items():
        dotted = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(_flatten(value, f"{dotted}."))
        else:
            flat[dotted] = value
    return flat


def test_default_toml_values_agree_with_schema_defaults() -> None:
    # Guards a real problem: default.toml is documentation only -- it is never auto-loaded, so
    # shipped behaviour comes from the schema. Three values had silently drifted apart, and because
    # the README tells users to start from this file, copying it CHANGED behaviour:
    # tilesets_root pointed at /tilesets instead of /World/tilesets (so tileset_server_url and every
    # tile tunable silently no-opped), delete_cache_on_launch was true (the exact setting the
    # troubleshooting section warns causes worst-case hitching), and domain_id pinned 13 instead of
    # inheriting $ROS_DOMAIN_ID. Validity alone was tested; agreement was not.
    schema_defaults = _flatten(load_config().model_dump(mode="json"))
    from_file = _flatten(load_config(path=DEFAULT_CONFIG).model_dump(mode="json"))
    divergences = {
        key: (schema_defaults.get(key), from_file.get(key))
        for key in sorted(set(schema_defaults) | set(from_file))
        if schema_defaults.get(key) != from_file.get(key)
    }
    unexplained = {k: v for k, v in divergences.items() if k not in _INTENTIONAL_DIVERGENCES}
    assert not unexplained, (
        "config/default.toml disagrees with the schema defaults for: "
        + "; ".join(f"{k}: schema={s!r} file={f!r}" for k, (s, f) in unexplained.items())
        + ". Either fix the file or add the key to _INTENTIONAL_DIVERGENCES with a reason."
    )
