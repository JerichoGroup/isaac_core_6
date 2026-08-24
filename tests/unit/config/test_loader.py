"""Tests for the configuration loader and merge machinery."""

from pathlib import Path
from typing import Any

from pydantic import ValidationError
import pytest

from isaac_core.config.loader import deep_merge, dump_toml, load, load_with_provenance
from isaac_core.config.schema import IsaacCoreConfig

# --------------------------------------------------------------------------- #
# deep_merge — basics
# --------------------------------------------------------------------------- #


def test_deep_merge_empty_dicts() -> None:
    assert deep_merge({}, {}) == {}


def test_deep_merge_override_adds_keys() -> None:
    result = deep_merge({"a": 1}, {"b": 2})
    assert result == {"a": 1, "b": 2}


def test_deep_merge_override_wins_on_conflict() -> None:
    result = deep_merge({"a": 1}, {"a": 2})
    assert result == {"a": 2}


def test_deep_merge_nested_dicts_are_merged_recursively() -> None:
    base = {"sim": {"headless": False, "scene": "earth"}}
    override = {"sim": {"headless": True}}
    result = deep_merge(base, override)
    assert result == {"sim": {"headless": True, "scene": "earth"}}


def test_deep_merge_deeply_nested() -> None:
    base = {"a": {"b": {"c": 1, "d": 2}}}
    override = {"a": {"b": {"c": 99}}}
    result = deep_merge(base, override)
    assert result == {"a": {"b": {"c": 99, "d": 2}}}


# --------------------------------------------------------------------------- #
# deep_merge — lists replace rather than concatenate
# --------------------------------------------------------------------------- #


def test_deep_merge_lists_replace_wholesale() -> None:
    # Appending would make it impossible to shorten a list.
    base = {"features": {"enabled": ["sensor", "bbox"]}}
    override = {"features": {"enabled": ["sensor"]}}
    result = deep_merge(base, override)
    assert result["features"]["enabled"] == ["sensor"]


def test_deep_merge_tuple_replaced_by_list() -> None:
    base = {"resolution": (1280, 720)}
    override = {"resolution": [3840, 2160]}
    result = deep_merge(base, override)
    assert result["resolution"] == [3840, 2160]


# --------------------------------------------------------------------------- #
# deep_merge — immutability
# --------------------------------------------------------------------------- #


def test_deep_merge_does_not_mutate_base() -> None:
    base = {"sim": {"headless": False, "scene": "earth"}}
    override = {"sim": {"headless": True}}
    base_copy = {"sim": {"headless": False, "scene": "earth"}}
    deep_merge(base, override)
    assert base == base_copy


def test_deep_merge_does_not_mutate_override() -> None:
    base = {"sim": {"headless": False}}
    override = {"sim": {"headless": True, "extra": "val"}}
    override_copy = {"sim": {"headless": True, "extra": "val"}}
    deep_merge(base, override)
    assert override == override_copy


def test_deep_merge_result_is_independent_of_inputs() -> None:
    base: dict[str, Any] = {"nested": {"key": "original"}}
    override: dict[str, Any] = {"other": "value"}
    result = deep_merge(base, override)
    result["nested"]["key"] = "modified"
    assert base["nested"]["key"] == "original"


# --------------------------------------------------------------------------- #
# load — TOML file path
# --------------------------------------------------------------------------- #


def test_load_from_explicit_path(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = true\n")
    config = load(path=config_file, environ={})
    assert config.sim.headless is True


def test_load_from_isaac_core_config_env_var(tmp_path: Path) -> None:
    config_file = tmp_path / "env.toml"
    config_file.write_text('[logging]\nlevel = "debug"\n')
    environ = {"ISAAC_CORE_CONFIG": str(config_file)}
    config = load(environ=environ)
    assert config.logging.level == "debug"


def test_explicit_path_overrides_env_var(tmp_path: Path) -> None:
    # `path` takes precedence over $ISAAC_CORE_CONFIG.
    env_file = tmp_path / "env.toml"
    env_file.write_text('[logging]\nlevel = "debug"\n')
    path_file = tmp_path / "explicit.toml"
    path_file.write_text('[logging]\nlevel = "warning"\n')
    environ = {"ISAAC_CORE_CONFIG": str(env_file)}
    config = load(path=path_file, environ=environ)
    assert config.logging.level == "warning"


def test_nonexistent_config_file_produces_clear_error(tmp_path: Path) -> None:
    missing = tmp_path / "does_not_exist.toml"
    with pytest.raises(FileNotFoundError, match="does_not_exist.toml"):
        load(path=missing, environ={})


def test_malformed_toml_produces_clear_error(tmp_path: Path) -> None:
    bad = tmp_path / "bad.toml"
    bad.write_text("[broken\n")
    with pytest.raises(ValueError, match="malformed TOML"):
        load(path=bad, environ={})


# --------------------------------------------------------------------------- #
# load — precedence: later wins
# --------------------------------------------------------------------------- #


def test_env_overrides_file(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = false\n")
    environ = {"ISAAC_CORE__SIM__HEADLESS": "true"}
    config = load(path=config_file, environ=environ)
    assert config.sim.headless is True


def test_cli_overrides_env(tmp_path: Path) -> None:
    environ = {"ISAAC_CORE__LOGGING__LEVEL": "debug"}
    config = load(environ=environ, cli_overrides={"logging.level": "error"})
    assert config.logging.level == "error"


def test_cli_overrides_file(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text('[logging]\nlevel = "info"\n')
    config = load(path=config_file, environ={}, cli_overrides={"logging.level": "warning"})
    assert config.logging.level == "warning"


def test_all_four_layers_at_once(tmp_path: Path) -> None:
    # Layer 1 (pydantic default) sets logging.level to "info".
    # Layer 2 (file) sets sim.headless=true and logging.level="debug".
    # Layer 3 (env) sets logging.level="warning".
    # Layer 4 (cli) sets logging.level="error".
    config_file = tmp_path / "test.toml"
    config_file.write_text('[sim]\nheadless = true\n[logging]\nlevel = "debug"\n')
    environ = {"ISAAC_CORE__LOGGING__LEVEL": "warning"}
    config = load(path=config_file, environ=environ, cli_overrides={"logging.level": "error"})
    # CLI wins over all.
    assert config.logging.level == "error"
    # File wins over defaults.
    assert config.sim.headless is True


def test_file_overrides_pydantic_defaults(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = true\n")
    config = load(path=config_file, environ={})
    # Pydantic default is False; file overrides it.
    assert config.sim.headless is True


# --------------------------------------------------------------------------- #
# load — validation error path
# --------------------------------------------------------------------------- #


def test_env_var_for_nonexistent_schema_key_produces_validation_error() -> None:
    # An env var that targets a key not in the schema must error, not be ignored.
    environ = {"ISAAC_CORE__SIM__TOTALLY_FAKE_KEY": "something"}
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        load(environ=environ)


def test_invalid_value_type_from_cli_produces_validation_error() -> None:
    with pytest.raises(ValidationError):
        load(environ={}, cli_overrides={"sim.headless": "not_a_bool"})


# --------------------------------------------------------------------------- #
# load — no file, no env, no cli gives defaults
# --------------------------------------------------------------------------- #


def test_load_with_nothing_gives_pydantic_defaults() -> None:
    config = load(environ={})
    expected = IsaacCoreConfig()
    assert config == expected


# --------------------------------------------------------------------------- #
# provenance
# --------------------------------------------------------------------------- #


def test_provenance_attributes_file_source(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = true\n")
    _, provenance = load_with_provenance(path=config_file, environ={})
    assert provenance["sim.headless"] == f"file:{config_file}"


def test_provenance_attributes_env_source() -> None:
    environ = {"ISAAC_CORE__LOGGING__LEVEL": "debug"}
    _, provenance = load_with_provenance(environ=environ)
    assert provenance["logging.level"] == "env:ISAAC_CORE__LOGGING__LEVEL"


def test_provenance_attributes_cli_source() -> None:
    _, provenance = load_with_provenance(environ={}, cli_overrides={"logging.level": "error"})
    assert provenance["logging.level"] == "cli"


def test_provenance_last_source_wins(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text('[logging]\nlevel = "debug"\n')
    environ = {"ISAAC_CORE__LOGGING__LEVEL": "warning"}
    _, provenance = load_with_provenance(path=config_file, environ=environ, cli_overrides={"logging.level": "error"})
    # CLI wins over env wins over file.
    assert provenance["logging.level"] == "cli"


def test_provenance_different_keys_from_different_sources(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = true\n")
    environ = {"ISAAC_CORE__LOGGING__LEVEL": "debug"}
    _, provenance = load_with_provenance(
        path=config_file, environ=environ, cli_overrides={"geo.enu_reference.alt_m": 100.0}
    )
    assert provenance["sim.headless"] == f"file:{config_file}"
    assert provenance["logging.level"] == "env:ISAAC_CORE__LOGGING__LEVEL"
    assert provenance["geo.enu_reference.alt_m"] == "cli"


def test_provenance_defaults_not_tracked() -> None:
    # Keys that came from pydantic defaults are not tracked in provenance.
    _, provenance = load_with_provenance(environ={})
    assert "sim.headless" not in provenance


# --------------------------------------------------------------------------- #
# dump_toml
# --------------------------------------------------------------------------- #


def test_dump_toml_returns_string() -> None:
    config = IsaacCoreConfig()
    result = dump_toml(config)
    assert isinstance(result, str)


def test_dump_toml_contains_key_values() -> None:
    config = IsaacCoreConfig()
    result = dump_toml(config)
    assert "headless" in result
    assert "scene" in result


def test_dump_toml_round_trip_produces_equal_config(tmp_path: Path) -> None:
    # The critical property: dump -> reload -> same config.
    original = IsaacCoreConfig()
    dumped = dump_toml(original)
    toml_file = tmp_path / "dumped.toml"
    toml_file.write_text(dumped)
    reloaded = load(path=toml_file, environ={})
    assert reloaded == original


def test_dump_toml_round_trip_with_custom_values(tmp_path: Path) -> None:
    from isaac_core.config.schema import CameraConfig, LoggingConfig, VehicleConfig

    original = IsaacCoreConfig(
        sim={"headless": True, "scene": "custom_scene"},
        logging=LoggingConfig(level="debug"),
        vehicles={
            "lead": VehicleConfig(
                udp_port=40000,
                cameras={"eo": CameraConfig(fov_deg=90.0, resolution=(1920, 1080))},
            ),
        },
    )
    dumped = dump_toml(original)
    toml_file = tmp_path / "dumped.toml"
    toml_file.write_text(dumped)
    reloaded = load(path=toml_file, environ={})
    assert reloaded == original


def test_dump_toml_round_trip_with_prim_overrides(tmp_path: Path) -> None:
    original = IsaacCoreConfig(
        prim_overrides=[
            {"prim": "/Environment/drone_0/Graph/node", "attribute": "inputs:enabled", "value": False},
        ]
    )
    dumped = dump_toml(original)
    toml_file = tmp_path / "dumped.toml"
    toml_file.write_text(dumped)
    reloaded = load(path=toml_file, environ={})
    assert reloaded == original
