"""Tests for configuration source layers."""

from pathlib import Path

import pytest

from isaac_core.config.sources import cli_source, defaults_source, env_source, toml_file_source

# --------------------------------------------------------------------------- #
# defaults_source
# --------------------------------------------------------------------------- #


def test_defaults_source_returns_empty_dict() -> None:
    # The pydantic field defaults ARE the shipped defaults; no data to load.
    assert defaults_source() == {}


def test_defaults_source_return_type_is_dict() -> None:
    result = defaults_source()
    assert isinstance(result, dict)


# --------------------------------------------------------------------------- #
# toml_file_source — happy path
# --------------------------------------------------------------------------- #


def test_toml_file_source_parses_valid_file(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = true\n")
    result = toml_file_source(config_file)
    assert result == {"sim": {"headless": True}}


def test_toml_file_source_returns_nested_structure(tmp_path: Path) -> None:
    config_file = tmp_path / "test.toml"
    config_file.write_text("[vehicles.drone_0.cameras.eo]\nfov_deg = 90.0\n")
    result = toml_file_source(config_file)
    assert result["vehicles"]["drone_0"]["cameras"]["eo"]["fov_deg"] == 90.0


# --------------------------------------------------------------------------- #
# toml_file_source — error paths
# --------------------------------------------------------------------------- #


def test_toml_file_source_raises_on_nonexistent_file(tmp_path: Path) -> None:
    missing = tmp_path / "missing.toml"
    with pytest.raises(FileNotFoundError, match="missing.toml"):
        toml_file_source(missing)


def test_toml_file_source_names_file_in_not_found_error(tmp_path: Path) -> None:
    # Guard: the error message must include the path so users know which file failed.
    missing = tmp_path / "specific_name.toml"
    with pytest.raises(FileNotFoundError, match="specific_name.toml"):
        toml_file_source(missing)


def test_toml_file_source_raises_on_malformed_toml(tmp_path: Path) -> None:
    bad_file = tmp_path / "bad.toml"
    bad_file.write_text("[invalid\nnot_closed")
    with pytest.raises(ValueError, match="malformed TOML"):
        toml_file_source(bad_file)


def test_toml_file_source_names_file_in_parse_error(tmp_path: Path) -> None:
    bad_file = tmp_path / "broken_syntax.toml"
    bad_file.write_text("key = [unclosed\n")
    with pytest.raises(ValueError, match="broken_syntax.toml"):
        toml_file_source(bad_file)


# --------------------------------------------------------------------------- #
# env_source — nesting via __
# --------------------------------------------------------------------------- #


def test_env_source_splits_on_double_underscore() -> None:
    environ = {"ISAAC_CORE__SIM__HEADLESS": "true"}
    result = env_source(environ)
    assert result == {"sim": {"headless": True}}


def test_env_source_deep_nesting() -> None:
    environ = {"ISAAC_CORE__VEHICLES__DRONE_0__CAMERAS__EO__FOV_DEG": "90.0"}
    result = env_source(environ)
    assert result["vehicles"]["drone_0"]["cameras"]["eo"]["fov_deg"] == 90.0


def test_env_source_lowercases_all_segments() -> None:
    environ = {"ISAAC_CORE__CESIUM__TILESET_SERVER_URL": "http://example.com"}
    result = env_source(environ)
    assert "cesium" in result
    assert "tileset_server_url" in result["cesium"]


def test_env_source_ignores_unrelated_variables() -> None:
    environ = {
        "ISAAC_CORE__SIM__HEADLESS": "true",
        "PATH": "/usr/bin",
        "HOME": "/home/user",
        "ISAAC_CORE_CONFIG": "/some/path.toml",
    }
    result = env_source(environ)
    # Only the double-underscore prefixed var should appear.
    assert result == {"sim": {"headless": True}}


def test_env_source_custom_prefix() -> None:
    environ = {"MYAPP__KEY": "val"}
    result = env_source(environ, prefix="MYAPP")
    assert result == {"key": "val"}


def test_env_source_returns_empty_for_no_matches() -> None:
    environ = {"PATH": "/usr/bin", "HOME": "/home/user"}
    result = env_source(environ)
    assert result == {}


def test_env_source_skips_empty_segments() -> None:
    # After removing the prefix+separator, "____KEY" splits on "__" as ["", "KEY"].
    # The empty segment makes this invalid and it should be skipped.
    environ = {"ISAAC_CORE____KEY": "val"}
    result = env_source(environ)
    assert result == {}


# --------------------------------------------------------------------------- #
# env_source — coercion
# --------------------------------------------------------------------------- #


def test_env_coerce_true() -> None:
    result = env_source({"ISAAC_CORE__KEY": "true"})
    assert result["key"] is True


def test_env_coerce_false() -> None:
    result = env_source({"ISAAC_CORE__KEY": "false"})
    assert result["key"] is False


def test_env_coerce_true_case_insensitive() -> None:
    result = env_source({"ISAAC_CORE__KEY": "True"})
    assert result["key"] is True


def test_env_coerce_false_case_insensitive() -> None:
    result = env_source({"ISAAC_CORE__KEY": "FALSE"})
    assert result["key"] is False


def test_env_coerce_integer() -> None:
    result = env_source({"ISAAC_CORE__KEY": "42"})
    assert result["key"] == 42
    assert isinstance(result["key"], int)


def test_env_coerce_negative_integer() -> None:
    result = env_source({"ISAAC_CORE__KEY": "-7"})
    assert result["key"] == -7
    assert isinstance(result["key"], int)


def test_env_coerce_float() -> None:
    result = env_source({"ISAAC_CORE__KEY": "3.14"})
    assert result["key"] == pytest.approx(3.14)
    assert isinstance(result["key"], float)


def test_env_coerce_negative_float() -> None:
    result = env_source({"ISAAC_CORE__KEY": "-0.5"})
    assert result["key"] == pytest.approx(-0.5)


def test_env_coerce_json_array() -> None:
    result = env_source({"ISAAC_CORE__KEY": "[1280, 720]"})
    assert result["key"] == [1280, 720]


def test_env_coerce_json_object() -> None:
    result = env_source({"ISAAC_CORE__KEY": '{"a": 1}'})
    assert result["key"] == {"a": 1}


def test_env_coerce_string_stays_string() -> None:
    # A value that looks like nothing else must stay a string.
    result = env_source({"ISAAC_CORE__KEY": "hello world"})
    assert result["key"] == "hello world"
    assert isinstance(result["key"], str)


def test_env_coerce_url_stays_string() -> None:
    # URLs must not be misinterpreted as numeric or JSON.
    result = env_source({"ISAAC_CORE__CESIUM__TILESET_SERVER_URL": "http://10.20.15.122:8088"})
    assert result["cesium"]["tileset_server_url"] == "http://10.20.15.122:8088"


def test_env_coerce_path_stays_string() -> None:
    result = env_source({"ISAAC_CORE__KEY": "/opt/data/scene.usd"})
    assert result["key"] == "/opt/data/scene.usd"
    assert isinstance(result["key"], str)


def test_env_coerce_empty_string_stays_string() -> None:
    result = env_source({"ISAAC_CORE__KEY": ""})
    assert result["key"] == ""


def test_env_coerce_zero_is_int() -> None:
    result = env_source({"ISAAC_CORE__KEY": "0"})
    assert result["key"] == 0
    assert isinstance(result["key"], int)


def test_env_coerce_malformed_json_stays_string() -> None:
    # Starts with [ but isn't valid JSON -- stays a string.
    result = env_source({"ISAAC_CORE__KEY": "[unclosed"})
    assert result["key"] == "[unclosed"
    assert isinstance(result["key"], str)


# --------------------------------------------------------------------------- #
# cli_source
# --------------------------------------------------------------------------- #


def test_cli_source_expands_dotted_key() -> None:
    result = cli_source({"sim.headless": True})
    assert result == {"sim": {"headless": True}}


def test_cli_source_expands_deeply_nested_key() -> None:
    result = cli_source({"vehicles.drone_0.cameras.eo.fov_deg": 90.0})
    assert result["vehicles"]["drone_0"]["cameras"]["eo"]["fov_deg"] == 90.0


def test_cli_source_multiple_keys() -> None:
    result = cli_source({"sim.headless": True, "logging.level": "debug"})
    assert result["sim"]["headless"] is True
    assert result["logging"]["level"] == "debug"


def test_cli_source_single_segment_key() -> None:
    result = cli_source({"headless": True})
    assert result == {"headless": True}


def test_cli_source_empty_overrides() -> None:
    result = cli_source({})
    assert result == {}


def test_cli_source_coerces_string_values_like_env() -> None:
    # --set values are always strings; they must coerce the same way env vars do, so
    # `--set` and ISAAC_CORE__* behave identically, including for list fields.
    assert cli_source({"sim.extensions": '["a", "b"]'}) == {"sim": {"extensions": ["a", "b"]}}
    assert cli_source({"sim.headless": "true"}) == {"sim": {"headless": True}}
    assert cli_source({"sim.control_plane.port": "9000"}) == {"sim": {"control_plane": {"port": 9000}}}
    assert cli_source({"vehicles.drone_0.cameras.eo.fov_deg": "60.0"}) == {
        "vehicles": {"drone_0": {"cameras": {"eo": {"fov_deg": 60.0}}}}
    }


def test_cli_source_leaves_plain_strings_alone() -> None:
    assert cli_source({"sim.scene": "earth"}) == {"sim": {"scene": "earth"}}


def test_cli_source_passes_non_string_values_through() -> None:
    # The devkit and tests pass already-typed values programmatically; those must not be
    # run through string coercion (a bool has no .lower()).
    assert cli_source({"sim.headless": True}) == {"sim": {"headless": True}}
    assert cli_source({"sim.control_plane.port": 8760}) == {"sim": {"control_plane": {"port": 8760}}}
