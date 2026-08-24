"""Tests for the config dump and config explain subcommands."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

from isaac_core.cli.config_cmd import run_config_dump, run_config_explain
from isaac_core.cli.main import main


def test_config_dump_exits_zero(capsys: pytest.CaptureFixture[str]) -> None:
    code = run_config_dump()
    assert code == 0
    out = capsys.readouterr().out
    assert len(out) > 0


def test_config_dump_output_is_parseable_toml(capsys: pytest.CaptureFixture[str]) -> None:
    if sys.version_info >= (3, 11):
        import tomllib
    else:
        import tomli as tomllib

    code = run_config_dump()
    assert code == 0
    output = capsys.readouterr().out

    # Should parse without error
    parsed = tomllib.loads(output)
    assert isinstance(parsed, dict)
    # Should contain expected top-level sections
    assert "sim" in parsed


def test_config_dump_roundtrips_through_loader(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    if sys.version_info >= (3, 11):
        pass
    else:
        pass

    from isaac_core.config import load

    # Dump the default config
    code = run_config_dump()
    assert code == 0
    output = capsys.readouterr().out

    # Write to a temp file and reload
    config_file = tmp_path / "dumped.toml"
    config_file.write_text(output)

    # Should load without validation error
    config = load(path=config_file)
    assert config.sim is not None


def test_config_dump_with_custom_config(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    config_file = tmp_path / "custom.toml"
    config_file.write_text("[sim]\nheadless = true\n")

    code = run_config_dump(config_path=str(config_file))
    assert code == 0
    out = capsys.readouterr().out
    assert "headless" in out


def test_config_dump_with_missing_file(capsys: pytest.CaptureFixture[str]) -> None:
    code = run_config_dump(config_path="/nonexistent/config.toml")
    assert code == 1
    err = capsys.readouterr().err
    assert "error" in err.lower()


def test_config_explain_default_value(capsys: pytest.CaptureFixture[str]) -> None:
    # A key that comes from defaults (not set by file/env/cli)
    code = run_config_explain("sim.headless")
    assert code == 0
    out = capsys.readouterr().out
    assert "sim.headless" in out
    assert "default" in out.lower()


def test_config_explain_file_source(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    # Set a value via a config file, and explain should report file source
    config_file = tmp_path / "explain.toml"
    config_file.write_text("[sim]\nheadless = true\n")

    code = run_config_explain("sim.headless", config_path=str(config_file))
    assert code == 0
    out = capsys.readouterr().out
    assert "sim.headless" in out
    assert "file:" in out


def test_config_explain_env_source(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    # Set a value via env var, explain should report env source
    monkeypatch.setenv("ISAAC_CORE__SIM__HEADLESS", "true")

    code = run_config_explain("sim.headless")
    assert code == 0
    out = capsys.readouterr().out
    assert "sim.headless" in out
    assert "env:" in out.lower()


def test_config_explain_nonexistent_key(capsys: pytest.CaptureFixture[str]) -> None:
    code = run_config_explain("nonexistent.key.that.does.not.exist")
    assert code == 1
    err = capsys.readouterr().err
    assert "does not exist" in err


def test_config_dump_via_main(capsys: pytest.CaptureFixture[str]) -> None:
    # Test going through main() dispatch
    code = main(["config", "dump"])
    assert code == 0
    out = capsys.readouterr().out
    assert "sim" in out


def test_config_explain_via_main(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["config", "explain", "sim.headless"])
    assert code == 0
    out = capsys.readouterr().out
    assert "sim.headless" in out
