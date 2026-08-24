"""Tests for the isaac-core CLI main entry point."""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_core.cli.main import main


def test_no_args_prints_help_exits_zero(capsys: pytest.CaptureFixture[str]) -> None:
    # No subcommand prints usage and exits 0
    code = main([])
    assert code == 0
    out = capsys.readouterr().out
    assert "isaac-core" in out


def test_help_flag_exits_zero() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["--help"])
    assert exc_info.value.code == 0


def test_run_help_exits_zero() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["run", "--help"])
    assert exc_info.value.code == 0


def test_doctor_help_exits_zero() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["doctor", "--help"])
    assert exc_info.value.code == 0


def test_config_help_exits_zero() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["config", "--help"])
    assert exc_info.value.code == 0


def test_config_dump_help_exits_zero() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["config", "dump", "--help"])
    assert exc_info.value.code == 0


def test_config_explain_help_exits_zero() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["config", "explain", "--help"])
    assert exc_info.value.code == 0


def test_run_exits_nonzero_runtime_not_implemented(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    # run needs an Isaac install -- fake one
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1-rc.7\n")

    monkeypatch.setenv("ISAACSIM_PATH", str(fake_isaac))

    code = main(["run", "--isaac-path", str(fake_isaac)])
    assert code == 1
    out = capsys.readouterr().out
    assert "not yet implemented" in out.lower()


def test_run_fails_without_isaac_install(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    # Remove env var and ensure no probe succeeds
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    code = main(["run"])
    assert code == 1
    captured = capsys.readouterr()
    assert "cannot find" in captured.err.lower() or "error" in captured.err.lower()


def test_config_no_subcommand_exits_nonzero(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["config"])
    assert code == 1


def test_run_with_config_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    # Build a fake Isaac install
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.1.0\n")

    # Create a minimal config file
    config_file = tmp_path / "test.toml"
    config_file.write_text("[sim]\nheadless = true\n")

    code = main(["run", "--config", str(config_file), "--isaac-path", str(fake_isaac)])
    assert code == 1  # runtime not implemented
    out = capsys.readouterr().out
    assert "Headless:  True" in out or "headless" in out.lower()


def test_run_with_invalid_config_file(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    bad_config = tmp_path / "nonexistent.toml"
    code = main(["run", "--config", str(bad_config)])
    assert code == 1
    assert "error" in capsys.readouterr().err.lower()
