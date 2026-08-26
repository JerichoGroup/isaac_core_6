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


def _fake_isaac_install(root: Path) -> Path:
    """Create a directory that passes IsaacInstall's structural checks."""
    root.mkdir(parents=True, exist_ok=True)
    (root / "python.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (root / "isaac-sim.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (root / "VERSION").write_text("6.0.1-rc.7\n", encoding="utf-8")
    return root


def test_run_dry_run_reports_without_launching(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    install = _fake_isaac_install(tmp_path / "isaacsim")
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(install),))
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)

    called: list[list[str]] = []

    def _record(cmd: list[str], *args: object, **kwargs: object) -> int:
        called.append(cmd)
        return 0

    monkeypatch.setattr("subprocess.call", _record)

    assert main(["run", "--dry-run"]) == 0
    assert "Dry run: not launching." in capsys.readouterr().out
    assert not called, "dry run must not launch anything"


def test_run_launches_the_sim_module_in_isaacs_interpreter(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # Decision D7: launch a MODULE inside Isaac's python, never a path into this repo.
    install = _fake_isaac_install(tmp_path / "isaacsim")
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(install),))
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)

    called: list[list[str]] = []

    def fake_call(cmd: list[str], *args: object, **kwargs: object) -> int:
        called.append(cmd)
        return 0

    monkeypatch.setattr("subprocess.call", fake_call)

    assert main(["run"]) == 0
    assert len(called) == 1
    command = called[0]
    assert command[0] == str(install / "python.sh")
    assert command[1:3] == ["-m", "isaac_core.sim"]
    assert "--config" in command
    # The forwarded config must be a resolved temp file, not a path inside the repo.
    forwarded = Path(command[command.index("--config") + 1])
    assert forwarded.is_file()
    assert "isaac-core-" in str(forwarded)


def test_run_propagates_the_simulator_exit_code(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    install = _fake_isaac_install(tmp_path / "isaacsim")
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(install),))
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.setattr("subprocess.call", lambda *a, **k: 42)

    assert main(["run"]) == 42
