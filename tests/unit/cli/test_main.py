"""Tests for the isaac-core CLI main entry point."""

from __future__ import annotations

from pathlib import Path
import signal
import subprocess
from typing import cast
from unittest import mock

import pytest

from isaac_core.cli.main import _terminate_process_group, main


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


def _recording_popen(recorded: list[list[str]], *, returncode: int) -> object:
    """Return a fake ``Popen`` class that records the command and exits with *returncode*.

    The launcher owns the simulator process now, rather than handing control to
    ``subprocess.call``, because returning from a Ctrl-C without killing the child left Isaac
    running and it would open a window minutes later.

    Args:
        recorded: List that receives each command.
        returncode: Exit status the fake process reports.

    Returns:
        A class usable in place of ``subprocess.Popen``.

    """

    class _FakePopen:
        def __init__(self, cmd: list[str], *args: object, **kwargs: object) -> None:
            recorded.append(cmd)
            self.pid = 4242
            self.returncode = returncode

        def wait(self, timeout: float | None = None) -> int:
            return returncode

        def poll(self) -> int:
            return returncode

    return _FakePopen


def test_run_launches_the_sim_module_in_isaacs_interpreter(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # Decision D7: launch a MODULE inside Isaac's python, never a path into this repo.
    install = _fake_isaac_install(tmp_path / "isaacsim")
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(install),))
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)

    called: list[list[str]] = []

    monkeypatch.setattr("subprocess.Popen", _recording_popen(called, returncode=0))

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
    monkeypatch.setattr("subprocess.Popen", _recording_popen([], returncode=42))

    assert main(["run"]) == 42


def test_ctrl_c_during_launch_kills_the_simulator_process_group() -> None:
    # A Ctrl-C used to return 130 while leaving the child alive: Isaac ignores SIGTERM, kept
    # initialising, and opened a window belonging to a command that had already exited. The launcher
    # must signal the whole group and not return until the process is gone.
    signals: list[tuple[int, int]] = []

    class _StubbornProcess:
        """Ignores SIGTERM, like Isaac, and only dies on SIGKILL."""

        pid = 7777

        def __init__(self) -> None:
            self._alive = True

        def poll(self) -> int | None:
            return None if self._alive else -9

        def wait(self, timeout: float | None = None) -> int:
            if self._alive:
                raise subprocess.TimeoutExpired(cmd="isaac", timeout=timeout or 0.0)
            return -9

        def kill_with(self, signal_number: int) -> None:
            if signal_number == signal.SIGKILL:
                self._alive = False

    process = _StubbornProcess()

    def fake_killpg(group: int, signal_number: int) -> None:
        signals.append((group, signal_number))
        process.kill_with(signal_number)

    with (
        mock.patch("os.getpgid", return_value=process.pid),
        mock.patch("os.killpg", side_effect=fake_killpg),
    ):
        _terminate_process_group(cast("subprocess.Popen[bytes]", process))

    assert [s for _, s in signals] == [signal.SIGTERM, signal.SIGKILL], f"expected SIGTERM then SIGKILL, got {signals}"
    assert process.poll() is not None, "the process was left running"


def test_terminating_an_already_dead_process_signals_nothing() -> None:
    # Signalling a pid that has exited can hit a recycled pid, so an early exit matters.
    class _Finished:
        pid = 5555

        def poll(self) -> int:
            return 0

    with mock.patch("os.killpg", side_effect=AssertionError("must not signal a finished process")):
        _terminate_process_group(cast("subprocess.Popen[bytes]", _Finished()))


def test_run_names_a_config_file_it_is_not_loading(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # Reported from a fresh machine: edits to config/default.toml had no effect, because that file is
    # documentation and is never auto-loaded. The only clue was the word "(defaults)", so the run
    # banner now names the file it is ignoring.
    from contextlib import redirect_stdout
    import io

    from isaac_core.cli.main import _warn_about_an_unloaded_config_file

    monkeypatch.chdir(tmp_path)
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "default.toml").write_text("", encoding="utf-8")
    buffer = io.StringIO()
    with redirect_stdout(buffer):
        _warn_about_an_unloaded_config_file()
    printed = buffer.getvalue()
    assert "config/default.toml" in printed
    assert "NOT loaded" in printed
    assert "--config config/default.toml" in printed


def test_run_says_nothing_when_there_is_no_config_file_to_confuse_anyone(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from contextlib import redirect_stdout
    import io

    from isaac_core.cli.main import _warn_about_an_unloaded_config_file

    monkeypatch.chdir(tmp_path)
    buffer = io.StringIO()
    with redirect_stdout(buffer):
        _warn_about_an_unloaded_config_file()
    assert buffer.getvalue() == ""
