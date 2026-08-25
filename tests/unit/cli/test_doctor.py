"""Tests for the isaac-core doctor environment diagnostic."""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_core.cli.doctor import (
    _check_config,
    _check_dependency,
    _check_extensions_linked,
    _check_isaac_install,
    _check_isaac_python,
    _check_isaac_version,
    _check_python_version,
    _check_ros2,
    doctor_checks,
    run_doctor,
)
from isaac_core.cli.main import main
from isaac_core.install import IsaacInstall


def test_doctor_never_raises_even_when_everything_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    # Simulate a completely broken environment
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    monkeypatch.delenv("ISAAC_CORE_CONFIG", raising=False)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    # Must not raise
    code = run_doctor()
    # Should return non-zero because Isaac isn't findable
    assert isinstance(code, int)


def test_doctor_returns_nonzero_on_fail(monkeypatch: pytest.MonkeyPatch) -> None:
    # Without Isaac Sim, extensions check will FAIL
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    code = run_doctor()
    # FAIL on Isaac Sim not found -> non-zero
    assert code == 1


def test_doctor_returns_zero_in_complete_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # Build a complete fake environment
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1-rc.7\n")

    exts_user = fake_isaac / "extsUser"
    exts_user.mkdir()
    # Create fake extension symlinks
    for ext in ("isaac_core_ogn.math", "isaac_core_ogn.position"):
        (exts_user / ext).symlink_to(tmp_path)

    monkeypatch.setenv("ISAACSIM_PATH", str(fake_isaac))
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(fake_isaac),))

    # doctor should pass (no FAILs)
    code = run_doctor()
    assert code == 0


def test_doctor_via_main_entrypoint(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    # Just verify it doesn't raise
    code = main(["doctor"])
    assert isinstance(code, int)


def test_check_python_version_passes() -> None:
    # We're running on >= 3.10 in this test env
    status, msg = _check_python_version()
    assert status == "PASS"
    assert "Python" in msg


def test_check_dependency_numpy() -> None:
    status, msg = _check_dependency("numpy", "1.24")
    assert status == "PASS"
    assert "numpy" in msg


def test_check_dependency_missing() -> None:
    status, msg = _check_dependency("nonexistent_package_xyz_999", "1.0")
    assert status == "FAIL"
    assert "not importable" in msg


def test_check_ros2_not_available(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    # rclpy is not importable in test env
    status, msg = _check_ros2()
    assert status == "WARN"
    assert "rclpy" in msg.lower() or "ros" in msg.lower()


def test_check_isaac_install_with_valid_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1\n")

    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(fake_isaac),))
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)

    status, msg, install = _check_isaac_install()
    assert status == "PASS"
    assert install is not None


def test_check_isaac_install_not_found(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)

    status, msg, install = _check_isaac_install()
    assert status == "FAIL"
    assert install is None


def test_check_isaac_version_supported(tmp_path: Path) -> None:
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1-rc.7\n")

    install = IsaacInstall(fake_isaac)
    status, msg = _check_isaac_version(install)
    assert status == "PASS"


def test_check_isaac_version_unsupported(tmp_path: Path) -> None:
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("2023.1.1\n")

    install = IsaacInstall(fake_isaac)
    status, msg = _check_isaac_version(install)
    assert status == "WARN"
    assert "not 6.x" in msg


def test_check_isaac_python_present(tmp_path: Path) -> None:
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1\n")

    install = IsaacInstall(fake_isaac)
    status, msg = _check_isaac_python(install)
    assert status == "PASS"


def test_check_extensions_linked_all_present(tmp_path: Path) -> None:
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1\n")
    exts_user = fake_isaac / "extsUser"
    exts_user.mkdir()

    for ext in ("isaac_core_ogn.math", "isaac_core_ogn.position"):
        (exts_user / ext).symlink_to(tmp_path)

    install = IsaacInstall(fake_isaac)
    status, msg = _check_extensions_linked(install)
    assert status == "PASS"


def test_check_extensions_linked_missing(tmp_path: Path) -> None:
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1\n")
    exts_user = fake_isaac / "extsUser"
    exts_user.mkdir()
    # Only link one
    (exts_user / "isaac_core_ogn.math").symlink_to(tmp_path)

    install = IsaacInstall(fake_isaac)
    status, msg = _check_extensions_linked(install)
    assert status == "FAIL"
    assert "not linked" in msg


def test_check_config_loads() -> None:
    status, msg = _check_config()
    # Default config should load fine
    assert status in ("PASS", "WARN")


def test_doctor_checks_returns_list(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # Set up a fake install so we get a full check run
    fake_isaac = tmp_path / "isaacsim"
    fake_isaac.mkdir()
    (fake_isaac / "python.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake_isaac / "VERSION").write_text("6.0.1\n")
    exts_user = fake_isaac / "extsUser"
    exts_user.mkdir()
    for ext in ("isaac_core_ogn.math", "isaac_core_ogn.position"):
        (exts_user / ext).symlink_to(tmp_path)

    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(fake_isaac),))
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)

    results = doctor_checks()
    assert len(results) > 0
    for status, msg in results:
        assert status in ("PASS", "WARN", "FAIL")
        assert isinstance(msg, str)
