"""Tests for Isaac Sim installation discovery and validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_core.install import IsaacInstall, IsaacInstallError, _is_valid_install


def _make_fake_install(base: Path, version: str = "6.0.1-rc.7") -> Path:
    """Build a minimal fake Isaac Sim install tree."""
    isaac_dir = base / "isaacsim"
    isaac_dir.mkdir(parents=True)
    (isaac_dir / "python.sh").write_text("#!/bin/sh\n")
    (isaac_dir / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (isaac_dir / "VERSION").write_text(f"{version}\n")
    return isaac_dir


def test_is_valid_install_true(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path)
    assert _is_valid_install(fake) is True


def test_is_valid_install_false_missing_python_sh(tmp_path: Path) -> None:
    fake = tmp_path / "isaacsim"
    fake.mkdir()
    (fake / "isaac-sim.sh").write_text("#!/bin/sh\n")
    (fake / "VERSION").write_text("6.0.1\n")
    # Missing python.sh
    assert _is_valid_install(fake) is False


def test_is_valid_install_false_missing_version(tmp_path: Path) -> None:
    fake = tmp_path / "isaacsim"
    fake.mkdir()
    (fake / "python.sh").write_text("#!/bin/sh\n")
    (fake / "isaac-sim.sh").write_text("#!/bin/sh\n")
    # Missing VERSION
    assert _is_valid_install(fake) is False


def test_is_valid_install_false_not_a_directory(tmp_path: Path) -> None:
    not_dir = tmp_path / "not_a_dir"
    not_dir.write_text("just a file")
    assert _is_valid_install(not_dir) is False


def test_is_valid_install_false_nonexistent(tmp_path: Path) -> None:
    assert _is_valid_install(tmp_path / "does_not_exist") is False


def test_locate_explicit_path_wins(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path)
    install = IsaacInstall.locate(explicit_path=fake)
    assert install.root == fake


def test_locate_config_path_used_when_no_explicit(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path)
    install = IsaacInstall.locate(config_path=fake, environ={})
    assert install.root == fake


def test_locate_ignores_the_environment_entirely(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # A shell variable naming an install is a setting the user cannot see and usually cannot remember
    # setting, and it is stale more often than not. Nothing in resolution reads the environment now, so
    # a perfectly valid install named only by the old variable must NOT be found.
    fake = _make_fake_install(tmp_path)
    monkeypatch.setenv("ISAACSIM_PATH", str(fake))
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    with pytest.raises(IsaacInstallError):
        IsaacInstall.locate()

    # And passing it in explicitly through the ignored parameter changes nothing either.
    with pytest.raises(IsaacInstallError):
        IsaacInstall.locate(environ={"ISAACSIM_PATH": str(fake)})


def test_locate_ignores_stale_env_var(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # Stale env var points at a directory that lacks marker files
    stale_dir = tmp_path / "old_isaac"
    stale_dir.mkdir()
    # It has no python.sh / isaac-sim.sh / VERSION

    # Real install is at a probe candidate
    real = _make_fake_install(tmp_path / "real")

    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(real),))

    install = IsaacInstall.locate(environ={"ISAACSIM_PATH": str(stale_dir)})
    # Should skip stale dir and find the real one via probing
    assert install.root == real


def test_locate_env_var_directory_does_not_exist(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # Env var points at a non-existent directory
    real = _make_fake_install(tmp_path)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(real),))

    install = IsaacInstall.locate(environ={"ISAACSIM_PATH": "/nonexistent/path"})
    assert install.root == real


def test_locate_probes_candidates(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # No explicit, no config, no env var -- falls through to probing
    fake = _make_fake_install(tmp_path)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(fake),))

    install = IsaacInstall.locate(environ={})
    assert install.root == fake


def test_locate_extra_candidates(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # Built-in candidates are empty, but extra_candidates finds it
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())
    fake = _make_fake_install(tmp_path)

    install = IsaacInstall.locate(environ={}, extra_candidates=[fake])
    assert install.root == fake


def test_locate_raises_when_nothing_found(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    with pytest.raises(IsaacInstallError) as exc_info:
        IsaacInstall.locate(environ={})
    assert "Cannot find" in str(exc_info.value)


def test_locate_raises_with_tried_locations(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # An explicit path that does not validate must fail LOUDLY, naming that path -- not
    # silently fall through to probing and use a different install than the user asked for.
    bad_path = tmp_path / "bad"
    bad_path.mkdir()
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    with pytest.raises(IsaacInstallError) as exc_info:
        IsaacInstall.locate(explicit_path=bad_path, environ={})
    error_msg = str(exc_info.value)
    assert str(bad_path) in error_msg
    assert "not a valid" in error_msg


def test_locate_falls_through_when_no_explicit_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # With no explicit path, an empty environment and no probe candidates, locate reports
    # the tried locations and how to fix it.
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())
    with pytest.raises(IsaacInstallError) as exc_info:
        IsaacInstall.locate(environ={})
    assert "Cannot find a valid Isaac Sim installation" in str(exc_info.value)


def test_version_parsed_correctly(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="6.0.1-rc.7+release.42383")
    install = IsaacInstall(fake)
    assert install.version == "6.0.1-rc.7+release.42383"
    assert install.major_version == 6


def test_version_2023_parsed(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="2023.1.1")
    install = IsaacInstall(fake)
    assert install.major_version == 2023


def test_version_malformed_returns_zero(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="unknown-version")
    install = IsaacInstall(fake)
    assert install.major_version == 0


def test_is_supported_true_for_6x(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="6.0.1-rc.7")
    install = IsaacInstall(fake)
    assert install.is_supported() is True


def test_is_supported_false_for_2023(tmp_path: Path) -> None:
    # 2023.x used the old omni.isaac.* namespace and is not supported
    fake = _make_fake_install(tmp_path, version="2023.1.1")
    install = IsaacInstall(fake)
    assert install.is_supported() is False


def test_is_supported_false_for_4x(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="4.5.0")
    install = IsaacInstall(fake)
    assert install.is_supported() is False


def test_python_path(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path)
    install = IsaacInstall(fake)
    assert install.python_path == fake / "python.sh"


def test_launcher_path(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path)
    install = IsaacInstall(fake)
    assert install.launcher_path == fake / "isaac-sim.sh"


def test_exts_user_dir(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path)
    install = IsaacInstall(fake)
    assert install.exts_user_dir == fake / "extsUser"


def test_support_message_supported(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="6.0.1")
    install = IsaacInstall(fake)
    msg = install.support_message
    assert "supported" in msg.lower()


def test_support_message_unsupported(tmp_path: Path) -> None:
    fake = _make_fake_install(tmp_path, version="4.2.0")
    install = IsaacInstall(fake)
    msg = install.support_message
    assert "NOT supported" in msg


def test_locate_prefers_explicit_over_env(tmp_path: Path) -> None:
    # Explicit path is preferred over env var even if env is valid
    explicit_dir = _make_fake_install(tmp_path / "explicit")
    env_dir = _make_fake_install(tmp_path / "env")

    install = IsaacInstall.locate(
        explicit_path=explicit_dir,
        environ={"ISAACSIM_PATH": str(env_dir)},
    )
    assert install.root == explicit_dir


def test_locate_prefers_config_over_env(tmp_path: Path) -> None:
    config_dir = _make_fake_install(tmp_path / "config")
    env_dir = _make_fake_install(tmp_path / "env")

    install = IsaacInstall.locate(
        config_path=config_dir,
        environ={"ISAACSIM_PATH": str(env_dir)},
    )
    assert install.root == config_dir
