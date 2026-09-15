"""Tests that a stale ISAACSIM_PATH cannot win over a supported install.

This guards a real trap observed on a developer machine: ``$ISAACSIM_PATH`` still
pointed at an Isaac Sim 2023.1.1 install that passed every structural check
(python.sh, isaac-sim.sh and VERSION all present) while Isaac Sim 6 sat elsewhere.
Resolving structurally-valid-first picked the dead install, and everything
downstream -- the bundled python, the extsUser directory, extension linking --
silently pointed at a version where none of our extensions can even load, because
``omni.isaac.*`` was renamed to ``isaacsim.*`` in 4.5.

So validity alone must not win: a candidate has to be structurally valid AND a
supported version to be chosen outright.
"""

from pathlib import Path

import pytest

from isaac_core.install import IsaacInstall, IsaacInstallError


def _make_install(root: Path, version: str) -> Path:
    """Create a directory that looks exactly like an Isaac Sim install."""
    root.mkdir(parents=True, exist_ok=True)
    (root / "python.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (root / "isaac-sim.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (root / "VERSION").write_text(f"{version}\n", encoding="utf-8")
    return root


def test_supported_probe_beats_a_structurally_valid_stale_env_var(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    stale = _make_install(tmp_path / "isaac_sim-2023.1.1", "2023.1.1-rc.8+2023.1.688")
    current = _make_install(tmp_path / "isaacsim", "6.0.1-rc.7+release.42383")

    monkeypatch.setenv("ISAACSIM_PATH", str(stale))
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(current),))

    resolved = IsaacInstall.locate()
    assert resolved.root == current
    assert resolved.is_supported()


def test_stale_env_var_is_used_only_when_nothing_supported_exists(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Falling back is better than refusing to run: doctor can then report the real
    # problem instead of the tooling claiming no Isaac Sim exists at all.
    stale = _make_install(tmp_path / "isaac_sim-2023.1.1", "2023.1.1")

    monkeypatch.setenv("ISAACSIM_PATH", str(stale))
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    resolved = IsaacInstall.locate()
    assert resolved.root == stale
    assert not resolved.is_supported()


def test_explicit_path_wins_even_when_unsupported(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # An explicit path is an instruction, not a guess, so targeting an old install
    # deliberately must still work.
    old = _make_install(tmp_path / "old", "2023.1.1")
    current = _make_install(tmp_path / "isaacsim", "6.0.1")

    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(current),))

    assert IsaacInstall.locate(explicit_path=old).root == old


def test_year_based_versions_are_not_supported(tmp_path: Path) -> None:
    old = _make_install(tmp_path / "old", "2023.1.1")
    assert not IsaacInstall(old).is_supported()


def test_six_point_zero_is_supported(tmp_path: Path) -> None:
    current = _make_install(tmp_path / "isaacsim", "6.0.1-rc.7+release.42383")
    assert IsaacInstall(current).is_supported()


def test_error_lists_what_was_tried_including_the_unsupported_reason(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: (str(tmp_path / "nope"),))

    with pytest.raises(IsaacInstallError) as excinfo:
        IsaacInstall.locate()
    assert "nope" in str(excinfo.value)


def test_probe_candidates_are_home_relative_not_hardcoded_to_one_user() -> None:
    # A literal /home/<someone>/isaacsim would silently fail for every other
    # team member, so the home-relative entries must resolve to the current user.
    from isaac_core.install import _probe_candidates

    candidates = _probe_candidates()
    assert any(str(Path.home()) in candidate for candidate in candidates)
