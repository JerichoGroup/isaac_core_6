"""Tests for scene path resolution.

`sim.scene` accepts three forms: an absolute path, a path relative to the working directory,
and a bare name resolved against the asset search paths and built-in scenes. The relative
form was reported broken -- a non-absolute path fell straight through to the
name-search and failed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_core.config import load
from isaac_core.sim.__main__ import _resolve_scene_path

# Scenes ship inside the package, beside the layers. There is no repo-relative fallback, so an
# installed package resolves a scene exactly as a source checkout does.
_PACKAGED_SCENES = Path(__file__).resolve().parents[3] / "src" / "isaac_core" / "assets" / "scenes"


def test_absolute_path_is_used_directly() -> None:
    absolute = (_PACKAGED_SCENES / "earth.usda").resolve()
    config = load(cli_overrides={"sim.scene": str(absolute)})
    assert _resolve_scene_path(config) == absolute


def test_relative_path_resolves_against_the_working_directory(monkeypatch: pytest.MonkeyPatch) -> None:
    # The reported bug: a relative path with a slash must resolve, not fall through to the
    # bare-name search and fail.
    monkeypatch.chdir(Path(__file__).resolve().parents[3])
    config = load(cli_overrides={"sim.scene": "./src/isaac_core/assets/scenes/earth.usda"})
    resolved = _resolve_scene_path(config)
    assert resolved.is_file()
    assert resolved.name == "earth.usda"


def test_bare_name_resolves_against_repo_scenes() -> None:
    config = load(cli_overrides={"sim.scene": "earth"})
    resolved = _resolve_scene_path(config)
    assert resolved.is_file()
    assert resolved.name == "earth.usda"


def test_unknown_scene_raises(tmp_path: Path) -> None:
    config = load(cli_overrides={"sim.scene": "does-not-exist-anywhere"})
    with pytest.raises(FileNotFoundError, match="not found"):
        _resolve_scene_path(config)


def test_a_tilde_path_is_expanded(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # A user is likely to write ~/scenes/foo.usda; that must expand rather than being
    # treated as a literal directory named "~".
    scene = tmp_path / "custom.usda"
    scene.write_text("#usda 1.0\n", encoding="utf-8")
    monkeypatch.setenv("HOME", str(tmp_path))
    config = load(cli_overrides={"sim.scene": "~/custom.usda"})
    assert _resolve_scene_path(config) == scene.resolve()
