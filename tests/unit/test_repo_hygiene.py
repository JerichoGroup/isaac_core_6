"""Guards for repository hygiene: what ships, what is ignored, and what the metadata claims.

Every assertion here corresponds to something that was wrong before Phase 4, so these are
regression guards rather than style preferences.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Any

import pytest
import tomli

REPO_ROOT = Path(__file__).resolve().parents[2]
PYPROJECT = REPO_ROOT / "pyproject.toml"
GITIGNORE = REPO_ROOT / ".gitignore"


def _pyproject() -> dict[str, Any]:
    """Return the parsed pyproject.toml.

    Typed as ``Any`` at the value level on purpose: this is a parsed TOML document, and narrowing
    every nested lookup would obscure what the assertions are actually checking.
    """
    return tomli.loads(PYPROJECT.read_text(encoding="utf-8"))


def _tracked_files() -> list[str]:
    """Return every file git tracks, or skip when git is unavailable."""
    result = subprocess.run(
        ["git", "ls-files"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        pytest.skip("git is not available, or this is not a repository")
    return [line for line in result.stdout.splitlines() if line]


# -- nothing generated is committed ------------------------------------------ #


@pytest.mark.parametrize(
    "fragment",
    ["__pycache__", ".pyc", ".egg-info", ".mypy_cache", ".ruff_cache", ".pytest_cache", "ogn_generated"],
)
def test_no_generated_artefact_is_tracked(fragment: str) -> None:
    offenders = [path for path in _tracked_files() if fragment in path]
    assert not offenders, f"generated files are committed: {offenders[:5]}"


def test_the_gitignore_covers_this_project_s_own_artefacts() -> None:
    # The stock GitHub Python template covers caches but knows nothing about Isaac Sim: captures,
    # the scratch directories used to debug against a live simulator, and the directories Kit leaves
    # behind when it crashes.
    text = GITIGNORE.read_text(encoding="utf-8")
    for pattern in ("__pycache__/", "*.egg-info/", "isaac_core_out/", ".validate/", "carb.*/"):
        assert pattern in text, f".gitignore is missing {pattern}"


# -- the metadata describes what actually ships ------------------------------ #


def test_every_console_script_points_at_something_importable() -> None:
    import importlib

    scripts = _pyproject()["project"]["scripts"]
    assert scripts, "no console scripts declared"
    for name, target in scripts.items():
        module_name, _, function = str(target).partition(":")
        module = importlib.import_module(module_name)
        assert hasattr(module, function), f"{name} points at {target}, which does not exist"


def test_the_package_declares_keywords_and_classifiers() -> None:
    # Without these the package is undiscoverable and its supported Python versions are a guess.
    project = _pyproject()["project"]
    assert project.get("keywords"), "no keywords declared"
    classifiers = project.get("classifiers") or []
    assert any("Programming Language :: Python :: 3.10" in c for c in classifiers)
    assert any("Typing :: Typed" in c for c in classifiers)


def test_the_typed_marker_exists_and_is_packaged() -> None:
    # Claiming `Typing :: Typed` without shipping py.typed means a consumer's type checker silently
    # ignores every annotation in a codebase that is checked under strict-ish mypy.
    marker = REPO_ROOT / "src" / "isaac_core" / "py.typed"
    assert marker.is_file(), "src/isaac_core/py.typed is missing"
    package_data = _pyproject()["tool"]["setuptools"].get("package-data", {})
    assert "py.typed" in package_data.get("isaac_core", []), "py.typed is not in package-data"


def test_the_declared_license_file_exists() -> None:
    project = _pyproject()["project"]
    assert project.get("license"), "no license declared"
    assert (REPO_ROOT / "LICENSE").is_file(), "LICENSE file is missing"


# -- the config reference stays a reference ---------------------------------- #


def test_the_config_reference_does_not_document_removed_pose_sources() -> None:
    # `script`, `replay` and `mavlink` were removed from PoseSource because nothing dispatched on
    # them: a vehicle configured that way composed and never moved. The reference must not offer them.
    from isaac_core.contracts.frames import PoseSource

    text = (REPO_ROOT / "config" / "default.toml").read_text(encoding="utf-8")
    valid = {member.value for member in PoseSource}
    for removed in ("script", "replay", "mavlink"):
        if removed in valid:
            continue
        assert f'pose_source = "{removed}"' not in text
        assert f"#   {removed} " not in text, f"default.toml still documents pose_source {removed!r}"


def test_the_config_reference_states_it_is_not_auto_loaded() -> None:
    # It is documentation: the shipped defaults come from the schema. Presenting this file as the
    # base layer of the precedence chain made copying it change behaviour.
    text = (REPO_ROOT / "config" / "default.toml").read_text(encoding="utf-8")
    assert "not loaded automatically" in text


def test_every_markdown_link_between_docs_resolves() -> None:
    # Moving documents around silently breaks cross-references, and a broken link is the first thing
    # a reader hits. The README has its own guard; this covers docs/ linking to each other.
    import re

    broken: list[str] = []
    pages = [*(REPO_ROOT / "docs").rglob("*.md"), REPO_ROOT / "README.md"]
    for page in pages:
        text = page.read_text(encoding="utf-8")
        for link in re.findall(r"\]\(([^)\s#]+\.md)\)", text):
            if link.startswith(("http://", "https://")):
                continue
            if not (page.parent / link).resolve().exists():
                broken.append(f"{page.relative_to(REPO_ROOT)} -> {link}")
    assert not broken, f"broken links between documents: {broken}"


def test_there_is_exactly_one_documentation_index() -> None:
    # docs/README.md was a second index, covering three of ten documents and describing two of them
    # inaccurately. The README's table is the index and is link-tested; a stale rival is worse than
    # none.
    assert not (REPO_ROOT / "docs" / "README.md").exists(), "docs/README.md is a second, untested index"


def test_every_data_file_in_the_package_is_declared_as_package_data() -> None:
    # Data files are not shipped unless listed. The four shipped layer manifests were missing, so a
    # non-editable install had no layers and every feature failed to compose -- invisible under the
    # editable install everything here actually uses.
    import fnmatch

    package_root = REPO_ROOT / "src" / "isaac_core"
    patterns = _pyproject()["tool"]["setuptools"]["package-data"]["isaac_core"]
    undeclared: list[str] = []
    for path in sorted(package_root.rglob("*")):
        if path.is_dir() or path.suffix in (".py", ".pyc") or "__pycache__" in str(path):
            continue
        relative = path.relative_to(package_root).as_posix()
        if not any(fnmatch.fnmatch(relative, pattern) for pattern in patterns):
            undeclared.append(relative)
    assert not undeclared, f"these ship only by accident: {undeclared}"


def test_the_shipped_layer_manifests_are_present() -> None:
    # A guard on the guard above: if the layers moved, the pattern would match nothing and the test
    # would pass while shipping nothing.
    manifests = sorted((REPO_ROOT / "src" / "isaac_core" / "assets" / "layers").glob("*/layer.toml"))
    assert len(manifests) >= 4, f"expected the four shipped layers, found {[m.parent.name for m in manifests]}"


def test_the_shipped_scene_and_layers_resolve_without_the_repo() -> None:
    # A new project installs this package, brings its own scene and maybe its own layer, and never
    # clones the repo. That only works if the shipped assets live inside the package: they used to sit
    # in a repo-level usd/ directory, so resolution fell back to a path that does not exist once
    # installed. The resolvers already looked in the packaged locations, so nothing but the files moved.
    from isaac_core.config import IsaacCoreConfig
    from isaac_core.sim.__main__ import _resolve_layer_search_paths, _resolve_scene_path

    package_root = Path(_resolve_scene_path(IsaacCoreConfig())).parents[1]
    assert package_root.name == "assets", f"the scene resolved outside the package, to {package_root}"
    for search_path in _resolve_layer_search_paths(IsaacCoreConfig()):
        assert "assets" in search_path.parts, f"layer search path {search_path} is outside the package"


def test_no_code_resolves_assets_relative_to_the_repo_root() -> None:
    # parents[3] from src/isaac_core/sim/ is the repo root, which does not exist for an installed
    # package. Reaching for it is how the old layout silently worked in development only.
    offenders = [
        path.relative_to(REPO_ROOT).as_posix()
        for path in (REPO_ROOT / "src").rglob("*.py")
        if "parents[3]" in path.read_text(encoding="utf-8")
    ]
    assert not offenders, f"these reach outside the installed package: {offenders}"
