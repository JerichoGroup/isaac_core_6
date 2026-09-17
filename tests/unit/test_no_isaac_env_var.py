"""Guards that no environment variable can name an Isaac Sim install, and none is advertised.

A shell variable pointing at an install is a setting the user cannot see, usually cannot remember
setting, and which is stale more often than not. V3 removed it from resolution entirely. These tests
keep it removed from both the code and the things a new user reads.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Final

import pytest

from isaac_core.config import load
from isaac_core.install import IsaacInstall, IsaacInstallError

REPO_ROOT: Final = Path(__file__).resolve().parents[2]

# The only files allowed to name the old variable. Everything here either records history or proves
# the variable is ignored; nothing here instructs a user to set it.
_HISTORICAL: Final = frozenset({"migrating_from_2023.md", "development-log.md", "roadmap.md", "v3_plan.md"})

# Directories that are generated, vendored or otherwise not ours to police.
_SKIP_DIRS: Final = frozenset(
    {".git", "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".import_linter_cache"}
)

# Tests may name the variable: their whole job is proving it does nothing. The setup script may
# explain in one comment why its local is lowercase.
_ALLOWED_TO_MENTION: Final = frozenset({"tests", "scripts/setup.sh"})


def _is_exempt(path: Path) -> bool:
    """Return whether a file is allowed to mention the removed variable."""
    relative = path.relative_to(REPO_ROOT)
    if any(part in _SKIP_DIRS for part in relative.parts):
        return True
    if path.name in _HISTORICAL:
        return True
    return any(str(relative) == allowed or str(relative).startswith(f"{allowed}/") for allowed in _ALLOWED_TO_MENTION)


def _make_install(root: Path, version: str = "6.0.1") -> Path:
    """Create a directory that passes the structural install check."""
    root.mkdir(parents=True, exist_ok=True)
    (root / "python.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (root / "python.sh").chmod(0o755)
    (root / "isaac-sim.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (root / "isaac-sim.sh").chmod(0o755)
    (root / "VERSION").write_text(f"{version}\n", encoding="utf-8")
    return root


@pytest.mark.parametrize("variable", ["ISAACSIM_PATH", "ISAAC_PATH", "ISAAC_SIM_PATH"])
def test_no_environment_variable_can_name_an_install(
    variable: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # A valid, supported install reachable only through a variable must not be found.
    hidden = _make_install(tmp_path / "isaacsim")
    monkeypatch.setenv(variable, str(hidden))
    monkeypatch.setattr("isaac_core.install._probe_candidates", lambda: ())

    with pytest.raises(IsaacInstallError):
        IsaacInstall.locate()


def test_the_source_does_not_read_any_isaac_environment_variable() -> None:
    # Grepping the source is the only way to catch a variable being reintroduced somewhere new.
    offenders: list[str] = []
    for path in (REPO_ROOT / "src").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), start=1):
            if re.search(r"""(environ|getenv)[^\n]*ISAAC[A-Z_]*PATH""", line):
                offenders.append(f"{path.relative_to(REPO_ROOT)}:{number}")
    assert not offenders, f"these read an Isaac path from the environment: {offenders}"


def test_nothing_in_the_repo_mentions_an_isaac_path_variable() -> None:
    """Every tracked file, not just the ones I remembered to look at.

    The first version of this guard checked ``src/**/*.py``, the shell scripts and ``docs/``, with a
    condition sloppy enough to permit ``$ISAAC_PATH`` in a script. It therefore passed while
    ``pyproject.toml`` and ``requirements.txt`` both still told the user to run
    ``$ISAACSIM_PATH/python.sh``. Checking everything and exempting a named list is the only version of
    this test that can be trusted.
    """
    offenders: list[str] = []
    for path in sorted(REPO_ROOT.rglob("*")):
        if not path.is_file() or _is_exempt(path):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for number, line in enumerate(text.splitlines(), start=1):
            if re.search(r"ISAAC(SIM)?_PATH", line):
                offenders.append(f"{path.relative_to(REPO_ROOT)}:{number}: {line.strip()[:70]}")
    assert not offenders, f"these still mention a removed Isaac path variable: {offenders}"


def test_no_user_facing_page_tells_anyone_about_the_variable() -> None:
    # A new user must never be told to set it, or that it exists.
    offenders: list[str] = []
    pages = [REPO_ROOT / "README.md", *(REPO_ROOT / "docs").rglob("*.md")]
    for page in pages:
        if page.name in _HISTORICAL:
            continue
        for number, line in enumerate(page.read_text(encoding="utf-8").splitlines(), start=1):
            if re.search(r"\$ISAAC[A-Z_]*PATH", line):
                offenders.append(f"{page.relative_to(REPO_ROOT)}:{number}: {line.strip()[:60]}")
    assert not offenders, f"these advertise the removed variable: {offenders}"


def test_the_config_key_that_replaces_it_exists_and_is_honoured(tmp_path: Path) -> None:
    # Error messages and the doctor hint named sim.isaac_sim_path before the field existed, so setting
    # it was rejected as an unknown key. With no environment variable left, this is the only way to
    # name a non-standard install once instead of passing --isaac-path to every command.
    install = _make_install(tmp_path / "elsewhere")
    config = load(cli_overrides={"sim.isaac_sim_path": str(install)})
    assert config.sim.isaac_sim_path == str(install)
    assert IsaacInstall.locate(config_path=Path(config.sim.isaac_sim_path)).root == install


def test_the_config_key_defaults_to_unset() -> None:
    from isaac_core.config import IsaacCoreConfig

    assert IsaacCoreConfig().sim.isaac_sim_path is None


def test_every_locate_caller_passes_the_configured_path() -> None:
    # The key is useless if a caller resolves the install without it, which is what shipped before:
    # three call sites, none of them passing config.
    sources = {
        path: path.read_text(encoding="utf-8")
        for path in (REPO_ROOT / "src").rglob("*.py")
        if "IsaacInstall.locate(" in path.read_text(encoding="utf-8")
    }
    assert sources, "no locate() call sites found, so this guard is checking nothing"
    bare: list[str] = []
    for path, text in sources.items():
        for number, line in enumerate(text.splitlines(), start=1):
            if "IsaacInstall.locate()" in line:
                bare.append(f"{path.relative_to(REPO_ROOT)}:{number}")
    assert not bare, f"these resolve the install while ignoring sim.isaac_sim_path: {bare}"
