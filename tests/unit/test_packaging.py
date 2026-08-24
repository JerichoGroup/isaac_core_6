"""
Tests that the packaging metadata cannot quietly drift apart.

The team installs system-wide from ``requirements.txt`` rather than using a venv,
but ``pyproject.toml`` still has to declare the same dependencies for the package
to install correctly anywhere else. That is two sources of truth for one fact,
which is exactly the kind of duplication that rotted the previous generation, so
these tests fail the moment the two disagree.

Deliberately parses no TOML: it must run before ``tomli`` is installed, since
telling someone what to install is the first thing a fresh checkout has to do.
"""

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[2]

PRE_COMMIT_PINNED_TOOLS = ("ruff", "mypy", "gitlint")


def _normalise(requirement: str) -> str:
    """Strip all whitespace so formatting differences never fail a comparison."""
    return re.sub(r"\s+", "", requirement)


def _read_requirements(name: str) -> list[str]:
    """Return the requirement specifiers from a requirements file."""
    lines = (REPO_ROOT / name).read_text(encoding="utf-8").splitlines()
    return [_normalise(line) for line in lines if line.strip() and not line.lstrip().startswith(("#", "-r "))]


def _pyproject_runtime_dependencies() -> list[str]:
    """Return the ``[project].dependencies`` entries without parsing TOML."""
    text = (REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    match = re.search(r"\ndependencies = \[(.*?)\]", text, re.DOTALL)
    assert match is not None, "could not locate [project].dependencies in pyproject.toml"
    return [
        _normalise(entry.strip().strip('",')) for entry in match.group(1).splitlines() if entry.strip().startswith('"')
    ]


def test_requirements_and_pyproject_declare_the_same_runtime_dependencies() -> None:
    from_requirements = sorted(_read_requirements("requirements.txt"))
    from_pyproject = sorted(_pyproject_runtime_dependencies())

    assert from_requirements == from_pyproject, (
        "requirements.txt and [project.dependencies] in pyproject.toml have "
        f"drifted apart.\n  requirements.txt: {from_requirements}\n"
        f"  pyproject.toml:   {from_pyproject}"
    )


def test_dev_requirements_pin_exact_versions() -> None:
    # Tooling must be pinned, not ranged: a different ruff or mypy version gives
    # different answers than the commit gate, which is worse than no tooling.
    for requirement in _read_requirements("requirements-dev.txt"):
        assert "==" in requirement, f"{requirement!r} in requirements-dev.txt must be pinned with '=='"


def test_dev_requirements_match_the_versions_pre_commit_actually_enforces() -> None:
    # The commit gate uses the `rev:` values in .pre-commit-config.yaml. If the
    # terminal tooling is a different version, local runs disagree with the gate.
    config = (REPO_ROOT / ".pre-commit-config.yaml").read_text(encoding="utf-8")

    pinned: dict[str, str] = {}
    for requirement in _read_requirements("requirements-dev.txt"):
        if "==" in requirement:
            name, version = requirement.split("==", 1)
            pinned[name] = version

    for tool in PRE_COMMIT_PINNED_TOOLS:
        expected = pinned[tool]
        assert f"rev: v{expected}" in config, (
            f"requirements-dev.txt pins {tool}=={expected}, but "
            f".pre-commit-config.yaml has no matching 'rev: v{expected}'"
        )


def test_requirements_documents_the_dependencies_pip_cannot_provide() -> None:
    # These come from Isaac Sim, ROS 2 or apt. Someone reading requirements.txt
    # must not conclude that pip install is sufficient to run the simulator.
    text = (REPO_ROOT / "requirements.txt").read_text(encoding="utf-8")
    for name in ("omni", "rclpy", "python3-gi", "python3-tk"):
        assert name in text, f"requirements.txt should mention {name}"
