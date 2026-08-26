"""
Guard against the Isaac Sim 4.5 / 6.0 API removals reappearing in code.

Two whole namespaces are gone, and both are easy to reintroduce because our Isaac
imports are *lazy strings* handed to ``importlib.import_module`` -- invisible to ruff,
to mypy, and to the kernel purity test. The only other thing that catches them is
launching Isaac Sim, which is slow, so catch them here.

``omni.isaac.*``
    Renamed wholesale to ``isaacsim.*`` in Isaac Sim 4.5.

``SimulationContext`` / ``isaacsim.core.api``
    Removed in Isaac Sim 6.0, which replaced the Core API with
    ``isaacsim.core.experimental.*``. Verified absent from the 6.0.1 install: no
    ``isaacsim.core.api`` extension and no ``SimulationContext`` class anywhere in it.
    Play/pause/step come from ``isaacsim.core.experimental.utils.app`` and physics from
    ``isaacsim.core.simulation_manager.SimulationManager`` -- what Isaac's own
    standalone examples use.

Analysis is over the AST, not the text, so docstrings explaining the rename (there are
several, deliberately) cannot trip it. Only real imports, real ``import_module`` string
arguments, and real attribute or name references count.
"""

import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]

SEARCH_ROOTS = (REPO_ROOT / "src", REPO_ROOT / "extensions", REPO_ROOT / "scripts")

# Dotted module prefixes that no longer exist, mapped to what to use instead.
DEAD_MODULE_PREFIXES = {
    "omni.isaac": "isaacsim.* (every omni.isaac.* extension was renamed in Isaac Sim 4.5)",
    "isaacsim.core.api": ("isaacsim.core.experimental.* (the Core API was removed in Isaac Sim 6.0)"),
}

# Names that no longer exist as importable symbols.
DEAD_NAMES = {
    "SimulationContext": (
        "isaacsim.core.experimental.utils.app for play/pause/step and "
        "isaacsim.core.simulation_manager.SimulationManager for physics"
    ),
}


def _python_files() -> list[Path]:
    """Return every Python file under the searched roots."""
    return sorted(path for root in SEARCH_ROOTS for path in root.rglob("*.py"))


def _dead_module_hits(tree: ast.AST) -> list[tuple[int, str]]:
    """Return (line, module) for every import or import_module of a dead module."""
    hits: list[tuple[int, str]] = []

    def is_dead(module: str) -> str | None:
        for prefix in DEAD_MODULE_PREFIXES:
            if module == prefix or module.startswith(f"{prefix}."):
                return prefix
        return None

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if is_dead(alias.name):
                    hits.append((node.lineno, alias.name))
        elif isinstance(node, ast.ImportFrom) and node.module and is_dead(node.module):
            hits.append((node.lineno, node.module))
        elif isinstance(node, ast.Call):
            # importlib.import_module("omni.isaac.core") and friends.
            for arg in node.args:
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str) and is_dead(arg.value):
                    hits.append((node.lineno, arg.value))
    return hits


def _dead_name_hits(tree: ast.AST) -> list[tuple[int, str]]:
    """Return (line, name) for every reference to a removed symbol."""
    hits: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id in DEAD_NAMES:
            hits.append((node.lineno, node.id))
        elif isinstance(node, ast.Attribute) and node.attr in DEAD_NAMES:
            hits.append((node.lineno, node.attr))
        elif isinstance(node, ast.Constant) and isinstance(node.value, str) and node.value in DEAD_NAMES:
            # A string constant naming the symbol, as passed to getattr.
            hits.append((node.lineno, node.value))
    return hits


def test_python_files_were_found() -> None:
    # Without this, both tests below would pass vacuously.
    assert _python_files(), "no Python files found to scan"


def test_no_code_imports_a_removed_isaac_module() -> None:
    offenders: list[str] = []
    for path in _python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for line, module in _dead_module_hits(tree):
            offenders.append(f"{path.relative_to(REPO_ROOT)}:{line}: {module}")

    assert not offenders, (
        "these modules were removed from Isaac Sim and do not exist in the 6.0.1 "
        "install:\n  "
        + "\n  ".join(offenders)
        + "\nReplacements: "
        + "; ".join(f"{k} -> {v}" for k, v in DEAD_MODULE_PREFIXES.items())
    )


def test_no_code_references_a_removed_isaac_symbol() -> None:
    offenders: list[str] = []
    for path in _python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for line, name in _dead_name_hits(tree):
            offenders.append(f"{path.relative_to(REPO_ROOT)}:{line}: {name}")

    assert not offenders, (
        "these symbols were removed from Isaac Sim 6.0:\n  "
        + "\n  ".join(offenders)
        + "\nReplacements: "
        + "; ".join(f"{k} -> {v}" for k, v in DEAD_NAMES.items())
    )


@pytest.mark.parametrize(
    "snippet",
    [
        'importlib.import_module("omni.isaac.core")',
        "import omni.isaac.core",
        "from omni.isaac.core import SimulationContext",
        'importlib.import_module("isaacsim.core.api")',
    ],
    ids=["import_module", "import", "from_import", "core_api"],
)
def test_the_guard_actually_detects_dead_usage(snippet: str) -> None:
    # A guard nobody has seen fail is a guard nobody should trust.
    tree = ast.parse(snippet)
    assert _dead_module_hits(tree), f"guard missed {snippet!r}"


def test_the_guard_ignores_prose_mentioning_the_old_names() -> None:
    # install.py deliberately explains the rename in its docstrings; that must not trip.
    tree = ast.parse('"""Every omni.isaac.* extension was renamed, and SimulationContext is gone."""\n')
    assert not _dead_module_hits(tree)
    assert not _dead_name_hits(tree)
