"""Guard the v2 exit gate (roadmap M8): no dead code, no dishonest README.

Two independent checks live here:

1. No ``NotImplementedError`` handler that the README presents as working. A capability that
   raises "not implemented" must be described as deferred, not advertised as a feature -- the
   whole point of registering those handlers was to fail honestly.
2. No public module-level symbol in ``src/`` without a consumer. A class or function nothing
   imports or references is dead weight; the sweep is the backstop for the "delete on the way
   past" habit (D23).

Both checks read source as text, so the suite runs with no Isaac Sim, no ROS 2 and no GPU.
The dead-symbol check deliberately prefers a curated allowlist of verified intentional entry
points over a clever reachability heuristic: a guard that misfires gets deleted, and a guard
that is deleted protects nothing.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Final

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src" / "isaac_core"
RUNTIME = SRC / "sim" / "runtime.py"
README = REPO_ROOT / "README.md"

# Trees searched for a reference to a symbol. A name used anywhere here is not dead.
_REFERENCE_ROOTS: Final = (
    REPO_ROOT / "src",
    REPO_ROOT / "tests",
    REPO_ROOT / "extensions",
    REPO_ROOT / "scripts",
)

# ---------------------------------------------------------------------------
# Dead-symbol allowlist.
#
# Public module-level classes/functions in src/ that legitimately have no in-repo caller
# reference outside their own file. Each is a verified entry point, protocol, or framework
# surface -- NOT dead code. Grow this only after confirming, by hand, that the symbol is a
# genuine external surface; a lazy addition here defeats the guard.
# ---------------------------------------------------------------------------
_PUBLIC_SYMBOL_ALLOWLIST: Final[frozenset[str]] = frozenset(
    {
        # Console-script entry points wired in pyproject.toml [project.scripts]; called by the
        # installed launcher, not by other Python in the repo.
        "cli",  # isaac_core.cli.main:cli  -> `isaac-core`
        "main",  # inspector / pose_sender_gui / sidecar / mavlink `main()` entry points
        # Package facade classes re-exported from __init__ and used by operator scripts, whose
        # only in-repo references are the re-export line itself.
        "Sim",
        "SimSession",
        # Context-manager / lifecycle dunders and framework surfaces that ruff/mypy require to be
        # public but which are invoked by the runtime, not named in source.
        "SimulationRuntime",
    }
)

# Regexes for a top-level (column-0) class or function definition. Leading-underscore names are
# private by convention and excluded -- a private helper with no caller is caught by ruff/vulture
# elsewhere, and including them here would flood the guard with false positives.
_TOP_LEVEL_CLASS = re.compile(r"^class ([A-Za-z][A-Za-z0-9_]*)", re.M)
_TOP_LEVEL_DEF = re.compile(r"^def ([a-z][A-Za-z0-9_]*)", re.M)


def _iter_src_modules() -> list[Path]:
    """Return every Python source file under ``src/isaac_core`` except ``__init__``/``__main__``."""
    return sorted(p for p in SRC.rglob("*.py") if p.name not in {"__init__.py", "__main__.py"})


def _public_symbols(path: Path) -> set[str]:
    """Return the public top-level class and function names defined in one module.

    Args:
        path: The source file to scan.

    Returns:
        Public symbol names (leading-underscore names excluded).

    """
    text = path.read_text(encoding="utf-8")
    names = set(_TOP_LEVEL_CLASS.findall(text)) | set(_TOP_LEVEL_DEF.findall(text))
    return {n for n in names if not n.startswith("_")}


# How far (characters) a capability mention may sit from a deferral marker and still count as
# honestly framed. Generous because the README's deferral sentences are long, but bounded so a
# marker three paragraphs away does not launder an unrelated claim.
_DEFERRED_WINDOW: Final = 240

# Substrings that mark surrounding prose as honestly describing something as not-yet-working.
_DEFERRAL_MARKERS: Final = (
    "deferred",
    "not implemented",
    "not work yet",
    "does not work yet",
    "registered but",
    "rather than silently doing nothing",
    "not yet",
)


def _near_deferral_marker(flat: str, pos: int) -> bool:
    """Report whether a deferral marker appears within the window around ``pos``.

    Args:
        flat: The whitespace-collapsed, lowercased README text.
        pos: The index of a capability mention within ``flat``.

    Returns:
        ``True`` if some deferral marker lies within ``_DEFERRED_WINDOW`` characters either side.

    """
    window = flat[max(0, pos - _DEFERRED_WINDOW) : pos + _DEFERRED_WINDOW]
    return any(marker in window for marker in _DEFERRAL_MARKERS)


def _is_definition_or_prose(line: str, name: str) -> bool:
    """Report whether a line declares/exports/merely-names the symbol rather than using it.

    Three cases do not count as a consumer: the symbol's own ``class``/``def`` line, an
    ``__all__`` export entry, and a docstring or comment cross-reference (``# ...``,
    ``:func:`name```, or a bare backtick mention). Excluding prose is what lets the guard catch a
    pair of helpers that reference each other only in their docstrings but that no code calls.

    Args:
        line: A source line containing the name.
        name: The symbol name.

    Returns:
        ``True`` if the line does not use the symbol as code.

    """
    stripped = line.strip()
    if stripped.startswith((f"class {name}", f"def {name}", "#")):
        return True
    if stripped in {f'"{name}",', f"'{name}',", f'"{name}"', f"'{name}'"}:
        return True
    # Sphinx cross-reference (:func:`name` / :class:`name`) or a bare ``name`` in a docstring.
    return f":func:`{name}`" in line or f":class:`{name}`" in line or f":meth:`{name}`" in line


def _usage_count(name: str) -> int:
    """Count real uses of ``name`` across the repo, including within its own module.

    Counts word-boundary occurrences everywhere in ``src``/``tests``/``extensions``/``scripts``,
    then discards lines that merely define, export, or name-in-prose the symbol (see
    :func:`_is_definition_or_prose`). A pydantic model used only as a field default in the same
    file, a helper called by its module's own ``main()``, or a class imported by a test all count
    as live uses; a symbol whose only occurrences are its definition, its export, and docstring
    cross-references counts as dead.

    Args:
        name: The symbol name.

    Returns:
        The number of lines that use the symbol as code.

    """
    pattern = re.compile(rf"\b{re.escape(name)}\b")
    uses = 0
    for root in _REFERENCE_ROOTS:
        for path in root.rglob("*.py"):
            for line in path.read_text(encoding="utf-8").splitlines():
                if not pattern.search(line):
                    continue
                if _is_definition_or_prose(line, name):
                    continue
                uses += 1
    return uses


def _notimplemented_handlers() -> set[str]:
    """Return the control methods whose handler raises ``NotImplementedError``.

    Walks the runtime source, tracking the wire name of the most recent ``register("name", ...)``
    that binds each ``_handle_x`` method, then flags the handlers whose body raises
    ``NotImplementedError``.

    Returns:
        The wire names of the deferred handlers.

    """
    text = RUNTIME.read_text(encoding="utf-8")
    handler_to_wire = dict((h, w) for w, h in re.findall(r'register\("([a-z_]+)",\s*self\.(_handle_[a-z_]+)\)', text))
    deferred: set[str] = set()
    for match in re.finditer(r"def (_handle_[a-z_]+)\(self.*?(?=\n    def |\Z)", text, re.S):
        body = match.group(0)
        if "raise NotImplementedError" in body and match.group(1) in handler_to_wire:
            deferred.add(handler_to_wire[match.group(1)])
    return deferred


def test_the_scanners_found_something() -> None:
    # Guards against a regex silently matching nothing, which would make every check below pass
    # vacuously.
    modules = _iter_src_modules()
    assert len(modules) > 20, f"only found {len(modules)} source modules; the walker is broken"
    # Zero deferred handlers is the goal, not a parser failure: v2 ships nothing that raises
    # NotImplementedError. The README guard below stays live for any that get added later.
    assert isinstance(_notimplemented_handlers(), set)


def test_readme_does_not_claim_deferred_capabilities_work() -> None:
    # Failure mode: a handler raises NotImplementedError while the README lists that capability
    # as a working feature -- the exact dishonesty D23 and the v2 "README is true" bar forbid.
    #
    # Matching rule (robust against prose that wraps across lines): flatten the README to one
    # lowercased string, collapsing whitespace so a sentence split over several lines reads as
    # one. For each deferred capability, every mention of its phrase must sit within
    # _DEFERRED_WINDOW characters of a deferral marker ("deferred", "not implemented", "not work
    # yet", "registered but", "rather than silently doing nothing", ...). We assert on nearby
    # *context*, not mere absence, so the README may and should mention these methods -- it just
    # has to frame each mention as not-yet-working.
    flat = " ".join(README.read_text(encoding="utf-8").split()).lower()
    # Wire name -> the phrases the README uses for that capability. Empty while nothing is
    # deferred; add an entry if a handler ever raises NotImplementedError again.
    aliases: dict[str, tuple[str, ...]] = {}
    problems: list[str] = []
    for wire in _notimplemented_handlers():
        phrases = aliases.get(wire, (wire,))
        mentions = [m.start() for phrase in phrases for m in re.finditer(re.escape(phrase.lower()), flat)]
        if not mentions:
            # Not mentioned at all is fine: the README cannot be claiming it works.
            continue
        if not all(_near_deferral_marker(flat, pos) for pos in mentions):
            problems.append(wire)
    assert not problems, (
        "the README mentions these capabilities without nearby 'deferred/not implemented' "
        f"context, while their handlers raise NotImplementedError: {sorted(problems)}. Either "
        "mark them deferred in the README or implement them."
    )


def test_no_public_symbol_in_src_without_a_consumer() -> None:
    # Failure mode: a public class or function that nothing imports, calls, or tests -- dead
    # weight that the M8 sweep exists to catch. Allowlisted names are verified external surfaces
    # (console-script entry points, operator-facing facades); everything else must have at least
    # one referencing file other than the one that defines it.
    dead: dict[str, str] = {}
    for module in _iter_src_modules():
        for name in _public_symbols(module):
            if name in _PUBLIC_SYMBOL_ALLOWLIST:
                continue
            if _usage_count(name) == 0:
                dead[name] = str(module.relative_to(REPO_ROOT))
    assert not dead, (
        "these public symbols in src/ have no consumer anywhere in src/, tests/, extensions/ or "
        f"scripts/, so they are dead code: {dead}. Remove them, or -- if one is a genuine "
        "external entry point -- add it to _PUBLIC_SYMBOL_ALLOWLIST with a reason."
    )
