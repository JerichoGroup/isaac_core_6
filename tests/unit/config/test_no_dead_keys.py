"""
Guard against config keys that are declared but never read.

A field that parses and validates but that nothing consumes is worse than not offering it:
setting it looks like it worked. Two such keys were reported as bugs by the team before this
guard existed, and an audit then found ten more.

The known-dead set below is an explicit, shrinking backlog. Adding to it should be a
deliberate decision, not an accident.
"""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
from typing import Final

REPO_ROOT = Path(__file__).resolve().parents[3]
SCHEMA = REPO_ROOT / "src" / "isaac_core" / "config" / "schema.py"

# Directories that legitimately consume config values.
_SEARCH_ROOTS: Final = ("src/isaac_core", "extensions", "scripts")

# Fields known to be unapplied, tracked in docs/roadmap.md. Shrink this; do not grow it.
_KNOWN_DEAD: Final[frozenset[str]] = frozenset()


def _schema_fields() -> list[str]:
    """Return every field name declared on a config model."""
    text = SCHEMA.read_text(encoding="utf-8")
    return sorted(set(re.findall(r"^\s{4}([a-z_][a-z0-9_]*)\s*:\s*[^=\n]+=", text, re.M)))


def _is_referenced(name: str) -> bool:
    """Report whether a field name appears anywhere outside the schema itself."""
    result = subprocess.run(
        [
            "grep",
            "-rn",
            "--include=*.py",
            "--include=*.toml",
            "--include=*.ogn",
            name,
            *_SEARCH_ROOTS,
        ],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        check=False,
    )
    return any("config/schema.py" not in line for line in result.stdout.splitlines())


def test_schema_fields_were_found() -> None:
    # Without this the test below could pass by parsing nothing.
    fields = _schema_fields()
    assert len(fields) > 40, f"only found {len(fields)} fields; the parser is probably broken"


def test_no_new_dead_config_keys() -> None:
    dead = {name for name in _schema_fields() if not _is_referenced(name)}
    new_dead = dead - _KNOWN_DEAD
    assert not new_dead, (
        "these config keys are declared but never read, so setting them silently does " f"nothing: {sorted(new_dead)}"
    )


def test_the_known_dead_list_is_still_accurate() -> None:
    # Keeps the backlog honest: once a key is wired up, it must leave the list, otherwise
    # the list stops meaning anything.
    dead = {name for name in _schema_fields() if not _is_referenced(name)}
    fixed = _KNOWN_DEAD - dead
    assert not fixed, f"these are no longer dead and should leave _KNOWN_DEAD: {sorted(fixed)}"
