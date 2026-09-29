"""No archaeology in `src` or `scripts`.

The constraint a reader needs is what the code requires *now*. Narrating what a previous generation did makes
a file harder to read and dates it: "year-based versions predate the 6.x numbering" tells you less than "only
6.x and newer are supported".

Nothing enforced this, which is why nine sites had accumulated by the end of V3. They are gone; this stops
them coming back.

`docs/` is deliberately out of scope -- the engineering log and the migration notes exist precisely to hold
history. So are `tests/`, where naming the thing a test proves is rejected describes behaviour rather than
narrating the past: `test_is_supported_false_for_2023` is a better name than any euphemism for it.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Final

REPO_ROOT: Final = Path(__file__).resolve().parents[2]

# Words that signal narration rather than a constraint. Deliberately narrow: a broad list would trip over
# ordinary English ("the legacy of", "previously computed") and get suppressed rather than obeyed.
_PATTERNS: Final[tuple[tuple[str, str], ...]] = (
    (r"\b20(1[0-9]|2[0-4])\b", "a year, which dates the file instead of stating the constraint"),
    (r"previous generation", "describes an earlier codebase"),
    (r"\bthe old repo\b", "describes an earlier codebase"),
    (r"used to be\b", "narrates what changed rather than what is required"),
    (r"\bpredates?\b", "narrates history"),
    (r"\blegacy\b", "narrates history"),
)

# Each exemption needs a reason. A bare skip list rots into a place where violations hide.
_EXEMPT: Final[dict[str, str]] = {
    # Describes where a target sits during a manoeuvre, not the repo's past: the aim is recomputed per pose
    # because the bearing changes as the vehicle travels.
    "src/isaac_core/devkit/bot.py": "'where the target used to be relative to the aircraft' is geometry",
}


def _scanned_files() -> list[Path]:
    """Return the Python files this rule covers.

    Returns:
        Every tracked ``.py`` file under ``src`` and ``scripts``.

    """
    found: list[Path] = []
    for directory in ("src", "scripts"):
        for path in sorted((REPO_ROOT / directory).rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            found.append(path)
    return found


def test_the_scanner_found_files() -> None:
    # Without this the check below could pass by reading nothing, which is the shape of a vacuous guard.
    files = _scanned_files()
    assert len(files) > 30, f"only found {len(files)} files; the walker is broken"


def test_no_archaeology_in_src_or_scripts() -> None:
    offences: list[str] = []
    for path in _scanned_files():
        relative = path.relative_to(REPO_ROOT).as_posix()
        if relative in _EXEMPT:
            continue
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            for pattern, why in _PATTERNS:
                if re.search(pattern, line, re.IGNORECASE):
                    offences.append(f"{relative}:{number}: {why} -- {line.strip()[:90]}")
    assert not offences, "archaeology in src/ or scripts/:\n" + "\n".join(offences)


def test_every_exemption_still_applies() -> None:
    # An exemption naming a file that no longer offends is a stale allowlist entry, and those are how a
    # guard quietly stops guarding.
    for relative, reason in _EXEMPT.items():
        path = REPO_ROOT / relative
        assert path.is_file(), f"exemption names a missing file: {relative}"
        text = path.read_text(encoding="utf-8")
        assert any(re.search(pattern, text, re.IGNORECASE) for pattern, _ in _PATTERNS), (
            f"{relative} no longer contains anything matching the patterns, so its exemption "
            f"({reason}) can be removed"
        )
