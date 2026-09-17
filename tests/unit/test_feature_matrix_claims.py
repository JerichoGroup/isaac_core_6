"""Keep the feature matrix's claims true.

`docs/dev/feature_matrix.md` is the document that certifies what works, which makes a wrong statement
in it worse than a wrong statement anywhere else. Two kinds of rot are mechanically detectable and both
have already happened:

*Counts.* "13 `Method` enum members" was written when there were 13 and read as fact when there were 15.
Any hand-maintained number in a certifying document eventually lies.

*Contradictions.* A row read "Entry-point layers from another package | pass" while the roadmap listed
the same capability as not built, and the test it cited never mentioned entry points. Two documents
disagreeing means at least one is wrong, and a reader cannot tell which.

What cannot be checked mechanically is whether a cited test exercises the surface a user touches. That
is a judgement, and getting it wrong is what let six bugs ship past a green suite; the rule it produced
is written into the matrix under F8.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys
from typing import Final

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from isaac_core.config import IsaacCoreConfig
from isaac_core.control.messages import Method
from isaac_core.devkit.session import SimSession

REPO_ROOT: Final = Path(__file__).resolve().parents[2]
MATRIX: Final = REPO_ROOT / "docs" / "dev" / "feature_matrix.md"
ROADMAP: Final = REPO_ROOT / "docs" / "dev" / "roadmap.md"


def _matrix_text() -> str:
    """Return the matrix's contents."""
    return MATRIX.read_text(encoding="utf-8")


def test_the_matrix_exists() -> None:
    # Without this the checks below would pass by reading nothing.
    assert MATRIX.is_file()
    assert len(_matrix_text()) > 2000


def test_the_method_count_matches_the_enum() -> None:
    text = _matrix_text()
    match = re.search(r"(\d+) `Method` enum members", text)
    assert match, "the matrix no longer states a Method count; update this test with it"
    assert int(match.group(1)) == len(
        list(Method)
    ), f"the matrix says {match.group(1)} Method members, the enum has {len(list(Method))}"


def test_the_handler_count_matches_the_runtime() -> None:
    text = _matrix_text()
    match = re.search(r"(\d+) registered handlers", text)
    assert match, "the matrix no longer states a handler count"
    runtime = (REPO_ROOT / "src" / "isaac_core" / "sim" / "runtime.py").read_text(encoding="utf-8")
    actual = len(set(re.findall(r"def (_handle_[a-z_]+)", runtime)))
    assert int(match.group(1)) == actual, f"the matrix says {match.group(1)} handlers, the runtime has {actual}"


def test_the_session_member_count_matches_the_class() -> None:
    text = _matrix_text()
    match = re.search(r"(\d+) public members on `SimSession`", text)
    assert match, "the matrix no longer states a SimSession member count"
    actual = len([name for name in dir(SimSession) if not name.startswith("_")])
    assert int(match.group(1)) == actual, f"the matrix says {match.group(1)} members, the class has {actual}"


def test_the_config_section_count_matches_the_schema() -> None:
    text = _matrix_text()
    match = re.search(r"(\d+) top-level sections", text)
    assert match, "the matrix no longer states a config section count"
    actual = len(IsaacCoreConfig.model_fields)
    assert int(match.group(1)) == actual, f"the matrix says {match.group(1)} sections, the schema has {actual}"


def test_the_shipped_layer_count_matches_the_package() -> None:
    text = _matrix_text()
    assert "Four shipped layers" in text, "the matrix no longer states a layer count"
    layers = sorted(p.parent.name for p in (REPO_ROOT / "src/isaac_core/assets/layers").glob("*/layer.toml"))
    assert len(layers) == 4, f"the matrix says four shipped layers, the package has {layers}"
    for layer in layers:
        assert f"`{layer}`" in text, f"the shipped layer {layer} is not named in the matrix"


def test_the_console_script_count_matches_pyproject() -> None:
    text = _matrix_text()
    assert "Four console scripts" in text, "the matrix no longer states a script count"
    scripts = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8"))["project"]["scripts"]
    assert len(scripts) == 4, f"the matrix says four console scripts, pyproject declares {sorted(scripts)}"


def test_every_cited_evidence_file_exists() -> None:
    # A row citing a file that has been renamed or deleted certifies nothing.
    text = _matrix_text()
    # Cells cite a path when the directory disambiguates and a bare filename when a second file is
    # listed alongside the first, so resolution is by basename anywhere under tests/ or src/.
    known: set[str] = set()
    for directory in ("tests", "src", "scripts"):
        known |= {path.name for path in REPO_ROOT.joinpath(directory).rglob("*.py")}
    missing = [
        cited for cited in sorted(set(re.findall(r"`([a-z_0-9/]+\.py)`", text))) if Path(cited).name not in known
    ]
    assert not missing, f"the matrix cites files that do not exist: {missing}"


def test_no_row_claims_a_capability_the_roadmap_calls_unbuilt() -> None:
    # The failure this guards: "Entry-point layers from another package | pass" sat in the matrix while
    # the roadmap's live-gaps section described the same thing as not built.
    matrix = _matrix_text()
    roadmap = ROADMAP.read_text(encoding="utf-8")
    gap_headings = re.findall(r"^## (.+)$", roadmap.split("## Version 1")[0], re.M)

    problems: list[str] = []
    for heading in gap_headings:
        # A gap's distinguishing words, ignoring filler, so the match is not defeated by rewording.
        keywords = [word for word in re.findall(r"[a-z]{5,}", heading.lower()) if word not in {"about"}]
        if not keywords:
            continue
        for line in matrix.splitlines():
            lowered = line.lower()
            if not lowered.strip().startswith("|") or "| pass |" not in lowered:
                continue
            if all(keyword in lowered for keyword in keywords):
                problems.append(f"{heading!r} is a roadmap gap but the matrix row says pass: {line.strip()[:80]}")
    assert not problems, problems


def test_the_audit_rule_is_recorded() -> None:
    # The lesson is worth more than the individual fixes, so it has to survive in the document that
    # people read before writing a new row.
    text = _matrix_text()
    assert "F8" in text, "the audit finding is not recorded"
    assert "same entry point a user" in text, "the rule the audit produced is not stated"
