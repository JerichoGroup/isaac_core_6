"""Guards that keep the README's factual claims true as the code changes."""

from __future__ import annotations

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[2]
README = REPO_ROOT / "README.md"

# Images are produced by hand and tracked in docs/dev/usd_build_sheet.md. A missing image renders as a
# broken image and breaks nothing else, so they are reported separately from document links.
_IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".gif", ".svg")


def _relative_links() -> list[str]:
    """Return every repo-relative link target in the README."""
    text = README.read_text(encoding="utf-8")
    return [
        target
        for target in re.findall(r"\]\(([^)\s]+)\)", text)
        if not target.startswith(("http://", "https://", "#", "mailto:"))
    ]


def test_every_readme_document_link_resolves() -> None:
    # A README that points at documents which do not exist is the first thing a newcomer hits.
    missing = [
        link
        for link in _relative_links()
        if not link.lower().endswith(_IMAGE_SUFFIXES) and not (REPO_ROOT / link.split("#")[0]).exists()
    ]
    assert not missing, f"README links to missing files: {missing}"


def test_readme_documents_only_layers_that_ship() -> None:
    shipped = {p.name for p in (REPO_ROOT / "src" / "isaac_core" / "assets" / "layers").iterdir() if p.is_dir()}
    text = README.read_text(encoding="utf-8")
    # Layer ids appear in [features] examples as quoted strings.
    claimed = set(re.findall(r'"(camera_udp|camera_ros|distance_sensor|bbox|sat)"', text))
    unknown = claimed - shipped
    assert not unknown, f"README documents layers that do not ship: {sorted(unknown)}"


def test_readme_does_not_claim_unimplemented_capability() -> None:
    # v2 ships nothing that raises NotImplementedError. If that changes, the README must say so,
    # and tests/unit/test_no_dead_code.py enforces the framing. This guards the inverse: the README
    # should not carry a stale "does not work yet" list once the capability is gone.
    text = README.read_text(encoding="utf-8").lower()
    for phrase in ("what does not work yet", "not implemented yet"):
        assert phrase not in text, f"README still contains a stale section: {phrase!r}"


def test_readme_test_count_matches_the_suite() -> None:
    # The count appeared three different ways across the docs (1103, 1279, 1558) while the suite was
    # at 1571, which makes a reader distrust every other number in the file.
    text = README.read_text(encoding="utf-8")
    claimed = {int(n) for n in re.findall(r"([0-9]{3,5}) tests pass", text)}
    if not claimed:
        return
    collected = sum(
        1
        for path in (REPO_ROOT / "tests").rglob("test_*.py")
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.startswith("def test_")
    )
    # Parametrized tests mean the collected count exceeds the function count, so assert the README
    # is not wildly stale rather than exactly equal.
    assert claimed, "no count found"
    for value in claimed:
        assert value >= collected, (
            f"README claims {value} tests but {collected} test functions exist before parametrisation; "
            "the number looks stale"
        )


def test_every_toml_example_in_the_readme_actually_loads() -> None:
    # The camera example shipped with `width`/`height`, which are read-only properties rather than
    # config keys, so the one snippet a reader is most likely to copy was rejected at load. Every
    # model forbids extra keys, so a wrong key name is always a hard error and always catchable here.
    import tempfile

    from isaac_core.config import load

    failures: list[str] = []
    for block in re.findall(r"```toml\n(.*?)```", README.read_text(encoding="utf-8"), re.S):
        # Layer manifests and packaging snippets are TOML but are not simulator config.
        if "[[bindings]]" in block or block.strip().startswith("id =") or "[project" in block or "[tool." in block:
            continue
        with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as handle:
            handle.write(block)
            path = Path(handle.name)
        try:
            load(path=path)
        except Exception as error:
            failures.append(f"{block.strip().splitlines()[0]}: {type(error).__name__}: {error}")
        finally:
            path.unlink(missing_ok=True)
    assert not failures, f"README TOML examples that do not load: {failures}"


def test_no_document_uses_the_equals_form_of_set() -> None:
    # `--set` is nargs=2, so `--set key=value` is rejected with "expected 2 arguments". The equals
    # form had spread to five files, and every one of those command lines would have failed.
    # Only pages a user is told to read. docs/dev/ and the engineering log record the bugs we fixed, so
    # they have to be able to write the wrong form down.
    offenders: list[str] = []
    pages = [
        README,
        *(page for page in (REPO_ROOT / "docs").glob("*.md") if page.name != "development-log.md"),
        REPO_ROOT / "config" / "default.toml",
    ]
    for page in pages:
        for number, line in enumerate(page.read_text(encoding="utf-8").splitlines(), start=1):
            if re.search(r"--set\s+[A-Za-z_][A-Za-z0-9_.]*=", line):
                offenders.append(f"{page.relative_to(REPO_ROOT)}:{number}")
    assert not offenders, f"--set takes two arguments, not key=value: {offenders}"


def test_the_readme_does_not_claim_layers_can_be_registered_by_entry_point() -> None:
    # Layer discovery only scans directories on assets.layer_search_paths plus the packaged layers.
    # There is no importlib.metadata lookup, so a layer registered only by entry point is never found.
    text = README.read_text(encoding="utf-8")
    assert "isaac_core.layers" not in text, "entry-point layer registration is documented but not implemented"
