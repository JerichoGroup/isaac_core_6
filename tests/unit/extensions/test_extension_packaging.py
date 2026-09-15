"""Contract tests for Kit extension packaging metadata.

These cover the parts of an extension that are declaration rather than code: the
``[package]`` block, the icon asset, and the docs entry. Kit reads all of it, but
none of it is exercised by the .ogn or node tests, so an omission here is silent
until someone opens the extension manager and sees a broken entry -- which is
exactly how the missing ``data/`` directory was found (by eye, not by a test).

Everything is parsed as data, so these run with no Isaac Sim present.
"""

from pathlib import Path
import struct
import sys

import pytest

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

REPO_ROOT = Path(__file__).resolve().parents[3]
EXTENSIONS_ROOT = REPO_ROOT / "extensions"

# Kit shows the icon at a small size in the extension manager, so a very large
# source image buys nothing. This ceiling also keeps us under the default 500 kB
# of the check-added-large-files hook, which only inspects files once they are
# staged -- so an oversized icon passes locally and then blocks a commit.
MAX_ICON_KB = 500

EXTENSION_TOMLS = sorted(EXTENSIONS_ROOT.rglob("config/extension.toml"))


def _extension_root(toml_path: Path) -> Path:
    """Return the extension directory that owns a config/extension.toml."""
    return toml_path.parent.parent


def _load(toml_path: Path) -> dict[str, object]:
    """Parse an extension.toml into a plain dict."""
    return tomllib.loads(toml_path.read_text(encoding="utf-8"))


def _png_dimensions(path: Path) -> tuple[int, int]:
    """Return a PNG's pixel width and height, read straight from its IHDR chunk."""
    header = path.read_bytes()[16:24]
    width, height = struct.unpack(">II", header)
    return (width, height)


def test_extensions_were_discovered() -> None:
    # Guard against the glob silently matching nothing and every test below
    # vacuously passing.
    assert EXTENSION_TOMLS, f"no extension.toml found under {EXTENSIONS_ROOT}"


def test_every_extension_has_a_data_directory() -> None:
    for toml_path in EXTENSION_TOMLS:
        data_dir = _extension_root(toml_path) / "data"
        assert data_dir.is_dir(), f"{_extension_root(toml_path).name} has no data/ directory"


@pytest.mark.parametrize("toml_path", EXTENSION_TOMLS, ids=lambda p: _extension_root(p).name)
def test_extension_declares_an_icon(toml_path: Path) -> None:
    package = _load(toml_path)["package"]
    assert isinstance(package, dict)
    assert "icon" in package, f"{_extension_root(toml_path).name} declares no package.icon"


@pytest.mark.parametrize("toml_path", EXTENSION_TOMLS, ids=lambda p: _extension_root(p).name)
def test_declared_icon_file_exists(toml_path: Path) -> None:
    package = _load(toml_path)["package"]
    assert isinstance(package, dict)
    icon = package["icon"]
    assert isinstance(icon, str)
    resolved = _extension_root(toml_path) / icon
    assert resolved.is_file(), f"icon {icon!r} declared but missing at {resolved}"


@pytest.mark.parametrize("toml_path", EXTENSION_TOMLS, ids=lambda p: _extension_root(p).name)
def test_declared_icon_lives_under_data(toml_path: Path) -> None:
    package = _load(toml_path)["package"]
    assert isinstance(package, dict)
    icon = package["icon"]
    assert isinstance(icon, str)
    assert icon.startswith("data/"), f"icon {icon!r} should live under data/"


@pytest.mark.parametrize("toml_path", EXTENSION_TOMLS, ids=lambda p: _extension_root(p).name)
def test_declared_icon_is_a_valid_png(toml_path: Path) -> None:
    package = _load(toml_path)["package"]
    assert isinstance(package, dict)
    icon = package["icon"]
    assert isinstance(icon, str)
    resolved = _extension_root(toml_path) / icon
    assert resolved.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n", f"{icon} is not a PNG"
    width, height = _png_dimensions(resolved)
    assert width > 0
    assert height > 0


@pytest.mark.parametrize("toml_path", EXTENSION_TOMLS, ids=lambda p: _extension_root(p).name)
def test_declared_readme_exists(toml_path: Path) -> None:
    package = _load(toml_path)["package"]
    assert isinstance(package, dict)
    readme = package.get("readme")
    assert isinstance(readme, str), f"{_extension_root(toml_path).name} declares no package.readme"
    assert (_extension_root(toml_path) / readme).is_file()


@pytest.mark.parametrize("toml_path", EXTENSION_TOMLS, ids=lambda p: _extension_root(p).name)
def test_extension_declares_title_and_description(toml_path: Path) -> None:
    package = _load(toml_path)["package"]
    assert isinstance(package, dict)
    for field in ("version", "title", "description"):
        value = package.get(field)
        assert isinstance(value, str) and value.strip(), f"package.{field} missing or empty"


def test_no_icon_would_trip_the_large_files_hook_once_staged() -> None:
    # check-added-large-files only inspects files that are STAGED, so an oversized
    # icon passes every local run and then blocks the commit. Catch it here instead.
    oversized = []
    for toml_path in EXTENSION_TOMLS:
        package = _load(toml_path)["package"]
        assert isinstance(package, dict)
        icon = package["icon"]
        assert isinstance(icon, str)
        resolved = _extension_root(toml_path) / icon
        size_kb = resolved.stat().st_size // 1024
        if size_kb > MAX_ICON_KB:
            width, height = _png_dimensions(resolved)
            oversized.append(f"{resolved.relative_to(REPO_ROOT)} is {size_kb} kB ({width}x{height})")

    assert not oversized, (
        "icon(s) exceed "
        f"{MAX_ICON_KB} kB and will be rejected by check-added-large-files when staged:\n  "
        + "\n  ".join(oversized)
        + "\nEither downscale the image or raise the hook's --maxkb."
    )
