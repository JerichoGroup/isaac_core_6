"""Layer registration by entry point.

The point of this feature is that installing a layer is just installing a package. So the important test
is not that a mocked entry point resolves — it is that a **real distribution** on `sys.path`, declaring a
real entry point, contributes its directory and that a layer inside it is then discovered.

This closed a documentation lie: the README once described the entry-point form, the feature matrix
claimed it passed citing a test that never mentioned entry points, and `docs/dev/roadmap.md` listed it as
a gap. Only the roadmap was right.

Nothing here installs anything. A distribution is faked on disk in `tmp_path` with the `.dist-info`
metadata `importlib.metadata` reads, which is the same thing pip would leave behind.
"""

from __future__ import annotations

import importlib
import importlib.metadata
from pathlib import Path
import sys
import textwrap
from typing import Any

import pytest

from isaac_core.sim.discovery import ENTRY_POINT_GROUP, discover_layers, entry_point_search_paths


def _write_layer(directory: Path, layer_id: str) -> Path:
    """Write a minimal behaviour-only layer manifest into a directory.

    Args:
        directory: Where the layer package lives.
        layer_id: The layer's id, used as its folder name.

    Returns:
        The layer directory.

    """
    layer = directory / layer_id
    layer.mkdir(parents=True, exist_ok=True)
    (layer / "layer.toml").write_text(
        textwrap.dedent(f"""
            id = "{layer_id}"
            mount = "/World/Environment/{{instance}}"
            requires = []
            provides = ["{layer_id.upper()}"]
        """).strip()
        + "\n",
        encoding="utf-8",
    )
    return layer


def _install_fake_distribution(root: Path, package: str, *, entry_value: str, layer_id: str) -> Path:
    """Create a package and the `.dist-info` metadata that declares its layer entry point.

    This is what pip leaves on disk, so `importlib.metadata` finds it with no patching.

    Args:
        root: A directory to put on `sys.path`.
        package: Importable package name.
        entry_value: The entry point's value, e.g. ``"my_pkg.layers"``.
        layer_id: A layer to place inside the package.

    Returns:
        The package's layers directory.

    """
    layers_dir = root / package / "layers"
    layers_dir.mkdir(parents=True, exist_ok=True)
    (root / package / "__init__.py").write_text("", encoding="utf-8")
    (layers_dir / "__init__.py").write_text("", encoding="utf-8")
    _write_layer(layers_dir, layer_id)

    dist_info = root / f"{package}-1.0.0.dist-info"
    dist_info.mkdir(parents=True, exist_ok=True)
    (dist_info / "METADATA").write_text(f"Metadata-Version: 2.1\nName: {package}\nVersion: 1.0.0\n", encoding="utf-8")
    (dist_info / "entry_points.txt").write_text(
        f"[{ENTRY_POINT_GROUP}]\n{layer_id} = {entry_value}\n", encoding="utf-8"
    )
    return layers_dir


@pytest.fixture
def on_path(tmp_path: Path) -> Any:
    """Put a temporary directory on `sys.path` and remove it afterwards.

    Yields:
        The directory, ready to hold fake distributions.

    """
    root = tmp_path / "site-packages"
    root.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(root))
    importlib.invalidate_caches()
    try:
        yield root
    finally:
        sys.path.remove(str(root))
        for name in [n for n in sys.modules if n.startswith("fake_layer_pkg")]:
            del sys.modules[name]
        importlib.invalidate_caches()


# -- the real path: an installed package announces a layer ---------------------- #


def test_a_package_entry_point_contributes_its_directory(on_path: Path) -> None:
    expected = _install_fake_distribution(
        on_path, "fake_layer_pkg_a", entry_value="fake_layer_pkg_a.layers", layer_id="thermal"
    )
    assert expected.resolve() in entry_point_search_paths()


def test_a_layer_in_an_installed_package_is_actually_discovered(on_path: Path) -> None:
    # The end-to-end claim: declare an entry point, and the layer is available by id.
    _install_fake_distribution(on_path, "fake_layer_pkg_b", entry_value="fake_layer_pkg_b.layers", layer_id="thermal")
    manifests = discover_layers(entry_point_search_paths())
    assert "thermal" in manifests, sorted(manifests)
    assert manifests["thermal"].provides == ("THERMAL",)


def test_the_group_name_is_the_documented_one() -> None:
    # Changing this silently breaks every third-party layer package that already declares it.
    assert ENTRY_POINT_GROUP == "isaac_core.layers"


def test_no_entry_points_means_no_paths_not_an_error() -> None:
    # The overwhelmingly common case: nobody has installed a layer package.
    assert isinstance(entry_point_search_paths(), tuple)


# -- broken declarations must not stop the simulator --------------------------- #


def test_an_entry_point_naming_a_missing_module_is_skipped(on_path: Path) -> None:
    # One misdeclared third-party package must not stop everything else composing.
    dist_info = on_path / "fake_layer_pkg_c-1.0.0.dist-info"
    dist_info.mkdir(parents=True)
    (dist_info / "METADATA").write_text("Metadata-Version: 2.1\nName: fake_layer_pkg_c\nVersion: 1.0.0\n")
    (dist_info / "entry_points.txt").write_text(f"[{ENTRY_POINT_GROUP}]\nbroken = does_not_exist_at_all.layers\n")
    importlib.invalidate_caches()
    assert entry_point_search_paths() is not None


def test_a_working_entry_point_still_resolves_alongside_a_broken_one(on_path: Path) -> None:
    expected = _install_fake_distribution(
        on_path, "fake_layer_pkg_d", entry_value="fake_layer_pkg_d.layers", layer_id="working"
    )
    dist_info = on_path / "fake_layer_pkg_e-1.0.0.dist-info"
    dist_info.mkdir(parents=True)
    (dist_info / "METADATA").write_text("Metadata-Version: 2.1\nName: fake_layer_pkg_e\nVersion: 1.0.0\n")
    (dist_info / "entry_points.txt").write_text(f"[{ENTRY_POINT_GROUP}]\nbroken = nope_not_here.layers\n")
    importlib.invalidate_caches()

    paths = entry_point_search_paths()
    assert expected.resolve() in paths, "a broken sibling entry point suppressed a working one"


def test_a_duplicate_directory_is_contributed_once(on_path: Path) -> None:
    # Two entry points naming the same directory would otherwise make discovery see the same layer twice
    # and raise DuplicateLayerError, which would be our bug rather than the user's.
    layers = _install_fake_distribution(
        on_path, "fake_layer_pkg_f", entry_value="fake_layer_pkg_f.layers", layer_id="once"
    )
    dist_info = on_path / "fake_layer_pkg_f-1.0.0.dist-info"
    (dist_info / "entry_points.txt").write_text(
        f"[{ENTRY_POINT_GROUP}]\nfirst = fake_layer_pkg_f.layers\nsecond = fake_layer_pkg_f.layers\n"
    )
    importlib.invalidate_caches()
    paths = entry_point_search_paths()
    assert paths.count(layers.resolve()) == 1, paths


def test_the_shipped_layers_are_not_reached_by_entry_point() -> None:
    # The package's own layers come from the built-in directory, not an entry point. If they arrived by
    # both routes, discovery would see duplicate ids and refuse to start.
    for path in entry_point_search_paths():
        assert "isaac_core/assets/layers" not in str(
            path
        ), f"the shipped layers are also exposed as an entry point via {path}"
