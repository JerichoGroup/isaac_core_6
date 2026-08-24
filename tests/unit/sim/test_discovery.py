"""Tests for isaac_core.sim.discovery."""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_core.sim.discovery import (
    DuplicateLayerError,
    MalformedManifestError,
    discover_layers,
)

# -- Helpers ------------------------------------------------------------------

_VALID_MANIFEST = """\
id = "{layer_id}"
usd = "{layer_id}.usda"
mount = "/Environment/{layer_id}"
"""


def _make_layer(base: Path, layer_id: str, content: str | None = None) -> Path:
    """Create a layer directory with a layer.toml."""
    layer_dir = base / layer_id
    layer_dir.mkdir(parents=True, exist_ok=True)
    toml_file = layer_dir / "layer.toml"
    toml_file.write_text(
        content if content is not None else _VALID_MANIFEST.format(layer_id=layer_id),
        encoding="utf-8",
    )
    return layer_dir


# -- Happy path ---------------------------------------------------------------


def test_discovers_multiple_layers_across_multiple_paths(tmp_path: Path) -> None:
    path_a = tmp_path / "a"
    path_b = tmp_path / "b"
    path_a.mkdir()
    path_b.mkdir()

    _make_layer(path_a, "sensor_a")
    _make_layer(path_b, "sensor_b")

    result = discover_layers([path_a, path_b])
    assert set(result.keys()) == {"sensor_a", "sensor_b"}
    assert result["sensor_a"].usd == "sensor_a.usda"
    assert result["sensor_b"].usd == "sensor_b.usda"


def test_directory_without_layer_toml_is_skipped(tmp_path: Path) -> None:
    search = tmp_path / "search"
    search.mkdir()

    # A subdirectory with no layer.toml
    (search / "empty_dir").mkdir()
    # A subdirectory with a layer.toml
    _make_layer(search, "real_layer")

    result = discover_layers([search])
    assert list(result.keys()) == ["real_layer"]


def test_nonexistent_search_path_is_skipped(tmp_path: Path) -> None:
    missing = tmp_path / "does_not_exist"
    result = discover_layers([missing])
    assert result == {}


def test_ordering_is_deterministic(tmp_path: Path) -> None:
    search = tmp_path / "layers"
    search.mkdir()
    # Create layers in non-alphabetical order
    _make_layer(search, "zebra")
    _make_layer(search, "alpha")
    _make_layer(search, "middle")

    result = discover_layers([search])
    # discover iterates sorted children, so the insertion order in the dict
    # reflects alphabetical order of subdirectory names.
    assert list(result.keys()) == ["alpha", "middle", "zebra"]


def test_single_layer_in_single_path(tmp_path: Path) -> None:
    search = tmp_path / "layers"
    search.mkdir()
    _make_layer(search, "thermal")

    result = discover_layers([search])
    assert "thermal" in result
    assert result["thermal"].mount == "/Environment/thermal"


# -- Error cases --------------------------------------------------------------


def test_duplicate_id_across_paths_raises_naming_both_files(tmp_path: Path) -> None:
    path_a = tmp_path / "a"
    path_b = tmp_path / "b"
    path_a.mkdir()
    path_b.mkdir()

    _make_layer(path_a, "sensor")
    _make_layer(path_b, "sensor")

    with pytest.raises(DuplicateLayerError) as exc_info:
        discover_layers([path_a, path_b])

    msg = str(exc_info.value)
    # Both file paths must appear in the error message
    assert str(path_a / "sensor" / "layer.toml") in msg
    assert str(path_b / "sensor" / "layer.toml") in msg


def test_malformed_toml_names_the_file(tmp_path: Path) -> None:
    search = tmp_path / "layers"
    search.mkdir()

    layer_dir = search / "broken"
    layer_dir.mkdir()
    toml_file = layer_dir / "layer.toml"
    toml_file.write_text("id = [not valid toml", encoding="utf-8")

    with pytest.raises(MalformedManifestError) as exc_info:
        discover_layers([search])

    assert str(toml_file) in str(exc_info.value)


def test_invalid_manifest_content_raises_malformed(tmp_path: Path) -> None:
    # Valid TOML but invalid manifest (missing required fields)
    search = tmp_path / "layers"
    search.mkdir()

    _make_layer(search, "bad", content='unknown_key = "hello"\n')

    with pytest.raises(MalformedManifestError) as exc_info:
        discover_layers([search])

    assert "layer.toml" in str(exc_info.value)


def test_files_in_search_dir_are_not_treated_as_layers(tmp_path: Path) -> None:
    # Only subdirectories are considered, not files directly in the search path
    search = tmp_path / "layers"
    search.mkdir()
    (search / "not_a_dir.toml").write_text('id = "nope"\n', encoding="utf-8")

    result = discover_layers([search])
    assert result == {}
