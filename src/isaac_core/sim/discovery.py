"""Layer discovery: scan search directories for ``layer.toml`` manifests.

The discovery logic replaces the hardcoded ``OPTIONAL_USDS`` dict. Adding a
feature layer is now a directory-drop: place a folder containing ``layer.toml``
anywhere on the configured search path and it is picked up automatically.

Search order matters: earlier paths take priority, and a duplicate layer id
found in a later path raises immediately rather than silently shadowing.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from isaac_core.sim.manifest import LayerManifest, load_manifest


class DuplicateLayerError(Exception):
    """Raised when two manifests declare the same layer id."""


class MalformedManifestError(Exception):
    """Raised when a manifest file cannot be parsed or validated."""


def discover_layers(search_paths: Sequence[Path]) -> dict[str, LayerManifest]:
    """Scan directories for ``*/layer.toml`` and return validated manifests keyed by id.

    Each element of ``search_paths`` is a directory. Its immediate subdirectories
    are inspected for a ``layer.toml`` file. If found, the manifest is loaded and
    validated. Later search paths must NOT silently shadow earlier ones: a
    duplicate id raises :class:`DuplicateLayerError` naming both file paths.

    Args:
        search_paths: Directories to scan, in priority order.

    Returns:
        Mapping from layer id to its manifest.

    Raises:
        DuplicateLayerError: If two manifests share the same id.
        MalformedManifestError: If a ``layer.toml`` exists but cannot be parsed or
            validated. The message includes the offending file path.

    """
    found: dict[str, tuple[LayerManifest, Path]] = {}

    for search_dir in search_paths:
        if not search_dir.is_dir():
            continue
        for child in sorted(search_dir.iterdir()):
            if not child.is_dir():
                continue
            toml_path = child / "layer.toml"
            if not toml_path.is_file():
                continue

            try:
                manifest = load_manifest(toml_path)
            except (ValueError, FileNotFoundError) as exc:
                msg = f"malformed manifest at {toml_path}: {exc}"
                raise MalformedManifestError(msg) from exc

            if manifest.id in found:
                existing_path = found[manifest.id][1]
                msg = f"duplicate layer id {manifest.id!r}: found in both " f"{existing_path} and {toml_path}"
                raise DuplicateLayerError(msg)

            found[manifest.id] = (manifest, toml_path)

    return {layer_id: manifest for layer_id, (manifest, _path) in found.items()}


__all__ = [
    "DuplicateLayerError",
    "MalformedManifestError",
    "discover_layers",
]
