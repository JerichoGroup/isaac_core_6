"""Layer discovery: scan search directories for ``layer.toml`` manifests.

The discovery logic replaces the hardcoded ``OPTIONAL_USDS`` dict. Adding a
feature layer is now a directory-drop: place a folder containing ``layer.toml``
anywhere on the configured search path and it is picked up automatically.

Search order matters: earlier paths take priority, and a duplicate layer id
found in a later path raises immediately rather than silently shadowing.
"""

from __future__ import annotations

from collections.abc import Sequence
import importlib.metadata
import importlib.resources
import logging
from pathlib import Path

from isaac_core.sim.manifest import LayerManifest, load_manifest

logger = logging.getLogger(__name__)

# Entry-point group a layer package declares to announce itself, so installing a layer is just
# installing a package. Each entry resolves to a directory holding one or more `<id>/layer.toml`.
ENTRY_POINT_GROUP = "isaac_core.layers"


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


def entry_point_search_paths() -> tuple[Path, ...]:
    """Return layer directories contributed by installed packages.

    A package declares itself like this, and nothing else is needed to install a layer:

    ```toml
    [project.entry-points."isaac_core.layers"]
    thermal = "my_package.layers"
    ```

    The value names either a package whose directory holds the layers, or a module attribute that is a
    path. A package is the useful form, because it travels with the wheel and needs no absolute path.

    A broken entry point is logged and skipped rather than raised: one uninstalled or misdeclared
    third-party package must not stop the simulator composing everything else.

    Returns:
        Directories to add to the layer search path, in the order the entry points were found, with
        duplicates removed.

    """
    found: list[Path] = []
    for entry in _entry_points():
        directory = _resolve_entry_point(entry)
        if directory is not None and directory not in found:
            found.append(directory)
    return tuple(found)


def _entry_points() -> tuple[importlib.metadata.EntryPoint, ...]:
    """Return the entry points in the layer group.

    Returns:
        The declared entry points, or an empty tuple when the group is absent.

    """
    try:
        return tuple(importlib.metadata.entry_points(group=ENTRY_POINT_GROUP))
    except Exception:
        # A corrupt distribution on the path must not stop the simulator starting.
        logger.warning("could not read the %r entry-point group", ENTRY_POINT_GROUP, exc_info=True)
        return ()


def _resolve_entry_point(entry: importlib.metadata.EntryPoint) -> Path | None:
    """Resolve one entry point to a directory of layers.

    Args:
        entry: The declared entry point.

    Returns:
        The directory, or ``None`` when it cannot be resolved or is not a directory.

    """
    try:
        target = entry.load()
    except Exception:
        logger.warning("layer entry point %r could not be loaded; skipping", entry.name, exc_info=True)
        return None

    candidate: Path | None = None
    if isinstance(target, str | Path):
        candidate = Path(target)
    elif hasattr(target, "__path__"):
        # A package: its directory holds the layer subdirectories.
        paths = list(target.__path__)
        candidate = Path(paths[0]) if paths else None
    elif callable(target):
        try:
            candidate = Path(target())
        except Exception:
            logger.warning("layer entry point %r raised when called; skipping", entry.name, exc_info=True)
            return None

    if candidate is None:
        logger.warning(
            "layer entry point %r resolved to %r, which is not a package, path or callable; skipping",
            entry.name,
            type(target).__name__,
        )
        return None

    resolved = candidate.expanduser().resolve()
    if not resolved.is_dir():
        logger.warning("layer entry point %r resolved to %s, which is not a directory; skipping", entry.name, resolved)
        return None
    logger.info("layer entry point %r contributes %s", entry.name, resolved)
    return resolved
