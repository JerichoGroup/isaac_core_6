"""Layer manifest model: validated, typed description of a feature layer.

A feature layer is a self-contained USD file (carrying its own OmniGraph and
prims) that can be composed onto a base stage at runtime. Its ``layer.toml``
manifest declares everything the compositor needs — where to mount it, what
stage capabilities it requires, and how to bind configuration values into prim
attributes — so that adding a new feature is a directory-drop, not a code change.
"""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Final

from pydantic import BaseModel, ConfigDict, field_validator, model_validator

from isaac_core.contracts.prims import INSTANCE, MOUNT, placeholders
from isaac_core.contracts.topics import validate_segment

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

# Placeholder for a vehicle's camera key, resolved at compose time. Lets a binding
# template the camera name instead of hardcoding one (e.g. "eo"), so renaming a
# camera or giving a vehicle a differently-named camera does not require editing the
# manifest.
CAMERA: Final[str] = "camera"

# The set of placeholders the system can supply at compose time.
KNOWN_PLACEHOLDERS: Final[frozenset[str]] = frozenset({MOUNT, INSTANCE, CAMERA})


class Binding(BaseModel):
    """Map a configuration value or runtime-derived value to a prim attribute.

    Exactly one of ``config`` or ``resolve`` must be provided. ``config`` is a
    dotted key into the loaded configuration (e.g. ``"cameras.eo.fov_deg"``);
    ``resolve`` names a value computed at runtime (e.g. ``"enu_origin"``).
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    prim: str
    attribute: str
    config: str | None = None
    resolve: str | None = None

    @model_validator(mode="after")
    def _exactly_one_source(self) -> "Binding":
        """Reject bindings that have both or neither source."""
        has_config = self.config is not None
        has_resolve = self.resolve is not None
        if has_config == has_resolve:
            msg = (
                f"binding for '{self.prim}:{self.attribute}' must specify exactly one "
                f"of 'config' or 'resolve', got {'both' if has_config else 'neither'}"
            )
            raise ValueError(msg)
        return self

    @field_validator("prim")
    @classmethod
    def _validate_prim_placeholders(cls, value: str) -> str:
        """Reject templates referencing unknown placeholders."""
        unknown = placeholders(value) - KNOWN_PLACEHOLDERS
        if unknown:
            msg = (
                f"prim template {value!r} uses unknown placeholder(s): "
                f"{', '.join(sorted(unknown))}; allowed: {', '.join(sorted(KNOWN_PLACEHOLDERS))}"
            )
            raise ValueError(msg)
        return value


class LayerManifest(BaseModel):
    """Validated description of a feature layer, loaded from ``layer.toml``.

    The manifest is the single source of truth for how a layer participates in
    stage composition. It replaces the hardcoded ``OPTIONAL_USDS`` dict and the
    branch-per-feature pattern it would otherwise need.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: str
    usd: str
    mount: str
    requires: tuple[str, ...] = ()
    provides: tuple[str, ...] = ()
    provision: bool = False
    bindings: tuple[Binding, ...] = ()

    @field_validator("id")
    @classmethod
    def _validate_id(cls, value: str) -> str:
        """Ensure the layer id is a valid topic segment (reuses the contracts check)."""
        validate_segment(value)
        return value

    @field_validator("mount")
    @classmethod
    def _validate_mount_placeholders(cls, value: str) -> str:
        """Reject mount templates with unknown or self-referential placeholders.

        ``{mount}`` is specifically forbidden here, even though it is a valid
        placeholder in *bindings*. The mount field defines what ``{mount}`` resolves
        to, so referencing it produces a circular template. That previously passed
        validation and then failed at compose time with a bare ``KeyError: 'mount'``,
        which says nothing about the actual mistake.
        """
        found = placeholders(value)

        if MOUNT in found:
            msg = (
                f"mount template {value!r} references {{{MOUNT}}}, which is circular: "
                f"this field defines what {{{MOUNT}}} means. Use {{{INSTANCE}}} instead, "
                f'e.g. "/Environment/{{{INSTANCE}}}".'
            )
            raise ValueError(msg)

        if CAMERA in found:
            msg = (
                f"mount template {value!r} references {{{CAMERA}}}, which is not available here: "
                f"a mount is per-vehicle and is rendered before any camera is chosen. "
                f"Use {{{CAMERA}}} in a binding's prim or config instead."
            )
            raise ValueError(msg)

        unknown = found - KNOWN_PLACEHOLDERS
        if unknown:
            msg = (
                f"mount template {value!r} uses unknown placeholder(s): "
                f"{', '.join(sorted(unknown))}; allowed: {', '.join(sorted(KNOWN_PLACEHOLDERS))}"
            )
            raise ValueError(msg)
        return value


def load_manifest(path: Path) -> LayerManifest:
    """Parse and validate a ``layer.toml`` file into a :class:`LayerManifest`.

    Args:
        path: Absolute or relative path to the ``layer.toml`` file.

    Returns:
        A validated manifest.

    Raises:
        FileNotFoundError: If ``path`` does not exist.
        ValueError: If parsing fails or validation rejects the content. The
            exception message includes the file path for diagnostics.

    """
    try:
        raw = path.read_bytes()
        data = tomllib.loads(raw.decode())
    except Exception as exc:
        msg = f"failed to parse manifest {path}: {exc}"
        raise ValueError(msg) from exc

    try:
        return LayerManifest(**data)
    except Exception as exc:
        msg = f"invalid manifest {path}: {exc}"
        raise ValueError(msg) from exc


__all__ = [
    "Binding",
    "CAMERA",
    "KNOWN_PLACEHOLDERS",
    "LayerManifest",
    "load_manifest",
]
