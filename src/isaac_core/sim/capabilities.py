"""
Stage capabilities: probing and representing what a USD stage offers.

The previous generation crashed outright if the stage lacked a ``/bboxes`` or
``/tilesets`` prim. This module replaces that behaviour with a probing step that
runs after the base stage opens: the result is a :class:`StageCapabilities` value
that layers can test with :meth:`~StageCapabilities.has`. Unmet requirements
cause a layer to be **skipped with a reason**, not a crash.

The actual USD inspection is done through the narrow :class:`StageInspector`
protocol, which a later Isaac-backed implementation will satisfy. Tests use the
included :class:`FakeStageInspector`.
"""

from __future__ import annotations

from enum import Enum, unique
from typing import Protocol, runtime_checkable

from isaac_core.contracts.prims import BBOXES_ROOT, TILESETS_ROOT


@unique
class StageCapability(Enum):
    """
    A capability that a USD stage may or may not provide.

    Layers declare which capabilities they require via their manifest's
    ``requires`` field, using the enum *name* (e.g. ``"TILESETS_ROOT"``).
    """

    TILESETS_ROOT = "tilesets_root"
    BBOXES_ROOT = "bboxes_root"
    GEOREFERENCE = "georeference"
    CAMERA = "camera"
    SEMANTICS = "semantics"


@runtime_checkable
class StageInspector(Protocol):
    """
    Narrow interface for querying a USD stage's structure.

    A real implementation wraps ``pxr.Usd.Stage`` calls; tests use
    :class:`FakeStageInspector`. The surface is deliberately minimal so that
    the coupling to Isaac stays thin.
    """

    def prim_exists(self, path: str) -> bool:
        """
        Report whether a prim exists at ``path`` in the open stage.

        Args:
            path: Absolute USD prim path.

        Returns:
            ``True`` if the prim is valid and defined.

        """
        ...

    def has_attribute(self, path: str, attribute: str) -> bool:
        """
        Report whether a prim at ``path`` has the named attribute.

        Args:
            path: Absolute USD prim path.
            attribute: Attribute name (e.g. ``"cesium:georeferenceBinding"``).

        Returns:
            ``True`` if the attribute exists on the prim.

        """
        ...


class FakeStageInspector:
    """
    In-memory stage inspector for testing without Isaac Sim.

    Args:
        prims: Set of prim paths that exist.
        attributes: Mapping from ``(path, attribute)`` to whether it exists.

    """

    def __init__(
        self,
        prims: frozenset[str] | None = None,
        attributes: dict[tuple[str, str], bool] | None = None,
    ) -> None:
        """Initialise with known prims and attributes."""
        self._prims: frozenset[str] = prims or frozenset()
        self._attributes: dict[tuple[str, str], bool] = attributes or {}

    def prim_exists(self, path: str) -> bool:
        """Report whether the path is in the known set."""
        return path in self._prims

    def has_attribute(self, path: str, attribute: str) -> bool:
        """Report whether the (path, attribute) pair is in the known set."""
        return self._attributes.get((path, attribute), False)


class StageCapabilities:
    """
    Frozen set of capabilities detected in the current stage.

    Constructed by :func:`probe`; layers test individual capabilities via
    :meth:`has`.
    """

    def __init__(self, capabilities: frozenset[StageCapability]) -> None:
        """Initialise from a frozenset of capabilities."""
        self._capabilities = capabilities

    def has(self, capability: StageCapability) -> bool:
        """
        Check whether a specific capability is present.

        Args:
            capability: The capability to check.

        Returns:
            ``True`` if the stage provides this capability.

        """
        return capability in self._capabilities

    def has_named(self, name: str) -> bool:
        """
        Check a capability by its enum name string (as used in manifests).

        Args:
            name: Enum member name, e.g. ``"TILESETS_ROOT"``.

        Returns:
            ``True`` if the stage provides this capability.

        Raises:
            KeyError: If ``name`` is not a valid :class:`StageCapability` member.

        """
        return StageCapability[name] in self._capabilities

    @property
    def present(self) -> frozenset[StageCapability]:
        """Return the full set of detected capabilities."""
        return self._capabilities

    def __repr__(self) -> str:
        """Return a readable representation."""
        names = sorted(c.name for c in self._capabilities)
        return f"StageCapabilities({{{', '.join(names)}}})"


# Prim path that Cesium georeference prims are expected to carry.
_GEOREFERENCE_ATTR: str = "cesium:georeferenceBinding"

# A conventional camera prim path prefix.
_CAMERA_ROOT: str = "/Environment"

# Semantic labels are applied via a "semantics" schema.
_SEMANTICS_ATTR: str = "semantic:Semantics:params:semanticType"


def probe(inspector: StageInspector) -> StageCapabilities:
    """
    Detect stage capabilities by inspecting known prim paths and attributes.

    This is the sole point where prim-existence checks for capability detection
    live. The mapping from prim structure to capabilities is intentionally
    centralised here.

    Args:
        inspector: An object satisfying the :class:`StageInspector` protocol.

    Returns:
        The detected capabilities.

    """
    caps: set[StageCapability] = set()

    if inspector.prim_exists(TILESETS_ROOT):
        caps.add(StageCapability.TILESETS_ROOT)

    if inspector.prim_exists(BBOXES_ROOT):
        caps.add(StageCapability.BBOXES_ROOT)

    if inspector.prim_exists("/CesiumGeoreference"):
        caps.add(StageCapability.GEOREFERENCE)

    if inspector.prim_exists("/Environment/main_camera"):
        caps.add(StageCapability.CAMERA)

    if inspector.prim_exists("/semantics"):
        caps.add(StageCapability.SEMANTICS)

    return StageCapabilities(frozenset(caps))


__all__ = [
    "FakeStageInspector",
    "StageCapabilities",
    "StageCapability",
    "StageInspector",
    "probe",
]
