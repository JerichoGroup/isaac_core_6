"""USD stage inspector backed by a real ``pxr.Usd.Stage``.

Implement the :class:`~isaac_core.sim.capabilities.StageInspector` protocol,
providing the narrow bridge between the pure planning/capability logic and the
actual Isaac Sim stage. All ``pxr`` access is done through
:func:`importlib.import_module` so that importing this module does NOT pull in
``pxr`` at the top level -- the kernel purity test asserts ``isaac_core.sim``
imports cleanly with omni/carb/pxr blocked.
"""

from __future__ import annotations

import importlib
from typing import Any


def _sdf() -> Any:
    """Lazily import and return the pxr.Sdf module."""
    return importlib.import_module("pxr.Sdf")


class UsdStageInspector:
    """Inspect a live USD stage by delegating to ``pxr.Usd.Stage`` methods.

    Satisfy the :class:`~isaac_core.sim.capabilities.StageInspector` protocol.

    Args:
        stage: An open ``pxr.Usd.Stage`` instance.

    """

    def __init__(self, stage: Any) -> None:
        """Initialise with an open USD stage."""
        self._stage = stage

    @property
    def stage(self) -> Any:
        """Return the underlying stage."""
        return self._stage

    def prim_exists(self, path: str) -> bool:
        """Report whether a prim exists at ``path`` in the open stage.

        Args:
            path: Absolute USD prim path.

        Returns:
            ``True`` if the prim is valid and defined.

        """
        sdf = _sdf()
        sdf_path = sdf.Path(path)
        prim = self._stage.GetPrimAtPath(sdf_path)
        return bool(prim.IsValid())

    def has_attribute(self, path: str, attribute: str) -> bool:
        """Report whether a prim at ``path`` has the named attribute.

        Args:
            path: Absolute USD prim path.
            attribute: Attribute name.

        Returns:
            ``True`` if the attribute exists on the prim.

        """
        sdf = _sdf()
        sdf_path = sdf.Path(path)
        prim = self._stage.GetPrimAtPath(sdf_path)
        if not prim.IsValid():
            return False
        return bool(prim.HasAttribute(attribute))

    def read_double(self, path: str, attribute: str) -> float | None:
        """Read a double-valued attribute from a prim.

        Args:
            path: Absolute USD prim path.
            attribute: Attribute name.

        Returns:
            The value, or ``None`` if the prim/attribute does not exist or is not
            a double.

        """
        sdf = _sdf()
        sdf_path = sdf.Path(path)
        prim = self._stage.GetPrimAtPath(sdf_path)
        if not prim.IsValid():
            return None
        attr = prim.GetAttribute(attribute)
        if not attr.IsValid():
            return None
        value = attr.Get()
        if isinstance(value, (int, float)):
            return float(value)
        return None


__all__ = [
    "UsdStageInspector",
]
