"""
USD stage inspector backed by a real ``pxr.Usd.Stage``.

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


class StagePrimMissingError(Exception):
    """Raised when a required prim does not exist in the open stage."""


def _sdf() -> Any:  # noqa: ANN401
    """Lazily import and return the pxr.Sdf module."""
    return importlib.import_module("pxr.Sdf")


class UsdStageInspector:
    """
    Inspect a live USD stage by delegating to ``pxr.Usd.Stage`` methods.

    Satisfy the :class:`~isaac_core.sim.capabilities.StageInspector` protocol.

    Args:
        stage: An open ``pxr.Usd.Stage`` instance.

    """

    def __init__(self, stage: Any) -> None:  # noqa: ANN401
        """Initialise with an open USD stage."""
        self._stage = stage

    @property
    def stage(self) -> Any:  # noqa: ANN401
        """Return the underlying stage."""
        return self._stage

    def prim_exists(self, path: str) -> bool:
        """
        Report whether a prim exists at ``path`` in the open stage.

        Args:
            path: Absolute USD prim path.

        Returns:
            ``True`` if the prim is valid and defined.

        """
        sdf = _sdf()
        sdf_path = sdf.Path(path)
        prim = self._stage.GetPrimAtPath(sdf_path)
        return prim.IsValid()  # type: ignore[no-any-return]

    def has_attribute(self, path: str, attribute: str) -> bool:
        """
        Report whether a prim at ``path`` has the named attribute.

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
        return prim.HasAttribute(attribute)  # type: ignore[no-any-return]

    def read_double(self, path: str, attribute: str) -> float | None:
        """
        Read a double-valued attribute from a prim.

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


def require_prim(stage: Any, path: str) -> Any:  # noqa: ANN401
    """
    Return the prim at ``path``, raising if it does not exist.

    This is the "raises" contract (defect #5): callers who cannot continue without
    the prim should use this. For optional lookups, use :func:`find_prim`.

    Args:
        stage: An open USD stage.
        path: Absolute prim path.

    Returns:
        The valid prim.

    Raises:
        StagePrimMissingError: If the prim does not exist or is not valid.

    """
    sdf = _sdf()
    sdf_path = sdf.Path(path)
    prim = stage.GetPrimAtPath(sdf_path)
    if not prim.IsValid():
        msg = f"required prim {path!r} does not exist in the open stage"
        raise StagePrimMissingError(msg)
    return prim


def find_prim(stage: Any, path: str) -> Any:  # noqa: ANN401
    """
    Return the prim at ``path``, or ``None`` if it does not exist.

    This is the "returns None" contract (defect #5): callers who have a fallback
    should use this. For mandatory lookups, use :func:`require_prim`.

    Args:
        stage: An open USD stage.
        path: Absolute prim path.

    Returns:
        The prim if valid, or ``None``.

    """
    sdf = _sdf()
    sdf_path = sdf.Path(path)
    prim = stage.GetPrimAtPath(sdf_path)
    if prim.IsValid():
        return prim
    return None


__all__ = [
    "StagePrimMissingError",
    "UsdStageInspector",
    "find_prim",
    "require_prim",
]
