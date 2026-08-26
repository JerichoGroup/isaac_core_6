"""
Stage composer: open the base scene, mount and reference feature layers.

Open the base USD stage, then for each enabled layer in the plan create a mount
prim and add the layer's USD file as a reference (relying on the layer's
``defaultPrim`` being set to ``Root``). This replaces
``Simulation._add_external_usds`` from the previous generation, which iterated a
hardcoded dict.

All ``omni`` and ``pxr`` access is done through :func:`importlib.import_module`
so that importing this module does NOT pull them in at the top level.
"""

from __future__ import annotations

from collections.abc import Sequence
import importlib
import logging
from pathlib import Path
from typing import Any, Final

from isaac_core.config import IsaacCoreConfig
from isaac_core.sim.capabilities import probe
from isaac_core.sim.configurator import AttributeWrite, compute_writes
from isaac_core.sim.georeference import describe_mismatch, resolve_enu_reference
from isaac_core.sim.planner import FeaturePlan, PlannedLayer
from isaac_core.sim.stage import UsdStageInspector

logger = logging.getLogger(__name__)


def _omni_usd() -> Any:  # noqa: ANN401
    """Lazily import and return the omni.usd module."""
    return importlib.import_module("omni.usd")


def _gf() -> Any:  # noqa: ANN401
    """Return the lazily imported ``pxr.Gf`` module."""
    return importlib.import_module("pxr.Gf")


def _sdf() -> Any:  # noqa: ANN401
    """Lazily import and return the pxr.Sdf module."""
    return importlib.import_module("pxr.Sdf")


def mount_path_for_layer(planned: PlannedLayer, instance: str) -> str:
    """
    Derive the mount prim path for a planned layer.

    Args:
        planned: A planned layer from the feature plan.
        instance: Instance identifier, normally the vehicle id.

    Returns:
        The absolute prim path at which to create the mount.

    """
    # This hardcoded instance="default" until 2026-08-25, which mounted every layer at
    # /Environment/default regardless of which vehicle it belonged to. No binding's
    # prim path then matched, and a swarm would have collapsed onto a single mount.
    return planned.manifest.mount.format(instance=instance)


def layer_usd_path(planned: PlannedLayer, search_paths: tuple[Path, ...]) -> Path | None:
    """
    Resolve a layer's USD file to an absolute path.

    Search the configured layer search paths, then treat the manifest's ``usd``
    as relative to the manifest's own location if not found elsewhere.

    Args:
        planned: A planned layer.
        search_paths: Asset search paths from config.

    Returns:
        Absolute path to the layer's USD file, or ``None`` if not found.

    """
    usd_relative = planned.manifest.usd
    for search_dir in search_paths:
        candidate = search_dir / planned.manifest.id / usd_relative
        if candidate.is_file():
            return candidate.resolve()
    return None


def compose_stage(
    config: IsaacCoreConfig,
    plan: FeaturePlan,
    scene_path: Path,
    layer_search_paths: tuple[Path, ...],
) -> Any:  # noqa: ANN401
    """
    Open the base scene, mount feature layers, and apply configuration.

    This is the single entry point for stage composition. It:

    1. Opens the base scene USD.
    2. For each enabled layer, creates the mount prim and adds the layer USD as a
       reference.
    3. Resolves the ENU reference from the scene georeference.
    4. Computes and applies attribute writes from layer bindings and prim overrides.
    5. Prints the enabled/skipped capability report.

    All Isaac-dependent calls are here, behind importlib lazy imports.

    Args:
        config: The resolved configuration.
        plan: The feature plan.
        scene_path: Absolute path to the base scene USD file.
        layer_search_paths: Directories to search for layer USD files.

    Returns:
        The opened ``pxr.Usd.Stage``.

    Raises:
        FileNotFoundError: If the scene file does not exist.
        RuntimeError: If the stage cannot be opened.

    """
    if not scene_path.is_file():
        msg = f"scene file not found: {scene_path}"
        raise FileNotFoundError(msg)

    omni_usd = _omni_usd()
    usd_context = omni_usd.get_context()
    result = usd_context.open_stage(str(scene_path))
    if not result:
        msg = f"failed to open stage: {scene_path}"
        raise RuntimeError(msg)

    stage = usd_context.get_stage()

    # The instance is the vehicle id, so prim paths match the manifests' {instance}
    # templates and each aircraft in a swarm gets its own mount.
    instance = next(iter(config.vehicles))
    _mount_layers(stage, plan, layer_search_paths, instance)

    inspector = UsdStageInspector(stage)
    probe(inspector)

    resolved_enu = resolve_enu_reference(config, inspector)
    mismatch = describe_mismatch(resolved_enu, inspector)
    if mismatch:
        logger.warning(mismatch)

    report = plan.render_report()
    if report:
        logger.info("Feature layers:\n%s", report)

    camera_prim = _resolve_camera_prim(config, stage)
    writes = compute_writes(config, plan, resolved_enu, camera_prim=camera_prim)
    _apply_stage_writes(stage, writes)

    return stage


def _mount_layers(
    stage: Any,  # noqa: ANN401
    plan: FeaturePlan,
    layer_search_paths: tuple[Path, ...],
    instance: str,
) -> None:
    """
    Create mount prims and add layer references.

    Args:
        stage: The open stage.
        plan: The feature plan.
        layer_search_paths: Directories to resolve layer USD files from.
        instance: Instance identifier, normally the vehicle id.

    """
    for planned in plan.enabled:
        mount = mount_path_for_layer(planned, instance)
        usd_path = layer_usd_path(planned, layer_search_paths)
        if usd_path is None:
            logger.warning(
                "layer %r USD file %r not found in search paths; skipping mount",
                planned.manifest.id,
                planned.manifest.usd,
            )
            continue
        _define_and_reference(stage, mount, usd_path)


def _define_and_reference(stage: Any, mount_path: str, usd_path: Path) -> None:  # noqa: ANN401
    """
    Define a prim at ``mount_path`` and add a reference to the layer USD.

    Args:
        stage: The open stage.
        mount_path: Absolute prim path for the mount.
        usd_path: Absolute path to the layer's USD file.

    """
    sdf = _sdf()
    prim = stage.DefinePrim(mount_path)
    references = prim.GetReferences()
    references.AddReference(sdf.Reference(str(usd_path)))
    logger.debug("mounted %s -> %s", mount_path, usd_path)


def _resolve_camera_prim(config: IsaacCoreConfig, stage: Any) -> str | None:  # noqa: ANN401
    """
    Determine the active camera prim path for bindings.

    Args:
        config: The resolved configuration.
        stage: The open stage.

    Returns:
        The camera prim path, or ``None`` if not found.

    """
    first_vehicle_id = next(iter(config.vehicles))
    first_camera_id = next(iter(config.vehicles[first_vehicle_id].cameras))
    mount = config.resolved_mount(first_vehicle_id)
    camera_path = f"{mount}/Camera_{first_camera_id}"

    sdf = _sdf()
    prim = stage.GetPrimAtPath(sdf.Path(camera_path))
    if prim.IsValid():
        return camera_path
    return None


def _apply_stage_writes(stage: Any, writes: list[AttributeWrite]) -> None:  # noqa: ANN401
    """
    Apply computed attribute writes to the stage.

    Args:
        stage: The open stage.
        writes: Ordered writes.

    """
    sdf = _sdf()
    for w in writes:
        prim = stage.GetPrimAtPath(sdf.Path(w.prim))
        if not prim.IsValid():
            logger.warning("write target prim %r does not exist; skipping", w.prim)
            continue
        attr = prim.GetAttribute(w.attribute)
        if not attr.IsValid():
            logger.warning(
                "attribute %r not found on prim %r; skipping",
                w.attribute,
                w.prim,
            )
            continue
        try:
            attr.Set(_coerce_for_attribute(attr, w.value))
        except Exception as exc:  # noqa: BLE001 - USD raises Tf.ErrorException
            logger.warning("could not write %r to %s.%s: %s", w.value, w.prim, w.attribute, exc)


# USD type names whose values must be constructed as a Gf vector or quaternion rather
# than passed as a plain Python sequence. Writing a list to a `double3` raises
# "Type mismatch ... expected 'GfVec3d', got 'vector<VtValue>'", which is why the
# configurator stays pure (plain Python values) and coercion happens here, at the only
# point that knows the USD schema.
_GF_CONSTRUCTORS: Final[dict[str, str]] = {
    "double3": "Vec3d",
    "float3": "Vec3f",
    "half3": "Vec3h",
    "int3": "Vec3i",
    "vector3d": "Vec3d",
    "vector3f": "Vec3f",
    "point3d": "Vec3d",
    "point3f": "Vec3f",
    "normal3d": "Vec3d",
    "normal3f": "Vec3f",
    "color3d": "Vec3d",
    "color3f": "Vec3f",
    "double2": "Vec2d",
    "float2": "Vec2f",
    "int2": "Vec2i",
    "double4": "Vec4d",
    "float4": "Vec4f",
    "quatd": "Quatd",
    "quatf": "Quatf",
    "quath": "Quath",
}


def _coerce_for_attribute(attr: Any, value: Any) -> Any:  # noqa: ANN401
    """
    Convert a plain Python value into the type a USD attribute expects.

    The configurator is deliberately pure and emits plain Python (floats, strings,
    lists), because that keeps it testable without USD. USD, however, rejects a list
    written to a ``double3`` attribute. This is the single place that bridges the two,
    and it is Isaac-coupled by design.

    Args:
        attr: The USD attribute being written.
        value: The plain Python value from the configurator.

    Returns:
        The value, converted if the attribute's type requires it.

    """
    gf = _gf()
    type_name = str(attr.GetTypeName())
    constructor_name = _GF_CONSTRUCTORS.get(type_name)

    if constructor_name is None or isinstance(value, str) or not isinstance(value, Sequence):
        return value

    constructor = getattr(gf, constructor_name, None)
    if constructor is None:
        return value

    # Quaternions take (real, imaginary-vector); Gf accepts four scalars for Quat*.
    return constructor(*(float(component) for component in value))


__all__ = [
    "compose_stage",
    "layer_usd_path",
    "mount_path_for_layer",
]
