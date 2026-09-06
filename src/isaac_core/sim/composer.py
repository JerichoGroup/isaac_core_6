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
from typing import TYPE_CHECKING
from urllib.parse import urlsplit, urlunsplit

if TYPE_CHECKING:
    from collections.abc import Callable
from typing import Any, Final

from isaac_core.config import IsaacCoreConfig
from isaac_core.contracts.prims import BBOXES_ROOT, render
from isaac_core.sim.capabilities import probe
from isaac_core.sim.configurator import (
    AttributeWrite,
    compute_writes,
)
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


def _usd() -> Any:  # noqa: ANN401
    """Return the lazily imported ``pxr.Usd`` module."""
    return importlib.import_module("pxr.Usd")


def _usdlux() -> Any:  # noqa: ANN401
    """Return the lazily imported ``pxr.UsdLux`` module."""
    return importlib.import_module("pxr.UsdLux")


def _usdgeom() -> Any:  # noqa: ANN401
    """Return the lazily imported ``pxr.UsdGeom`` module."""
    return importlib.import_module("pxr.UsdGeom")


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
    settle: "Callable[[], None] | None" = None,
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
        settle: Optional callable invoked after the base scene opens and again after the
            layers mount, used to let the application finish loading in between. Layers
            can contain graphs that OmniGraph evaluates in the render pipeline, and
            mounting them before the renderer has produced a frame crashed startup
            intermittently. ``None`` skips the waits, which keeps this testable without
            a running application.

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

    # Establish metres-per-unit before anything reads geometry: the pose pipeline and
    # camera intrinsics all assume 1 unit == 1 metre.
    apply_stage_units(stage, config.sim.stage_units_in_meters)

    # Let the base scene finish loading before any layer graph exists. See `settle`.
    if settle is not None:
        settle()

    # Each planned layer carries the vehicle it was planned for; this is only the fallback for a
    # plan that predates per-vehicle planning.
    instance = next(iter(config.vehicles))
    _mount_layers(stage, plan, layer_search_paths, instance)

    # And again, so the newly mounted graphs are fully resolved before physics starts.
    if settle is not None:
        settle()

    inspector = UsdStageInspector(stage)
    probe(inspector)

    resolved_enu = resolve_enu_reference(config, inspector)
    mismatch = describe_mismatch(resolved_enu, inspector)
    if mismatch:
        logger.warning(mismatch)

    report = plan.render_report()
    if report:
        logger.info("Feature layers:\n%s", report)

    apply_tileset_server_url(stage, config.cesium.tileset_server_url, config.cesium.tilesets_root)

    camera_prim = _resolve_camera_prim(config, stage)
    writes = compute_writes(config, plan, resolved_enu, camera_prim=camera_prim)
    _apply_stage_writes(stage, writes)

    apply_target_semantics(stage, BBOXES_ROOT)

    apply_hdri(stage, config.assets.hdri)

    return stage


# Prim path for the environment dome light created from `assets.hdri`.
_HDRI_DOME_PATH = "/World/Environment/EnvironmentLight"


# Cesium's on-disk request cache. Grows without bound over long sessions -- the team saw
# it reach 12 GB -- and there is no config inside Cesium to cap it, so the only lever is to
# clear it at launch. The .sqlite file has -wal and -shm companions that must go too.
_CESIUM_CACHE_DIR = Path("~/.cache/ov").expanduser()
_CESIUM_CACHE_GLOB = "cesium-request-cache.sqlite*"


def delete_cesium_cache(cache_dir: Path = _CESIUM_CACHE_DIR) -> int:
    """
    Delete Cesium's request cache files.

    Args:
        cache_dir: Directory holding the cache, overridable for testing.

    Returns:
        The number of files removed.

    """
    removed = 0
    for cache_file in cache_dir.glob(_CESIUM_CACHE_GLOB):
        try:
            cache_file.unlink()
            removed += 1
        except OSError as exc:
            logger.warning("could not delete Cesium cache file %s: %s", cache_file, exc)
    if removed:
        logger.info("deleted %d Cesium cache file(s) from %s", removed, cache_dir)
    else:
        logger.info("no Cesium cache files to delete in %s", cache_dir)
    return removed


# Floating-point slack when comparing metres-per-unit values. USD stores the value as a
# double and a scene authored in the GUI can land a hair off 1.0; anything inside this
# window is treated as equal so a rounding artefact is not reported as a conflict.
_METERS_PER_UNIT_TOL: Final = 1e-9


def apply_stage_units(stage: Any, meters_per_unit: float) -> bool:  # noqa: ANN401
    """
    Set the stage's metres-per-unit metadata from config.

    The whole pipeline assumes one stage unit is one metre: the ENU translate values the
    pose graph writes are metres, and the camera intrinsics are authored in millimetres
    against a metre stage. Changing this rescales the entire world relative to the poses,
    so anything other than ``1.0`` is applied but WARNED about loudly rather than accepted
    silently as if it were a supported mode.

    A scene that already declares a different metres-per-unit than config is a real
    conflict -- one of the two is wrong about what the numbers mean -- so it is reported
    before config overwrites it.

    ``pxr`` is imported lazily so this module imports without Isaac Sim.

    Args:
        stage: The open USD stage.
        meters_per_unit: The configured metres per stage unit, from ``sim.stage_units_in_meters``.

    Returns:
        ``True`` if the stage metadata was written.

    """
    usdgeom = _usdgeom()

    existing = usdgeom.GetStageMetersPerUnit(stage)
    if abs(existing - meters_per_unit) > _METERS_PER_UNIT_TOL:
        logger.warning(
            "scene declares metersPerUnit=%s but config sim.stage_units_in_meters=%s; "
            "overwriting the scene with the configured value",
            existing,
            meters_per_unit,
        )

    if abs(meters_per_unit - 1.0) > _METERS_PER_UNIT_TOL:
        logger.warning(
            "sim.stage_units_in_meters=%s is not 1.0; the rest of the pipeline assumes 1 unit == 1 metre "
            "(ENU translates are metres, camera intrinsics are authored against a metre stage), so this "
            "silently rescales the world relative to the poses. Setting it anyway, but this is unsupported.",
            meters_per_unit,
        )

    usdgeom.SetStageMetersPerUnit(stage, meters_per_unit)
    logger.info("set stage metersPerUnit to %s", meters_per_unit)
    return True


def apply_hdri(stage: Any, hdri: str | None) -> bool:  # noqa: ANN401
    """
    Light the scene from an HDRI image.

    `assets.hdri` is a path to an ``.exr`` / ``.hdr`` latlong image. If the scene already
    has a dome light -- the shipped ``earth`` scene has one at
    ``/World/Environment/DomeLight`` -- its texture is repointed at the image, which is what
    a user actually wants: change the sky, not add a second competing light. Only when no
    dome light exists is one created.

    Args:
        stage: The open USD stage.
        hdri: Path to the HDRI image. Empty or ``None`` does nothing, so a scene with its
            own lighting is left untouched.

    Returns:
        ``True`` if a dome light was created or updated.

    """
    if not hdri:
        return False

    path = Path(hdri).expanduser()
    if not path.is_file():
        logger.warning("assets.hdri %s does not exist; not applying an HDRI", path)
        return False

    usdlux = _usdlux()
    existing = [usdlux.DomeLight(prim) for prim in stage.Traverse() if prim.IsA(usdlux.DomeLight)]

    if existing:
        for dome in existing:
            dome.CreateTextureFileAttr().Set(str(path))
            dome.CreateTextureFormatAttr().Set("latlong")
            logger.info("repointed dome light %s at HDRI %s", dome.GetPath(), path)
        return True

    # No dome light in the scene: create one so the HDRI still lights something.
    sdf = _sdf()
    dome = usdlux.DomeLight.Define(stage, sdf.Path(_HDRI_DOME_PATH))
    dome.CreateTextureFileAttr().Set(str(path))
    # latlong matches the equirectangular .exr the team uses; the alternative is a cube map.
    dome.CreateTextureFormatAttr().Set("latlong")
    logger.info("no existing dome light; created %s using HDRI %s", _HDRI_DOME_PATH, path)
    return True


# USD attribute on a Cesium tileset prim that holds the tileset.json URL.
_CESIUM_URL_ATTR: Final = "cesium:url"


def _parse_server_override(override: str) -> tuple[str, str, str] | None:
    """
    Split a server override into its scheme, network location and path prefix.

    The override supplies only the server: scheme, host and (optional) port. A missing
    scheme is tolerated -- ``newhost:9000`` and ``//newhost:9000`` are parsed as a network
    location rather than a scheme, defaulting the scheme to ``http``. Any path component on
    the override is treated as a prefix prepended to each tileset's own path; the normal
    case (no path) leaves each path untouched. A query or fragment on the override is
    ignored, since those belong to the individual tileset.

    Args:
        override: The configured server override, for example ``http://newhost:9000``.

    Returns:
        A ``(scheme, netloc, path_prefix)`` triple, or ``None`` if the override has no
        parseable host.

    """
    split = urlsplit(override)
    scheme, netloc, path = split.scheme, split.netloc, split.path

    # A bare "host:port" parses with the host as the scheme and no netloc; re-parse it as
    # a network location so authority-only overrides work without a scheme. Only do this
    # when the override carries no "scheme://" marker, so a malformed "http://" stays
    # hostless rather than being mangled into a host named "http".
    if not netloc and "://" not in override:
        split = urlsplit(f"//{override.lstrip('/')}")
        scheme, netloc, path = split.scheme, split.netloc, split.path

    if not split.hostname:
        return None

    scheme = scheme or "http"
    path_prefix = path.rstrip("/")
    return scheme, netloc, path_prefix


def _rewrite_url(original: str, scheme: str, netloc: str, path_prefix: str) -> str:
    """
    Swap the server of ``original`` while preserving its path, query and fragment.

    A relative or path-only original (no scheme and no host) is treated as a path under
    the new server, so it gains the override's scheme and host rather than being left
    server-less.

    Args:
        original: The tileset's current URL.
        scheme: Replacement scheme.
        netloc: Replacement network location (host and optional port).
        path_prefix: Path fragment prepended to the original path, usually empty.

    Returns:
        The rewritten URL.

    """
    parts = urlsplit(original)
    path = parts.path if parts.path.startswith("/") or not parts.path else f"/{parts.path}"
    combined_path = f"{path_prefix}{path}" if path_prefix else path
    return urlunsplit((scheme, netloc, combined_path, parts.query, parts.fragment))


def apply_tileset_server_url(
    stage: Any,  # noqa: ANN401
    url: str | None,
    tilesets_root: str,
) -> int:
    """
    Repoint every Cesium tileset under ``tilesets_root`` at a different server.

    Lets a team member switch tile servers from config instead of hand-editing the scene
    in the GUI, which matters because the URL is otherwise baked into the USD. Only the
    scheme, host and port are taken from ``url``; each tileset keeps its own path and query,
    so N tilesets that share a server but differ by path all move together. A bad override
    (unparseable or hostless) is logged and ignored rather than blanking out the terrain.

    Args:
        stage: The open USD stage.
        url: Server override supplying scheme/host/port. ``None`` or empty leaves the
            scene's own URLs untouched.
        tilesets_root: Prim path whose subtree is searched for tilesets.

    Returns:
        Number of tilesets repointed.

    """
    if not url:
        return 0

    parsed = _parse_server_override(url)
    if parsed is None:
        logger.warning("tileset server override %r has no host; leaving tileset URLs untouched", url)
        return 0
    scheme, netloc, path_prefix = parsed

    root = stage.GetPrimAtPath(tilesets_root)
    if not root.IsValid():
        logger.warning("tilesets root %s does not exist; not applying the tileset URL", tilesets_root)
        return 0

    changed = 0
    for prim in _usd().PrimRange(root):
        attribute = prim.GetAttribute(_CESIUM_URL_ATTR)
        if not attribute.IsValid():
            continue
        previous = attribute.Get()
        if not previous:
            logger.warning("tileset %s has an empty %s; leaving it alone", prim.GetPath(), _CESIUM_URL_ATTR)
            continue
        rewritten = _rewrite_url(previous, scheme, netloc, path_prefix)
        if rewritten == previous:
            continue
        attribute.Set(rewritten)
        logger.info("tileset %s: %s -> %s", prim.GetPath(), previous, rewritten)
        changed += 1

    return changed


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
        # Each planned layer records the vehicle it belongs to, so a swarm mounts one copy per
        # aircraft. Falling back to the caller's instance keeps a plan built before per-vehicle
        # planning working unchanged.
        layer_instance = planned.instance if planned.instance != "default" else instance
        mount = mount_path_for_layer(planned, layer_instance)
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

    Tries ``sim.viewport_camera`` first, because that is already the single place naming the
    vehicle's camera -- the runtime points the main viewport at it -- so a layer binding and the
    viewport cannot disagree about which camera is "the" camera.

    Falls back to searching the vehicle's mount for the first ``Camera``-typed prim, which keeps
    a custom camera layer working without anyone editing config.

    This used to guess ``{mount}/Camera_{camera_id}``, a path no shipped layer has ever used
    (the real one is ``{mount}/Xform/main_camera_01``). It therefore always returned ``None``,
    and any layer binding ``resolve = "camera_prim"`` aborted composition with
    "no camera path is available" -- which is what stopped the bbox layer from ever loading.

    Args:
        config: The resolved configuration.
        stage: The open stage.

    Returns:
        The camera prim path, or ``None`` if no camera can be found.

    """
    sdf = _sdf()
    first_vehicle_id = next(iter(config.vehicles))
    mount = config.resolved_mount(first_vehicle_id)

    configured = config.sim.viewport_camera
    if configured:
        candidate = render(configured, instance=first_vehicle_id, mount=mount)
        if stage.GetPrimAtPath(sdf.Path(candidate)).IsValid():
            return candidate
        logger.debug("sim.viewport_camera %r is not on the stage; searching the mount", candidate)

    mount_prim = stage.GetPrimAtPath(sdf.Path(mount))
    if not mount_prim.IsValid():
        return None
    usd = importlib.import_module("pxr.Usd")
    for prim in usd.PrimRange(mount_prim):
        if prim.GetTypeName() == "Camera":
            found = str(prim.GetPath())
            logger.debug("resolved camera prim by type search: %s", found)
            return found
    return None


def apply_target_semantics(stage: Any, targets_root: str) -> int:  # noqa: ANN401
    """
    Give every child of the targets root a semantic label.

    Isaac's bounding-box annotators report only prims carrying a ``SemanticsAPI``, and they key
    off it entirely -- an unlabelled prim is invisible to them no matter how solid it looks on
    screen. The 2023 scene labelled each target by hand in the GUI, which is easy to forget and
    silently yields an empty detection list.

    Applied at composition time instead, so adding an object to the scene is all a user has to
    do. The label is the prim's own name, which is also what the projector publishes as
    ``target_name``, keeping the two consistent by construction.

    Args:
        stage: The open stage.
        targets_root: Absolute path of the scope holding the tracked objects.

    Returns:
        Number of prims labelled.

    """
    sdf = _sdf()
    root = stage.GetPrimAtPath(sdf.Path(targets_root))
    if not root.IsValid():
        logger.debug("targets root %s absent; no semantics to apply", targets_root)
        return 0

    try:
        # Top-level `Semantics`, not `pxr.Semantics`: the latter is deprecated in Isaac Sim 6 and
        # warns on import.
        semantics = importlib.import_module("Semantics")
    except ImportError:
        logger.warning("Semantics module unavailable; bbox annotators will report nothing")
        return 0

    labelled = 0
    for prim in root.GetChildren():
        try:
            api = semantics.SemanticsAPI.Apply(prim, "Semantics")
            api.CreateSemanticTypeAttr().Set("class")
            api.CreateSemanticDataAttr().Set(prim.GetName())
        except Exception as exc:  # noqa: BLE001 - USD raises Tf.ErrorException
            logger.warning("could not label %s: %s", prim.GetPath(), exc)
            continue
        labelled += 1

    logger.info("labelled %d bbox target(s) under %s", labelled, targets_root)
    return labelled


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
    "apply_stage_units",
    "compose_stage",
    "layer_usd_path",
    "mount_path_for_layer",
]
