"""
OmniGraph node: raycast from a prim and report the distance to the first hit.

Shaped to feed Isaac's generic ``isaacsim.ros2.bridge.ROS2Publisher`` configured for
``sensor_msgs/Range``: wire ``range_m``, ``min_range_out`` and ``max_range_out`` to the
publisher's matching inputs.

All *interpretation* of the reading is delegated to
:mod:`isaac_core.contracts.rangefinder`, which is unit-tested without Isaac Sim. This module
only performs the raycast and hands the raw result over, keeping the boundary logic (no
detection, too close, beyond range) in one tested place.

Isaac Sim 6 does ship a ``RaycastSensor``, but it is a Python runtime class rather than an
OmniGraph node or a USD schema, so it cannot be placed in an action graph. The PhysX
scene-query interface used here is what Isaac exposes for exactly this, which keeps the
sensing pipeline inside the graph like every other feature layer.
"""

import importlib

import carb
from isaac_core_ogn.sensors.ogn.OgnDistanceSensorDatabase import OgnDistanceSensorDatabase

from isaac_core.contracts.rangefinder import NO_DETECTION, resolve_range

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | RANGE |"


def _sensor_prim_path(db: OgnDistanceSensorDatabase) -> str | None:
    """
    Return the first target of the ``sensorPrim`` relationship, or ``None``.

    Args:
        db: OmniGraph node database.

    Returns:
        The prim path as a string, or ``None`` when nothing is wired.

    """
    targets = db.inputs.sensorPrim
    if targets is None:
        return None
    try:
        first = targets[0]
    except (IndexError, TypeError):
        return None
    return str(first)


# One raycast sequence per node instance, keyed by node path. A sequence is the render
# raycast's per-frame idiom: submit this frame's ray, read the latest result that is ready.
# Created lazily because the interface is unavailable until the extension has started.
_SEQUENCES: dict[str, int] = {}


def _raycast_interface() -> object | None:
    """
    Return the render raycast query interface, or ``None`` if unavailable.

    Deliberately the **render** raycast rather than PhysX. PhysX only hits collision geometry,
    and Cesium 3D Tiles terrain has none -- it is rendered geometry with no colliders -- so a
    PhysX raycast over the terrain reports "no detection" forever. This is the interface the
    2023 sensor used, for the same reason.

    Returns:
        The acquired interface, or ``None`` when the extension is not loaded.

    """
    try:
        module = importlib.import_module("omni.kit.raycast.query")
    except ImportError:
        return None
    try:
        return module.acquire_raycast_query_interface()
    except Exception:  # noqa: BLE001 - carb raises bare errors when the interface is absent
        return None


def _cast(
    db: OgnDistanceSensorDatabase,
    origin: object,
    direction: object,
    min_range: float,
    max_range: float,
) -> tuple[bool | None, float]:
    """
    Submit this frame's ray and read the latest ready result.

    The result is whatever the renderer has finished, so it can lag the submitted ray by a
    frame. That is inherent to an asynchronous query and is why a sequence is used: it holds the
    last valid value rather than flickering to "no hit" while a query is in flight.

    Args:
        db: OmniGraph node database.
        origin: Ray origin in world space.
        direction: Ray direction in world space.
        min_range: Closest reportable distance in metres.
        max_range: Furthest reportable distance in metres.

    Returns:
        ``(hit, distance)``, or ``(None, 0.0)`` when the interface is unavailable.

    """
    interface = _raycast_interface()
    if interface is None:
        carb.log_warn(f"{_LOG_PREFIX} omni.kit.raycast.query unavailable; reporting no detection")
        return None, 0.0

    module = importlib.import_module("omni.kit.raycast.query")
    key = str(db.node.get_prim_path())
    sequence = _SEQUENCES.get(key)
    if sequence is None:
        sequence = interface.add_raycast_sequence()
        _SEQUENCES[key] = sequence

    ray = module.Ray(
        (float(origin[0]), float(origin[1]), float(origin[2])),
        (float(direction[0]), float(direction[1]), float(direction[2])),
        min_t=0.0,
        max_t=max_range,
    )
    interface.submit_ray_to_raycast_sequence(sequence, ray)

    status, _submitted, result = interface.get_latest_result_from_raycast_sequence(sequence)
    if status != module.Result.SUCCESS or not result.valid:
        return False, 0.0
    return True, float(result.hit_t)


class OgnDistanceSensor:
    """OmniGraph node: PhysX raycast distance sensor."""

    @staticmethod
    def compute(db: OgnDistanceSensorDatabase) -> bool:
        """
        Cast a ray from the sensor prim and write the resulting range.

        Args:
            db: OmniGraph node database.

        Returns:
            ``True`` always. A bad configuration or a missing prim is logged once per tick
            and reported as "no detection" rather than failing the graph, so one bad tick
            cannot stop the simulation -- consistent with the other nodes here.

        """
        import importlib  # noqa: PLC0415

        min_range = float(db.inputs.min_range_m)
        max_range = float(db.inputs.max_range_m)

        # Pass the rated band through so the publisher reads it from one source of truth.
        db.outputs.min_range_out = min_range
        db.outputs.max_range_out = max_range

        prim_path = _sensor_prim_path(db)
        if prim_path is None:
            carb.log_warn(f"{_LOG_PREFIX} No sensorPrim wired; reporting no detection")
            db.outputs.range_m = NO_DETECTION
            db.outputs.hit = False
            return True

        omni_usd = importlib.import_module("omni.usd")
        usd_geom = importlib.import_module("pxr.UsdGeom")
        usd_mod = importlib.import_module("pxr.Usd")

        stage = omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path) if stage is not None else None
        if prim is None or not prim.IsValid():
            carb.log_warn(f"{_LOG_PREFIX} sensorPrim {prim_path} is not valid")
            db.outputs.range_m = NO_DETECTION
            db.outputs.hit = False
            return True

        transform = usd_geom.Xformable(prim).ComputeLocalToWorldTransform(usd_mod.TimeCode.Default())
        origin = transform.ExtractTranslation()
        # Local -Z, matching the USD camera convention, so a sensor parented to the camera
        # points where the camera looks.
        rotation = transform.ExtractRotationMatrix()
        direction = -rotation.GetRow(2)

        hit, distance = _cast(db, origin, direction, min_range, max_range)
        if hit is None:
            db.outputs.range_m = NO_DETECTION
            db.outputs.hit = False
            return True

        try:
            db.outputs.range_m = resolve_range(hit, distance, min_range, max_range)
        except ValueError as exc:
            carb.log_warn(f"{_LOG_PREFIX} {exc}")
            db.outputs.range_m = NO_DETECTION
            db.outputs.hit = False
            return True

        db.outputs.hit = hit
        return True
