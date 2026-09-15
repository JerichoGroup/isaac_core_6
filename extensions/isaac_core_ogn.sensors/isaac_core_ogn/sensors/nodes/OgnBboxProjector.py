"""OmniGraph node: report 2D bounding boxes for the tracked objects in a camera's view.

Boxes come from Isaac's own synthetic-data annotators, not from arithmetic here. That matters for
two reasons. They are measured from the rendered image, so a box cannot drift away from what the
camera actually shows -- a hand-rolled projection has to be kept in step with the camera's lens,
the viewport resolution and the render product, and silently produces plausible boxes in the
wrong place when any of those disagree. And the tight/loose pair gives **real occlusion**:
``loose`` bounds an object's full extent even when it is hidden, ``tight`` bounds only the visible
part and drops the object entirely when it is fully hidden. That is exactly the ``in_frame`` vs
``is_visible`` distinction, and it cannot be computed from geometry alone.

Outputs are parallel arrays named identically to ``isaac_core_ros2_msgs/FrameBboxes`` fields, so
each wires straight into Isaac's generic ROS 2 publisher.

Targets must carry a ``SemanticsAPI`` label: the annotators key off semantics and an unlabelled
prim is invisible to them. ``composer.apply_target_semantics`` applies one to every child of the
targets root at startup, so nobody has to remember.
"""

import importlib
import math

import carb
from isaac_core_ogn.sensors.ogn.OgnBboxProjectorDatabase import OgnBboxProjectorDatabase

from isaac_core.contracts.bbox import ARRAY_FIELDS, BboxArrays, BboxDetection
from isaac_core.geo.rotations import quaternion_to_euler

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | BBOX |"

# Box reported for a target the annotators do not mention at all.
_ZERO_BOX = (0, 0, 0, 0)

# Sensors enabled per viewport, keyed by node path, so the (non-trivial) enable happens once.
_ENABLED: dict[str, bool] = {}


def _prim_path(value: object) -> str | None:
    """Return a cleaned absolute prim path from a token input, or ``None``.

    Args:
        value: The token value read off the node database.

    Returns:
        The prim path, or ``None`` when unset or not absolute.

    """
    if value is None:
        return None
    text = str(value).strip()
    if not text.startswith("/"):
        return None
    return text


def _viewport_for(camera_path: str) -> object | None:
    """Return the viewport rendering the given camera.

    Prefers a viewport whose camera actually matches, so a second vehicle's boxes cannot be
    measured against the wrong picture. Falls back to the active viewport, which is the common
    single-vehicle case because ``sim.viewport_camera`` already points it at the drone camera.

    Args:
        camera_path: Absolute path of the camera prim.

    Returns:
        A viewport handle, or ``None`` when the viewport utility is unavailable.

    """
    try:
        utility = importlib.import_module("omni.kit.viewport.utility")
    except ImportError:
        return None

    try:
        instances = utility.get_viewport_window_instances()
    except Exception:
        instances = None

    if instances:
        for window in instances:
            api = getattr(window, "viewport_api", None)
            if api is not None and str(getattr(api, "camera_path", "")) == camera_path:
                return api

    try:
        return utility.get_active_viewport()
    except Exception:
        return None


def _annotator_rows(viewport: object, node_key: str) -> tuple[dict[str, object], dict[str, object]] | None:
    """Return the tight and loose annotator results, keyed by prim path.

    Enabling the sensors is done once per node: it allocates render resources, and doing it every
    frame would be both wasteful and disruptive to the render graph.

    Args:
        viewport: The viewport to read from.
        node_key: Unique key for this node instance.

    Returns:
        ``(tight, loose)`` dictionaries keyed by prim path, or ``None`` when unavailable.

    """
    try:
        sd = importlib.import_module("omni.syntheticdata")
        native = importlib.import_module("omni.syntheticdata._syntheticdata")
    except ImportError:
        return None

    sensor_types = [
        native.SensorType.BoundingBox2DTight,
        native.SensorType.BoundingBox2DLoose,
    ]
    if not _ENABLED.get(node_key):
        try:
            sd.sensors.enable_sensors(viewport, sensor_types)
        except Exception as exc:
            carb.log_warn(f"{_LOG_PREFIX} could not enable bbox sensors: {exc}")
            return None
        _ENABLED[node_key] = True

    try:
        tight = sd.sensors.get_bounding_box_2d_tight(viewport)
        loose = sd.sensors.get_bounding_box_2d_loose(viewport)
    except Exception as exc:
        carb.log_warn(f"{_LOG_PREFIX} annotators not ready: {exc}")
        return None

    # `name` is the full prim path, which is what lets a target be matched exactly rather than
    # by semantic label (labels are not unique).
    return ({str(row["name"]): row for row in tight}, {str(row["name"]): row for row in loose})


def _anchor_lla(target: object) -> tuple[float, float, float]:
    """Read a target's geodetic position from its Cesium globe anchor.

    Returns NaN rather than zero for a target with no anchor: zero is a real place off the coast
    of Africa, so a consumer could not tell "unknown" from "there". NaN is checkable.

    Args:
        target: The target prim.

    Returns:
        Latitude in degrees, longitude in degrees, altitude in metres.

    """
    values = []
    for attribute in ("cesium:anchor:latitude", "cesium:anchor:longitude", "cesium:anchor:height"):
        attr = target.GetAttribute(attribute)  # type: ignore[attr-defined]
        value = attr.Get() if attr is not None and attr.IsValid() else None
        values.append(math.nan if value is None else float(value))
    return values[0], values[1], values[2]


def _local_euler(target: object) -> tuple[float, float, float]:
    """Read a target's local orientation as euler XYZ radians.

    Uses the authored ``xformOp:orient`` -- the orientation set in the GUI -- matching what the
    published message documents, rather than the fully composed world rotation.

    Args:
        target: The target prim.

    Returns:
        Roll, pitch and yaw in radians. Zeros when no orientation is authored.

    """
    attr = target.GetAttribute("xformOp:orient")  # type: ignore[attr-defined]
    quat = attr.Get() if attr is not None and attr.IsValid() else None
    if quat is None:
        return 0.0, 0.0, 0.0
    # Gf.Quat is not iterable; real and imaginary parts come out separately.
    imaginary = quat.GetImaginary()
    return quaternion_to_euler(float(quat.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2]))


def _box_of(row: object) -> tuple[int, int, int, int]:
    """Return a annotator row's pixel box.

    Args:
        row: One row of an annotator's structured array.

    Returns:
        ``(x1, y1, x2, y2)`` in pixels.

    """
    return (int(row["x_min"]), int(row["y_min"]), int(row["x_max"]), int(row["y_max"]))  # type: ignore[index]


def _write_empty(db: OgnBboxProjectorDatabase) -> None:
    """Write empty arrays and a zero count to every output.

    Args:
        db: OmniGraph node database.

    """
    for name in ARRAY_FIELDS:
        setattr(db.outputs, name, [])
    db.outputs.count = 0


def _write_arrays(db: OgnBboxProjectorDatabase, arrays: BboxArrays) -> None:
    """Write the accumulated parallel arrays and their shared length to the outputs.

    Args:
        db: OmniGraph node database.
        arrays: The parallel output arrays built over this tick's targets.

    """
    for name, values in arrays.as_outputs().items():
        setattr(db.outputs, name, values)
    db.outputs.count = len(arrays)


def _append_target(
    arrays: BboxArrays,
    target: object,
    tight: dict[str, object],
    loose: dict[str, object],
    camera_position: object,
    usd_geom: object,
    time_code: object,
) -> None:
    """Append one target's box, visibility and pose to the output arrays.

    Every target gets an entry even when the annotators do not mention it, because a consumer
    matches detections by index across the arrays and a skipped target would misalign them all.

    Args:
        arrays: The accumulating parallel arrays.
        target: The target prim.
        tight: Tight annotator rows keyed by prim path.
        loose: Loose annotator rows keyed by prim path.
        camera_position: The camera's world translation.
        usd_geom: The imported ``pxr.UsdGeom`` module.
        time_code: The USD time code to sample at.

    """
    path = str(target.GetPath())  # type: ignore[attr-defined]
    loose_row = loose.get(path)
    tight_row = tight.get(path)

    # Loose gives the object's full extent, which is what a consumer wants to draw; tight only
    # answers whether any of it is actually visible.
    box = _box_of(loose_row) if loose_row is not None else _ZERO_BOX
    latitude, longitude, altitude = _anchor_lla(target)
    roll, pitch, yaw = _local_euler(target)
    position = usd_geom.Xformable(target).ComputeLocalToWorldTransform(time_code).ExtractTranslation()  # type: ignore[attr-defined]

    arrays.append(
        BboxDetection(
            target_name=target.GetName(),  # type: ignore[attr-defined]
            x1=box[0],
            y1=box[1],
            x2=box[2],
            y2=box[3],
            in_frame=loose_row is not None,
            is_visible=tight_row is not None,
            lat=latitude,
            lon=longitude,
            alt=altitude,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            distance_x=float(position[0] - camera_position[0]),  # type: ignore[index]
            distance_y=float(position[1] - camera_position[1]),  # type: ignore[index]
            distance_z=float(position[2] - camera_position[2]),  # type: ignore[index]
        )
    )


class OgnBboxProjector:
    """OmniGraph node: 2D bounding boxes from Isaac's synthetic-data annotators."""

    @staticmethod
    def compute(db: OgnBboxProjectorDatabase) -> bool:
        """Report a box, visibility and pose for every target under the targets root.

        Args:
            db: OmniGraph node database.

        Returns:
            ``True`` always. Anything missing -- an unset path, no viewport, annotators not yet
            ready -- is logged and reported as empty arrays rather than failing the graph, so one
            bad tick cannot stop the simulation.

        """
        camera_path = _prim_path(db.inputs.cameraPath)
        targets_path = _prim_path(db.inputs.targetsRootPath)
        if camera_path is None or targets_path is None:
            missing = "cameraPath" if camera_path is None else "targetsRootPath"
            carb.log_warn(f"{_LOG_PREFIX} {missing} is unset or not absolute; emitting empty arrays")
            _write_empty(db)
            return True

        omni_usd = importlib.import_module("omni.usd")
        usd_geom = importlib.import_module("pxr.UsdGeom")
        usd_mod = importlib.import_module("pxr.Usd")

        stage = omni_usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(camera_path) if stage is not None else None
        targets_prim = stage.GetPrimAtPath(targets_path) if stage is not None else None
        if camera_prim is None or not camera_prim.IsValid() or targets_prim is None or not targets_prim.IsValid():
            bad = camera_path if camera_prim is None or not camera_prim.IsValid() else targets_path
            carb.log_warn(f"{_LOG_PREFIX} prim {bad} is not valid; emitting empty arrays")
            _write_empty(db)
            return True

        viewport = _viewport_for(camera_path)
        if viewport is None:
            carb.log_warn(f"{_LOG_PREFIX} no viewport for {camera_path}; emitting empty arrays")
            _write_empty(db)
            return True

        rows = _annotator_rows(viewport, str(db.node.get_prim_path()))
        if rows is None:
            _write_empty(db)
            return True
        tight, loose = rows

        time_code = usd_mod.TimeCode.Default()
        camera_position = usd_geom.Xformable(camera_prim).ComputeLocalToWorldTransform(time_code).ExtractTranslation()

        arrays = BboxArrays()
        for target in targets_prim.GetChildren():
            _append_target(arrays, target, tight, loose, camera_position, usd_geom, time_code)

        _write_arrays(db, arrays)
        return True
