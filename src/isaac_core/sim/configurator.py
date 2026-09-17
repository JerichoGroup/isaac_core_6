"""Pure configuration applicator: decide what to write, never how to write it.

Given a resolved configuration, a feature plan and the ENU reference, this module
produces a deterministic, ordered list of ``(prim_path, attribute_name, value)``
tuples. A separate ``AttributeWriter`` implementation performs the actual USD
writes, making the entire logic here testable against a recording fake with no
Isaac dependency.

This is the replacement for the scattered ``_configure_camera``,
``_configure_extensions_ros2`` and ``_set_cesium_tilesets_url`` methods that the
alternative is one method per feature, each hard-coding prim paths and duplicating
config lookups. Here, wiring is declared in layer manifests and applied uniformly.

Every runtime resolver is scoped to an explicit vehicle and camera threaded from
:func:`compute_writes`, never picked with ``next(iter(...))``. There is no module-level
"current vehicle" state. The identity a resolver uses should ultimately come from the
plan: a :class:`~isaac_core.sim.planner.PlannedLayer` is planned for one instance. It does
not record it yet, so :func:`compute_writes` takes ``vehicle_id`` as a parameter instead;
when ``PlannedLayer`` gains an ``instance`` field, per-layer scoping is a matter of reading
it off each planned layer inside the loop rather than passing one identity for the whole
call.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import logging
import math
from typing import Any, Protocol, runtime_checkable

from isaac_core.config import IsaacCoreConfig
from isaac_core.config.schema import CameraConfig
from isaac_core.contracts.topics import MAVROS_LLA_LEAF, MAVROS_ORIENTATION_LEAF
from isaac_core.sim.georeference import ResolvedEnuReference
from isaac_core.sim.planner import FeaturePlan, ResolvedBinding

logger = logging.getLogger(__name__)


class ConfigKeyError(Exception):
    """Raised when a binding references a config key that does not exist."""


@dataclass(frozen=True, slots=True)
class AttributeWrite:
    """One concrete write: apply ``value`` to ``attribute`` on ``prim``.

    Ordering matters: later writes to the same (prim, attribute) pair win.
    """

    prim: str
    attribute: str
    value: Any


@runtime_checkable
class AttributeWriter(Protocol):
    """Narrow interface for writing a single attribute value to a USD prim.

    A real implementation uses ``pxr.Usd.Stage``; tests use :class:`RecordingWriter`.
    """

    def write(self, prim: str, attribute: str, value: Any) -> None:
        """Set an attribute on a prim.

        Args:
            prim: Absolute USD prim path.
            attribute: Attribute name.
            value: The value to write.

        """
        ...


@dataclass(slots=True)
class RecordingWriter:
    """In-memory writer for testing: records every write in order.

    Use ``writes`` to inspect the full sequence after configuration is applied.
    """

    writes: list[AttributeWrite] = field(default_factory=list)

    def write(self, prim: str, attribute: str, value: Any) -> None:
        """Record a write."""
        self.writes.append(AttributeWrite(prim=prim, attribute=attribute, value=value))


def _lookup_config(config: IsaacCoreConfig, dotted_key: str) -> Any:
    """Traverse the config model by a dotted path.

    Supports both pydantic model attributes and dict keys for the ``vehicles``
    and ``layers`` dicts. Examples: ``"vehicles.drone_0.camera.fov_deg"``,
    ``"cesium.tileset_server_url"``.

    Args:
        config: The resolved configuration.
        dotted_key: A dotted path into the config, e.g. ``"sim.headless"``.

    Returns:
        The value at the specified path.

    Raises:
        ConfigKeyError: If any segment of the path does not exist.

    """
    parts = dotted_key.split(".")
    current: Any = config
    traversed: list[str] = []

    for part in parts:
        traversed.append(part)
        if isinstance(current, dict):
            try:
                current = current[part]
            except KeyError:
                msg = f"config key {dotted_key!r} not found: " f"{'.'.join(traversed)} does not exist"
                raise ConfigKeyError(msg) from None
        elif hasattr(current, part):
            current = getattr(current, part)
        else:
            msg = f"config key {dotted_key!r} not found: " f"{'.'.join(traversed)} does not exist"
            raise ConfigKeyError(msg)

    return current


def _resolve_runtime_value(
    name: str,
    *,
    config: IsaacCoreConfig,
    enu_reference: ResolvedEnuReference,
    camera_prim: str | None,
    vehicle_id: str,
) -> Any:
    """Compute a runtime-derived value by its resolve name.

    Every resolver produces the value for the specific ``vehicle_id``, supplied by the caller
    (ultimately the plan) rather than picked with ``next(iter(...))``, so the same
    resolvers serve every aircraft in a swarm without any global "current vehicle" state.

    Args:
        name: The resolve identifier from the binding.
        config: The resolved configuration.
        enu_reference: The computed ENU reference for this run.
        camera_prim: The active camera prim path, if known.
        vehicle_id: The vehicle this binding belongs to.

    Returns:
        The value to write.

    Raises:
        ConfigKeyError: If the resolve name is unknown.

    """
    from isaac_core.contracts.prims import BBOXES_ROOT
    from isaac_core.contracts.topics import BBOX, GLOBAL_POSE

    resolvers: dict[str, Any] = {
        "enu_origin": lambda: [
            enu_reference.reference.lat_deg,
            enu_reference.reference.lon_deg,
            enu_reference.reference.alt_m,
        ],
        "camera_prim": lambda: _resolve_camera_prim_value(camera_prim),
        "bboxes_root": lambda: BBOXES_ROOT,
        "udp_port": lambda: config.resolved_udp_port(vehicle_id),
        "image_topic": lambda: _resolve_image_topic(config, vehicle_id),
        "global_pose_topic": lambda: _resolve_vehicle_topic(config, GLOBAL_POSE, vehicle_id),
        "distance_topic": lambda: _resolve_distance_topic(config, vehicle_id),
        "bbox_topic": lambda: _resolve_vehicle_topic(config, BBOX, vehicle_id),
        "rtsp_mount_path": lambda: _resolve_rtsp_mount_path(config, vehicle_id),
        "rtsp_port": lambda: config.resolved_rtsp_port(vehicle_id),
        "camera_horizontal_aperture": lambda: _resolve_horizontal_aperture(config, vehicle_id),
        "camera_vertical_aperture": lambda: _resolve_vertical_aperture(config, vehicle_id),
        "lla_topic": lambda: _resolve_mavros_topic(config, MAVROS_LLA_LEAF, vehicle_id),
        "orientation_topic": lambda: _resolve_mavros_topic(config, MAVROS_ORIENTATION_LEAF, vehicle_id),
    }

    resolver = resolvers.get(name)
    if resolver is None:
        msg = f"unknown resolve name {name!r}"
        raise ConfigKeyError(msg)
    return resolver()


def _first_vehicle_id(config: IsaacCoreConfig) -> str:
    """Return the first vehicle id in declaration order.

    Args:
        config: The resolved configuration.

    Returns:
        The first vehicle's key.

    """
    return config.first_vehicle_id


def _resolve_mavros_topic(config: IsaacCoreConfig, leaf: str, vehicle_id: str) -> str:
    """Return a vehicle's MAVROS topic, explicit if configured, else derived.

    MAVROS topic names default to ``None`` in config, meaning "derive from this
    vehicle's ``mavros_namespace``". Deriving keeps a swarm working, where each aircraft
    has its own MAVROS namespace, without anyone spelling out four topic names per
    vehicle.

    Args:
        config: The resolved configuration.
        leaf: Topic path below the MAVROS namespace, e.g. ``"local_position/pose"``.
        vehicle_id: The vehicle whose MAVROS topic to resolve.

    Returns:
        The fully resolved topic name.

    """
    vehicle = config.vehicles[vehicle_id]
    explicit = vehicle.lla_topic if leaf == MAVROS_LLA_LEAF else vehicle.orientation_topic
    if explicit is not None:
        return explicit
    return f"{vehicle.mavros_namespace.rstrip('/')}/{leaf}"


def _resolve_camera_prim_value(camera_prim: str | None) -> str:
    """Return the camera prim or raise if not available.

    Args:
        camera_prim: The active camera prim path, or ``None``.

    Returns:
        The camera prim path.

    Raises:
        ConfigKeyError: If no camera path is available.

    """
    if camera_prim is None:
        msg = "resolve 'camera_prim' requested but no camera path is available"
        raise ConfigKeyError(msg)
    return camera_prim


def _camera_for(config: IsaacCoreConfig, vehicle_id: str) -> CameraConfig:
    """Return a specific vehicle's camera, defaulting to its first when unspecified.

    Args:
        config: The resolved configuration.
        vehicle_id: The vehicle whose camera to return.

    Returns:
        The vehicle's camera config.

    """
    return config.vehicles[vehicle_id].camera


# Default ``fov_deg`` declared on CameraConfig. Used to tell "the user left fov alone" from
# "the user set fov AND an explicit aperture", which is the collision worth warning about.
_DEFAULT_FOV_DEG: float = CameraConfig.model_fields["fov_deg"].default


def _resolve_horizontal_aperture(
    config: IsaacCoreConfig,
    vehicle_id: str | None = None,
) -> float:
    """Compute the camera horizontal aperture in mm, honouring an explicit override.

    A pinhole camera relates horizontal field of view, focal length and sensor width by
    ``aperture = 2 * focal_length * tan(fov / 2)``. Isaac's camera prim is driven by focal
    length and aperture, not by an fov attribute, so ``fov_deg`` only takes effect once it
    is turned into an aperture here. Before this it was read from config and applied
    nowhere, which is why changing it did nothing visible.

    When ``horizontal_aperture_mm`` is set it wins outright and ``fov_deg`` is ignored for
    this axis. Setting both an explicit aperture and a non-default ``fov_deg`` makes one of
    them meaningless, so the collision is announced -- naming the winner -- rather than left
    silent, matching how a prim_override that shadows a binding is reported.

    Args:
        config: The resolved configuration.
        vehicle_id: The vehicle whose camera to resolve, or ``None`` for the first vehicle.

    Returns:
        Horizontal aperture in millimetres.

    """
    resolved_vehicle_id = vehicle_id if vehicle_id is not None else _first_vehicle_id(config)
    camera = _camera_for(config, resolved_vehicle_id)
    if camera.horizontal_aperture_mm is not None:
        if camera.fov_deg != _DEFAULT_FOV_DEG:
            logger.warning(
                "camera has both horizontal_aperture_mm=%s and a non-default fov_deg=%s; the "
                "explicit aperture wins and fov_deg is ignored for the horizontal axis",
                camera.horizontal_aperture_mm,
                camera.fov_deg,
            )
        return camera.horizontal_aperture_mm
    return 2.0 * camera.focal_length_mm * math.tan(math.radians(camera.fov_deg) / 2.0)


def _resolve_vertical_aperture(
    config: IsaacCoreConfig,
    vehicle_id: str | None = None,
) -> float:
    """Compute the vertical aperture in mm, honouring an explicit override.

    Keeping the vertical aperture consistent with the resolution's aspect ratio avoids a
    stretched image, which is the usual symptom of setting one aperture and leaving the
    other at its authored default. When ``vertical_aperture_mm`` is set it wins outright and
    the derived value is ignored; otherwise it follows from the (possibly overridden)
    horizontal aperture and the aspect ratio.

    Args:
        config: The resolved configuration.
        vehicle_id: The vehicle whose camera to resolve, or ``None`` for the first vehicle.

    Returns:
        Vertical aperture in millimetres.

    """
    resolved_vehicle_id = vehicle_id if vehicle_id is not None else _first_vehicle_id(config)
    camera = _camera_for(config, resolved_vehicle_id)
    if camera.vertical_aperture_mm is not None:
        return camera.vertical_aperture_mm
    return _resolve_horizontal_aperture(config, resolved_vehicle_id) * (camera.height / camera.width)


def _resolve_rtsp_mount_path(config: IsaacCoreConfig, vehicle_id: str) -> str:
    """Return a camera's RTSP mount path, explicit if set, otherwise derived.

    Mirrors the topic convention: a single camera streams at ``/stream``, and once there is
    more than one vehicle or camera the path is namespaced so two streams cannot collide on
    the same port. The vehicle segment appears whenever the config is multi-vehicle, and the
    camera segment whenever the vehicle carries more than one camera; index ``i`` of one
    vehicle's stream can never alias another's.

    Args:
        config: The resolved configuration.
        vehicle_id: The vehicle whose camera to resolve.

    Returns:
        The mount path, always beginning with ``/``.

    """
    explicit = config.vehicles[vehicle_id].camera.rtsp_mount_path
    if explicit is not None:
        return explicit if explicit.startswith("/") else f"/{explicit}"

    parts = []
    if not config.is_single_vehicle:
        parts.append(vehicle_id)
    return "/" + "/".join([*parts, "stream"])


# The three topic resolvers below repeat an "explicit config value wins, else derive" shape. That
# is deliberate: what differs is the config field each reads and how each derives, which is the
# entire substance. Folding them into one parameterised helper would trade three readable lines for
# a callable or a dotted-path argument, and hide which field feeds which topic.
def _resolve_distance_topic(config: IsaacCoreConfig, vehicle_id: str) -> str:
    """Return the distance sensor's topic, explicit if set, otherwise derived.

    Same contract as the image and MAVROS topics: an explicit value in config wins, and the
    conventional namespaced name is derived otherwise. Deriving keeps a swarm working without
    anyone spelling out a topic per vehicle.

    Args:
        config: The resolved configuration.
        vehicle_id: The vehicle whose distance topic to resolve.

    Returns:
        The fully resolved topic name.

    """
    from isaac_core.contracts.topics import DISTANCE_SENSOR

    explicit = config.vehicles[vehicle_id].distance_sensor.topic
    if explicit is not None:
        return explicit
    return _resolve_vehicle_topic(config, DISTANCE_SENSOR, vehicle_id)


def _resolve_image_topic(config: IsaacCoreConfig, vehicle_id: str) -> str:
    """Return a camera's image topic, explicit if set, otherwise derived.

    Deriving keeps the namespacing convention working for a swarm, while an explicit
    ``image_topic`` in config is honoured as written -- the same contract the MAVROS
    topics use. Before this, the field was silently ignored and only the derived name
    ever reached the publisher.

    Args:
        config: The resolved configuration.
        vehicle_id: The vehicle whose camera to resolve.

    Returns:
        The fully resolved image topic name.

    """
    from isaac_core.contracts.topics import IMAGE_RGB

    explicit = config.vehicles[vehicle_id].camera.image_topic
    if explicit is not None:
        return explicit
    return _resolve_topic(config, IMAGE_RGB, vehicle_id, camera_scoped=True)


def _resolve_topic(
    config: IsaacCoreConfig,
    leaf: str,
    vehicle_id: str,
    *,
    camera_scoped: bool = False,
) -> str:
    """Resolve a topic name for a specific vehicle and camera.

    Args:
        config: The resolved configuration.
        leaf: Topic leaf name.
        vehicle_id: The vehicle whose topic to resolve.
        camera_scoped: Whether to include the camera in the resolution.

    Returns:
        The fully resolved topic name.

    """
    if camera_scoped:
        resolver = config.topic_resolver(vehicle_id)
        return resolver.resolve(leaf)
    return config.topic_resolver(vehicle_id).vehicle_scoped(leaf)


def _resolve_vehicle_topic(config: IsaacCoreConfig, leaf: str, vehicle_id: str) -> str:
    """Resolve a vehicle-scoped topic for a specific vehicle.

    Args:
        config: The resolved configuration.
        leaf: Topic leaf name.
        vehicle_id: The vehicle whose topic to resolve.

    Returns:
        The fully resolved topic name.

    """
    resolver = config.topic_resolver(vehicle_id)
    return resolver.vehicle_scoped(leaf)


def compute_horizontal_aperture(fov_deg: float, focal_length_mm: float) -> float:
    """Compute horizontal aperture from field of view and focal length.

    Uses the rectilinear projection formula::

        aperture = 2 * focal_length * tan(fov / 2)

    Aperture follows from the field of view and the sensor width.

    Args:
        fov_deg: Horizontal field of view in degrees.
        focal_length_mm: Focal length in millimetres.

    Returns:
        Horizontal aperture in millimetres.

    """
    fov_r = math.radians(fov_deg)
    return 2.0 * focal_length_mm * math.tan(fov_r / 2.0)


def compute_writes(
    config: IsaacCoreConfig,
    plan: FeaturePlan,
    enu_reference: ResolvedEnuReference,
    *,
    camera_prim: str | None = None,
    vehicle_id: str | None = None,
) -> list[AttributeWrite]:
    """Produce the ordered list of attribute writes for a simulation run.

    The order is deterministic and always:

    1. Layer bindings, in plan order (which is sorted by layer id), then in
       declaration order within each layer's manifest.
    2. ``[[prim_overrides]]`` from config, in declaration order.

    Overrides applied last guarantee the escape hatch wins over any layer binding
    that targets the same (prim, attribute) pair.

    Every runtime-derived value resolves for an explicit ``vehicle_id`` rather than
    silently for the first vehicle. When it is omitted it defaults to the first vehicle,
    preserving the single-vehicle behaviour existing callers rely on. A ``PlannedLayer``
    is planned for one instance today, so per-layer scoping is available once the plan
    carries that identity (see the module docstring's note on the planner).

    Args:
        config: The resolved configuration.
        plan: The feature plan from the planner.
        enu_reference: The computed ENU reference for this run.
        camera_prim: Path to the active camera prim, if known.
        vehicle_id: The vehicle these writes serve, or ``None`` for the first vehicle.

    Returns:
        Ordered writes ready to be applied.

    Raises:
        ConfigKeyError: If a binding references a config key or resolve name that
            does not exist.

    """
    resolved_vehicle_id = vehicle_id if vehicle_id is not None else _first_vehicle_id(config)

    writes: list[AttributeWrite] = []

    for planned in plan.enabled:
        # Per-layer identity: in a swarm each vehicle's camera layer resolves its own ports and
        # topics, so taking one identity for the whole call would give every vehicle the first
        # vehicle's values.
        layer_vehicle = planned.instance if planned.instance in config.vehicles else resolved_vehicle_id
        for binding in planned.resolved_bindings:
            value = _resolve_binding_value(
                binding,
                config=config,
                enu_reference=enu_reference,
                camera_prim=camera_prim,
                vehicle_id=layer_vehicle,
            )
            writes.append(AttributeWrite(prim=binding.prim, attribute=binding.attribute, value=value))

    bound_targets = {(w.prim, w.attribute) for w in writes}
    for override in config.prim_overrides:
        target = (override.prim, override.attribute)
        if target in bound_targets:
            # The override still wins, by design -- it is applied last. But a value set from
            # two places, where only one takes effect, is exactly the confusion that makes a
            # config file untrustworthy, so it is called out rather than left silent.
            logger.warning(
                "prim_override for %s.%s also has a layer binding; the override wins and the "
                "bound value (e.g. from a config field) is ignored",
                override.prim,
                override.attribute,
            )
        writes.append(AttributeWrite(prim=override.prim, attribute=override.attribute, value=override.value))

    return writes


def _resolve_binding_value(
    binding: ResolvedBinding,
    *,
    config: IsaacCoreConfig,
    enu_reference: ResolvedEnuReference,
    camera_prim: str | None,
    vehicle_id: str,
) -> Any:
    """Resolve the value for a single binding.

    Args:
        binding: The resolved binding from the plan.
        config: The resolved configuration.
        enu_reference: The computed ENU reference for this run.
        camera_prim: Active camera prim path.
        vehicle_id: The vehicle this binding serves.

    Returns:
        The value to write.

    Raises:
        ConfigKeyError: If the binding's source cannot be resolved.

    """
    if binding.config is not None:
        return _lookup_config(config, binding.config)
    if binding.resolve is not None:
        return _resolve_runtime_value(
            binding.resolve,
            config=config,
            enu_reference=enu_reference,
            camera_prim=camera_prim,
            vehicle_id=vehicle_id,
        )
    msg = f"binding for {binding.prim}:{binding.attribute} has neither config nor resolve"
    raise ConfigKeyError(msg)


def apply_writes(writer: AttributeWriter, writes: Sequence[AttributeWrite]) -> None:
    """Execute a sequence of attribute writes through the given writer.

    Args:
        writer: The writer implementation (real or fake).
        writes: Ordered writes to apply.

    """
    for w in writes:
        writer.write(w.prim, w.attribute, w.value)


__all__ = [
    "AttributeWrite",
    "AttributeWriter",
    "ConfigKeyError",
    "RecordingWriter",
    "apply_writes",
    "compute_horizontal_aperture",
    "compute_writes",
]
