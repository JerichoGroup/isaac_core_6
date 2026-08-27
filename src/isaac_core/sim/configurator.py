"""
Pure configuration applicator: decide what to write, never how to write it.

Given a resolved configuration, a feature plan and the ENU reference, this module
produces a deterministic, ordered list of ``(prim_path, attribute_name, value)``
tuples. A separate ``AttributeWriter`` implementation performs the actual USD
writes, making the entire logic here testable against a recording fake with no
Isaac dependency.

This is the replacement for the scattered ``_configure_camera``,
``_configure_extensions_ros2`` and ``_set_cesium_tilesets_url`` methods that the
previous generation fused into a 353-line god class. Each method hard-coded prim
paths and duplicated config lookups; here, wiring is declared in layer manifests
and applied uniformly.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import logging
import math
from typing import Any, Protocol, runtime_checkable

from isaac_core.config import IsaacCoreConfig
from isaac_core.config.schema import CameraConfig
from isaac_core.sim.georeference import ResolvedEnuReference
from isaac_core.sim.planner import FeaturePlan, ResolvedBinding

logger = logging.getLogger(__name__)


class ConfigKeyError(Exception):
    """Raised when a binding references a config key that does not exist."""


@dataclass(frozen=True, slots=True)
class AttributeWrite:
    """
    One concrete write: apply ``value`` to ``attribute`` on ``prim``.

    Ordering matters: later writes to the same (prim, attribute) pair win.
    """

    prim: str
    attribute: str
    value: Any


@runtime_checkable
class AttributeWriter(Protocol):
    """
    Narrow interface for writing a single attribute value to a USD prim.

    A real implementation uses ``pxr.Usd.Stage``; tests use :class:`RecordingWriter`.
    """

    def write(self, prim: str, attribute: str, value: Any) -> None:  # noqa: ANN401
        """
        Set an attribute on a prim.

        Args:
            prim: Absolute USD prim path.
            attribute: Attribute name.
            value: The value to write.

        """
        ...


@dataclass(slots=True)
class RecordingWriter:
    """
    In-memory writer for testing: records every write in order.

    Use ``writes`` to inspect the full sequence after configuration is applied.
    """

    writes: list[AttributeWrite] = field(default_factory=list)

    def write(self, prim: str, attribute: str, value: Any) -> None:  # noqa: ANN401
        """Record a write."""
        self.writes.append(AttributeWrite(prim=prim, attribute=attribute, value=value))


def _lookup_config(config: IsaacCoreConfig, dotted_key: str) -> Any:  # noqa: ANN401
    """
    Traverse the config model by a dotted path.

    Supports both pydantic model attributes and dict keys for the ``vehicles``
    and ``layers`` dicts. Examples: ``"vehicles.drone_0.cameras.eo.fov_deg"``,
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
) -> Any:  # noqa: ANN401
    """
    Compute a runtime-derived value by its resolve name.

    Args:
        name: The resolve identifier from the binding.
        config: The resolved configuration.
        enu_reference: The computed ENU reference for this run.
        camera_prim: The active camera prim path, if known.

    Returns:
        The value to write.

    Raises:
        ConfigKeyError: If the resolve name is unknown.

    """
    from isaac_core.contracts.topics import BBOX, DISTANCE_SENSOR, GIMBAL, GLOBAL_POSE

    resolvers: dict[str, Any] = {
        "enu_origin": lambda: [
            enu_reference.reference.lat_deg,
            enu_reference.reference.lon_deg,
            enu_reference.reference.alt_m,
        ],
        "camera_prim": lambda: _resolve_camera_prim_value(camera_prim),
        "udp_port": lambda: config.resolved_udp_port(next(iter(config.vehicles))),
        "image_topic": lambda: _resolve_image_topic(config),
        "global_pose_topic": lambda: _resolve_vehicle_topic(config, GLOBAL_POSE),
        "distance_topic": lambda: _resolve_vehicle_topic(config, DISTANCE_SENSOR),
        "bbox_topic": lambda: _resolve_vehicle_topic(config, BBOX),
        "gimbal_topic": lambda: _resolve_vehicle_topic(config, GIMBAL),
        "camera_horizontal_aperture": lambda: _resolve_horizontal_aperture(config),
        "camera_vertical_aperture": lambda: _resolve_vertical_aperture(config),
        "lla_topic": lambda: _resolve_mavros_topic(config, "global_position/global"),
        "orientation_topic": lambda: _resolve_mavros_topic(config, "local_position/pose"),
    }

    resolver = resolvers.get(name)
    if resolver is None:
        msg = f"unknown resolve name {name!r}"
        raise ConfigKeyError(msg)
    return resolver()


def _resolve_mavros_topic(config: IsaacCoreConfig, leaf: str) -> str:
    """
    Return a vehicle's MAVROS topic, explicit if configured, else derived.

    MAVROS topic names default to ``None`` in config, meaning "derive from this
    vehicle's ``mavros_namespace``". Deriving keeps a swarm working, where each aircraft
    has its own MAVROS namespace, without anyone spelling out four topic names per
    vehicle.

    Args:
        config: The resolved configuration.
        leaf: Topic path below the MAVROS namespace, e.g. ``"local_position/pose"``.

    Returns:
        The fully resolved topic name.

    """
    vehicle_id = next(iter(config.vehicles))
    vehicle = config.vehicles[vehicle_id]
    explicit = vehicle.lla_topic if leaf.startswith("global_position") else vehicle.orientation_topic
    if explicit is not None:
        return explicit
    return f"{vehicle.mavros_namespace.rstrip('/')}/{leaf}"


def _resolve_camera_prim_value(camera_prim: str | None) -> str:
    """
    Return the camera prim or raise if not available.

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


def _first_camera(config: IsaacCoreConfig) -> CameraConfig:
    """Return the first camera of the first vehicle."""
    vehicle_id = next(iter(config.vehicles))
    camera_id = next(iter(config.vehicles[vehicle_id].cameras))
    return config.vehicles[vehicle_id].cameras[camera_id]


def _resolve_horizontal_aperture(config: IsaacCoreConfig) -> float:
    """
    Compute the camera horizontal aperture in mm from its field of view.

    A pinhole camera relates horizontal field of view, focal length and sensor width by
    ``aperture = 2 * focal_length * tan(fov / 2)``. Isaac's camera prim is driven by focal
    length and aperture, not by an fov attribute, so ``fov_deg`` only takes effect once it
    is turned into an aperture here. Before this it was read from config and applied
    nowhere, which is why changing it did nothing visible.

    Args:
        config: The resolved configuration.

    Returns:
        Horizontal aperture in millimetres.

    """
    camera = _first_camera(config)
    return 2.0 * camera.focal_length_mm * math.tan(math.radians(camera.fov_deg) / 2.0)


def _resolve_vertical_aperture(config: IsaacCoreConfig) -> float:
    """
    Compute the vertical aperture from the horizontal aperture and the aspect ratio.

    Keeping the vertical aperture consistent with the resolution's aspect ratio avoids a
    stretched image, which is the usual symptom of setting one aperture and leaving the
    other at its authored default.

    Args:
        config: The resolved configuration.

    Returns:
        Vertical aperture in millimetres.

    """
    camera = _first_camera(config)
    return _resolve_horizontal_aperture(config) * (camera.height / camera.width)


def _resolve_image_topic(config: IsaacCoreConfig) -> str:
    """
    Return the first camera's image topic, explicit if set, otherwise derived.

    Deriving keeps the namespacing convention working for a swarm, while an explicit
    ``image_topic`` in config is honoured as written -- the same contract the MAVROS
    topics use. Before this, the field was silently ignored and only the derived name
    ever reached the publisher.

    Args:
        config: The resolved configuration.

    Returns:
        The fully resolved image topic name.

    """
    from isaac_core.contracts.topics import IMAGE_RGB  # noqa: PLC0415

    vehicle_id = next(iter(config.vehicles))
    camera_id = next(iter(config.vehicles[vehicle_id].cameras))
    explicit = config.vehicles[vehicle_id].cameras[camera_id].image_topic
    if explicit is not None:
        return explicit
    return _resolve_topic(config, IMAGE_RGB, camera_scoped=True)


def _resolve_topic(config: IsaacCoreConfig, leaf: str, *, camera_scoped: bool = False) -> str:
    """
    Resolve a topic name for the first vehicle and camera.

    Args:
        config: The resolved configuration.
        leaf: Topic leaf name.
        camera_scoped: Whether to include the camera in the resolution.

    Returns:
        The fully resolved topic name.

    """
    first_vehicle_id = next(iter(config.vehicles))
    if camera_scoped:
        first_camera_id = next(iter(config.vehicles[first_vehicle_id].cameras))
        resolver = config.topic_resolver(first_vehicle_id, first_camera_id)
        return resolver.resolve(leaf)
    return config.topic_resolver(first_vehicle_id).vehicle_scoped(leaf)


def _resolve_vehicle_topic(config: IsaacCoreConfig, leaf: str) -> str:
    """
    Resolve a vehicle-scoped topic for the first vehicle.

    Args:
        config: The resolved configuration.
        leaf: Topic leaf name.

    Returns:
        The fully resolved topic name.

    """
    first_vehicle_id = next(iter(config.vehicles))
    resolver = config.topic_resolver(first_vehicle_id)
    return resolver.vehicle_scoped(leaf)


def compute_horizontal_aperture(fov_deg: float, focal_length_mm: float) -> float:
    """
    Compute horizontal aperture from field of view and focal length.

    Uses the rectilinear projection formula::

        aperture = 2 * focal_length * tan(fov / 2)

    Matching the old repo's camera configuration logic.

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
) -> list[AttributeWrite]:
    """
    Produce the ordered list of attribute writes for a simulation run.

    The order is deterministic and always:

    1. Layer bindings, in plan order (which is sorted by layer id), then in
       declaration order within each layer's manifest.
    2. ``[[prim_overrides]]`` from config, in declaration order.

    Overrides applied last guarantee the escape hatch wins over any layer binding
    that targets the same (prim, attribute) pair.

    Args:
        config: The resolved configuration.
        plan: The feature plan from the planner.
        enu_reference: The computed ENU reference for this run.
        camera_prim: Path to the active camera prim, if known.

    Returns:
        Ordered writes ready to be applied.

    Raises:
        ConfigKeyError: If a binding references a config key or resolve name that
            does not exist.

    """
    writes: list[AttributeWrite] = []

    for planned in plan.enabled:
        for binding in planned.resolved_bindings:
            value = _resolve_binding_value(binding, config=config, enu_reference=enu_reference, camera_prim=camera_prim)
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
) -> Any:  # noqa: ANN401
    """
    Resolve the value for a single binding.

    Args:
        binding: The resolved binding from the plan.
        config: The resolved configuration.
        enu_reference: The computed ENU reference for this run.
        camera_prim: Active camera prim path.

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
        )
    msg = f"binding for {binding.prim}:{binding.attribute} has neither config nor resolve"
    raise ConfigKeyError(msg)


def apply_writes(writer: AttributeWriter, writes: Sequence[AttributeWrite]) -> None:
    """
    Execute a sequence of attribute writes through the given writer.

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
