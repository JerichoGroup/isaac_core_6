"""
Typed configuration schema.

Mirrors ``config/default.toml`` exactly. Validation happens in-process the moment
a config is loaded -- there is no separate command to run -- so a bad value is
reported with the field, the offending value and the reason, before Isaac Sim is
ever launched.

Two conventions worth knowing when reading this module:

Angles carry their unit in the name
    ``fov_deg``, ``start_roll_deg``, ``max_rate_deg_s``. Config is the boundary
    where degrees are acceptable; everything internal works in radians.

Optional means "derive it"
    Fields such as a camera's ``image_topic`` or a vehicle's ``udp_port`` default
    to ``None``, meaning *derive the conventional value*. Derivation needs to know
    how many vehicles and cameras exist, so it lives on
    :class:`IsaacCoreConfig`, which is the only object with that whole-picture
    view. Setting a value explicitly always wins.
"""

from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from isaac_core.contracts import prims, topics
from isaac_core.contracts.frames import PoseSource, RotationFrame
from isaac_core.contracts.ports import (
    DEFAULT_CONTROL_PLANE_PORT,
    DEFAULT_POSE_UDP_PORT,
    MAX_PORT,
    MIN_PORT,
    pose_port_for_index,
)

_MAX_FOV_DEG = 180.0
_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})

Port = Annotated[int, Field(ge=MIN_PORT, le=MAX_PORT)]
LogLevel = Literal["debug", "info", "warning", "error"]


class _Strict(BaseModel):
    """
    Base for every config model.

    ``extra="forbid"`` turns a typo into an immediate, located error rather than a
    silently ignored setting -- the failure mode that makes configuration files
    untrustworthy.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)


class EnuReference(_Strict):
    """
    Anchor for the local ENU tangent plane the simulation works in.

    Must agree with the scene's Cesium georeference, or the terrain and the
    aircraft will disagree about where they are.
    """

    lat_deg: float = Field(32.22481, ge=-90.0, le=90.0)
    lon_deg: float = Field(35.25621, ge=-180.0, le=180.0)
    alt_m: float = 516.7


class ControlPlaneConfig(_Strict):
    """
    JSON-RPC command channel settings.

    Defaults to loopback deliberately. This channel mutates a running simulation
    and writes files, so exposing it on a routable interface requires an explicit
    token.
    """

    host: str = "127.0.0.1"
    port: Port = DEFAULT_CONTROL_PLANE_PORT
    token: str = ""
    output_root: Path = Path("~/isaac_core_out")

    @model_validator(mode="after")
    def _require_token_when_not_loopback(self) -> "ControlPlaneConfig":
        """Refuse to expose an unauthenticated control channel off-host."""
        if self.host not in _LOOPBACK_HOSTS and not self.token:
            msg = (
                f"sim.control_plane.host is {self.host!r}, which is not loopback, so "
                f"sim.control_plane.token must be set. This channel can patch the "
                f"running simulation and write files; leaving it open is unsafe."
            )
            raise ValueError(msg)
        return self


class ViewportConfig(_Strict):
    """Which camera the GUI window looks through."""

    primary_camera: str | None = None


class SimConfig(_Strict):
    """Simulator lifecycle and window settings."""

    scene: str = "earth"
    headless: bool = False
    strict_features: bool = False
    physics_dt: float = Field(1.0 / 60.0, gt=0.0)
    stage_units_in_meters: float = Field(1.0, gt=0.0)
    control_plane: ControlPlaneConfig = ControlPlaneConfig()
    viewport: ViewportConfig = ViewportConfig()


class AssetsConfig(_Strict):
    """Where scenes, feature layers and optional heavy binaries are found."""

    search_paths: tuple[Path, ...] = ()
    layer_search_paths: tuple[Path, ...] = ()
    hdri: str | None = None


class GeoConfig(_Strict):
    """Georeferencing settings."""

    enu_reference: EnuReference = EnuReference()


class CesiumConfig(_Strict):
    """3D Tiles terrain settings."""

    tileset_server_url: str | None = None
    tilesets_root: str = prims.TILESETS_ROOT
    delete_cache_on_launch: bool = False


class FeaturesConfig(_Strict):
    """Which feature layers to compose onto the scene."""

    enabled: tuple[str, ...] = ()


class GimbalConfig(_Strict):
    """
    Camera gimbal starting attitude and slew behaviour.

    Defaults to ``RotationFrame.BODY`` because a gimbal is physically mounted on
    the airframe and moves with it. ``max_rate_deg_s`` of ``None`` reproduces the
    previous generation's instantaneous snapping.
    """

    start_roll_deg: float = 0.0
    start_pitch_deg: float = 0.0
    start_yaw_deg: float = 0.0
    max_rate_deg_s: float | None = Field(None, gt=0.0)
    rotation_frame: RotationFrame = RotationFrame.BODY
    topic: str | None = None


class CameraConfig(_Strict):
    """
    One camera on a vehicle.

    ``focal_length_mm`` together with ``fov_deg`` determines the horizontal
    aperture applied to the USD camera prim; the vertical aperture follows from
    the resolution aspect ratio.
    """

    resolution: tuple[Annotated[int, Field(gt=0)], Annotated[int, Field(gt=0)]] = (
        1280,
        720,
    )
    fov_deg: float = Field(78.1, gt=0.0, le=_MAX_FOV_DEG)
    focal_length_mm: float = Field(22.7885, gt=0.0)
    publish_rate_hz: float = Field(30.0, gt=0.0)
    image_topic: str | None = None
    raw_topic: str | None = None

    @property
    def width(self) -> int:
        """Horizontal resolution in pixels."""
        return self.resolution[0]

    @property
    def height(self) -> int:
        """Vertical resolution in pixels."""
        return self.resolution[1]


class VehicleConfig(_Strict):
    """
    One vehicle, with one or more cameras.

    Defaults to ``RotationFrame.WORLD`` for movement commands, matching the
    previous generation's ``UdpBot`` turn methods, which rotated about fixed world
    axes. Note this differs from the gimbal default of ``BODY`` -- both behaviours
    existed before, neither was selectable, and the physical defaults differ.
    """

    pose_source: PoseSource = PoseSource.UDP
    udp_port: Port | None = None
    mount: str | None = None
    mavros_namespace: str = "/mavros"
    lla_topic: str | None = None
    orientation_topic: str | None = None
    rotation_frame: RotationFrame = RotationFrame.WORLD
    gimbal: GimbalConfig = GimbalConfig()
    cameras: dict[str, CameraConfig] = Field(default_factory=lambda: {"eo": CameraConfig()})

    @model_validator(mode="after")
    def _require_at_least_one_camera(self) -> "VehicleConfig":
        """Require a camera, since a vehicle without one cannot produce imagery."""
        if not self.cameras:
            msg = "a vehicle must define at least one camera"
            raise ValueError(msg)
        return self


class Ros2Config(_Strict):
    """ROS 2 middleware settings."""

    domain_id: int = Field(13, ge=0, le=232)
    use_sim_time: bool = True


class SidecarServiceConfig(BaseModel):
    """
    One supervised sidecar service.

    Extra keys are permitted, unlike everywhere else, because each service kind
    defines its own parameters and those are validated by the service itself.
    """

    model_config = ConfigDict(extra="allow", frozen=True)

    kind: str


class SidecarConfig(_Strict):
    """Out-of-interpreter services, run as peers of the simulator."""

    enabled: bool = False
    services: dict[str, SidecarServiceConfig] = Field(default_factory=dict)


class LoggingConfig(_Strict):
    """Logging verbosity."""

    level: LogLevel = "info"
    isaac_logs: bool = False


class PrimOverride(_Strict):
    """
    A raw prim attribute write, applied after all layer bindings.

    The escape hatch for a knob no layer exposed. Sharp, occasionally correct.
    """

    prim: str
    attribute: str
    value: Any

    @model_validator(mode="after")
    def _validate_prim_path(self) -> "PrimOverride":
        """Reject a malformed prim path at load time rather than at compose time."""
        prims.validate_prim_path(self.prim)
        return self


class IsaacCoreConfig(_Strict):
    """
    The complete configuration for one simulation run.

    Also the only object that can resolve the conventional defaults, because
    those depend on how many vehicles and cameras exist. See
    :meth:`topic_resolver`, :meth:`resolved_udp_port` and :meth:`resolved_mount`.
    """

    sim: SimConfig = SimConfig()
    assets: AssetsConfig = AssetsConfig()
    geo: GeoConfig = GeoConfig()
    cesium: CesiumConfig = CesiumConfig()
    features: FeaturesConfig = FeaturesConfig()
    vehicles: dict[str, VehicleConfig] = Field(default_factory=lambda: {"drone_0": VehicleConfig()})
    layers: dict[str, dict[str, Any]] = Field(default_factory=dict)
    ros2: Ros2Config = Ros2Config()
    sidecar: SidecarConfig = SidecarConfig()
    logging: LoggingConfig = LoggingConfig()
    prim_overrides: tuple[PrimOverride, ...] = ()

    @model_validator(mode="after")
    def _validate_cross_references(self) -> "IsaacCoreConfig":
        """Check that names referring to other parts of the config actually exist."""
        if not self.vehicles:
            msg = "at least one vehicle must be configured"
            raise ValueError(msg)

        for vehicle_id in self.vehicles:
            topics.validate_segment(vehicle_id)
            for camera_id in self.vehicles[vehicle_id].cameras:
                topics.validate_segment(camera_id)

        primary = self.sim.viewport.primary_camera
        if primary is not None and primary not in self.camera_keys():
            msg = (
                f"sim.viewport.primary_camera is {primary!r}, which is not a "
                f"configured camera. Available: {sorted(self.camera_keys())}"
            )
            raise ValueError(msg)

        for feature_id in self.features.enabled:
            topics.validate_segment(feature_id)

        return self

    # -- derived values -------------------------------------------------- #

    @property
    def is_single_vehicle(self) -> bool:
        """Whether topic namespacing should collapse the vehicle level."""
        return len(self.vehicles) == 1

    def vehicle_index(self, vehicle_id: str) -> int:
        """
        Return the declaration order index of a vehicle.

        Args:
            vehicle_id: Key in ``vehicles``.

        Returns:
            Zero-based index, used for default port allocation.

        Raises:
            KeyError: If the vehicle is not configured.

        """
        try:
            return list(self.vehicles).index(vehicle_id)
        except ValueError as error:
            msg = f"unknown vehicle {vehicle_id!r}"
            raise KeyError(msg) from error

    def resolved_udp_port(self, vehicle_id: str) -> int:
        """
        Return a vehicle's pose port, explicit if set, otherwise ``base + index``.

        Args:
            vehicle_id: Key in ``vehicles``.

        Returns:
            The UDP port this vehicle's pose packets arrive on.

        """
        configured = self.vehicles[vehicle_id].udp_port
        if configured is not None:
            return configured
        return pose_port_for_index(self.vehicle_index(vehicle_id), DEFAULT_POSE_UDP_PORT)

    def resolved_mount(self, vehicle_id: str) -> str:
        """
        Return a vehicle's stage mount point, defaulting to ``/Environment/<id>``.

        Args:
            vehicle_id: Key in ``vehicles``.

        Returns:
            A validated absolute prim path.

        """
        configured = self.vehicles[vehicle_id].mount
        if configured is not None:
            return prims.validate_prim_path(configured)
        return prims.child(prims.ENVIRONMENT_ROOT, vehicle_id)

    def topic_resolver(self, vehicle_id: str, camera_id: str | None = None) -> topics.TopicResolver:
        """
        Return a resolver producing this vehicle's (and camera's) topic names.

        Namespace levels collapse when there is only one of something, so a single
        vehicle with a single camera yields the flat names the team already uses,
        and adding a second of either namespaces automatically.

        Args:
            vehicle_id: Key in ``vehicles``.
            camera_id: Key in that vehicle's ``cameras``, or ``None`` for
                vehicle-scoped data such as pose or range.

        Returns:
            A configured :class:`~isaac_core.contracts.topics.TopicResolver`.

        """
        vehicle_segment = None if self.is_single_vehicle else vehicle_id

        camera_segment = None
        if camera_id is not None and len(self.vehicles[vehicle_id].cameras) > 1:
            camera_segment = camera_id

        return topics.TopicResolver(vehicle=vehicle_segment, camera=camera_segment)

    def camera_keys(self) -> tuple[str, ...]:
        """Return every camera as ``"<vehicle_id>.<camera_id>"``, in declaration order."""
        return tuple(
            f"{vehicle_id}.{camera_id}"
            for vehicle_id, vehicle in self.vehicles.items()
            for camera_id in vehicle.cameras
        )


__all__ = [
    "AssetsConfig",
    "CameraConfig",
    "CesiumConfig",
    "ControlPlaneConfig",
    "EnuReference",
    "FeaturesConfig",
    "GeoConfig",
    "GimbalConfig",
    "IsaacCoreConfig",
    "LoggingConfig",
    "PrimOverride",
    "Ros2Config",
    "SidecarConfig",
    "SidecarServiceConfig",
    "SimConfig",
    "VehicleConfig",
    "ViewportConfig",
]
