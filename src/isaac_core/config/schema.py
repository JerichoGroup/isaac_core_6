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

import logging
from pathlib import Path
from typing import Annotated, Any, Final, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

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

# Each pose source implies the camera layer that reads it, so a vehicle declaring
# `pose_source = "udp"` does not also have to be listed in `[features] enabled`.
# Requiring both is a duplicate source of truth that can disagree with itself -- the
# same problem decision D18 removed for the ENU reference.
_POSE_SOURCE_LAYERS: Final[dict[str, str]] = {
    "udp": "camera_udp",
    "ros": "camera_ros",
}
_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})

# Emits the "MinimalRendering draws no terrain" warning. Named so a test can attach a
# handler to it directly -- pytest's ``caplog`` does not capture this project's warnings.
logger = logging.getLogger(__name__)

# RTX render modes SimulationApp accepts, mapping each lower-cased spelling to its
# canonical form. Isaac lower-cases the value before matching (see
# ``isaacsim.simulation_app.SimulationApp._set_render_settings``), so the accepted set is
# case-insensitive and "Minimal" is an alias for "MinimalRendering". Any value outside this
# map is passed straight through to Kit's ``/rtx/rendermode`` carb setting, where a typo
# fails late or silently rather than at config load -- exactly the foot-gun this validator
# closes.
_RENDERER_CANONICAL: Final[dict[str, str]] = {
    "raytracedlighting": "RaytracedLighting",
    "pathtracing": "PathTracing",
    "realtimepathtracing": "RealTimePathTracing",
    "minimalrendering": "MinimalRendering",
    "minimal": "MinimalRendering",
}

# Renderers that produce a healthy but empty scene over Cesium 3D Tiles terrain: the RTX
# passes those tilesets depend on do not run, so nothing is drawn and no error is raised.
# Selecting one is legitimate (profiling without terrain), so this only warns.
_RENDERERS_WITHOUT_TERRAIN: Final[frozenset[str]] = frozenset({"MinimalRendering"})

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

    enabled: bool = True
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


class SimConfig(_Strict):
    """Simulator lifecycle and window settings."""

    scene: str = "earth"
    headless: bool = False
    strict_features: bool = False
    # Kit extensions to enable before opening the stage. Configurable rather than
    # hardcoded so a stage needing an extra extension requires no code change.
    #
    # A missing extension degrades rather than crashes: OmniGraph logs
    # "Could not find node type interface" and those nodes do nothing, so dropping
    # one from this list is a safe way to bisect a problem.
    extensions: tuple[str, ...] = (
        "omni.graph.action",
        "omni.graph.nodes",
        "isaacsim.core.nodes",
        "isaacsim.ros2.bridge",
        # Native RTSP (D21). Always on: the stream is there for whoever wants it, and
        # costs nothing to ignore. Without this the RTSPCameraHelper node in the camera
        # layers logs "Could not find node type interface" and silently does nothing.
        "isaacsim.streaming.rtsp",
        "isaac_core_ogn.math",
        "isaac_core_ogn.position",
        "isaac_core_ogn.sensors",
    )
    # Kit experience (app config) to launch.
    #
    # Isaac's SimulationApp defaults to `isaacsim.exp.base.python.kit`, a deliberately
    # minimal app. That app omits the extensions that contribute property widgets and, more
    # importantly, behaves differently from the editor the team actually authors USD in --
    # which makes "it works in the GUI but not from the CLI" hard to reason about. Using the
    # full experience keeps the two consistent.
    #
    # A bare filename is resolved against Isaac's apps directory ($EXP_PATH). An absolute
    # path is used as given. Set to "" to accept SimulationApp's own default.
    experience: str = "isaacsim.exp.full.kit"

    # Extensions enabled during Kit startup, via `--enable`, rather than afterwards.
    #
    # This distinction is not cosmetic. An extension that contributes **USD schemas** must be
    # present before the schema registry initialises; enabling it later reports success and
    # silently does nothing useful. Measured on this install: enabling `cesium.omniverse`
    # after startup left `CesiumTilesetPrim` unregistered, so the prim resolved as untyped
    # with 5 raw attributes, `IsA(Xformable)` false, and no terrain drawn. Enabling it at
    # boot gave a registered schema, 28 attributes and `IsA(Xformable)` true.
    #
    # The visible symptom is a prim that has lost most of its properties in the GUI -- and no
    # error anywhere. Put schema-providing extensions here, everything else in `extensions`.
    boot_extensions: tuple[str, ...] = (
        "cesium.usd.plugins",
        "cesium.omniverse",
    )

    # Extra directories Kit should search for extensions.
    #
    # Isaac's headless python experience does not search Omniverse's user extension
    # registry, where extensions installed through the GUI's extension manager live. Cesium
    # for Omniverse is installed there, so without this the tileset prims in a scene load
    # as inert typed prims and no terrain ever appears -- with no error, because the prims
    # themselves are perfectly valid USD.
    #
    # Each existing path is passed to Kit as `--ext-folder`. Missing paths are skipped
    # rather than failing, so this default is portable across machines.
    extension_search_paths: tuple[str, ...] = ("~/.local/share/ov/data/exts/v2",)

    # Prim path the main viewport should look through when running with a GUI.
    #
    # Without this the viewport keeps Kit's default perspective camera, so the aircraft
    # camera can be tracking a pose perfectly while the window appears frozen -- which
    # reads as "the camera does not respond". `{instance}` is replaced with the vehicle id.
    #
    # Set to an empty string to leave the viewport alone.
    viewport_camera: str = "/World/Environment/{instance}/Xform/main_camera_01"

    # RTX render mode passed to SimulationApp.
    #
    # Isaac defaults to "RealTimePathTracing", which this project does not need: the camera
    # topic wants a correct image, not a photoreal one. Path tracing over streaming 3D
    # Tiles was also implicated in an intermittent startup segfault inside OmniGraph's
    # render-stage execution. "RaytracedLighting" is materially lighter and stable.
    #
    # Also accepts "PathTracing", "RealTimePathTracing" and "MinimalRendering" (alias
    # "Minimal"). Matching is case-insensitive, mirroring Isaac; an unknown value is
    # rejected at config load rather than passed through. "MinimalRendering" validates but
    # warns: it draws no Cesium 3D Tiles terrain.
    renderer: str = "RaytracedLighting"
    physics_dt: float = Field(1.0 / 60.0, gt=0.0)
    stage_units_in_meters: float = Field(1.0, gt=0.0)
    control_plane: ControlPlaneConfig = ControlPlaneConfig()

    @field_validator("renderer")
    @classmethod
    def _check_renderer(cls, value: str) -> str:
        """
        Reject an unknown renderer and warn when the choice draws no terrain.

        Validation happens here, at config load, because Isaac silently passes an
        unrecognised value through to a raw carb setting where it fails late or not at
        all. "MinimalRendering" is accepted but warned about: the scene stays healthy
        yet empty over Cesium terrain, a symptom indistinguishable from a broken tileset.

        Args:
            value: The renderer name as written in config.

        Returns:
            The value unchanged, so an explicit setting reaches Isaac verbatim.

        Raises:
            ValueError: If the value is not one Isaac accepts.

        """
        canonical = _RENDERER_CANONICAL.get(value.lower())
        if canonical is None:
            options = ", ".join(sorted(set(_RENDERER_CANONICAL.values())))
            raise ValueError(f"unknown renderer {value!r}; valid options are: {options}")
        if canonical in _RENDERERS_WITHOUT_TERRAIN:
            logger.warning(
                "renderer %r draws no Cesium 3D Tiles terrain: the scene will render but "
                "the terrain will be absent, which looks like a broken tileset. Use "
                "'RaytracedLighting' to see terrain; keep %s only for profiling without it.",
                value,
                canonical,
            )
        return value


class AssetsConfig(_Strict):
    """Where scenes, feature layers and optional heavy binaries are found."""

    search_paths: tuple[Path, ...] = ()
    layer_search_paths: tuple[Path, ...] = ()
    hdri: str | None = None


class GeoConfig(_Strict):
    """Georeferencing settings."""

    enu_reference: EnuReference = EnuReference()


class CesiumConfig(_Strict):
    """
    3D Tiles terrain settings.

    The tile-loading fields default to ``None``, meaning "leave Cesium's own default alone".
    They exist because tuning them previously required a ``prim_override``, not because the
    shipped defaults are wrong -- measurement showed the frame-rate hitch during flight is
    **cold-cache streaming**, not load concurrency, so changing concurrency does not fix it.
    """

    tileset_server_url: str | None = None
    tilesets_root: str = prims.TILESETS_ROOT
    # Deleting the cache forces every launch to stream terrain from scratch, which is exactly the
    # condition that produces the worst hitching. Off by default for that reason; turn it on only
    # to reclaim disk.
    delete_cache_on_launch: bool = False

    # Concurrent tile requests. Cesium's default is 20.
    max_simultaneous_tile_loads: int | None = Field(None, gt=0)
    # Pixel error allowed before a finer tile is fetched. Cesium's default is 16; larger values
    # mean coarser terrain and fewer tile loads.
    max_screen_space_error: float | None = Field(None, gt=0.0)
    # Tile cache ceiling in bytes. Cesium's default is 512 MiB. Raising it keeps terrain resident
    # for longer, which is the one knob here that genuinely reduces how often streaming goes cold.
    max_cached_bytes: int | None = Field(None, gt=0)
    # Prefetch neighbouring and parent tiles ahead of need.
    preload_ancestors: bool | None = None
    preload_siblings: bool | None = None


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


class DistanceSensorConfig(_Strict):
    """
    Rangefinder settings for a vehicle's distance sensor.

    The rated band is a sensor property, not a preference: readings outside it are reported
    as the `sensor_msgs/Range` out-of-band values rather than clamped, so a consumer can tell
    "nothing detected" from "something at exactly max range".
    """

    min_range_m: float = Field(0.2, ge=0.0)
    # 5 km, not the 100 m a ground rangefinder would use: this sensor points down from an
    # aircraft typically 500-2000 m above terrain, so a 100 m ray never reaches the ground and
    # the sensor reports "no detection" forever while looking perfectly healthy.
    max_range_m: float = Field(5000.0, gt=0.0)
    topic: str | None = None

    @model_validator(mode="after")
    def _require_a_positive_band(self) -> "DistanceSensorConfig":
        """Reject a band where every reading would be meaningless."""
        if self.max_range_m <= self.min_range_m:
            message = (
                f"distance_sensor.max_range_m ({self.max_range_m}) must exceed " f"min_range_m ({self.min_range_m})"
            )
            raise ValueError(message)
        return self


class CameraConfig(_Strict):
    """
    One camera on a vehicle.

    ``focal_length_mm`` together with ``fov_deg`` determines the horizontal
    aperture applied to the USD camera prim; the vertical aperture follows from
    the resolution aspect ratio. Both apertures can instead be set directly with
    ``horizontal_aperture_mm`` / ``vertical_aperture_mm``, in which case
    ``fov_deg`` is ignored for that axis.
    """

    resolution: tuple[Annotated[int, Field(gt=0)], Annotated[int, Field(gt=0)]] = (
        1280,
        720,
    )
    fov_deg: float = Field(78.1, gt=0.0, le=_MAX_FOV_DEG)
    focal_length_mm: float = Field(22.7885, gt=0.0)
    image_topic: str | None = None

    # Distance in scene units the lens is focused at. Only visible when depth of field is
    # on, i.e. when ``f_stop`` is non-zero.
    focus_distance: float = Field(400.0, gt=0.0)

    # Lens f-number. USD treats an ``fStop`` of 0 as "depth of field OFF", producing the
    # everything-in-focus pinhole image this project has always rendered. The default keeps
    # that behaviour; set a positive value (and ``focus_distance``) for a photographic
    # blur, and be aware that leaving it at 0 means the effect is disabled, not merely wide.
    f_stop: float = Field(0.0, ge=0.0)

    # Direct sensor-aperture override, in millimetres. ``None`` means derive the aperture
    # from ``fov_deg`` and ``focal_length_mm`` (the historical behaviour). Setting either
    # value wins for that axis and makes ``fov_deg`` irrelevant to it; setting both an
    # explicit aperture and a non-default ``fov_deg`` is flagged by the configurator.
    horizontal_aperture_mm: float | None = Field(None, gt=0.0)
    vertical_aperture_mm: float | None = Field(None, gt=0.0)

    # RTSP stream settings for this camera (D21: always streaming, no enable flag).
    #
    # `rtsp_mount_path` of ``None`` means derive: ``/stream`` for a single camera, and a
    # namespaced path once there is more than one, mirroring how topics are derived.
    rtsp_port: Port = 8554
    rtsp_mount_path: str | None = None

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
    distance_sensor: DistanceSensorConfig = DistanceSensorConfig()
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

    # DDS domain the ROS 2 bridge publishes on.
    #
    # ``None`` means inherit ``$ROS_DOMAIN_ID`` from the environment (0 if unset), which is
    # what the bridge's context node does by default and what the team expects. An explicit
    # integer is exported to ``ROS_DOMAIN_ID`` before the bridge starts, so config can
    # override the environment when needed. A hardcoded default here was misleading: it
    # looked authoritative but never reached the bridge.
    domain_id: int | None = Field(None, ge=0, le=232)


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

    # Whether to let Kit's own log stream through to the terminal.
    #
    # False quiets Kit to warnings and errors only. Isaac otherwise prints thousands of
    # startup lines that bury our own output -- a full launch emitted over 3,400 lines.
    # Our own logger is unaffected either way; this only governs Kit's stream.
    isaac_logs: bool = False

    # Python loggers raised to WARNING when `isaac_logs` is false.
    #
    # These are named explicitly because they set their own level and attach their own
    # handler, so raising the root logger does not touch them. `ogn_registration` alone
    # accounted for 2,721 lines of a 3,400-line launch.
    quiet_loggers: tuple[str, ...] = (
        "ogn_registration",
        "AutoNode",
        "omni",
        "omni.kit",
        "usd_validation_nvidia",
        "matplotlib",
        "asyncio",
        "isaacsim",
        "cesium",
    )


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

    def resolved_rtsp_port(self, vehicle_id: str, camera_id: str) -> int:
        """
        Return a camera's RTSP port, offset per vehicle so a swarm cannot collide.

        Every vehicle mounts its own copy of the camera layer, so with a shared port the second
        aircraft's RTSP server fails to bind: Isaac reports
        ``Error binding to address 0.0.0.0:8554: Address already in use`` and that vehicle simply
        has no stream. Offsetting by the vehicle's index mirrors how pose ports work.

        An explicitly configured port is honoured verbatim, so a single-vehicle setup keeps 8554
        and a user pinning a port gets exactly that.

        Args:
            vehicle_id: Key in ``vehicles``.
            camera_id: Key in that vehicle's ``cameras``.

        Returns:
            The port this camera's RTSP server should bind.

        """
        camera = self.vehicles[vehicle_id].cameras[camera_id]
        if "rtsp_port" in camera.model_fields_set:
            return camera.rtsp_port
        return camera.rtsp_port + self.vehicle_index(vehicle_id)

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

    def required_feature_ids(self) -> tuple[str, ...]:
        """
        Return every feature layer this configuration needs, in a stable order.

        The union of what ``[features] enabled`` lists explicitly and the camera layer
        implied by each vehicle's ``pose_source``. Deriving the implied layers means a
        minimal config -- one vehicle on UDP -- composes and flies with nothing listed
        under ``[features]`` at all, and it is impossible to select a pose source
        without the layer that reads it.

        Pose sources with no corresponding shipped layer (``script``, ``replay``,
        ``mavlink``) contribute nothing; those need an explicit entry.

        Returns:
            Feature layer ids, explicit ones first, then derived, without duplicates.

        """
        ordered: list[str] = list(self.features.enabled)
        for vehicle in self.vehicles.values():
            # .value, not str(): PoseSource subclasses str but is not a StrEnum, so
            # str() yields "PoseSource.UDP" on Python 3.10.
            implied = _POSE_SOURCE_LAYERS.get(vehicle.pose_source.value)
            if implied is not None and implied not in ordered:
                ordered.append(implied)
        return tuple(ordered)

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
    "DistanceSensorConfig",
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
]
