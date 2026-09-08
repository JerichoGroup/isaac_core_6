"""
Simulation runtime: app lifecycle, step loop and control-plane handlers.

This is the thinnest possible shell over Isaac Sim's application lifecycle. It
owns ONLY:

- Creating and shutting down ``SimulationApp``.
- The physics step loop.
- Starting the control server and registering handlers.

It must NOT mention any feature by name -- that is defect #13 from the old repo.
Features are composed via the layer system; the runtime iterates the plan, never
branches on a feature id.

All ``omni`` access is done through :func:`importlib.import_module` so that
importing this module does NOT pull in Isaac dependencies at the top level.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass, field
import importlib
import logging
import math
import os
from pathlib import Path
import queue
import socket
import struct
import sys
import threading
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

from isaac_core.config import IsaacCoreConfig
from isaac_core.contracts import packet as packet_spec
from isaac_core.contracts.gimbal import GimbalAngles, slew_towards
from isaac_core.contracts.prims import render
from isaac_core.control import ControlServer, InvalidParamsError
from isaac_core.sim.planner import FeaturePlan

logger = logging.getLogger(__name__)

# Frames pumped after changing the capture resolution, so the renderer converges at the new size.
CAPTURE_SETTLE_FRAMES = 8

# Frames to wait for an asynchronous capture to write its file before giving up on confirming it.
CAPTURE_WAIT_FRAMES = 120

# How long a caller waits for a capture before giving up. Generous because the request is served
# across several frames, but bounded so a stalled renderer cannot hang a script forever.
CAPTURE_TIMEOUT_S: float = 20.0

# Config paths that something re-reads while the simulator runs, and so can be patched safely.
# Anything applied once at composition time is deliberately absent: patching it would change the
# config object while the stage kept the old value.
PATCHABLE_CONFIG_KEYS: frozenset[str] = frozenset({"gimbal.max_rate_deg_s"})

# How long a control-plane handler waits for the step loop to run its task.
#
# Generous because a single frame can take a while on a cold stage, but bounded so a
# wedged step loop surfaces as a clear error rather than a hung client.
MAIN_THREAD_TASK_TIMEOUT_S: float = 10.0

# Frames to pump after enabling extensions, before touching the stage.
#
# This is the fix for an intermittent startup segfault that killed roughly half of all
# launches. Measured on this install, opening or creating a stage immediately after
# enabling `isaacsim.ros2.bridge` crashed 4 of 6 runs inside Kit's parallel graph executor
# (`omni.graph.core` -> `omni.graph.image.core` -> `omni.kit.exec.core` -> TBB). Pumping
# frames first before any stage operation crashed 0 of 6. The bridge evidently needs a few
# update cycles to finish registering its render-stage hooks, and a stage arriving mid-way
# through that races it.
#
# Cheap insurance: 60 frames is well under a second of wall clock at startup.
EXTENSION_WARMUP_FRAMES: int = 60

# Frames to pump after the base scene opens and again after feature layers mount.
#
# Same reasoning at smaller scale: layers can carry graphs that OmniGraph evaluates in the
# render pipeline, and mounting one before the renderer has produced a frame is the same
# class of race.
STAGE_SETTLE_FRAMES: int = 30

# File descriptor of the stream the pre-Kit banners are written to.
#
# The launcher banners come out of plain Python ``print()`` calls in Isaac's
# ``SimulationApp.__init__`` ("Starting kit application with the following args: ...") and out
# of Kit's own C++ startup writing to the process's stdout. Neither obeys a log level or the
# ``--/app/enableStdoutOutput=false`` setting, because both happen before or beneath our
# logging configuration. Redirecting this descriptor -- 1, POSIX stdout -- around the
# construction of ``SimulationApp`` is the only lever that catches them. Only stdout is
# touched: our own logging and every Python traceback go to stderr (fd 2), so a crash during
# construction still surfaces in full.
STDOUT_FILENO: int = 1


@dataclass
class _MainThreadTask:
    """
    A callable queued for execution on the thread that owns the step loop.

    USD, Fabric and the Kit application are not thread-safe. Reading a prim or calling
    ``update_app()`` directly from the control server's thread races the step loop: it
    appeared to work when the stage was idle, then timed out as soon as pose packets were
    arriving and OmniGraph was writing transforms. Handlers therefore hand work here and
    wait for the loop to run it.
    """

    call: "Callable[[], Any]"
    done: threading.Event = field(default_factory=threading.Event)
    result: Any = None
    error: BaseException | None = None


# Kit extensions the composed stages need. The base isaacsim.exp.base.python app
# enables none of these, and without them OmniGraph reports "Could not find node type
# interface" for every node and the graphs quietly do nothing.
#
# Ours provide the pose maths and UDP ingestion; isaacsim.ros2.bridge provides every
# ROS 2 publisher/subscriber (it is C++, which is why it works at all -- see
# docs/ros2_and_python.md).
REQUIRED_EXTENSIONS: tuple[str, ...] = (
    "omni.graph.action",
    "omni.graph.nodes",
    "isaacsim.ros2.bridge",
    "isaacsim.core.nodes",
    "isaac_core_ogn.math",
    "isaac_core_ogn.position",
)


def _json_safe_usd(value: Any) -> Any:  # noqa: ANN401
    """
    Convert any read USD attribute value into a JSON-serialisable form.

    Scalars (bool/int/float/str) pass through; Gf vectors and quaternions become lists;
    anything else is stringified. ``None`` stays ``None``.

    Args:
        value: A value read from a USD attribute.

    Returns:
        A JSON-serialisable representation.

    """
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "GetReal") or (hasattr(value, "__len__")):
        return _usd_value_to_list(value)
    return str(value)


def _usd_value_to_list(value: Any) -> list[float] | None:  # noqa: ANN401
    """
    Convert a USD attribute value into a JSON-serialisable list of floats.

    ``Gf.Vec3d`` is iterable but ``Gf.Quatd`` is not -- it exposes ``GetReal()`` and
    ``GetImaginary()`` instead -- so a single ``[float(v) for v in ...]`` raises
    ``TypeError: 'Quatd' object is not iterable``. Quaternions are returned in
    ``(w, x, y, z)`` order, matching how USD writes them in .usda text.

    Args:
        value: A value read from a USD attribute.

    Returns:
        The components as floats, or ``None`` if the attribute was unset.

    """
    if value is None:
        return None
    real = getattr(value, "GetReal", None)
    imaginary = getattr(value, "GetImaginary", None)
    if callable(real) and callable(imaginary):
        return [float(real()), *(float(component) for component in imaginary())]
    try:
        return [float(component) for component in value]
    except TypeError:
        return None


@dataclass
class _CaptureRequest:
    """An in-flight frame capture, advanced one step per simulation frame."""

    target: Path
    width: int | None
    height: int | None
    stage: str = "start"
    frames: int = 0
    original: tuple[int, int] = (0, 0)
    used: tuple[int, int] = (0, 0)
    render_product: str | None = None
    result: dict[str, Any] = field(default_factory=dict)
    error: str | None = None
    done: threading.Event = field(default_factory=threading.Event)


@dataclass(frozen=True, slots=True)
class _GimbalTarget:
    """A commanded gimbal attitude in degrees."""

    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def _as_mapping(params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
    """
    Return JSON-RPC params as a mapping.

    Args:
        params: The raw params.

    Returns:
        A mapping, empty when none were supplied.

    Raises:
        InvalidParamsError: If positional params were supplied, which these methods do not take.

    """
    if params is None:
        return {}
    if isinstance(params, list):
        raise InvalidParamsError("this method takes named params, not positional")
    return params


def _optional_positive_int(values: dict[str, Any], name: str) -> int | None:
    """
    Read an optional positive integer from params.

    Args:
        values: The params mapping.
        name: Parameter name.

    Returns:
        The value, or ``None`` when absent.

    Raises:
        InvalidParamsError: If present but not a positive integer.

    """
    if name not in values or values[name] is None:
        return None
    try:
        parsed = int(values[name])
    except (TypeError, ValueError) as exc:
        raise InvalidParamsError(f"{name} must be an integer") from exc
    if parsed <= 0:
        raise InvalidParamsError(f"{name} must be greater than zero")
    return parsed


def _optional_float(values: dict[str, Any], name: str, fallback: float) -> float:
    """
    Read a named float from params, falling back when absent.

    Args:
        values: The params mapping.
        name: Parameter name.
        fallback: Value to use when the parameter is absent.

    Returns:
        The parsed float.

    Raises:
        InvalidParamsError: If present but not a number.

    """
    if name not in values or values[name] is None:
        return fallback
    try:
        return float(values[name])
    except (TypeError, ValueError) as exc:
        raise InvalidParamsError(f"{name} must be a number") from exc


def _claim_capture_output(target: Path) -> bool:
    """
    Move Isaac's suffixed capture file to the path the caller asked for.

    Kit names captures after the render variable, so a request for ``shot.png`` lands as
    ``shot_LdrColorSD.png``. Returning that name to the caller would break the obvious next step
    of opening the path they passed, so the file is renamed into place instead.

    Args:
        target: The path the caller requested.

    Returns:
        ``True`` once the file exists at ``target``.

    """
    if target.exists():
        return True
    matches = sorted(
        (p for p in target.parent.glob(f"{target.stem}*{target.suffix}") if p != target),
        key=lambda p: p.stat().st_mtime,
    )
    if not matches:
        return False
    matches[-1].replace(target)
    for leftover in matches[:-1]:
        leftover.unlink(missing_ok=True)
    return True


def _encode_pose_packet(pose: dict[str, float]) -> bytes:
    """
    Build the 51-byte UDP pose packet for a pose given in degrees.

    Uses the shared wire contract rather than a private layout, so this cannot drift from what the
    receiver expects. Angles are converted to the radians the wire carries -- the degrees/radians
    boundary is the single most common source of confusion in this protocol, which is why every
    name here states its unit.

    Args:
        pose: ``lat_deg``, ``lon_deg``, ``alt_m``, ``roll_deg``, ``pitch_deg``, ``yaw_deg``.

    Returns:
        The encoded packet.

    """
    payload = struct.pack(
        packet_spec.PAYLOAD_FORMAT,
        pose["lat_deg"],
        pose["lon_deg"],
        pose["alt_m"],
        math.radians(pose["roll_deg"]),
        math.radians(pose["pitch_deg"]),
        math.radians(pose["yaw_deg"]),
    )
    return bytes(packet_spec.HEADER) + payload + bytes([packet_spec.checksum(payload)])


class SimulationRuntime:
    """
    Isaac Sim application lifecycle and step loop.

    Designed as a context manager::

        with SimulationRuntime(config, plan) as rt:
            rt.run()

    """

    def __init__(self, config: IsaacCoreConfig, plan: FeaturePlan) -> None:
        """
        Initialise the runtime.

        The ``SimulationApp`` is NOT created here -- it is created in
        :meth:`start` so that callers have full control over when the heavy
        import happens (``__main__`` sets launch config first).

        Args:
            config: The resolved configuration.
            plan: The feature plan produced by the planner.

        """
        self._config = config
        self._plan = plan
        self._app: Any = None
        # Isaac Sim 6 API handles, bound in run(). See run()'s docstring: there is no
        # SimulationContext in 6.0 -- the Core API was replaced by
        # isaacsim.core.experimental.*.
        self._app_utils: Any = None
        self._simulation_manager: Any = None
        self._control_server: ControlServer | None = None
        self._main_thread_tasks: queue.Queue[_MainThreadTask] = queue.Queue()
        self._loop_thread_id: int | None = None
        self._running = False
        # Gimbal aiming state. `_gimbal_target` is set by the control plane and cleared once
        # reached; `_gimbal_current` is where the gimbal actually is, integrated per frame.
        self._gimbal_target: _GimbalTarget | None = None
        self._gimbal_current: GimbalAngles | None = None
        self._gimbal_prim_path: str | None = None
        self._capture_request: _CaptureRequest | None = None
        # Set by reset(); the loop presses play one frame later, since play() in the same
        # frame as stop() is ignored by Kit's timeline.
        self._pending_play = False
        # Set once the stage is composed, so clients can tell a live port from a usable sim.
        self._stage_composed = False
        # Runtime config patches, consulted ahead of the frozen config models.
        self._config_overrides: dict[str, Any] = {}

    @property
    def config(self) -> IsaacCoreConfig:
        """Return the active configuration."""
        return self._config

    @property
    def plan(self) -> FeaturePlan:
        """Return the active feature plan."""
        return self._plan

    @property
    def is_running(self) -> bool:
        """Return whether the step loop is active."""
        return self._running

    def _apply_ros_domain(self) -> None:
        """
        Export ``ROS_DOMAIN_ID`` when the config sets one explicitly.

        The ROS 2 bridge's context node reads ``$ROS_DOMAIN_ID`` (``useDomainIDEnvVar`` is
        on by default), so exporting here before the bridge starts is what makes
        ``ros2.domain_id`` actually take effect. ``None`` leaves the environment untouched,
        so the bridge simply inherits whatever the shell already set.
        """
        domain_id = self._config.ros2.domain_id
        if domain_id is None:
            logger.debug("ros2.domain_id unset; inheriting ROS_DOMAIN_ID=%s", os.environ.get("ROS_DOMAIN_ID", "0"))
            return
        env_value = os.environ.get("ROS_DOMAIN_ID")
        if env_value is not None and env_value != str(domain_id):
            logger.warning(
                "ros2.domain_id=%d in config overrides ROS_DOMAIN_ID=%s from the environment",
                domain_id,
                env_value,
            )
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        logger.info("exported ROS_DOMAIN_ID=%d from config", domain_id)

    def start(self) -> None:
        """
        Create the SimulationApp, open the stage, and start the control server.

        Call this AFTER the SimulationApp launch config has been set (in
        ``__main__``). The SimulationApp import triggers Kit initialisation.
        """
        self._apply_ros_domain()

        if self._config.cesium.delete_cache_on_launch:
            # Must happen before the app starts: cesium.omniverse is a boot extension and
            # opens the request-cache sqlite during Kit init. Deleting it afterwards, while
            # Cesium holds it open, produced intermittent "disk I/O error" log spam. Doing
            # it here, before the app exists, means the files are simply absent when Cesium
            # first opens them and it recreates them cleanly.
            from isaac_core.sim.composer import delete_cesium_cache

            delete_cesium_cache()

        isaacsim = importlib.import_module("isaacsim")
        sim_app_cls = getattr(isaacsim, "SimulationApp")

        launch_config = {
            "headless": self._config.sim.headless,
            "width": 1280,
            "height": 720,
            "renderer": self._config.sim.renderer,
            "extra_args": self._kit_startup_args(),
        }
        self._silence_warp_banner()
        with self._suppressed_startup_stdout():
            self._app = sim_app_cls(launch_config, experience=self._resolve_experience())

        self._enable_required_extensions()

        if self._config.sim.control_plane.enabled:
            self._start_control_server()
            if self._control_server is not None:
                logger.info(
                    "control plane listening on %s:%d",
                    self._control_server.host,
                    self._control_server.port,
                )
        else:
            # Disabling it is legitimate: a batch render or a crash bisection has no
            # need for remote control, and one fewer background thread is one fewer
            # thing interacting with Kit's event loop.
            logger.info("control plane disabled by configuration")

    def _silence_warp_banner(self) -> None:
        """
        Set Warp's ``quiet`` flag before Warp initialises, when Isaac logs are off.

        Warp prints a multi-line "Warp <version> initialized: ... Devices: ..." banner to
        stdout the first time it initialises, guarded solely by ``warp.config.quiet`` -- there
        is no environment variable for it. Setting the flag here, before ``SimulationApp`` is
        constructed and long before any extension pulls Warp in, means the flag is already in
        place when ``warp.init()`` runs. Warp's own docstring notes that errors and warnings are
        unaffected by ``quiet``, so this hides the banner without hiding a real problem.

        Left untouched when ``logging.isaac_logs`` is set, so a debugging run still sees it. A
        missing Warp import is not an error: a build without Warp simply has no banner to quiet.
        """
        if self._config.logging.isaac_logs:
            return
        try:
            warp_config = importlib.import_module("warp.config")
        except ImportError:
            logger.debug("warp not importable; no Warp banner to silence")
            return
        # The module is imported dynamically, so its ``quiet`` attribute is invisible to the
        # type checker even though Warp defines it; the Any handle makes the write explicit.
        config: Any = warp_config
        config.quiet = True
        logger.debug("set warp.config.quiet to suppress the Warp init banner")

    @contextlib.contextmanager
    def _suppressed_startup_stdout(self) -> "Iterator[None]":
        """
        Redirect POSIX stdout to the null device for the duration of the block.

        This is the only lever that catches the launcher banners Isaac's ``SimulationApp``
        prints with plain ``print()`` and the lines Kit's C++ startup writes to the process's
        stdout: both bypass every log level and the ``enableStdoutOutput`` setting. Only fd 1 is
        redirected, and only at the file-descriptor level so the C++ side is covered too; fd 2
        (stderr) is deliberately left alone, so our own logging and any Python traceback raised
        while the app is constructed still reach the terminal in full.

        The original descriptor is restored in a ``finally``, unconditionally, even if
        construction raises -- a silent boot that also swallowed a crash would be far worse than
        a noisy one. Disabled entirely when ``logging.isaac_logs`` is set, which then shows
        every startup line.
        """
        if self._config.logging.isaac_logs:
            yield
            return
        sys.stdout.flush()
        saved_fd = os.dup(STDOUT_FILENO)
        null_fd = os.open(os.devnull, os.O_WRONLY)
        try:
            os.dup2(null_fd, STDOUT_FILENO)
            yield
        finally:
            sys.stdout.flush()
            os.dup2(saved_fd, STDOUT_FILENO)
            os.close(saved_fd)
            os.close(null_fd)

    def _set_viewport_camera(self) -> None:
        """
        Point the main viewport at the configured camera, when running with a GUI.

        Skipped when headless, where there is no viewport to retarget, and skipped if the
        prim does not exist so a scene without that camera still runs.
        """
        template = self._config.sim.viewport_camera
        if self._config.sim.headless or not template:
            return

        instance = next(iter(self._config.vehicles))
        camera_path = template.replace("{instance}", instance)

        omni_usd = importlib.import_module("omni.usd")
        stage = omni_usd.get_context().get_stage()
        if stage is None or not stage.GetPrimAtPath(camera_path).IsValid():
            logger.warning("viewport camera %s does not exist; leaving the viewport alone", camera_path)
            return

        try:
            viewport_utils = importlib.import_module("omni.kit.viewport.utility")
            viewport = viewport_utils.get_active_viewport()
        except (ImportError, RuntimeError) as exc:
            logger.warning("could not access the viewport: %s", exc)
            return
        if viewport is None:
            logger.warning("no active viewport to retarget")
            return

        viewport.camera_path = camera_path
        logger.info("viewport looking through %s", camera_path)

    def _quiet_noisy_loggers(self) -> None:
        """
        Raise the level of Isaac's chattiest Python loggers.

        Only takes effect if called after the extensions have loaded: each sets its own
        level on the way up, overwriting anything configured earlier. `ogn_registration`
        alone accounted for 2,721 lines of a 3,400-line launch.
        """
        if self._config.logging.isaac_logs:
            return
        for name in self._config.logging.quiet_loggers:
            logging.getLogger(name).setLevel(logging.WARNING)
        logger.debug("quieted %d Isaac loggers", len(self._config.logging.quiet_loggers))

    def _resolve_experience(self) -> str:
        """
        Resolve the configured Kit experience to a path SimulationApp accepts.

        Args:
            None.

        Returns:
            An absolute ``.kit`` path, or an empty string to accept Isaac's own default.

        """
        configured = self._config.sim.experience
        if not configured:
            return ""

        candidate = Path(configured).expanduser()
        if candidate.is_absolute():
            if candidate.is_file():
                return str(candidate)
            logger.warning("experience %s does not exist; using Isaac's default", candidate)
            return ""

        # Bare filename: resolve against Isaac's apps directory, which Kit exports as
        # EXP_PATH inside the simulator's own interpreter.
        exp_path = os.environ.get("EXP_PATH")
        if not exp_path:
            logger.warning("EXP_PATH is unset; using Isaac's default experience")
            return ""

        resolved = Path(exp_path) / configured
        if not resolved.is_file():
            logger.warning("experience %s not found; using Isaac's default", resolved)
            return ""

        logger.info("using Kit experience %s", resolved)
        return str(resolved)

    def _kit_startup_args(self) -> list[str]:
        """
        Build the Kit startup arguments: extension folders, boot enables and log levels.

        Only existing directories are included: a missing path is a machine difference,
        not an error, and passing a nonexistent folder makes Kit complain on every launch.

        Returns:
            Flat argument list, empty if no configured path exists.

        """
        args: list[str] = []
        for raw in self._config.sim.extension_search_paths:
            folder = Path(raw).expanduser()
            if folder.is_dir():
                args.extend(["--ext-folder", str(folder)])
            else:
                logger.debug("skipping missing extension folder %s", folder)

        # Must be enabled by Kit itself, before the USD schema registry initialises.
        for extension in self._config.sim.boot_extensions:
            args.extend(["--enable", extension])

        args.extend(self._log_level_args())
        return args

    def _log_level_args(self) -> list[str]:
        """
        Build Kit logging arguments honouring ``logging.isaac_logs``.

        Kit's log level has to be set as a startup argument. Setting it later still leaves
        the thousands of lines it prints while loading extensions, which is exactly the
        noise the setting exists to remove.

        Returns:
            Kit arguments, empty when Isaac's own logs are wanted.

        """
        if self._config.logging.isaac_logs:
            return []
        return [
            "--/log/level=warning",
            "--/log/outputStreamLevel=warning",
            "--/log/debugConsoleLevel=warning",
            # Kit prints ~500 "[ext: ...] startup" lines straight to stdout, which no log
            # level affects. Our own logging goes to stderr, so it survives this.
            "--/app/enableStdoutOutput=false",
        ]

    def _enable_required_extensions(self) -> None:
        """
        Enable the Kit extensions the composed stages depend on.

        Without this, OmniGraph loads a stage and reports
        ``Could not find node type interface for ...`` for every node -- our own and
        the ROS 2 bridge's alike -- and the graphs silently do nothing. The base
        ``isaacsim.exp.base.python`` app does not enable them.

        Failures are logged rather than raised: a stage that does not use the ROS 2
        bridge should still run if only that extension is unavailable, and the startup
        report makes the omission visible.
        """
        app_utils = importlib.import_module("isaacsim.core.experimental.utils.app")

        for extension in self._config.sim.extensions:
            try:
                enabled = app_utils.enable_extension(extension)
            except (ImportError, RuntimeError, ValueError) as exc:
                logger.warning("could not enable extension %s: %s", extension, exc)
                continue
            if enabled:
                logger.debug("enabled extension %s", extension)
            else:
                logger.warning("extension %s could not be enabled; nodes from it will be missing", extension)

        # Extensions set their own Python logger levels as they load, so quieting them
        # before this point has no effect -- it has to happen afterwards.
        self._quiet_noisy_loggers()

        # Let the newly enabled extensions settle before any stage operation.
        self._pump(app_utils, EXTENSION_WARMUP_FRAMES)
        logger.debug("warmed up %d frames after enabling extensions", EXTENSION_WARMUP_FRAMES)

    def open_stage(self, scene_path: Path, layer_search_paths: tuple[Path, ...]) -> None:
        """
        Open the scene and compose feature layers.

        Args:
            scene_path: Path to the base scene USD.
            layer_search_paths: Search paths for layer USD files.

        """
        from isaac_core.sim.composer import compose_stage

        app_utils = importlib.import_module("isaacsim.core.experimental.utils.app")

        def settle() -> None:
            """Let the application catch up between composition steps."""
            self._pump(app_utils, STAGE_SETTLE_FRAMES)

        compose_stage(
            self._config,
            self._plan,
            scene_path,
            layer_search_paths,
            settle=settle,
        )
        self._stage_composed = True

        self._set_viewport_camera()

    def run(self) -> None:
        """
        Enter the simulation step loop.

        Block until the application shuts down or :meth:`stop` is called.

        Isaac Sim 6 removed ``SimulationContext`` along with the whole
        ``isaacsim.core.api`` extension -- the Core API was replaced by
        ``isaacsim.core.experimental.*``. Play/pause/step now go through
        ``isaacsim.core.experimental.utils.app`` (a thin wrapper over
        ``omni.timeline``) and physics through ``SimulationManager``, which is what
        Isaac's own standalone examples use.

        The loop deliberately runs while the *application* runs, not while the
        timeline is playing. The previous generation gated on ``is_playing()``, so
        pausing terminated the process; here a pause leaves the loop turning and the
        control plane responsive, which is what makes remote ``pause``/``resume``
        useful.
        """
        app_utils = importlib.import_module("isaacsim.core.experimental.utils.app")
        simulation_manager = importlib.import_module("isaacsim.core.simulation_manager")
        manager = simulation_manager.SimulationManager

        manager.setup_simulation(dt=self._config.sim.physics_dt)
        manager.initialize_physics()

        self._app_utils = app_utils
        self._simulation_manager = manager

        app_utils.play()
        self._running = True
        self._loop_thread_id = threading.get_ident()
        logger.info("simulation running")

        try:
            while self._running and self._app.is_running():
                app_utils.update_app()
                self._step_pending_play()
                self._step_gimbal()
                self._step_capture()
                self._drain_main_thread_tasks()
        finally:
            self._running = False
            logger.info("simulation stopped")

    def _pump(self, app_utils: Any, frames: int) -> None:  # noqa: ANN401
        """
        Advance the application a fixed number of frames.

        Used to let subsystems finish initialising at points where a stage operation would
        otherwise race them -- see :data:`EXTENSION_WARMUP_FRAMES`.

        Args:
            app_utils: The ``isaacsim.core.experimental.utils.app`` module.
            frames: Number of update cycles to run.

        """
        for _ in range(frames):
            app_utils.update_app()

    def _drain_main_thread_tasks(self) -> None:
        """Run every queued task, recording its result or exception for the caller."""
        while True:
            try:
                task = self._main_thread_tasks.get_nowait()
            except queue.Empty:
                return
            try:
                task.result = task.call()
            except BaseException as exc:  # noqa: BLE001
                # Held and re-raised in the requesting thread: a handler bug must not
                # take down the step loop, and the caller needs the real error.
                task.error = exc
            finally:
                task.done.set()

    def _on_main_thread(self, call: "Callable[[], Any]") -> Any:  # noqa: ANN401
        """
        Run ``call`` on the step-loop thread and return its result.

        Args:
            call: Zero-argument callable touching USD, Fabric or the Kit application.

        Returns:
            Whatever ``call`` returns.

        Raises:
            TimeoutError: If the step loop does not run the task in time.

        """
        if self._loop_thread_id is None or threading.get_ident() == self._loop_thread_id:
            # Before the loop starts, or already on it: nothing to hand off to.
            return call()

        task = _MainThreadTask(call=call)
        self._main_thread_tasks.put(task)
        if not task.done.wait(timeout=MAIN_THREAD_TASK_TIMEOUT_S):
            message = f"the simulation step loop did not run the task within {MAIN_THREAD_TASK_TIMEOUT_S}s"
            raise TimeoutError(message)
        if task.error is not None:
            raise task.error
        return task.result

    def stop(self) -> None:
        """Stop the step loop and shut down."""
        self._running = False
        if self._app_utils is not None:
            self._app_utils.stop()
        if self._control_server is not None:
            self._control_server.stop()
            self._control_server = None

    def close(self) -> None:
        """Shut down the SimulationApp."""
        self.stop()
        if self._app is not None:
            self._app.close()
            self._app = None

    def __enter__(self) -> "SimulationRuntime":
        """Enter the context manager."""
        return self

    def __exit__(self, *_: object) -> None:
        """Exit the context manager, closing everything."""
        self.close()

    def _start_control_server(self) -> None:
        """Start the JSON-RPC control server and register handlers."""
        self._control_server = ControlServer(self._config.sim.control_plane)
        self._control_server.register("ping", self._handle_ping)
        self._control_server.register("get_pose", self._handle_get_pose)
        self._control_server.register("set_gimbal", self._handle_set_gimbal)
        self._control_server.register("set_pose", self._handle_set_pose)
        self._control_server.register("load_scene", self._handle_load_scene)
        self._control_server.register("get_state", self._handle_get_state)
        self._control_server.register("get_capabilities", self._handle_get_capabilities)
        self._control_server.register("get_config", self._handle_get_config)
        self._control_server.register("pause", self._handle_pause)
        self._control_server.register("resume", self._handle_resume)
        self._control_server.register("step", self._handle_step)
        self._control_server.register("capture_frame", self._handle_capture_frame)
        self._control_server.register("set_config", self._handle_set_config)
        self._control_server.register("reset", self._handle_reset)
        self._control_server.register("enable_feature", self._handle_enable_feature)
        self._control_server.register("disable_feature", self._handle_disable_feature)
        self._control_server.register("get_runtime_values", self._handle_get_runtime_values)
        self._control_server.register("read_prim_attribute", self._handle_read_prim_attribute)
        self._control_server.start()

    def _handle_ping(self, params: dict[str, Any] | list[Any] | None) -> str:
        """Respond to a ping request."""
        return "pong"

    def _stage(self) -> Any:  # noqa: ANN401
        """
        Return the currently open USD stage, or ``None``.

        Returns:
            The stage, or ``None`` before one is open.

        """
        omni_usd = importlib.import_module("omni.usd")
        return omni_usd.get_context().get_stage()

    @property
    def _frame_dt_s(self) -> float:
        """
        Return the per-frame timestep used for slewing.

        Uses the configured physics dt rather than measured wall time so a commanded slew takes
        the same number of simulated seconds regardless of how fast the machine renders.
        """
        return float(self._config.sim.physics_dt)

    def _resolve_gimbal_prim(self) -> None:
        """
        Locate the prim carrying the gimbal offset inputs, once.

        Resolved from the first vehicle's mount rather than hardcoded, so a renamed vehicle still
        works. Left as ``None`` when the pose graph is absent, which simply makes ``set_gimbal``
        a no-op instead of an error -- a scene with no camera layer has no gimbal to move.
        """
        if self._gimbal_prim_path is not None:
            return
        vehicle_id = next(iter(self._config.vehicles))
        mount = self._config.resolved_mount(vehicle_id)
        candidate = f"{mount}/PoseSync/global_position_to_local_position"
        stage = self._stage()
        if stage is None:
            return
        sdf = importlib.import_module("pxr.Sdf")
        if stage.GetPrimAtPath(sdf.Path(candidate)).IsValid():
            self._gimbal_prim_path = candidate
            logger.debug("gimbal offsets resolved to %s", candidate)

    def _write_float_attribute(self, prim_path: str, attribute: str, value: float) -> None:
        """
        Write one float attribute on the simulation thread.

        Args:
            prim_path: Absolute prim path.
            attribute: Attribute name.
            value: Value to write.

        """
        stage = self._stage()
        if stage is None:
            return
        sdf = importlib.import_module("pxr.Sdf")
        prim = stage.GetPrimAtPath(sdf.Path(prim_path))
        if not prim.IsValid():
            return
        attr = prim.GetAttribute(attribute)
        if attr.IsValid():
            attr.Set(float(value))

    def _handle_set_gimbal(self, params: dict[str, Any] | list[Any] | None) -> dict[str, float]:
        """
        Aim the gimbal at a new attitude, slewing there if a rate limit is configured.

        Only records the target. The prim writes happen on the simulation loop, which is the one
        thread allowed to touch USD -- so this handler never needs the main-thread task queue and
        cannot block the caller behind a frame.

        Args:
            params: ``roll_deg``, ``pitch_deg`` and ``yaw_deg``; any omitted axis holds.

        Returns:
            The requested target in degrees.

        Raises:
            RpcError: If a supplied angle is not a number.

        """
        values = _as_mapping(params)
        gimbal = self._config.vehicles[next(iter(self._config.vehicles))].gimbal
        held = self._gimbal_current.to_degrees() if self._gimbal_current is not None else None
        current = _GimbalTarget(
            roll_deg=held[0] if held else gimbal.start_roll_deg,
            pitch_deg=held[1] if held else gimbal.start_pitch_deg,
            yaw_deg=held[2] if held else gimbal.start_yaw_deg,
        )
        target = _GimbalTarget(
            roll_deg=_optional_float(values, "roll_deg", current.roll_deg),
            pitch_deg=_optional_float(values, "pitch_deg", current.pitch_deg),
            yaw_deg=_optional_float(values, "yaw_deg", current.yaw_deg),
        )
        self._gimbal_target = target
        logger.info(
            "gimbal target set to roll=%.2f pitch=%.2f yaw=%.2f deg",
            target.roll_deg,
            target.pitch_deg,
            target.yaw_deg,
        )
        return {"roll_deg": target.roll_deg, "pitch_deg": target.pitch_deg, "yaw_deg": target.yaw_deg}

    def _step_pending_play(self) -> None:
        """
        Press play if a reset asked for it on a previous frame.

        Separated from the reset handler because Kit ignores ``play()`` issued in the same frame as
        ``stop()``, which left a reset simulation stopped.
        """
        if not self._pending_play:
            return
        self._pending_play = False
        timeline = importlib.import_module("omni.timeline").get_timeline_interface()
        timeline.play()
        logger.info("reset: timeline resumed")

    def _step_gimbal(self) -> None:
        """
        Move the gimbal one frame's worth toward its target.

        Called every frame from the simulation loop. Does nothing until a target has been
        commanded, so a run that never touches the gimbal pays nothing and leaves the config's
        start angles exactly as composed.
        """
        target = self._gimbal_target
        if target is None:
            return
        self._resolve_gimbal_prim()
        if self._gimbal_prim_path is None:
            return

        limit = self._gimbal_max_rate_deg_s()
        target_angles = GimbalAngles.from_degrees(target.roll_deg, target.pitch_deg, target.yaw_deg)
        if self._gimbal_current is None:
            gimbal = self._config.vehicles[next(iter(self._config.vehicles))].gimbal
            self._gimbal_current = GimbalAngles.from_degrees(
                gimbal.start_roll_deg, gimbal.start_pitch_deg, gimbal.start_yaw_deg
            )
        else:
            # A rate of None means unlimited, matching the config default and the previous
            # generation's unconditional snap.
            rate_r_s = math.radians(limit) if limit is not None else 0.0
            self._gimbal_current = slew_towards(self._gimbal_current, target_angles, rate_r_s, self._frame_dt_s)

        roll_deg, pitch_deg, yaw_deg = self._gimbal_current.to_degrees()
        for attribute, value in (
            ("inputs:offset_roll_deg", roll_deg),
            ("inputs:offset_pitch_deg", pitch_deg),
            ("inputs:offset_yaw_deg", yaw_deg),
        ):
            self._write_float_attribute(self._gimbal_prim_path, attribute, value)

        if self._gimbal_current.as_tuple() == target_angles.as_tuple():
            self._gimbal_target = None
            logger.debug("gimbal reached target")

    def _handle_get_pose(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Return the live local transform of a vehicle's moved prim.

        Reads what the pose graph has actually written, which is the only way to
        confirm from outside the process that UDP or ROS input is reaching the stage.
        Verifying that by eye in the viewport does not scale, and nothing else exposes
        it.

        Args:
            params: Optional mapping with ``vehicle``; defaults to the first vehicle.

        Returns:
            The vehicle id, prim path, and its translate and orient, or an ``error``
            entry explaining why they could not be read.

        """
        vehicle_id = self._vehicle_from(_as_mapping(params))

        mount = self._config.resolved_mount(vehicle_id)
        prim_path = f"{mount}/Xform"
        result: dict[str, Any] = {"vehicle": vehicle_id, "prim": prim_path}
        result.update(self._on_main_thread(lambda: self._read_prim_transform(prim_path)))
        return result

    def _read_prim_transform(self, prim_path: str) -> dict[str, Any]:
        """
        Read a prim's translate and orient from the stage.

        Must run on the step-loop thread -- see :class:`_MainThreadTask`.

        Args:
            prim_path: Absolute path of the prim to read.

        Returns:
            ``translate`` and ``orient`` entries, or an ``error`` entry explaining why
            they could not be read.

        """
        omni_usd = importlib.import_module("omni.usd")
        stage = omni_usd.get_context().get_stage()
        if stage is None:
            return {"error": "no stage open"}

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return {"error": f"prim {prim_path} does not exist"}

        transform: dict[str, Any] = {}
        for attribute, key in (("xformOp:translate", "translate"), ("xformOp:orient", "orient")):
            attr = prim.GetAttribute(attribute)
            transform[key] = _usd_value_to_list(attr.Get()) if attr.IsValid() else None
        return transform

    def _handle_read_prim_attribute(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Read one live attribute value off a prim in the running stage.

        The general "read what is actually there" primitive: given a prim path and an
        attribute name, return the value currently on the stage, not what config asked for.

        Args:
            params: Mapping with ``prim`` and ``attribute``.

        Returns:
            ``{"prim", "attribute", "value"}`` (value ``None`` if the prim or attribute is
            absent or unset), or an ``error`` entry.

        """
        if not isinstance(params, dict) or "prim" not in params or "attribute" not in params:
            raise InvalidParamsError("read_prim_attribute requires params.prim and params.attribute")
        prim_path = str(params["prim"])
        attribute = str(params["attribute"])
        value = self._on_main_thread(lambda: self._read_attr(prim_path, attribute))
        return {"prim": prim_path, "attribute": attribute, "value": value}

    def _handle_get_runtime_values(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Return the effective values actually applied on the running stage.

        Reads the real attribute values off the composed prims -- port, rotation frame,
        topic names, camera intrinsics, tileset URLs -- rather than echoing config, so an
        inspector reports what is genuinely running. Config is consulted only for the prim
        paths (the mount), which are not themselves stage-readable.

        Args:
            params: Optional mapping with ``vehicle``; defaults to the first vehicle.

        Returns:
            A dict of live values, or an ``error`` entry.

        """
        vehicle_id = next(iter(self._config.vehicles))
        if isinstance(params, dict) and "vehicle" in params:
            vehicle_id = str(params["vehicle"])
        mount = self._config.resolved_mount(vehicle_id)
        tilesets_root = self._config.cesium.tilesets_root
        values: dict[str, Any] = self._on_main_thread(
            lambda: self._read_runtime_values(vehicle_id, mount, tilesets_root)
        )
        return values

    def _read_attr(self, prim_path: str, attribute: str) -> Any:  # noqa: ANN401
        """
        Read a single attribute value from the stage, JSON-serialisable. Main-thread only.

        Args:
            prim_path: Absolute prim path.
            attribute: Attribute name.

        Returns:
            The value as a scalar or list, or ``None`` if absent/unset.

        """
        omni_usd = importlib.import_module("omni.usd")
        stage = omni_usd.get_context().get_stage()
        if stage is None:
            return None
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None
        attr = prim.GetAttribute(attribute)
        return _json_safe_usd(attr.Get()) if attr.IsValid() else None

    def _read_runtime_values(self, vehicle_id: str, mount: str, tilesets_root: str) -> dict[str, Any]:
        """
        Read the curated set of effective values off the stage. Main-thread only.

        Args:
            vehicle_id: The vehicle to report.
            mount: The vehicle's resolved mount prim path.
            tilesets_root: Prim path under which Cesium tilesets live.

        Returns:
            A dict of the real values found on the stage.

        """
        omni_usd = importlib.import_module("omni.usd")
        stage = omni_usd.get_context().get_stage()
        if stage is None:
            return {"error": "no stage open"}

        def rd(prim_path: str, attribute: str) -> Any:  # noqa: ANN401
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return None
            attr = prim.GetAttribute(attribute)
            return _json_safe_usd(attr.Get()) if attr.IsValid() else None

        pose_sync = f"{mount}/PoseSync"
        camera = f"{mount}/Xform/main_camera_01"
        values: dict[str, Any] = {
            "vehicle": vehicle_id,
            "mount": mount,
            "udp_port": rd(f"{pose_sync}/udp_to_global_position", "inputs:udp_port"),
            "rotation_frame": rd(f"{pose_sync}/global_position_to_local_position", "inputs:rotation_frame"),
            "enu_reference": rd(f"{pose_sync}/global_position_to_local_position", "inputs:enu_reference"),
            "global_pose_topic": rd(f"{pose_sync}/ros2_publisher", "inputs:topicName"),
            "image_topic": rd(f"{mount}/CameraImageExport/ros2_camera_helper", "inputs:topicName"),
            "camera": {
                "focalLength": rd(camera, "focalLength"),
                "horizontalAperture": rd(camera, "horizontalAperture"),
                "verticalAperture": rd(camera, "verticalAperture"),
            },
        }

        tilesets: dict[str, Any] = {}
        root = stage.GetPrimAtPath(tilesets_root)
        if root.IsValid():
            usd = importlib.import_module("pxr.Usd")
            for prim in usd.PrimRange(root):
                attr = prim.GetAttribute("cesium:url")
                if attr.IsValid() and attr.Get():
                    tilesets[str(prim.GetPath())] = attr.Get()
        values["tilesets"] = tilesets
        return values

    def _handle_get_state(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Return the current simulation state.

        ``ready`` is the field a client should wait on. The control plane starts listening
        *before* the stage is composed, so a connectable port means only "the process is alive":
        commands sent between those two moments are silently lost, because the graphs that would
        act on them do not exist yet. That is not theoretical -- a `set_pose` issued in that window
        left the camera motionless with no error anywhere.

        Args:
            params: Ignored.

        Returns:
            Running and readiness state plus the scene and headless flags.

        """
        del params
        return {
            "running": self._running,
            "ready": self._running and self._stage_composed,
            "stage_composed": self._stage_composed,
            "scene": self._config.sim.scene,
            "headless": self._config.sim.headless,
        }

    def _handle_get_capabilities(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """Return enabled and skipped layers."""
        return {
            "enabled": [p.manifest.id for p in self._plan.enabled],
            "skipped": [{"id": s.id, "reason": s.reason} for s in self._plan.skipped],
        }

    def _handle_get_config(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """Return the current config as a dict."""
        return self._config.model_dump(mode="json")  # type: ignore[no-any-return]

    def _handle_pause(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Pause the simulation.

        The pause goes through the step loop, not the control server's thread. ``pause`` is a
        thin wrapper over ``omni.timeline``, and a timeline transition fires Isaac's own
        extension callbacks (the throttling extension's ``_on_play``/``_on_stop`` among them)
        which assume they run on the main thread with a live asyncio event loop. Calling it from
        the control-server thread crashed inside Isaac's throttling extension with
        ``RuntimeError: There is no current event loop in thread 'isaac-core-control-server'``.

        Args:
            params: Ignored.

        Returns:
            ``"paused"``.

        """
        del params
        if self._app_utils is not None:
            self._on_main_thread(self._app_utils.pause)
        return "paused"

    def _handle_resume(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Resume the simulation.

        Dispatched to the step loop for the same reason as :meth:`_handle_pause`: ``play``
        wraps ``omni.timeline`` and its transition invokes Isaac extension callbacks that need
        the main thread's event loop. This was the reported bug -- ``resume`` from the control
        thread reached Isaac's throttling extension ``_on_play`` and died with
        ``RuntimeError: There is no current event loop in thread 'isaac-core-control-server'``.

        Args:
            params: Ignored.

        Returns:
            ``"resumed"``.

        """
        del params
        if self._app_utils is not None:
            self._on_main_thread(self._app_utils.play)
        return "resumed"

    def _handle_step(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Advance the simulation by ``count`` frames.

        The frames are pumped inside a single queued main-thread task rather than spread across
        ``count`` loop iterations like the capture state machine. ``step`` is a synchronous
        request-response call whose contract is "the simulation has advanced N frames by the time
        this returns", so the caller must block until all N are done; the capture machine can
        stretch over frames precisely because its caller waits on a separate event, not on the
        queued task. Pumping here is not re-entrant either -- the loop drains this task *between*
        its own ``update_app`` calls, so ``update_app`` is never called from inside itself.

        The bound is :data:`MAIN_THREAD_TASK_TIMEOUT_S`: the whole run of ``count`` frames must
        finish within that budget or the caller sees a ``TimeoutError``, so a very large ``count``
        on a slow stage can time out. That is deliberate -- a wedged or glacial loop surfaces as a
        clear error rather than a client hung forever. Callers wanting many frames should issue
        several ``step`` calls.

        Args:
            params: Optional mapping with ``count`` (a positive integer, default 1).

        Returns:
            ``"stepped"``.

        Raises:
            InvalidParamsError: If ``count`` is present but not a positive integer.

        """
        count = _optional_positive_int(_as_mapping(params), "count") or 1
        if self._app_utils is None:
            return "stepped"
        app_utils = self._app_utils

        def _pump() -> None:
            """Advance the simulation ``count`` frames on the loop thread."""
            timeline = importlib.import_module("omni.timeline").get_timeline_interface()
            # `update_app` renders a frame but does NOT advance a paused timeline, so stepping
            # while paused appeared to do nothing at all -- which is exactly when stepping is
            # useful. `forward_one_frame` moves the timeline itself; the render still needs
            # `update_app` afterwards for the new frame to appear.
            playing = timeline.is_playing()
            for _ in range(count):
                if not playing:
                    timeline.forward_one_frame()
                app_utils.update_app()

        self._on_main_thread(_pump)
        return "stepped"

    def _handle_capture_frame(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Capture the camera's current frame to an image on disk.

        This is 2023's ``--sat``, moved off ROS because a capture is a *command*, not telemetry
        (D20). ``width``/``height`` capture at a resolution independent of the viewport -- which
        2023 could not do -- by resizing the render product for the shot and restoring it after.

        The work is driven by the simulation loop across several frames rather than done inline,
        because changing the resolution needs frames to converge and the capture itself writes
        asynchronously. Doing it in a main-thread task would mean pumping frames from inside the
        loop's own frame, which is re-entrant and hangs.

        Args:
            params: ``path`` (required, confined under ``output_root``) plus optional ``width``
                and ``height`` in pixels.

        Returns:
            The absolute path written and the resolution actually captured.

        Raises:
            InvalidParamsError: If ``path`` is missing, only one dimension is given, or a
                dimension is not a positive integer.
            RuntimeError: If the capture does not complete, or no viewport exists.

        """
        from isaac_core.control.server import confine_path  # noqa: PLC0415

        values = _as_mapping(params)
        if "path" not in values:
            raise InvalidParamsError("capture_frame requires params.path")
        target = confine_path(values["path"], self._config.sim.control_plane.output_root)
        target.parent.mkdir(parents=True, exist_ok=True)

        width = _optional_positive_int(values, "width")
        height = _optional_positive_int(values, "height")
        if (width is None) != (height is None):
            raise InvalidParamsError("capture_frame needs both width and height, or neither")

        request = _CaptureRequest(target=target, width=width, height=height)
        self._capture_request = request
        if not request.done.wait(timeout=CAPTURE_TIMEOUT_S):
            self._capture_request = None
            message = f"capture did not complete within {CAPTURE_TIMEOUT_S}s"
            raise RuntimeError(message)
        if request.error is not None:
            raise RuntimeError(request.error)
        return request.result

    def _step_capture(self) -> None:
        """
        Advance an in-flight capture by one frame.

        Each state transition happens on a *different* loop iteration, which is what lets the
        renderer converge at a new resolution and the asynchronous file write land without ever
        calling ``update_app`` re-entrantly.
        """
        request = self._capture_request
        if request is None:
            return
        try:
            self._advance_capture(request)
        except Exception as exc:  # noqa: BLE001 - report to the caller rather than kill the loop
            logger.exception("capture failed")
            request.error = str(exc)
            self._capture_request = None
            request.done.set()

    def _advance_capture(self, request: _CaptureRequest) -> None:
        """
        Run one step of the capture state machine.

        Args:
            request: The in-flight capture.

        Raises:
            RuntimeError: If no viewport is available.

        """
        utility = importlib.import_module("omni.kit.viewport.utility")
        request.frames += 1

        if request.stage == "start":
            viewport = self._capture_viewport(utility)
            if viewport is None:
                message = "no viewport available to capture from"
                raise RuntimeError(message)
            request.render_product = self._camera_render_product()
            original = self._render_product_resolution(request.render_product) or tuple(
                int(v) for v in viewport.resolution
            )
            request.original = (original[0], original[1])
            if request.width is not None and request.height is not None:
                self._set_render_product_resolution(request.render_product, request.width, request.height)
            request.stage = "settle"
            request.frames = 0
            return

        viewport = self._capture_viewport(utility)
        if request.stage == "settle":
            if request.frames >= CAPTURE_SETTLE_FRAMES:
                used = self._render_product_resolution(request.render_product) or request.original
                request.used = (used[0], used[1])
                utility.capture_viewport_to_file(
                    viewport,
                    file_path=str(request.target),
                    render_product_path=request.render_product,
                )
                request.stage = "await_file"
                request.frames = 0
            return

        if request.stage == "await_file":
            if _claim_capture_output(request.target) or request.frames >= CAPTURE_WAIT_FRAMES:
                if request.used != request.original:
                    self._set_render_product_resolution(
                        request.render_product, request.original[0], request.original[1]
                    )
                request.stage = "finish"
                request.frames = 0
            return

        if not request.target.exists():
            request.error = f"capture produced no file at {request.target}"
        else:
            request.result = {
                "path": str(request.target),
                "width": request.used[0],
                "height": request.used[1],
            }
            logger.info("captured %dx%d frame to %s", request.used[0], request.used[1], request.target)
        self._capture_request = None
        request.done.set()

    def _render_product_resolution(self, render_product: str | None) -> tuple[int, int] | None:
        """
        Read a render product's pixel resolution.

        Args:
            render_product: The render product prim path, or ``None``.

        Returns:
            ``(width, height)``, or ``None`` when unavailable.

        """
        if render_product is None:
            return None
        stage = self._stage()
        if stage is None:
            return None
        sdf = importlib.import_module("pxr.Sdf")
        prim = stage.GetPrimAtPath(sdf.Path(render_product))
        if not prim.IsValid():
            return None
        attr = prim.GetAttribute("resolution")
        value = attr.Get() if attr.IsValid() else None
        if value is None:
            return None
        return int(value[0]), int(value[1])

    def _set_render_product_resolution(self, render_product: str | None, width: int, height: int) -> None:
        """
        Resize a render product so a capture can be taken at an arbitrary resolution.

        Resizing the render product rather than the viewport widget is what actually changes the
        captured image: the capture reads the render product, so changing only the widget left a
        4K request producing a 720p file.

        Args:
            render_product: The render product prim path, or ``None``.
            width: Width in pixels.
            height: Height in pixels.

        """
        if render_product is None:
            return
        stage = self._stage()
        if stage is None:
            return
        sdf = importlib.import_module("pxr.Sdf")
        gf = importlib.import_module("pxr.Gf")
        prim = stage.GetPrimAtPath(sdf.Path(render_product))
        if not prim.IsValid():
            return
        attr = prim.GetAttribute("resolution")
        if attr.IsValid():
            attr.Set(gf.Vec2i(int(width), int(height)))

    def _camera_render_product(self) -> str | None:
        """
        Return the camera layer's render product path, or ``None``.

        Read off the graph rather than guessed, because it is created at runtime by
        ``isaac_get_viewport_render_product``. In headless mode the *active* viewport has no
        colour resource to capture -- Kit reports "Capture of LdrColor was requested, but no valid
        resource!" -- while this render product is the one actually rendering, since it is what
        feeds the image topic.

        Returns:
            The render product path, or ``None`` when the camera graph is absent.

        """
        try:
            og = importlib.import_module("omni.graph.core")
        except ImportError:
            return None
        vehicle_id = next(iter(self._config.vehicles))
        mount = self._config.resolved_mount(vehicle_id)
        node_path = f"{mount}/CameraImageExport/isaac_get_viewport_render_product"
        try:
            node = og.Controller.node(node_path)
            value = og.Controller.get(node.get_attribute("outputs:renderProductPath"))
        except Exception:  # noqa: BLE001 - OmniGraph raises bare errors for a missing node
            return None
        text = str(value).strip()
        return text or None

    def _capture_viewport(self, utility: Any) -> Any:  # noqa: ANN401
        """
        Return the viewport showing the vehicle camera, or the active one.

        Args:
            utility: The imported ``omni.kit.viewport.utility`` module.

        Returns:
            A viewport API handle, or ``None``.

        """
        wanted = self._config.sim.viewport_camera
        if wanted:
            vehicle_id = next(iter(self._config.vehicles))
            wanted = render(wanted, instance=vehicle_id, mount=self._config.resolved_mount(vehicle_id))
            try:
                for window in utility.get_viewport_window_instances() or ():
                    api = getattr(window, "viewport_api", None)
                    if api is not None and str(getattr(api, "camera_path", "")) == wanted:
                        return api
            except Exception:  # noqa: BLE001 - the utility raises bare errors with no windows
                pass
        return utility.get_active_viewport()

    def _handle_load_scene(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Swap the open scene at runtime.

        Registered so the call fails with an explanation rather than "method not found", which
        reads like a version mismatch between client and simulator.

        Deferred deliberately, with a concrete reason: swapping the scene means closing a stage
        that Cesium, the ROS bridge and several OmniGraph graphs all hold references to. Every
        stage-lifecycle shortcut tried so far in this project has produced a silent abort rather
        than an error -- the startup warm-up frames exist because of exactly that. Restarting the
        process is currently the safe way to change scene.

        Args:
            params: Ignored.

        Raises:
            NotImplementedError: Always.

        """
        del params
        message = (
            "load_scene is not implemented: swapping the stage at runtime means closing one that "
            "Cesium, the ROS bridge and the action graphs still reference, which aborts the "
            "process rather than erroring. Restart the simulator with a different sim.scene."
        )
        raise NotImplementedError(message)

    def _handle_set_config(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Patch a runtime-mutable config value.

        The patchable set is an explicit allowlist, not a general deep-merge, and it is short by
        design. Most config is applied **once** at composition -- a binding writes it to a prim and
        the value is never consulted again -- so "patching" it would change the config object while
        the stage kept the old value: a silent lie, and worse than refusing. Only fields something
        re-reads while running are accepted; everything else says so and names the restart.

        Args:
            params: ``key`` (a dotted config path) and ``value``.

        Returns:
            The key, the previous value and the new value.

        Raises:
            InvalidParamsError: If the key is missing, not patchable, or the value is the wrong
                type.

        """
        values = _as_mapping(params)
        if "key" not in values or "value" not in values:
            raise InvalidParamsError("set_config requires params.key and params.value")
        key = str(values["key"])
        if key not in PATCHABLE_CONFIG_KEYS:
            allowed = ", ".join(sorted(PATCHABLE_CONFIG_KEYS))
            message = (
                f"{key!r} is not patchable at runtime. Most config is applied once when the stage "
                f"is composed, so changing it later would update the config object while the stage "
                f"kept the old value. Restart the simulator to change it. Patchable now: {allowed}"
            )
            raise InvalidParamsError(message)

        return self._patch_config_key(key, values["value"])

    def _patch_config_key(self, key: str, raw: Any) -> dict[str, Any]:  # noqa: ANN401
        """
        Apply one allowlisted config patch.

        Args:
            key: The dotted config path, already checked against the allowlist.
            raw: The requested value.

        Returns:
            The key, previous and new values.

        Raises:
            InvalidParamsError: If the value cannot be coerced to the field's type.

        """
        vehicle_id = next(iter(self._config.vehicles))
        gimbal = self._config.vehicles[vehicle_id].gimbal

        if key == "gimbal.max_rate_deg_s":
            previous = gimbal.max_rate_deg_s
            new: float | None
            if raw is None:
                new = None
            else:
                try:
                    new = float(raw)
                except (TypeError, ValueError) as exc:
                    raise InvalidParamsError("gimbal.max_rate_deg_s must be a number or null") from exc
                if new <= 0.0:
                    raise InvalidParamsError("gimbal.max_rate_deg_s must be greater than zero, or null for unlimited")
            # Config models are frozen, so the override lives beside them and is consulted first
            # by the slew step. Mutating the model is not an option and would not be safe anyway.
            self._config_overrides[key] = new
            logger.info("patched %s: %s -> %s", key, previous, new)
            return {"key": key, "previous": previous, "new": new}

        message = f"{key!r} is allowlisted but has no handler; this is a bug"
        raise InvalidParamsError(message)

    def _gimbal_max_rate_deg_s(self) -> float | None:
        """
        Return the effective gimbal slew limit, honouring a runtime patch.

        Returns:
            Degrees per second, or ``None`` for unlimited.

        """
        if "gimbal.max_rate_deg_s" in self._config_overrides:
            patched: float | None = self._config_overrides["gimbal.max_rate_deg_s"]
            return patched
        return self._config.vehicles[next(iter(self._config.vehicles))].gimbal.max_rate_deg_s

    def _handle_reset(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Restart the timeline and clear commanded state.

        Scope is deliberately narrow and stated plainly, because "reset" could mean three very
        different things. This resets **simulation time** to zero and drops any commanded gimbal
        target so the gimbal returns to its configured start angles. It does **not** reload the
        stage or re-open the scene: doing that at runtime means closing a stage other subsystems
        hold references to, which is the same territory as the startup crash the warm-up frames
        exist to avoid.

        Resetting simulation time is the useful part for repeatability: every published
        ``header.stamp`` derives from it, so a recording made after a reset starts from zero.

        Args:
            params: Ignored; accepted for forward compatibility.

        Returns:
            What was reset.

        """
        del params

        def _do() -> dict[str, Any]:
            timeline = importlib.import_module("omni.timeline").get_timeline_interface()
            timeline.stop()
            timeline.set_current_time(0.0)
            # `play()` in the same frame as `stop()` does not take -- the timeline needs a frame
            # in between, so a reset left the simulation stopped rather than restarted. Deferred
            # to the next loop iteration instead.
            self._pending_play = True
            return {"timeline": "restarted", "simulation_time": 0.0}

        self._gimbal_target = None
        self._gimbal_current = None
        result: dict[str, Any] = self._on_main_thread(_do)
        logger.info("reset: timeline restarted, gimbal target cleared")
        return {**result, "gimbal": "returned to configured start angles"}

    def _vehicle_from(self, values: dict[str, Any]) -> str:
        """
        Resolve which vehicle a request targets.

        Defaults to the first configured vehicle so single-vehicle callers need not name one, and
        rejects an unknown id by listing the real ones -- a typo would otherwise silently act on
        the wrong aircraft.

        Args:
            values: The request's params mapping.

        Returns:
            A valid vehicle id.

        Raises:
            InvalidParamsError: If a named vehicle does not exist.

        """
        requested = values.get("vehicle")
        if requested is None:
            return next(iter(self._config.vehicles))
        name = str(requested)
        if name not in self._config.vehicles:
            known = ", ".join(sorted(self._config.vehicles))
            message = f"unknown vehicle {name!r}; configured vehicles: {known}"
            raise InvalidParamsError(message)
        return name

    def _handle_set_pose(self, params: dict[str, Any] | list[Any] | None) -> dict[str, Any]:
        """
        Place the vehicle at a geodetic pose.

        Sends the pose through the **same UDP path a real sender uses** rather than writing the
        prim directly. Writing the prim looks simpler but does not work: the pose graph rewrites
        that transform every frame from whatever the receiver last held, so a direct write is
        overwritten within one frame. Feeding the receiver instead means the pose persists exactly
        as if it had arrived over the wire.

        Consequently the usual UDP rule applies: if something else is streaming to the same port,
        the last packet wins and this pose will be replaced by the next one.

        Args:
            params: ``lat_deg``, ``lon_deg``, ``alt_m`` and optional ``roll_deg``/``pitch_deg``/
                ``yaw_deg`` (degrees, NED -- converted to the radians the wire carries).

        Returns:
            The pose sent and the port it was sent to.

        Raises:
            InvalidParamsError: If a required field is missing or not a number.

        """
        values = _as_mapping(params)
        for required in ("lat_deg", "lon_deg", "alt_m"):
            if required not in values:
                raise InvalidParamsError(f"set_pose requires params.{required}")

        pose = {
            "lat_deg": _optional_float(values, "lat_deg", 0.0),
            "lon_deg": _optional_float(values, "lon_deg", 0.0),
            "alt_m": _optional_float(values, "alt_m", 0.0),
            "roll_deg": _optional_float(values, "roll_deg", 0.0),
            "pitch_deg": _optional_float(values, "pitch_deg", 0.0),
            "yaw_deg": _optional_float(values, "yaw_deg", 0.0),
        }
        vehicle_id = self._vehicle_from(values)
        port = self._config.resolved_udp_port(vehicle_id)
        packet = _encode_pose_packet(pose)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(packet, ("127.0.0.1", port))
        logger.info("set_pose sent to udp port %d: %s", port, pose)
        return {"sent": pose, "udp_port": port, "vehicle": vehicle_id}

    def _handle_enable_feature(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Enable a feature layer on the running stage.

        Registered so the call fails honestly. Toggling a layer at runtime means composing
        or removing it on the live stage (mount, resolve bindings, settle), which is not
        implemented yet -- features are selected at launch via config. See docs/roadmap.md.
        """
        raise NotImplementedError(
            "runtime feature toggling is not implemented yet; enable features in config before launch"
        )

    def _handle_disable_feature(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Disable a feature layer on the running stage.

        Registered so the call fails honestly; see :meth:`_handle_enable_feature`.
        """
        raise NotImplementedError(
            "runtime feature toggling is not implemented yet; disable features in config before launch"
        )


__all__ = [
    "SimulationRuntime",
]
