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

from dataclasses import dataclass, field
import importlib
import logging
import os
from pathlib import Path
import queue
import threading
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable

from isaac_core.config import IsaacCoreConfig
from isaac_core.control import ControlServer, InvalidParamsError
from isaac_core.sim.planner import FeaturePlan

logger = logging.getLogger(__name__)

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
        vehicle_id = next(iter(self._config.vehicles))
        if isinstance(params, dict) and "vehicle" in params:
            vehicle_id = str(params["vehicle"])

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
        """Return the current simulation state."""
        return {
            "running": self._running,
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
        """Pause the simulation."""
        if self._app_utils is not None:
            self._app_utils.pause()
        return "paused"

    def _handle_resume(self, params: dict[str, Any] | list[Any] | None) -> str:
        """Resume the simulation."""
        if self._app_utils is not None:
            self._app_utils.play()
        return "resumed"

    def _handle_step(self, params: dict[str, Any] | list[Any] | None) -> str:
        """Advance one step."""
        if self._app_utils is not None:
            self._on_main_thread(self._app_utils.update_app)
        return "stepped"

    def _handle_capture_frame(self, params: dict[str, Any] | list[Any] | None) -> dict[str, str]:
        """
        Capture the current frame to disk.

        Require ``params.path`` (confined under ``output_root``).
        """
        from isaac_core.control.server import confine_path

        if not isinstance(params, dict) or "path" not in params:
            raise InvalidParamsError("capture_frame requires params.path")
        output_root = self._config.sim.control_plane.output_root
        target = confine_path(params["path"], output_root)
        target.parent.mkdir(parents=True, exist_ok=True)
        raise NotImplementedError(
            "capture_frame requires Isaac viewport capture API; " "implement once the viewport is accessible"
        )

    def _handle_set_config(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Patch mutable config values at runtime.

        Only a safe subset is patchable; the full config is frozen.
        """
        raise NotImplementedError(
            "set_config runtime patching is deferred until the mutable " "subset is defined and tested"
        )

    def _handle_reset(self, params: dict[str, Any] | list[Any] | None) -> str:
        """
        Reset the simulation to its initial state.

        Registered so the call fails with a clear, honest error rather than a confusing
        "method not found": the devkit exposes ``SimSession.reset()``, and an unregistered
        method would look like a version mismatch. Runtime reset semantics (timeline, pose,
        or full stage reload) are not settled yet -- see docs/roadmap.md.
        """
        raise NotImplementedError(
            "reset is not implemented yet; restart the simulator, or track the runtime-control item in the roadmap"
        )

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
