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

    def start(self) -> None:
        """
        Create the SimulationApp, open the stage, and start the control server.

        Call this AFTER the SimulationApp launch config has been set (in
        ``__main__``). The SimulationApp import triggers Kit initialisation.
        """
        isaacsim = importlib.import_module("isaacsim")
        sim_app_cls = getattr(isaacsim, "SimulationApp")

        launch_config = {
            "headless": self._config.sim.headless,
            "width": 1280,
            "height": 720,
        }
        self._app = sim_app_cls(launch_config)

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

    def open_stage(self, scene_path: Path, layer_search_paths: tuple[Path, ...]) -> None:
        """
        Open the scene and compose feature layers.

        Args:
            scene_path: Path to the base scene USD.
            layer_search_paths: Search paths for layer USD files.

        """
        from isaac_core.sim.composer import compose_stage

        compose_stage(self._config, self._plan, scene_path, layer_search_paths)

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


__all__ = [
    "SimulationRuntime",
]
