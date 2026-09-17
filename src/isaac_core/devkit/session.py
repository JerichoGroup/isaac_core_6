"""Session management for scripting Isaac Sim.

``Sim.attach(host, port)`` connects to an already-running simulator using only the
control-plane contract, so it needs no filesystem knowledge and works against another
machine. ``Sim.launch(...)`` and ``Sim.attach(...)`` return the SAME
:class:`SimSession` type, so a script is identical regardless of how the sim started.

Every operation goes through :class:`~isaac_core.control.ControlClient`; readiness
uses the client's ``wait_until_ready``, NOT log scraping.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
import signal
import subprocess
import tempfile
from typing import Any

from isaac_core.contracts.ports import DEFAULT_CONTROL_PLANE_PORT
from isaac_core.control.client import DEFAULT_CALL_TIMEOUT_S, ControlClient
from isaac_core.control.messages import Method

logger = logging.getLogger(__name__)

# Seconds to let a launched simulator shut down cleanly before killing it.
_SHUTDOWN_TIMEOUT_S = 15.0


def _default_launcher(command: list[str]) -> Any:
    """Spawn the simulator as a child process.

    Injected rather than called directly so tests can drive :meth:`Sim.launch` without
    Isaac Sim present.

    Args:
        command: The full argument vector to execute.

    Returns:
        The spawned ``subprocess.Popen``.

    """

    # New session, so the simulator gets its own process group. `python.sh` does not `exec` --
    # it runs the interpreter as a child -- so killing only the wrapper orphans Isaac Sim and
    # leaves a GUI and a held GPU behind. Killing the group is what actually stops it.
    return subprocess.Popen(command, start_new_session=True)


class _ConfigProxy:
    """Proxy for config inspection and patching via the control plane.

    Provides ``config.get()`` and ``config.patch(...)`` on :class:`SimSession`.
    """

    def __init__(self, client: ControlClient) -> None:
        """Initialise with a connected control client.

        Args:
            client: The control client to use for RPC calls.

        """
        self._client = client

    def get(self) -> Any:
        """Retrieve the current configuration from the running sim.

        Returns:
            The configuration dict from the server.

        """
        return self._client.call(Method.GET_CONFIG.value)

    def patch(self, key: str, value: Any) -> Any:
        """Patch one runtime-mutable config key.

        Takes the key and value explicitly because that is the wire contract. A ``**kwargs``
        signature invited ``patch(gimbal_max_rate_deg_s=10.0)``, which the server rejects: the
        dotted key cannot be spelled as a Python identifier.

        Only a small allowlist is patchable, because most config is applied once when the stage is
        composed. An unpatchable key is refused with the list of those that are.

        Args:
            key: Dotted config path, for example ``"gimbal.max_rate_deg_s"``.
            value: The new value.

        Returns:
            The server's response.

        """
        return self._client.call(Method.SET_CONFIG.value, {"key": key, "value": value})


class SimSession:
    """Handle to a running simulation, returned by both ``Sim.launch()`` and ``Sim.attach()``.

    Exposes lifecycle control (pause, resume, step, reset), feature management,
    config patching, frame capture, and vehicle handles -- all routed through the
    JSON-RPC control plane. Works as a context manager for automatic cleanup, but
    does not require it.
    """

    def __init__(self, client: ControlClient, process: Any = None) -> None:
        """Initialise from an already-connected :class:`ControlClient`.

        Args:
            client: A connected control client.
            process: The simulator process this session started, if any. Only
                :meth:`Sim.launch` passes one; :meth:`Sim.attach` leaves it ``None`` so
                closing the session does not stop somebody else's simulator.

        """
        self._client = client
        self._config = _ConfigProxy(client)
        # Set only by Sim.launch: a session that started the simulator is responsible for
        # stopping it, otherwise a script that raises leaves Isaac Sim running.
        self._process = process

    @property
    def client(self) -> ControlClient:
        """Return the underlying control client."""
        return self._client

    @property
    def config(self) -> _ConfigProxy:
        """Access the config get/patch interface."""
        return self._config

    def state(self) -> Any:
        """Return the running simulation's state.

        Returns:
            A dict describing the run, e.g. ``{"running": True, "scene": "earth",
            "headless": False}``.

        """
        return self._client.call(Method.GET_STATE.value)

    def set_gimbal(
        self,
        *,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
        yaw_deg: float | None = None,
    ) -> Any:
        """Aim the camera gimbal, slewing there if a rate limit is configured.

        Returns as soon as the target is accepted, not when the gimbal arrives: with
        ``gimbal.max_rate_deg_s`` set the move takes real simulated time, and blocking the caller
        for it would make a script look hung. Poll :meth:`get_pose` to observe arrival.

        Any axis left as ``None`` holds its current angle, so a single axis can be nudged without
        restating the other two.

        Args:
            roll_deg: Target roll in degrees, or ``None`` to hold.
            pitch_deg: Target pitch in degrees, or ``None`` to hold.
            yaw_deg: Target yaw in degrees, or ``None`` to hold.

        Returns:
            The accepted target angles in degrees.

        """
        params = {"roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg}
        return self._client.call(Method.SET_GIMBAL.value, {k: v for k, v in params.items() if v is not None})

    def set_pose(
        self,
        *,
        vehicle: str | None = None,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
    ) -> Any:
        """Place the vehicle at a geodetic pose.

        Delivered through the same UDP path a real sender uses, so the usual rule applies: if
        something else is streaming poses to that port, the last packet wins and this one will be
        replaced by the next arrival. Useful for scripted positioning when nothing else is
        sending.

        Args:
            vehicle: Which vehicle to move; defaults to the first configured one.
            lat_deg: Latitude in degrees.
            lon_deg: Longitude in degrees.
            alt_m: Altitude in metres above sea level.
            roll_deg: Roll in degrees, NED.
            pitch_deg: Pitch in degrees, NED.
            yaw_deg: Yaw in degrees, NED.

        Returns:
            The pose sent and the UDP port used.

        """
        params: dict[str, Any] = {}
        if vehicle is not None:
            params["vehicle"] = vehicle
        return self._client.call(
            Method.SET_POSE.value,
            {
                **params,
                "lat_deg": lat_deg,
                "lon_deg": lon_deg,
                "alt_m": alt_m,
                "roll_deg": roll_deg,
                "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg,
            },
        )

    def get_pose(self, *, vehicle: str | None = None) -> Any:
        """Return the live prim transform of a vehicle's moved camera.

        This reads what the pose graph has actually written to the stage -- the only way
        to confirm from outside the process that UDP or ROS pose input is reaching the
        camera. Same data ``isaac-core-inspect`` prints.

        Returns:
            A dict with ``vehicle``, ``prim``, ``translate`` and ``orient`` (or an
            ``error`` entry if the prim could not be read).

        """
        return self._client.call(Method.GET_POSE.value, {"vehicle": vehicle} if vehicle else None)

    def capture_frame(
        self,
        path: str | Path,
        *,
        width: int | None = None,
        height: int | None = None,
    ) -> Any:
        """Capture the camera's current frame to an image file.

        Supplying ``width`` and ``height`` captures at that resolution regardless of the
        viewport's own size, so a high-resolution still can be taken from a small window; the
        viewport is restored afterwards. Both must be given together or neither.

        Args:
            path: Output path, resolved relative to the server's ``output_root``.
            width: Optional capture width in pixels.
            height: Optional capture height in pixels.

        Returns:
            A dict with the ``path`` written and the ``width``/``height`` actually captured.

        """
        params: dict[str, Any] = {"path": str(path)}
        if width is not None:
            params["width"] = width
        if height is not None:
            params["height"] = height
        return self._client.call(Method.CAPTURE_FRAME.value, params)

    def pause(self) -> Any:
        """Pause the simulation.

        Returns:
            The server's response.

        """
        return self._client.call(Method.PAUSE.value)

    def resume(self) -> Any:
        """Resume a paused simulation.

        Returns:
            The server's response.

        """
        return self._client.call(Method.RESUME.value)

    def step(self, count: int = 1) -> Any:
        """Advance the simulation by ``count`` physics steps.

        Args:
            count: Number of steps to advance.

        Returns:
            The server's response.

        """
        return self._client.call(Method.STEP.value, {"count": count})

    def reset(self) -> Any:
        """Reset the simulation to its initial state.

        Returns:
            The server's response.

        """
        return self._client.call(Method.RESET.value)

    def get_capabilities(self) -> Any:
        """Query the simulator's stage capabilities.

        Returns:
            A dict of capabilities from the server.

        """
        return self._client.call(Method.GET_CAPABILITIES.value)

    def wait_until_ready(self, timeout_s: float = 120.0) -> None:
        """Block until the simulator is usable.

        Args:
            timeout_s: Maximum time to wait.

        Raises:
            TimeoutError: If the simulator does not become ready in time.

        """
        self._client.wait_until_ready(timeout_s=timeout_s)

    def close(self) -> None:
        """Disconnect from the control server, and stop the simulator if we started it.

        A session from :meth:`Sim.attach` leaves the simulator alone -- it belongs to
        whoever launched it. A session from :meth:`Sim.launch` owns the process and
        terminates it, so ``with Sim.launch(...)`` cannot leak a running Isaac Sim.
        """
        self._client.close()
        if self._process is None:
            return
        if self._process.poll() is not None:
            return
        logger.info("terminating the simulator we launched")
        self._terminate_process_group()

    def _terminate_process_group(self) -> None:
        """Stop the simulator and every process it spawned.

        Two facts make the obvious ``terminate()`` insufficient, and together they are why a
        finished script could leave a live Isaac Sim GUI behind:

        * Isaac is started with ``installSignalHandlers=0``, so it **ignores SIGTERM**.
        * ``python.sh`` does not ``exec`` the interpreter, so the process we hold is a wrapper and
          Isaac is its child. Signalling only the wrapper orphans Isaac.

        So the whole process group is signalled, and SIGKILL follows if SIGTERM is ignored.
        """

        if self._process is None:
            return
        # An injected test double may have no pid; fall back to signalling the object itself.
        pid = getattr(self._process, "pid", None)
        group = None
        if isinstance(pid, int):
            try:
                group = os.getpgid(pid)
            except (ProcessLookupError, PermissionError, OSError):
                group = None

        def _signal(sig: int) -> None:
            if group is not None:
                try:
                    os.killpg(group, sig)
                    return
                except (ProcessLookupError, PermissionError, OSError):
                    pass
            send = getattr(self._process, "send_signal", None)
            terminate = getattr(self._process, "terminate", None)
            try:
                if callable(send):
                    send(sig)
                elif callable(terminate):
                    terminate()
            except (ProcessLookupError, OSError):
                pass

        _signal(signal.SIGTERM)
        try:
            self._process.wait(timeout=_SHUTDOWN_TIMEOUT_S)
            return
        except Exception:
            logger.warning("simulator ignored SIGTERM (expected); killing the process group")
        _signal(signal.SIGKILL)
        try:
            self._process.wait(timeout=_SHUTDOWN_TIMEOUT_S)
        except Exception:
            logger.warning("simulator process group did not exit after SIGKILL")

    def __enter__(self) -> SimSession:
        """Enter context manager -- return self."""
        return self

    def __exit__(self, *_args: object) -> None:
        """Exit context manager -- close the session."""
        self.close()


class Sim:
    """Facade for obtaining a :class:`SimSession`.

    Provides two entry points:

    ``Sim.attach(host, port)``
        Connect to an already-running simulator. Needs no filesystem knowledge at all --
        only the control-plane contract -- so it works against a remote machine.

    ``Sim.launch(...)``
        Resolve the Isaac Sim install, start a simulator, and wait for it to answer.
        The returned session owns the process and stops it on close.
    """

    @classmethod
    def attach(
        cls,
        host: str = "127.0.0.1",
        port: int = DEFAULT_CONTROL_PLANE_PORT,
        *,
        token: str | None = None,
        timeout_s: float = 30.0,
        call_timeout_s: float = DEFAULT_CALL_TIMEOUT_S,
    ) -> SimSession:
        """Connect to an already-running simulator.

        Waits until the server is ready (accepts connections and responds to
        ``ping``), then returns a :class:`SimSession`. No filesystem path, no repo
        path, no Isaac install knowledge required.

        Args:
            host: Control plane host.
            port: Control plane port.
            token: Authentication token (required for non-loopback hosts).
            timeout_s: Maximum time to wait for the server to become ready.
            call_timeout_s: How long any single call waits for a reply. Raise it for a heavy stage:
                methods that run on the simulator's main thread wait for that thread, and a
                two-vehicle stage was measured holding it for longer than the default.

        Returns:
            A connected :class:`SimSession`.

        Raises:
            TimeoutError: If the server does not become ready within ``timeout_s``.

        """
        client = ControlClient(host=host, port=port, token=token, call_timeout_s=call_timeout_s)
        client.wait_until_ready(timeout_s=timeout_s)
        logger.info("attached to simulation at %s:%d", host, port)
        return SimSession(client)

    @classmethod
    def launch(
        cls,
        *,
        host: str = "127.0.0.1",
        port: int = DEFAULT_CONTROL_PLANE_PORT,
        scene: str = "earth",
        headless: bool = False,
        timeout_s: float = 120.0,
        call_timeout_s: float = DEFAULT_CALL_TIMEOUT_S,
        overrides: dict[str, Any] | None = None,
        launcher: Any = None,
    ) -> SimSession:
        """Launch a new simulator instance and connect to it.

        Resolves the Isaac install (validating it rather than trusting
        the probed candidates), writes the fully resolved config to a temporary file, spawns
        ``python.sh -m isaac_core.sim`` in Isaac's bundled interpreter, and waits for the
        control plane to answer. Port opening is the readiness signal.

        The returned session OWNS the process: closing it stops the simulator. Use
        :meth:`attach` instead to connect to one somebody else started.

        Args:
            host: Control plane host for the new instance.
            port: Control plane port for the new instance.
            scene: Scene to load by logical name.
            headless: Whether to launch headless.
            timeout_s: Maximum time to wait for readiness after spawning.
            call_timeout_s: How long any single call waits for a reply, once running. Raise it for a
                heavy stage: main-thread methods wait for that thread, and a two-vehicle stage was
                measured holding it for longer than the default.
            overrides: Extra dotted config keys, e.g. ``{"features.enabled": ["bbox"]}`` or
                ``{"vehicles.wing.pose_source": "udp"}``. Anything the explicit arguments also
                set wins over these, so ``headless`` cannot be silently contradicted.
            launcher: Optional injected launcher callable. When the runtime exists,
                this will default to the standard process spawner.

        Returns:
            A connected :class:`SimSession`.

        Raises:
            IsaacInstallError: If no usable Isaac Sim install can be found.
            TimeoutError: If the control plane does not answer within ``timeout_s``.

        """

        from isaac_core.config import load
        from isaac_core.config.loader import dump_toml
        from isaac_core.install import IsaacInstall

        # The simulator reads one fully resolved TOML rather than a pile of flags, so
        # precedence is decided in exactly one place. Same mechanism the CLI uses.
        # Explicit arguments are applied last so they cannot be silently contradicted by an
        # override, which would make `headless=True` mean nothing.
        cli_overrides: dict[str, Any] = dict(overrides or {})
        cli_overrides.update(
            {
                "sim.scene": scene,
                "sim.headless": str(headless).lower(),
                "sim.control_plane.port": str(port),
            }
        )
        config = load(cli_overrides=cli_overrides)

        # Resolved after the config, so sim.isaac_sim_path can name an install outside the probed
        # locations. Nothing reads an environment variable, so without this a non-standard install
        # would be reachable only from the CLI.
        configured = config.sim.isaac_sim_path
        install = IsaacInstall.locate(config_path=Path(configured) if configured else None)
        config_dir = Path(tempfile.mkdtemp(prefix="isaac-core-launch-"))
        config_path = config_dir / "resolved.toml"
        config_path.write_text(dump_toml(config), encoding="utf-8")

        command = [str(install.python_path), "-m", "isaac_core.sim", "--config", str(config_path)]
        spawn = launcher if launcher is not None else _default_launcher
        logger.info("launching simulator: %s", " ".join(command))
        process = spawn(command)

        client = ControlClient(host=host, port=port, call_timeout_s=call_timeout_s)
        session = SimSession(client, process=process)
        try:
            session.wait_until_ready(timeout_s=timeout_s)
        except BaseException:
            # BaseException on purpose: KeyboardInterrupt is the common case. Pressing Ctrl-C
            # while "Launching..." was on screen used to leave a fully-booted Isaac Sim running
            # with nothing holding a reference to it, because the session that owns the process
            # was only constructed after the wait succeeded.
            logger.info("startup interrupted or timed out; stopping the simulator we launched")
            session.close()
            raise

        logger.info("launched simulation at %s:%d", host, port)
        return session


__all__ = [
    "Sim",
    "SimSession",
]
