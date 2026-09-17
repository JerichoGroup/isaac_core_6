"""Tkinter GUI for manually driving the camera via UDP pose packets.

Replaces the old ``debugger/udp_sender.py`` and ``debugger/ros_sender.py``. All
state and logic lives in :class:`PoseSenderController` which has no tkinter
dependency and is fully unit-testable. The thin :class:`PoseSenderView` reads and
writes the controller; it imports tkinter lazily so that
``import isaac_core.debug.pose_sender_gui`` succeeds without python3-tk installed.

Key improvements over the old GUI:

- Labels angles ``[deg]`` and converts to radians exactly once at the boundary.
- Derives the packet-structure table from :mod:`isaac_core.contracts.packet` so it
  cannot drift from the real format.
- Delegates all wire-level work to :mod:`isaac_core.protocol` and
  :mod:`isaac_core.devkit.transport`; contains zero struct/checksum code.
- Uses ``time.perf_counter`` with an accumulating deadline instead of
  ``time.sleep(1/rate)`` to prevent drift in the send loop.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Iterator
import contextlib
from dataclasses import dataclass
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import TYPE_CHECKING, Any, Final

if TYPE_CHECKING:
    from collections.abc import Callable
    import tkinter as tk

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.packet import (
    CHECKSUM_OFFSET,
    CHECKSUM_SIZE,
    FIELD_ORDER,
    HEADER,
    HEADER_SIZE,
    PACKET_SIZE,
    PAYLOAD_FORMAT,
)
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import PoseTransport, Ros2PoseTransport, UdpPoseTransport

# Maximum entries kept in the rolling log.
_MAX_LOG_ENTRIES: Final = 50

# Default values matching the shipped scene's Cesium georeference.
_DEFAULT_LAT_DEG: Final = 32.22481
_DEFAULT_LON_DEG: Final = 35.25621
_DEFAULT_ALT_M: Final = 1000.0
_DEFAULT_ROLL_DEG: Final = 0.0
_DEFAULT_PITCH_DEG: Final = 0.0
_DEFAULT_YAW_DEG: Final = 0.0
_DEFAULT_HOST: Final = "127.0.0.1"
_DEFAULT_PORT: Final = 33333
_DEFAULT_RATE_HZ: Final = 10.0

# The two wires a tab can use. Anything else is a pose source the simulator cannot listen to.
_SOURCE_UDP: Final = "udp"
_SOURCE_ROS: Final = "ros"

# Matches the schema default for vehicles.<id>.mavros_namespace.
_DEFAULT_ROS_NAMESPACE: Final = "/mavros"

# Control plane defaults, for reading the simulator's own pose back.
_DEFAULT_CONTROL_PORT: Final = 8760

# Short, because a readback runs on the UI's refresh tick and must never make the window hang.
_READBACK_TIMEOUT_S: Final = 1.0

# A stage transform is (x, y, z).
_TRANSLATE_COMPONENTS: Final = 3

# How long a closing tab waits for its sender thread, so a leaked sender cannot hold the UDP port.
_THREAD_JOIN_S: Final = 2.0

# "Fly there" duration, and how finely the ramp is stepped.
_RAMP_SECONDS: Final = 3.0
_RAMP_STEPS_PER_S: Final = 20

# How long a stream player gets to exit politely before it is killed.
_PLAYER_STOP_S: Final = 3.0

# The readback crosses the wire, so it runs on a slower tick than the widget refresh.
_READBACK_REFRESH_MS: Final = 1000

# Keys that nudge a field, so flying by hand does not mean dragging boxes.
_JOG_KEYS: Final[dict[str, tuple[str, int]]] = {
    "<Left>": ("yaw_deg", -1),
    "<Right>": ("yaw_deg", 1),
    "<Up>": ("pitch_deg", 1),
    "<Down>": ("pitch_deg", -1),
    "<Prior>": ("alt_m", 1),
    "<Next>": ("alt_m", -1),
}

# RTSP defaults, matching how the simulator allocates a stream per vehicle.
_DEFAULT_RTSP_PORT: Final = 8554

# Per-field nudge step sizes.
_STEP_LAT: Final = 0.001
_STEP_LON: Final = 0.001
_STEP_ALT: Final = 5.0
_STEP_ROLL: Final = 5.0
_STEP_PITCH: Final = 5.0
_STEP_YAW: Final = 5.0
_STEP_RATE: Final = 1.0


def _build_packet_table() -> list[str]:
    """Build the packet-structure reference table from live contract constants.

    Derives byte ranges, field names, and types from
    :mod:`isaac_core.contracts.packet` so the table cannot drift from reality.
    """
    # Field sizes: each float64 is 8 bytes.
    field_size = 8
    lines = [
        "Byte   | Field      | Size | Type    | Unit/Notes",
        "-------|------------|------|---------|-------------------",
        f"0      | header[0]  | 1    | uint8   | Fixed: 0x{HEADER[0]:02X}",
        f"1      | header[1]  | 1    | uint8   | Fixed: 0x{HEADER[1]:02X}",
    ]

    # Units for each field, derived from FIELD_ORDER names.
    units = {
        "lat_deg": "degrees",
        "lon_deg": "degrees",
        "alt_m": "metres MSL",
        "roll_r": "RADIANS, NED",
        "pitch_r": "RADIANS, NED",
        "yaw_r": "RADIANS, NED",
    }

    offset = HEADER_SIZE
    for field_name in FIELD_ORDER:
        unit = units.get(field_name, "")
        display_name = field_name.replace("_deg", "").replace("_r", "").replace("_m", "")
        lines.append(f"{offset:<6} | {display_name:<10} | {field_size}    | float64 | {unit}")
        offset += field_size

    lines.append(f"{CHECKSUM_OFFSET:<6} | checksum   | {CHECKSUM_SIZE}    | uint8   | XOR of bytes [2..50)")
    lines.append(f"Total: {PACKET_SIZE} bytes | Format: {PAYLOAD_FORMAT!r}")
    return lines


# How often the window pushes widget values into the controller.
_UI_REFRESH_MS: Final = 200

# Log lines visible in the window at once.
_LOG_LINES_SHOWN: Final = 4

# Pre-built table, available for the view and for tests.
PACKET_TABLE_LINES: Final = _build_packet_table()


class PoseSenderController:
    """All state and behaviour for the pose sender, with no UI dependency.

    Fields are stored in degrees (the user's unit); conversion to radians happens
    exactly once at the boundary when building a :class:`GeodeticPose`.
    """

    def __init__(self) -> None:
        """Initialise with default values."""
        self.lat_deg: float = _DEFAULT_LAT_DEG
        self.lon_deg: float = _DEFAULT_LON_DEG
        self.alt_m: float = _DEFAULT_ALT_M
        self.roll_deg: float = _DEFAULT_ROLL_DEG
        self.pitch_deg: float = _DEFAULT_PITCH_DEG
        self.yaw_deg: float = _DEFAULT_YAW_DEG
        self.host: str = _DEFAULT_HOST
        self.port: int = _DEFAULT_PORT
        self.rate_hz: float = _DEFAULT_RATE_HZ
        # Which wire this tab uses. Both transports satisfy PoseTransport, so nothing else in the
        # controller changes with the source -- which is what lets one window drive a UDP vehicle in
        # one tab and a ROS one in another.
        self.pose_source: str = _SOURCE_UDP
        # Where to read the simulator's own view back from. Separate from the pose target: poses go
        # out over UDP or ROS, while the readback is a control-plane query.
        self.control_host: str = _DEFAULT_HOST
        self.control_port: int = _DEFAULT_CONTROL_PORT
        # Which vehicle to read back, when the simulator has more than one. Empty means "let the
        # simulator pick its first", which is what a single-vehicle setup wants.
        self.vehicle: str = ""
        # Whether the simulator namespaces this vehicle's RTSP mount, which it does only when more
        # than one vehicle is configured. Set when a tab adopts a vehicle from a running simulator.
        self.stream_namespaced: bool = False
        # An empty override means "use the derived guess". A second vehicle, a remote host or a
        # hand-authored mount path all need a way in, and guessing cannot cover those.
        self.rtsp_url_override: str = ""
        self.ros_namespace: str = _DEFAULT_ROS_NAMESPACE
        self.name: str = "vehicle"

        self.locks: dict[str, bool] = {
            "lat_deg": False,
            "lon_deg": False,
            "alt_m": False,
            "roll_deg": False,
            "pitch_deg": False,
            "yaw_deg": False,
            "rate_hz": False,
        }

        self.paused: bool = False
        self.sent: int = 0
        self._log: deque[str] = deque(maxlen=_MAX_LOG_ENTRIES)
        self._transport: PoseTransport | None = None
        self._players: list[subprocess.Popen[bytes]] = []
        self._transport_key: str = ""

    @property
    def log_entries(self) -> list[str]:
        """Return the log ring buffer as a list, oldest first."""
        return list(self._log)

    def reset_to_defaults(self) -> None:
        """Reset all fields to their default values."""
        self.lat_deg = _DEFAULT_LAT_DEG
        self.lon_deg = _DEFAULT_LON_DEG
        self.alt_m = _DEFAULT_ALT_M
        self.roll_deg = _DEFAULT_ROLL_DEG
        self.pitch_deg = _DEFAULT_PITCH_DEG
        self.yaw_deg = _DEFAULT_YAW_DEG
        self.host = _DEFAULT_HOST
        self.port = _DEFAULT_PORT
        self.rate_hz = _DEFAULT_RATE_HZ

    def nudge(self, field: str, direction: int) -> None:
        """Adjust a field by one step in the given direction, unless locked.

        Args:
            field: The field name to nudge (e.g. ``"lat_deg"``).
            direction: ``+1`` or ``-1``.

        """
        if self.locks.get(field, False):
            return

        steps: dict[str, float] = {
            "lat_deg": _STEP_LAT,
            "lon_deg": _STEP_LON,
            "alt_m": _STEP_ALT,
            "roll_deg": _STEP_ROLL,
            "pitch_deg": _STEP_PITCH,
            "yaw_deg": _STEP_YAW,
            "rate_hz": _STEP_RATE,
        }

        step = steps.get(field)
        if step is None:
            return

        current = getattr(self, field, 0.0)
        new_value = current + direction * step
        new_value = self._clamp(field, new_value)
        setattr(self, field, round(new_value, 6))

    def _clamp(self, field: str, value: float) -> float:
        """Clamp a value to its valid range."""
        if field == "lat_deg":
            return max(-90.0, min(90.0, value))
        if field == "lon_deg":
            return max(-180.0, min(180.0, value))
        if field == "rate_hz":
            return max(0.1, value)
        return value

    def build_pose(self) -> GeodeticPose:
        """Build a :class:`GeodeticPose` from current field values.

        Converts degrees to radians at this boundary.
        """
        return GeodeticPose(
            position=Lla(
                lat_deg=self._clamp("lat_deg", self.lat_deg),
                lon_deg=self._clamp("lon_deg", self.lon_deg),
                alt_m=self.alt_m,
            ),
            orientation=Rpy.from_degrees(
                roll_deg=self.roll_deg,
                pitch_deg=self.pitch_deg,
                yaw_deg=self.yaw_deg,
                frame=Frame.NED,
            ),
        )

    def send_once(self) -> bool:
        """Build and send one pose packet. Return True on success, False on failure.

        Skips if paused. Logs the result.
        """
        if self.paused:
            return False

        pose = self.build_pose()

        try:
            transport = self._current_transport()
        except ImportError as exc:
            self._log.append(f"ERROR: {exc}")
            return False

        try:
            transport.send(pose)
        except (OSError, RuntimeError) as exc:
            self._log.append(f"ERROR: {exc}")
            return False

        self.sent += 1
        r_deg, p_deg, y_deg = pose.orientation.to_degrees()
        msg = (
            f"lat={pose.position.lat_deg:.6f} lon={pose.position.lon_deg:.6f} "
            f"alt={pose.position.alt_m:.2f} "
            f"R={r_deg:.1f}° P={p_deg:.1f}° Y={y_deg:.1f}° "
            f"→ {self.target_label}"
        )
        self._log.append(msg)
        return True

    def _current_transport(self) -> PoseTransport:
        """Return a transport matching the current source, rebuilding it if the target changed.

        Returns:
            A live transport for this tab.

        Raises:
            ImportError: If a ROS source is selected on a host without ROS 2.

        """
        if self._transport is not None and self._transport_key == self.target_key:
            return self._transport
        if self._transport is not None:
            self._transport.close()
        built: PoseTransport
        if self.pose_source == _SOURCE_ROS:
            built = Ros2PoseTransport(
                namespace=self.ros_namespace,
                node_name=f"isaac_core_pose_sender_{self.name}",
            )
        else:
            built = UdpPoseTransport(host=self.host, port=self.port)
        self._transport = built
        self._transport_key = self.target_key
        return built

    @property
    def target_key(self) -> str:
        """Return a string identifying where this tab is sending, for change detection."""
        if self.pose_source == _SOURCE_ROS:
            return f"ros:{self.ros_namespace}"
        return f"udp:{self.host}:{self.port}"

    @property
    def target_label(self) -> str:
        """Return the target in a form fit for a status line."""
        if self.pose_source == _SOURCE_ROS:
            return f"ROS 2 {self.ros_namespace}"
        return f"UDP {self.host}:{self.port}"

    def set_pose_source(self, source: str) -> None:
        """Switch this tab between the UDP and ROS 2 wires.

        Args:
            source: ``"udp"`` or ``"ros"``.

        Raises:
            ValueError: If the source is not one of the two.

        """
        if source not in (_SOURCE_UDP, _SOURCE_ROS):
            msg = f"pose_source must be {_SOURCE_UDP!r} or {_SOURCE_ROS!r}, got {source!r}"
            raise ValueError(msg)
        if source == self.pose_source:
            return
        self.pose_source = source
        self._log.append(f"pose source is now {self.target_label}")

    def as_python(self) -> str:
        """Return the current pose as a devkit call, for pasting into a script.

        Returns:
            A ``session.set_pose(...)`` line.

        """
        return (
            "session.set_pose("
            f"lat_deg={self.lat_deg:.6f}, lon_deg={self.lon_deg:.6f}, alt_m={self.alt_m:.2f}, "
            f"roll_deg={self.roll_deg:.1f}, pitch_deg={self.pitch_deg:.1f}, yaw_deg={self.yaw_deg:.1f})"
        )

    def as_toml(self) -> str:
        """Return the current position as a config fragment, for pasting into a config file.

        Returns:
            A ``[geo] enu_reference`` fragment.

        """
        return (
            "[geo]\n"
            f"enu_reference = {{ lat_deg = {self.lat_deg:.6f}, "
            f"lon_deg = {self.lon_deg:.6f}, alt_m = {self.alt_m:.2f} }}\n"
            # There is no config key for a starting attitude, so the angles ride along as a comment
            # rather than being silently dropped from what the user thought they were copying.
            f"# attitude when copied: roll = {self.roll_deg:.1f}, "
            f"pitch = {self.pitch_deg:.1f}, yaw = {self.yaw_deg:.1f} (degrees)"
        )

    def track_player(self, process: subprocess.Popen[bytes]) -> None:
        """Remember a stream player so closing this tab can stop it.

        Args:
            process: The spawned player.

        """
        self._players.append(process)

    def stop_players(self) -> None:
        """Stop every stream player this tab opened."""
        for process in self._players:
            if process.poll() is not None:
                continue
            process.terminate()
            try:
                process.wait(timeout=_PLAYER_STOP_S)
            except subprocess.TimeoutExpired:
                process.kill()
        self._players.clear()

    def log(self, message: str) -> None:
        """Append a line to this tab's log.

        Args:
            message: The line to show.

        """
        self._log.append(message)

    @property
    def rtsp_url(self) -> str:
        """Return the stream URL, either the one the user typed or the derived guess."""
        return self.rtsp_url_override or self.derived_rtsp_url

    @property
    def derived_rtsp_url(self) -> str:
        """Return the RTSP URL this tab's vehicle serves.

        Mirrors how the simulator allocates a stream: the port is ``8554 + vehicle index``, and the
        mount path carries the vehicle name only when more than one vehicle is configured. Guessing
        the single-vehicle form on a swarm sent the button at ``/stream`` while the simulator served
        ``/lead/stream``, so it opened nothing.

        Returns:
            An ``rtsp://`` URL.

        """
        index = max(0, self.port - _DEFAULT_PORT)
        mount = f"/{self.name}/stream" if self.stream_namespaced else "/stream"
        return f"rtsp://{self.host}:{_DEFAULT_RTSP_PORT + index}{mount}"

    def readback(self) -> str:
        """Return what the simulator says the vehicle's pose is, or why it cannot be read.

        This is the difference between "the camera is frozen" and "the camera is frozen and my
        packets are not arriving": one line showing the stage's own transform, beside the values this
        tab is sending.

        Returns:
            A short status line, never raising -- an unreachable simulator is normal, not an error.

        """
        try:
            from isaac_core.control.client import ControlClient
        except ImportError:  # pragma: no cover - control is a hard dependency
            return "control client unavailable"
        client = ControlClient(host=self.control_host, port=self.control_port)
        try:
            client.connect(timeout=_READBACK_TIMEOUT_S)
        except OSError as exc:
            return f"no simulator on {self.control_host}:{self.control_port} ({type(exc).__name__})"
        try:
            # The vehicle is only named when the user has set it to one the simulator knows. A tab's
            # name is a label for the operator; passing it as a vehicle id asked for "vehicle" while
            # the simulator had "drone_0", and the rejection was then reported as "no simulator".
            params = {"vehicle": self.vehicle} if self.vehicle else None
            pose = client.call("get_pose", params)
        except Exception as exc:
            return f"simulator refused the readback: {exc}"[:120]
        finally:
            client.close()

        translate = pose.get("translate") if isinstance(pose, dict) else None
        if not isinstance(translate, list) or len(translate) < _TRANSLATE_COMPONENTS:
            return f"simulator replied without a transform: {pose!r}"[:120]
        return f"stage x={translate[0]:.2f} y={translate[1]:.2f} z={translate[2]:.2f}"

    def readback_vehicle_names(self) -> tuple[str, ...]:
        """Return the vehicle names the simulator has configured, for naming tabs after them.

        Returns:
            The configured vehicle ids, or an empty tuple if the simulator cannot be reached.

        """
        try:
            from isaac_core.control.client import ControlClient

            client = ControlClient(host=self.control_host, port=self.control_port)
            client.connect(timeout=_READBACK_TIMEOUT_S)
            try:
                config = client.call("get_config")
            finally:
                client.close()
        except Exception:
            return ()
        vehicles = config.get("vehicles") if isinstance(config, dict) else None
        return tuple(vehicles) if isinstance(vehicles, dict) else ()

    def close(self) -> None:
        """Release the underlying transport and stop any stream player this tab opened."""
        self.stop_players()
        if self._transport is not None:
            self._transport.close()
            self._transport = None


class SenderTabs:
    """The set of send targets one window drives, one per tab.

    Kept separate from the widgets because the interesting part is not the notebook: it is that a new
    tab lands on the port the simulator will actually be listening on. The simulator allocates a
    vehicle's UDP port as ``base + vehicle index``, so tab *n* defaults to that same arithmetic and a
    two-vehicle swarm needs no thought from the user.
    """

    def __init__(self) -> None:
        """Start with a single tab on the default target."""
        self._controllers: list[PoseSenderController] = []
        self.add()

    def __len__(self) -> int:
        """Return how many tabs there are."""
        return len(self._controllers)

    def __getitem__(self, index: int) -> PoseSenderController:
        """Return the controller for one tab."""
        return self._controllers[index]

    def __iter__(self) -> "Iterator[PoseSenderController]":
        """Iterate the controllers in tab order."""
        return iter(self._controllers)

    @property
    def controllers(self) -> list[PoseSenderController]:
        """Return the controllers in tab order."""
        return list(self._controllers)

    def next_port(self) -> int:
        """Return the UDP port a new tab should default to.

        Returns:
            ``33333 + tab index``, matching how the simulator allocates a vehicle's port.

        """
        return _DEFAULT_PORT + len(self._controllers)

    def next_name(self) -> str:
        """Return the label a new tab should default to.

        Positional rather than vehicle-derived: a tab is a place to type, and mixing the two produced
        labels like ``drone_0, vehicle_2, vehicle_3`` where only the first came from the simulator.
        Which vehicle a tab reads back is a separate field the user can see.

        Returns:
            ``"tab_1"``, ``"tab_2"`` and so on.

        """
        return f"tab_{len(self._controllers) + 1}"

    def add(
        self, *, name: str | None = None, pose_source: str = _SOURCE_UDP, port: int | None = None
    ) -> PoseSenderController:
        """Add a tab and return its controller.

        Args:
            name: Tab name. Defaults to the next ``vehicle`` name.
            pose_source: Which wire the new tab uses.
            port: UDP port. Defaults to the next port in the sequence.

        Returns:
            The new controller.

        """
        controller = PoseSenderController()
        controller.name = name if name is not None else self.next_name()
        controller.port = port if port is not None else self.next_port()
        controller.set_pose_source(pose_source)
        if pose_source == _SOURCE_ROS:
            # A second ROS tab must not publish into the first one's namespace, or both vehicles get
            # both streams and neither operator can tell which is which.
            controller.ros_namespace = (
                f"/mavros_{controller.name}" if len(self._controllers) else _DEFAULT_ROS_NAMESPACE
            )
        self._controllers.append(controller)
        return controller

    def replace_first(self, controller: PoseSenderController) -> None:
        """Use *controller* as the first tab, discarding the default one.

        The CLI configures a controller from its flags before any window exists, so it has to become
        tab one rather than appearing beside a default tab nobody asked for.

        Args:
            controller: The controller to install as the first tab.

        """
        if not controller.name:
            controller.name = "tab_1"
        self._controllers[0].close()
        self._controllers[0] = controller

    def remove(self, index: int) -> None:
        """Close and drop one tab, keeping at least one.

        Args:
            index: Tab index to remove.

        Raises:
            ValueError: If this is the last remaining tab.

        """
        if len(self._controllers) <= 1:
            msg = "the last tab cannot be closed; the window would have nothing to send"
            raise ValueError(msg)
        self._controllers.pop(index).close()

    def close(self) -> None:
        """Close every tab's transport."""
        for controller in self._controllers:
            controller.close()


def _run_sender_loop(controller: PoseSenderController, stop_event: threading.Event) -> None:
    """Background send loop with drift-free pacing.

    Uses ``time.perf_counter`` with an accumulating deadline rather than
    ``time.sleep(1/rate)``. The ``pace()`` helper in ``devkit.transport`` is
    designed for an iterator-based model and does not fit a GUI callback loop
    where rate and field values change between ticks -- hence this direct
    implementation of the same algorithm.
    """
    next_time = time.perf_counter()
    while not stop_event.is_set():
        rate = max(0.1, controller.rate_hz)
        dt = 1.0 / rate
        now = time.perf_counter()
        sleep_duration = next_time - now
        if sleep_duration > 0:
            # Use the stop event's wait for interruptible sleep.
            if stop_event.wait(timeout=sleep_duration):
                break
        controller.send_once()
        next_time += dt
        # Prevent runaway catch-up if the loop fell far behind.
        actual_now = time.perf_counter()
        if next_time < actual_now - 1.0:
            next_time = actual_now


# Field name paired with its display label, in window order.
_FIELD_ROWS: Final[tuple[tuple[str, str], ...]] = (
    ("lat_deg", "Latitude [deg]"),
    ("lon_deg", "Longitude [deg]"),
    ("alt_m", "Altitude [m]"),
    ("roll_deg", "Roll [deg]"),
    ("pitch_deg", "Pitch [deg]"),
    ("yaw_deg", "Yaw [deg]"),
    ("rate_hz", "Send Rate [Hz]"),
)


def _build_field_rows(
    root: "tk.Misc",
    controller: PoseSenderController,
) -> tuple[dict[str, "tk.DoubleVar"], dict[str, "tk.BooleanVar"]]:
    """Build one labelled row per pose field, each with a lock and nudge buttons.

    Args:
        root: Parent widget.
        controller: Controller the widgets read and write.

    Returns:
        The value variables and the lock variables, both keyed by field name.

    """
    import tkinter as tk

    value_vars: dict[str, tk.DoubleVar] = {}
    lock_vars: dict[str, tk.BooleanVar] = {}

    for row, (attr, label) in enumerate(_FIELD_ROWS):
        tk.Label(root, text=label).grid(row=row, column=0, sticky="w", padx=4)

        lock_var = tk.BooleanVar(value=controller.locks.get(attr, False))
        lock_vars[attr] = lock_var
        tk.Checkbutton(root, variable=lock_var, text="lock").grid(row=row, column=1)

        def make_nudge(field: str, direction: int) -> "Callable[[], None]":
            def _nudge() -> None:
                controller.locks[field] = lock_vars[field].get()
                controller.nudge(field, direction)
                value_vars[field].set(getattr(controller, field))

            return _nudge

        tk.Button(root, text="<", width=2, command=make_nudge(attr, -1)).grid(row=row, column=2)
        value_var = tk.DoubleVar(value=getattr(controller, attr))
        value_vars[attr] = value_var
        tk.Entry(root, textvariable=value_var, width=15).grid(row=row, column=3)
        tk.Button(root, text=">", width=2, command=make_nudge(attr, 1)).grid(row=row, column=4)

    return value_vars, lock_vars


def _build_endpoint_rows(
    root: "tk.Misc",
    controller: PoseSenderController,
    row: int,
) -> tuple["tk.StringVar", "tk.IntVar", int]:
    """Build the host and port entry rows.

    Args:
        root: Parent widget.
        controller: Controller supplying the initial values.
        row: First grid row to use.

    Returns:
        The host variable, the port variable, and the next free row.

    """
    import tkinter as tk

    tk.Label(root, text="Host").grid(row=row, column=0, sticky="w", padx=4)
    host_var = tk.StringVar(value=controller.host)
    tk.Entry(root, textvariable=host_var, width=20).grid(row=row, column=2, columnspan=3)

    row += 1
    tk.Label(root, text="Port").grid(row=row, column=0, sticky="w", padx=4)
    port_var = tk.IntVar(value=controller.port)
    tk.Entry(root, textvariable=port_var, width=10).grid(row=row, column=2, columnspan=3)

    return host_var, port_var, row + 1


def _build_log_and_table(root: "tk.Misc", row: int) -> "tk.Text":
    """Build the rolling packet log and the static packet-structure reference table.

    Args:
        root: Parent widget.
        row: First grid row to use.

    Returns:
        The log text widget, which the sync callback refreshes.

    """
    import tkinter as tk

    tk.Label(root, text="Recent packets:").grid(row=row, column=0, columnspan=5, sticky="w")
    row += 1
    log_text = tk.Text(root, height=_LOG_LINES_SHOWN, width=90, state="disabled", bg="#f0f0f0")
    log_text.grid(row=row, column=0, columnspan=5, padx=5, pady=(0, 5))

    row += 1
    tk.Label(root, text="Packet structure (from isaac_core.contracts.packet):").grid(
        row=row, column=0, columnspan=5, sticky="w", pady=(10, 0)
    )
    row += 1
    table = tk.Text(root, height=len(PACKET_TABLE_LINES), width=90, bg="#e8e8e8")
    table.grid(row=row, column=0, columnspan=5, padx=5, pady=(0, 10))
    table.insert("end", "\n".join(PACKET_TABLE_LINES))
    table.config(state="disabled")

    return log_text


def _push_widgets_to_controller(
    controller: PoseSenderController,
    value_vars: dict[str, "tk.DoubleVar"],
    lock_vars: dict[str, "tk.BooleanVar"],
    host_var: "tk.StringVar",
    port_var: "tk.IntVar",
) -> None:
    """Copy the current widget values into the controller.

    A half-typed entry raises ``TclError``, which is expected and ignored: the field keeps
    its last good value until the user finishes typing.

    Args:
        controller: Destination for the values.
        value_vars: Field value variables.
        lock_vars: Field lock variables.
        host_var: Host entry variable.
        port_var: Port entry variable.

    """
    import tkinter as tk

    for attr, var in value_vars.items():
        try:
            setattr(controller, attr, var.get())
        except (tk.TclError, ValueError):
            continue
    for attr, lock_var in lock_vars.items():
        controller.locks[attr] = lock_var.get()
    try:
        controller.host = host_var.get()
        controller.port = port_var.get()
    except (tk.TclError, ValueError):
        return


@dataclass
class _Tab:
    """One tab's controller, widgets and sender thread.

    Held together because closing a tab has to stop its thread and its transport, and a leaked sender
    keeps a UDP port busy for the next tab that tries to use it.
    """

    controller: PoseSenderController
    frame: Any
    value_vars: dict[str, Any]
    lock_vars: dict[str, Any]
    host_var: Any
    port_var: Any
    source_var: Any
    namespace_var: Any
    stream_var: Any
    status_var: Any
    log_text: Any
    stop_event: threading.Event
    thread: threading.Thread

    def stop(self) -> None:
        """Stop this tab's sender and release its transport."""
        self.stop_event.set()
        self.thread.join(timeout=_THREAD_JOIN_S)
        self.controller.close()


def _ramp_controller(controller: PoseSenderController, target: dict[str, float], duration_s: float) -> None:
    """Walk a controller's fields to a target over *duration_s*, so a move is flown not teleported.

    Deliberately animates the controller rather than driving a second transport: the tab's own sender
    loop keeps sending at its own rate, so nothing has to arbitrate between two writers on one port.

    Args:
        controller: The tab's controller.
        target: Field names mapped to their final values.
        duration_s: How long the move takes.

    """
    start = {field: float(getattr(controller, field)) for field in target}
    steps = max(1, int(duration_s * _RAMP_STEPS_PER_S))
    for step in range(1, steps + 1):
        fraction = step / steps
        for field, final in target.items():
            setattr(controller, field, start[field] + fraction * (final - start[field]))
        time.sleep(duration_s / steps)


def _build_tab_buttons(
    frame: Any,
    controller: PoseSenderController,
    value_vars: dict[str, Any],
    host_var: Any,
    port_var: Any,
    row: int,
) -> None:
    """Build one tab's row of actions.

    Args:
        frame: The tab's frame.
        controller: The tab's controller.
        value_vars: The tab's field variables, so Reset and Fly there can read and write them.
        host_var: The tab's host variable.
        port_var: The tab's port variable.
        row: Grid row to place the buttons on.

    """
    import tkinter as tk

    buttons = tk.Frame(frame)
    buttons.grid(row=row, column=0, columnspan=5, sticky="w", pady=4)

    pause_button = tk.Button(buttons, text="Pause")

    def toggle_pause() -> None:
        controller.paused = not controller.paused
        pause_button.config(text="Resume" if controller.paused else "Pause")

    pause_button.config(command=toggle_pause)
    pause_button.pack(side="left", padx=2)

    def reset() -> None:
        controller.reset_to_defaults()
        for attr, var in value_vars.items():
            var.set(getattr(controller, attr))
        host_var.set(controller.host)
        port_var.set(controller.port)

    tk.Button(buttons, text="Reset", command=reset).pack(side="left", padx=2)

    def ramp() -> None:
        # Fly to whatever is typed in the boxes rather than jumping there, on a thread so the window
        # stays responsive while it happens. A jump is not a flight and looks wrong in a recording.
        target = {field: var.get() for field, var in value_vars.items() if field != "rate_hz"}
        threading.Thread(target=_ramp_controller, args=(controller, target, _RAMP_SECONDS), daemon=True).start()

    tk.Button(buttons, text=f"Fly there ({_RAMP_SECONDS:.0f}s)", command=ramp).pack(side="left", padx=2)

    def copy_python() -> None:
        frame.clipboard_clear()
        frame.clipboard_append(controller.as_python())
        controller.log("copied a set_pose call")

    def copy_toml() -> None:
        frame.clipboard_clear()
        frame.clipboard_append(controller.as_toml())
        controller.log("copied a [geo] fragment")

    tk.Button(buttons, text="Copy call", command=copy_python).pack(side="left", padx=2)
    tk.Button(buttons, text="Copy TOML", command=copy_toml).pack(side="left", padx=2)
    tk.Button(buttons, text="View stream", command=lambda: _open_stream(controller)).pack(side="left", padx=2)


def _build_tab(notebook: Any, controller: PoseSenderController) -> _Tab:
    """Build one tab's widgets and start its sender thread.

    Args:
        notebook: The parent notebook.
        controller: The controller this tab drives.

    Returns:
        The assembled tab.

    """
    import tkinter as tk

    frame = tk.Frame(notebook)
    value_vars, lock_vars = _build_field_rows(frame, controller)
    host_var, port_var, row = _build_endpoint_rows(frame, controller, len(_FIELD_ROWS))

    # -- one labelled row each, so nothing is an unexplained box ------------------- #
    tk.Label(frame, text="Topic namespace").grid(row=row, column=0, sticky="w", padx=4)
    namespace_var = tk.StringVar(value=controller.ros_namespace)
    tk.Entry(frame, textvariable=namespace_var, width=24).grid(row=row, column=1, columnspan=3, sticky="w")
    tk.Label(frame, text="(ROS 2 only)", fg="#666").grid(row=row, column=4, sticky="w")
    row += 1

    tk.Label(frame, text="Stream URL").grid(row=row, column=0, sticky="w", padx=4)
    stream_var = tk.StringVar(value=controller.derived_rtsp_url)
    tk.Entry(frame, textvariable=stream_var, width=34).grid(row=row, column=1, columnspan=4, sticky="w")
    row += 1

    tk.Label(frame, text="Pose source").grid(row=row, column=0, sticky="w", padx=4)
    source_var = tk.StringVar(value=controller.pose_source)

    def on_source_change() -> None:
        controller.set_pose_source(source_var.get())

    tk.Radiobutton(frame, text="UDP", variable=source_var, value=_SOURCE_UDP, command=on_source_change).grid(
        row=row, column=1, sticky="w"
    )
    tk.Radiobutton(frame, text="ROS 2", variable=source_var, value=_SOURCE_ROS, command=on_source_change).grid(
        row=row, column=2, sticky="w"
    )
    row += 1

    # -- what the simulator itself reports ---------------------------------------- #
    status_var = tk.StringVar(value="readback: not polled yet")
    tk.Label(frame, textvariable=status_var, anchor="w", fg="#0a3").grid(
        row=row, column=0, columnspan=5, sticky="w", padx=4
    )
    row += 1

    _build_tab_buttons(frame, controller, value_vars, host_var, port_var, row)
    row += 1

    log_text = _build_log_and_table(frame, row)

    # -- jog keys ------------------------------------------------------------------ #
    # Flying with sliders is the slowest part of using this window, so the arrows nudge attitude and
    # PageUp/PageDown nudge altitude, on whichever tab has focus.
    for key, (field, direction) in _JOG_KEYS.items():

        def make_jog(jog_field: str = field, jog_direction: int = direction) -> Callable[[Any], None]:
            def _jog(_event: Any) -> None:
                controller.nudge(jog_field, jog_direction)
                value_vars[jog_field].set(getattr(controller, jog_field))

            return _jog

        frame.bind_all(key, make_jog())

    stop_event = threading.Event()
    thread = threading.Thread(target=_run_sender_loop, args=(controller, stop_event), daemon=True)
    thread.start()

    return _Tab(
        controller=controller,
        frame=frame,
        value_vars=value_vars,
        lock_vars=lock_vars,
        host_var=host_var,
        port_var=port_var,
        source_var=source_var,
        namespace_var=namespace_var,
        stream_var=stream_var,
        status_var=status_var,
        log_text=log_text,
        stop_event=stop_event,
        thread=thread,
    )


def _open_stream(controller: PoseSenderController) -> None:
    """Open this tab's RTSP stream in an external player.

    Launched rather than embedded: decoding H.264 in-window would make ``opencv`` a hard dependency of
    a debug tool, and a player in its own window is what an operator wants beside the sender anyway.
    The point was not needing a second terminal, which this satisfies.

    Args:
        controller: The tab whose stream to open.

    """
    url = controller.rtsp_url
    player = shutil.which("ffplay") or shutil.which("vlc")
    if player is None:
        controller.log("no ffplay or vlc on PATH; install ffmpeg to view the stream")
        return
    try:
        # The player comes from PATH and the URL is either derived from config or typed by the user.
        # NOT start_new_session: a player in its own session outlived the window that opened it, so
        # closing the sender left an orphaned stream on screen with nothing to stop it.
        process = subprocess.Popen(
            [player, url] if player.endswith("vlc") else [player, "-loglevel", "error", url],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        controller.log(f"could not open the stream: {exc}")
        return
    controller.track_player(process)
    controller.log(f"opened {url}")


def _run_until_closed(root: Any, tabs: list[_Tab]) -> None:
    """Run the Tk event loop, closing cleanly on the window button or a single Ctrl-C.

    Tk prints exceptions raised inside callbacks and then carries on, so a ``KeyboardInterrupt``
    arriving during the periodic UI refresh was reported and swallowed -- which is why the first
    Ctrl-C appeared to do nothing and a second was needed to kill the process. Handling ``SIGINT``
    explicitly and scheduling the shutdown onto Tk's own loop makes the first one enough.

    Args:
        root: The Tk root window.
        tabs: Every tab, so each sender thread and transport is released.

    """
    import tkinter as tk

    closing = False

    def on_close() -> None:
        nonlocal closing
        if closing:
            return
        closing = True
        for tab in tabs:
            tab.stop()
        root.quit()

    def on_interrupt(_signal_number: int, _frame: object) -> None:
        root.after(0, on_close)

    root.protocol("WM_DELETE_WINDOW", on_close)
    previous_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, on_interrupt)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()
    finally:
        signal.signal(signal.SIGINT, previous_handler)
        on_close()
        with contextlib.suppress(tk.TclError):
            root.destroy()


def _adopt_simulator_vehicle(controller: PoseSenderController, taken: set[str]) -> None:
    """Point a tab at a real vehicle when the simulator can be asked which exist.

    Without this a tab reads back nothing useful on a swarm: the readback has to name a vehicle the
    simulator knows, and a tab's own label is chosen before anyone asks the simulator anything. When
    no simulator is listening the tab keeps its default label and reads back the first vehicle, which
    is right for the single-vehicle case.

    Args:
        controller: The new tab's controller.
        taken: Vehicle names already claimed by other tabs.

    """
    names = controller.readback_vehicle_names()
    available = [name for name in names if name not in taken]
    if not available:
        return
    # Only the vehicle to read back, not the tab's label: the label is positional so the tab strip
    # reads tab_1, tab_2 rather than a mix of simulator names and placeholders.
    controller.vehicle = available[0]
    # The simulator namespaces topics and RTSP mounts only when there is more than one vehicle.
    controller.stream_namespaced = len(names) > 1
    taken.add(available[0])


def _build_tab_controls(
    root: Any,
    notebook: Any,
    tab_set: SenderTabs,
    tabs: list[_Tab],
    add_tab: Callable[[PoseSenderController], None],
) -> None:
    """Build the add and close buttons that manage the tab set.

    Args:
        root: The window the buttons live in.
        notebook: The notebook whose selection decides which tab closes.
        tab_set: The model of send targets.
        tabs: The live tabs, kept in step with the model.
        add_tab: Callback that builds and shows a tab for a controller.

    """
    import tkinter as tk

    controls = tk.Frame(root)
    controls.grid(row=1, column=0, sticky="w", pady=4)

    def on_add() -> None:
        add_tab(tab_set.add())

    def on_close_tab() -> None:
        index = notebook.index(notebook.select())
        try:
            tab_set.remove(index)
        except ValueError as exc:
            # Refusing to close the last tab is a message, not a crash.
            tabs[index].controller.log(str(exc))
            return
        tabs.pop(index).stop()
        notebook.forget(index)

    tk.Button(controls, text="+ Add tab", command=on_add).pack(side="left", padx=4)
    tk.Button(controls, text="Close tab", command=on_close_tab).pack(side="left", padx=4)


def _enable_tab_renaming(notebook: Any, tabs: list[_Tab]) -> None:
    """Let a double-click on a tab label rename it.

    Args:
        notebook: The notebook whose labels can be renamed.
        tabs: The live tabs, so the controller's name follows the label.

    """
    import tkinter as tk
    from tkinter import simpledialog

    def on_rename(event: Any) -> None:
        try:
            index = notebook.index(f"@{event.x},{event.y}")
        except tk.TclError:
            # Double-clicked somewhere that is not a tab label.
            return
        chosen = simpledialog.askstring(
            "Rename tab", "Tab name:", initialvalue=notebook.tab(index, "text"), parent=notebook
        )
        if not chosen:
            return
        tabs[index].controller.name = chosen
        notebook.tab(index, text=chosen)

    notebook.bind("<Double-Button-1>", on_rename)


def launch_gui(controller: "PoseSenderController | None" = None) -> None:
    """Launch the tkinter GUI: one window, one tab per send target.

    Args:
        controller: Pre-configured controller for the first tab, or ``None`` for a default one.
            Taking it as an argument lets the CLI apply its options before any window exists.

    Raises:
        SystemExit: If tkinter is unavailable, with the apt package to install.

    """
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as exc:
        msg = "tkinter is not available. Install python3-tk:\n  sudo apt install python3-tk\n" f"Original error: {exc}"
        raise SystemExit(msg) from exc

    tab_set = SenderTabs()
    if controller is not None:
        # The CLI already configured a controller, so make it the first tab rather than a second one.
        tab_set.replace_first(controller)

    root = tk.Tk()
    root.title("Isaac Core - Pose Sender")

    notebook = ttk.Notebook(root)
    notebook.grid(row=0, column=0, sticky="nsew")
    tabs: list[_Tab] = []

    claimed: set[str] = set()

    def add_tab(new_controller: PoseSenderController) -> None:
        _adopt_simulator_vehicle(new_controller, claimed)
        tab = _build_tab(notebook, new_controller)
        tabs.append(tab)
        notebook.add(tab.frame, text=new_controller.name)
        notebook.select(len(tabs) - 1)

    for existing in tab_set.controllers:
        add_tab(existing)

    _build_tab_controls(root, notebook, tab_set, tabs, add_tab)
    _enable_tab_renaming(notebook, tabs)

    def sync_ui() -> None:
        for tab in tabs:
            _push_widgets_to_controller(tab.controller, tab.value_vars, tab.lock_vars, tab.host_var, tab.port_var)
            tab.controller.ros_namespace = tab.namespace_var.get() or _DEFAULT_ROS_NAMESPACE
            typed_url = tab.stream_var.get().strip()
            tab.controller.rtsp_url_override = "" if typed_url == tab.controller.derived_rtsp_url else typed_url
            tab.log_text.config(state="normal")
            tab.log_text.delete("1.0", "end")
            tab.log_text.insert("end", "\n".join(tab.controller.log_entries[-_LOG_LINES_SHOWN:]))
            tab.log_text.config(state="disabled")
        root.after(_UI_REFRESH_MS, sync_ui)

    def poll_readback() -> None:
        # On its own slower tick: a readback crosses the wire, so doing it at the UI refresh rate
        # would make the window stutter whenever no simulator is listening.
        for tab in tabs:
            tab.status_var.set(
                f"{tab.controller.target_label} | sent {tab.controller.sent} | {tab.controller.readback()}"
            )
        root.after(_READBACK_REFRESH_MS, poll_readback)

    root.after(_UI_REFRESH_MS, sync_ui)
    root.after(_READBACK_REFRESH_MS, poll_readback)
    _run_until_closed(root, tabs)


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line parser for the pose sender.

    Returns:
        A parser accepting the initial field values and a no-window smoke-test flag.

    """
    parser = argparse.ArgumentParser(
        prog="python -m isaac_core.debug.pose_sender_gui",
        description="Manually drive the simulated camera by sending UDP pose packets.",
    )
    parser.add_argument("--host", default=_DEFAULT_HOST, help="Target host")
    parser.add_argument("--port", type=int, default=_DEFAULT_PORT, help="Target UDP port")
    parser.add_argument("--rate-hz", type=float, default=_DEFAULT_RATE_HZ, help="Send rate")
    parser.add_argument("--lat-deg", type=float, default=_DEFAULT_LAT_DEG)
    parser.add_argument("--lon-deg", type=float, default=_DEFAULT_LON_DEG)
    parser.add_argument("--alt-m", type=float, default=_DEFAULT_ALT_M)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate the configuration and exit without opening a window",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point for ``python -m isaac_core.debug.pose_sender_gui``.

    Parses arguments BEFORE touching tkinter. An earlier version ignored ``argv`` and
    called :func:`launch_gui` unconditionally, so ``--help`` opened a window and blocked
    forever instead of printing usage -- unusable from a script or a CI check.

    Args:
        argv: Command-line arguments; ``None`` means ``sys.argv[1:]``.

    Returns:
        Process exit code.

    """
    args = build_parser().parse_args(argv)

    controller = PoseSenderController()
    controller.host = args.host
    controller.port = args.port
    controller.rate_hz = args.rate_hz
    controller.lat_deg = args.lat_deg
    controller.lon_deg = args.lon_deg
    controller.alt_m = args.alt_m

    if args.check:
        # Exercises everything except the window: field validation, pose construction
        # and encoding. Enough for a smoke test that cannot hang.
        pose = controller.build_pose()
        print(f"target   : {controller.host}:{controller.port} at {controller.rate_hz} Hz")
        print(f"position : {pose.position.as_tuple()}")
        print(f"attitude : {tuple(round(v, 4) for v in pose.orientation.to_degrees())} deg ({pose.frame.value})")
        print("ok: configuration valid, window not opened")
        return 0

    launch_gui(controller)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
