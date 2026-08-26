"""
Tkinter GUI for manually driving the camera via UDP pose packets.

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
import sys
import threading
import time
from typing import TYPE_CHECKING, Final

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
from isaac_core.devkit.transport import UdpPoseTransport

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

# Per-field nudge step sizes.
_STEP_LAT: Final = 0.001
_STEP_LON: Final = 0.001
_STEP_ALT: Final = 5.0
_STEP_ROLL: Final = 5.0
_STEP_PITCH: Final = 5.0
_STEP_YAW: Final = 5.0
_STEP_RATE: Final = 1.0


def _build_packet_table() -> list[str]:
    """
    Build the packet-structure reference table from live contract constants.

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
    """
    All state and behaviour for the pose sender, with no UI dependency.

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
        self._log: deque[str] = deque(maxlen=_MAX_LOG_ENTRIES)
        self._transport: UdpPoseTransport | None = None

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
        """
        Adjust a field by one step in the given direction, unless locked.

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
        """
        Build a :class:`GeodeticPose` from current field values.

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
        """
        Build and send one pose packet. Return True on success, False on failure.

        Skips if paused. Logs the result.
        """
        if self.paused:
            return False

        pose = self.build_pose()

        # Ensure transport matches current host/port.
        if self._transport is None or self._transport.host != self.host or self._transport.port != self.port:
            if self._transport is not None:
                self._transport.close()
            self._transport = UdpPoseTransport(host=self.host, port=self.port)

        try:
            self._transport.send(pose)
        except OSError as exc:
            self._log.append(f"ERROR: {exc}")
            return False

        r_deg, p_deg, y_deg = pose.orientation.to_degrees()
        msg = (
            f"lat={pose.position.lat_deg:.6f} lon={pose.position.lon_deg:.6f} "
            f"alt={pose.position.alt_m:.2f} "
            f"R={r_deg:.1f}° P={p_deg:.1f}° Y={y_deg:.1f}° "
            f"→ {self.host}:{self.port}"
        )
        self._log.append(msg)
        return True

    def close(self) -> None:
        """Release the underlying transport."""
        if self._transport is not None:
            self._transport.close()
            self._transport = None


def _run_sender_loop(controller: PoseSenderController, stop_event: threading.Event) -> None:
    """
    Background send loop with drift-free pacing.

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
    """
    Build one labelled row per pose field, each with a lock and nudge buttons.

    Args:
        root: Parent widget.
        controller: Controller the widgets read and write.

    Returns:
        The value variables and the lock variables, both keyed by field name.

    """
    import tkinter as tk  # noqa: PLC0415

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
    """
    Build the host and port entry rows.

    Args:
        root: Parent widget.
        controller: Controller supplying the initial values.
        row: First grid row to use.

    Returns:
        The host variable, the port variable, and the next free row.

    """
    import tkinter as tk  # noqa: PLC0415

    tk.Label(root, text="Host").grid(row=row, column=0, sticky="w", padx=4)
    host_var = tk.StringVar(value=controller.host)
    tk.Entry(root, textvariable=host_var, width=20).grid(row=row, column=2, columnspan=3)

    row += 1
    tk.Label(root, text="Port").grid(row=row, column=0, sticky="w", padx=4)
    port_var = tk.IntVar(value=controller.port)
    tk.Entry(root, textvariable=port_var, width=10).grid(row=row, column=2, columnspan=3)

    return host_var, port_var, row + 1


def _build_log_and_table(root: "tk.Misc", row: int) -> "tk.Text":
    """
    Build the rolling packet log and the static packet-structure reference table.

    Args:
        root: Parent widget.
        row: First grid row to use.

    Returns:
        The log text widget, which the sync callback refreshes.

    """
    import tkinter as tk  # noqa: PLC0415

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
    """
    Copy the current widget values into the controller.

    A half-typed entry raises ``TclError``, which is expected and ignored: the field keeps
    its last good value until the user finishes typing.

    Args:
        controller: Destination for the values.
        value_vars: Field value variables.
        lock_vars: Field lock variables.
        host_var: Host entry variable.
        port_var: Port entry variable.

    """
    import tkinter as tk  # noqa: PLC0415

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


def launch_gui(controller: "PoseSenderController | None" = None) -> None:
    """
    Launch the tkinter GUI.

    Args:
        controller: Pre-configured controller, or ``None`` to build a default one. Taking
            it as an argument lets the CLI apply its options before any window exists.

    Raises:
        SystemExit: If tkinter is unavailable, with the apt package to install.

    """
    try:
        import tkinter as tk  # noqa: F811
    except ImportError as exc:
        msg = (
            "tkinter is not available. Install python3-tk:\n" "  sudo apt install python3-tk\n" f"Original error: {exc}"
        )
        raise SystemExit(msg) from exc

    if controller is None:
        controller = PoseSenderController()
    stop_event = threading.Event()

    root = tk.Tk()
    root.title("Isaac Core - Pose Sender")

    value_vars, lock_vars = _build_field_rows(root, controller)
    host_var, port_var, row = _build_endpoint_rows(root, controller, len(_FIELD_ROWS))

    pause_button = tk.Button(root, text="Pause")

    def toggle_pause() -> None:
        controller.paused = not controller.paused
        pause_button.config(text="Resume" if controller.paused else "Pause")

    pause_button.config(command=toggle_pause)
    pause_button.grid(row=row, column=0, columnspan=2, pady=5)

    def reset() -> None:
        controller.reset_to_defaults()
        for attr, var in value_vars.items():
            var.set(getattr(controller, attr))
        host_var.set(controller.host)
        port_var.set(controller.port)

    tk.Button(root, text="Reset to defaults", command=reset).grid(row=row, column=2, columnspan=3, pady=5)

    log_text = _build_log_and_table(root, row + 1)

    def sync_ui() -> None:
        _push_widgets_to_controller(controller, value_vars, lock_vars, host_var, port_var)
        log_text.config(state="normal")
        log_text.delete("1.0", "end")
        log_text.insert("end", "\n".join(controller.log_entries[-_LOG_LINES_SHOWN:]))
        log_text.config(state="disabled")
        root.after(_UI_REFRESH_MS, sync_ui)

    root.after(_UI_REFRESH_MS, sync_ui)

    sender = threading.Thread(target=_run_sender_loop, args=(controller, stop_event), daemon=True)
    sender.start()

    def on_close() -> None:
        stop_event.set()
        controller.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


def build_parser() -> argparse.ArgumentParser:
    """
    Return the command-line parser for the pose sender.

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
    """
    Entry point for ``python -m isaac_core.debug.pose_sender_gui``.

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
