"""
Terminal inspector for a running Isaac Sim session's control plane.

Connects to :class:`~isaac_core.control.client.ControlClient`, queries state,
capabilities, pose and config, and prints them. Optionally polls ``get_pose`` at
an interval with a compact one-line-per-sample table for watching the camera move
in real time.

This is the simplest way to verify pose packets are reaching the stage without
launching the GUI — reading the live prim transform from outside the process.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from isaac_core.contracts.ports import DEFAULT_CONTROL_PLANE_PORT
from isaac_core.control.client import ControlClient

# Tolerance for treating a pose as "still at its default".
_IDENTITY_TOL = 1e-9


def _format_pose(pose: dict[str, Any]) -> str:
    """
    Format a pose dict as a compact one-line table row.

    Args:
        pose: Dict with ``translate`` and ``orient`` keys from ``get_pose``.

    Returns:
        A compact formatted string.

    """
    translate = pose.get("translate", [0, 0, 0])
    orient = pose.get("orient", [0, 0, 0, 0])
    tx, ty, tz = translate[0], translate[1], translate[2]
    ow, ox, oy, oz = orient[0], orient[1], orient[2], orient[3]
    return f"T=({tx:>10.3f}, {ty:>10.3f}, {tz:>10.3f})  Q=({ow:.4f}, {ox:.4f}, {oy:.4f}, {oz:.4f})"


def inspect_once(client: ControlClient) -> None:
    """
    Query and print the simulator's current state.

    Args:
        client: A connected ControlClient.

    """
    print("=== Simulator State ===")
    try:
        state = client.call("get_state")
        print(f"  State: {state}")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_state failed: {exc}")

    print()
    print("=== Capabilities ===")
    try:
        caps = client.call("get_capabilities")
        print(f"  Capabilities: {caps}")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_capabilities failed: {exc}")

    print()
    print("=== Current Pose ===")
    try:
        pose = client.call("get_pose")
        print(f"  {_format_pose(pose)}")
        translate = pose.get("translate") or [0, 0, 0]
        orient = pose.get("orient") or [1, 0, 0, 0]
        if all(abs(v) < _IDENTITY_TOL for v in translate) and abs(orient[0] - 1.0) < _IDENTITY_TOL:
            print("  (pose is at its default -- no UDP/ROS pose received yet; start a sender)")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_pose failed: {exc}")

    print()
    print("=== Config ===")
    try:
        config = client.call("get_config")
        if isinstance(config, dict):
            for key, value in sorted(config.items()):
                print(f"  {key}: {value}")
        else:
            print(f"  {config}")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_config failed: {exc}")

    _print_live_values(client)


def _print_live_values(client: ControlClient) -> None:
    """
    Print the effective values read directly off the running stage.

    This is the inspector's core job: report what is genuinely applied -- the port, rotation
    frame, topic names, camera intrinsics and tileset URLs actually on the prims -- rather
    than echoing config, where those fields are often ``None`` meaning "derive". Backed by
    the ``get_runtime_values`` control method, which reads the live attributes.

    Args:
        client: A connected ControlClient.

    """
    print()
    print("=== Live values (read from the running stage) ===")
    try:
        rv = client.call("get_runtime_values")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_runtime_values failed: {exc}")
        return
    if not isinstance(rv, dict):
        print(f"  {rv}")
        return
    if "error" in rv:
        print(f"  {rv['error']}")
        return

    print(f"  vehicle           : {rv.get('vehicle')}")
    print(f"  mount             : {rv.get('mount')}")
    print(f"  udp_port          : {rv.get('udp_port')}")
    print(f"  rotation_frame    : {rv.get('rotation_frame')}")
    print(f"  enu_reference     : {rv.get('enu_reference')}")
    print(f"  global_pose topic : {rv.get('global_pose_topic')}")
    print(f"  image topic       : {rv.get('image_topic')}")
    camera = rv.get("camera") or {}
    print(f"  camera focalLength: {camera.get('focalLength')}")
    print(f"  camera h/vAperture: {camera.get('horizontalAperture')} / {camera.get('verticalAperture')}")
    tilesets = rv.get("tilesets") or {}
    if tilesets:
        for path, url in tilesets.items():
            print(f"  tileset {path}: {url}")
    else:
        print("  tilesets          : (none found under tilesets_root)")


def poll_pose(client: ControlClient, interval: float, count: int | None = None) -> None:
    """
    Poll ``get_pose`` at an interval, printing a one-line table per sample.

    Args:
        client: A connected ControlClient.
        interval: Seconds between polls.
        count: Number of samples to take; None means poll until interrupted.

    """
    header = f"{'#':>5}  {'Translate (x, y, z)':^36}  {'Quaternion (w, x, y, z)':^32}"
    print(header)
    print("-" * len(header))

    sample = 0
    try:
        while count is None or sample < count:
            try:
                pose = client.call("get_pose")
                sample += 1
                line = f"{sample:>5}  {_format_pose(pose)}"
                print(line, flush=True)
            except Exception as exc:  # noqa: BLE001
                sample += 1
                print(f"{sample:>5}  ERROR: {exc}", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\nStopped after {sample} samples.")


def main(argv: list[str] | None = None) -> int:
    """
    Entry point for ``python -m isaac_core.debug.inspector``.

    Args:
        argv: Command-line arguments.

    Returns:
        Exit code: 0 on success, 1 on connection failure.

    """
    parser = argparse.ArgumentParser(
        prog="isaac-core-inspector",
        description="Inspect a running Isaac Sim session via the control plane.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Control plane host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_CONTROL_PLANE_PORT,
        help=f"Control plane port (default: {DEFAULT_CONTROL_PLANE_PORT})",
    )
    parser.add_argument(
        "--poll",
        type=float,
        default=None,
        metavar="INTERVAL",
        help="Poll get_pose at this interval in seconds (e.g. 0.5)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=None,
        help="Number of poll samples (default: unlimited until Ctrl-C)",
    )

    args = parser.parse_args(argv)

    client = ControlClient(host=args.host, port=args.port)
    try:
        client.connect(timeout=5.0)
    except (ConnectionRefusedError, OSError) as exc:
        print(
            f"ERROR: Cannot connect to {args.host}:{args.port}\n"
            f"  {exc}\n"
            f"\n"
            f"  Is the simulator running? Start it with:\n"
            f"    isaac-core run\n"
            f"  The control plane becomes available once the sim is fully initialised.",
            file=sys.stderr,
        )
        return 1

    try:
        if args.poll is not None:
            poll_pose(client, interval=args.poll, count=args.count)
        else:
            inspect_once(client)
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
