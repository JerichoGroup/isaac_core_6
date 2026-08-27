#!/usr/bin/env python3
"""
Send pose packets to a running Isaac Sim, for a first end-to-end test.

Runs on the *system* Python, not Isaac's, and needs no ROS 2. It exercises the whole
pure kernel -- ``isaac_core.vehicle`` generates the trajectory,
``isaac_core.protocol`` encodes the packets and ``isaac_core.devkit.transport``
ships them -- so if the camera moves, a large part of the stack is proven at once.

Usage, with the simulation playing and a ``camera_udp`` layer composed::

    PYTHONPATH=src ./scripts/send_test_pose.py hold
    PYTHONPATH=src ./scripts/send_test_pose.py orbit --radius-m 800 --duration-s 60
    PYTHONPATH=src ./scripts/send_test_pose.py path

Defaults match ``usd/scenes/earth.usda``'s Cesium georeference origin and the
``enu_reference`` default baked into the math node, so the aircraft starts over the
terrain rather than somewhere in the ocean.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
import math
import sys

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import UdpPoseTransport, pace
from isaac_core.vehicle import HoldTrajectory, OrbitTrajectory, PathTrajectory

# The Cesium georeference origin of usd/scenes/earth.usda, which is also the
# enu_reference default in GlobalPositionToLocalPosition. Starting here means the
# camera is over the terrain on the very first packet.
DEFAULT_LAT_DEG = 32.22481
DEFAULT_LON_DEG = 35.25621
DEFAULT_ALT_M = 1000.0

# Matches the previous generation's senders, and comfortably above the render rate.
DEFAULT_RATE_HZ = 30.0


def _hold(args: argparse.Namespace) -> Iterator[GeodeticPose]:
    """Build a stationary pose stream, the simplest possible smoke test."""
    pose = GeodeticPose(
        position=Lla(args.lat_deg, args.lon_deg, args.alt_m),
        # Wire angles are NED; UdpToGlobalPosition converts to ENU inside Isaac.
        orientation=Rpy.from_degrees(args.roll_deg, args.pitch_deg, args.yaw_deg, Frame.NED),
    )
    print(f"holding at {args.lat_deg}, {args.lon_deg}, {args.alt_m} m")
    return HoldTrajectory(pose).poses(args.rate_hz)


def _orbit(args: argparse.Namespace) -> Iterator[GeodeticPose]:
    """Build a circular orbit, which makes motion obvious in the viewport."""
    print(
        f"orbiting {args.lat_deg}, {args.lon_deg} at radius {args.radius_m} m, "
        f"height {args.alt_m} m, {args.speed_mps} m/s for {args.duration_s} s"
    )
    return OrbitTrajectory(
        center_lat_deg=args.lat_deg,
        center_lon_deg=args.lon_deg,
        radius_m=args.radius_m,
        height_m=args.alt_m,
        speed_mps=args.speed_mps,
        orbit_duration_s=args.duration_s,
        roll_r=math.radians(args.roll_deg),
        pitch_r=math.radians(args.pitch_deg),
    ).poses(args.rate_hz)


def _path(args: argparse.Namespace) -> Iterator[GeodeticPose]:
    """Build a short square circuit around the reference point."""
    offset = args.radius_m / 111_000.0  # rough degrees per metre, fine for a test
    waypoints = (
        Lla(args.lat_deg, args.lon_deg, args.alt_m),
        Lla(args.lat_deg + offset, args.lon_deg, args.alt_m),
        Lla(args.lat_deg + offset, args.lon_deg + offset, args.alt_m),
        Lla(args.lat_deg, args.lon_deg + offset, args.alt_m),
        Lla(args.lat_deg, args.lon_deg, args.alt_m),
    )
    print(f"flying a {len(waypoints)}-point circuit at {args.speed_mps} m/s")
    return PathTrajectory(
        waypoints=waypoints,
        speed_mps=args.speed_mps,
        roll_r=math.radians(args.roll_deg),
        pitch_r=math.radians(args.pitch_deg),
    ).poses(args.rate_hz)


_MODES = {"hold": _hold, "orbit": _orbit, "path": _path}


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line parser."""
    parser = argparse.ArgumentParser(
        description="Send pose packets to a running Isaac Sim for an end-to-end test.",
    )
    parser.add_argument("mode", choices=sorted(_MODES), help="Trajectory to fly")
    parser.add_argument("--host", default="127.0.0.1", help="Target host")
    parser.add_argument("--port", type=int, default=DEFAULT_POSE_UDP_PORT, help="Target UDP port")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--lat-deg", type=float, default=DEFAULT_LAT_DEG)
    parser.add_argument("--lon-deg", type=float, default=DEFAULT_LON_DEG)
    parser.add_argument("--alt-m", type=float, default=DEFAULT_ALT_M)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=-30.0, help="Negative looks down")
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--radius-m", type=float, default=500.0)
    parser.add_argument("--speed-mps", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float, default=60.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    """
    Send the selected trajectory until it ends or the user interrupts.

    Args:
        argv: Command-line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        Process exit code.

    """
    args = build_parser().parse_args(argv)

    poses = _MODES[args.mode](args)
    transport = UdpPoseTransport(host=args.host, port=args.port)

    print(f"sending to {args.host}:{args.port} at {args.rate_hz} Hz — Ctrl-C to stop")
    try:
        sent = pace(poses, transport, args.rate_hz)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 0
    else:
        print(f"sent {sent} packets")
        return 0
    finally:
        close = getattr(transport, "close", None)
        if callable(close):
            close()


if __name__ == "__main__":
    sys.exit(main())
