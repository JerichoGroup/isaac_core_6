"""System tests for a two-vehicle swarm.

Every assertion here corresponds to something that broke in this project while the unit suite stayed
green. They are written to measure the observable outcome -- pixels on disk, topics on the wire, the
stage transform -- rather than that a call returned without raising.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from tests.system.conftest import wait_for_stage_z

# Metres of tolerance when comparing a commanded altitude against the stage transform.
ALTITUDE_TOLERANCE_M = 1.0

# The scene's reference point, and an altitude comfortably above the terrain.
REFERENCE_LAT = 32.22481
REFERENCE_LON = 35.25621
TEST_ALTITUDE_M = 1500.0


def _reference_altitude(session: Any) -> float:
    """Return the ENU reference altitude the running simulator resolved."""
    return float(session.config.get()["geo"]["enu_reference"]["alt_m"])


def _png_size(path: Path) -> tuple[int, int]:
    """Return a PNG's pixel dimensions by reading its IHDR chunk.

    Deliberately not using an image library: the point is to measure what is on disk without
    depending on Pillow or OpenCV being installed.

    Args:
        path: The PNG file.

    Returns:
        ``(width, height)`` in pixels.

    """
    header = path.read_bytes()[:33]
    assert header[:8] == b"\x89PNG\r\n\x1a\n", f"{path} is not a PNG"
    width = int.from_bytes(header[16:20], "big")
    height = int.from_bytes(header[20:24], "big")
    return width, height


# -- swarm ------------------------------------------------------------------- #


def test_each_vehicle_has_its_own_prim_and_altitude(swarm_session: Any) -> None:
    reference_alt = _reference_altitude(swarm_session)

    def send_lead() -> None:
        swarm_session.set_pose(vehicle="lead", lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=1500.0)

    def send_wing() -> None:
        swarm_session.set_pose(vehicle="wing", lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=900.0)

    lead_z = wait_for_stage_z(swarm_session, 1500.0 - reference_alt, vehicle="lead", resend=send_lead)
    wing_z = wait_for_stage_z(swarm_session, 900.0 - reference_alt, vehicle="wing", resend=send_wing)
    lead = swarm_session.get_pose(vehicle="lead")
    wing = swarm_session.get_pose(vehicle="wing")
    assert lead["prim"] != wing["prim"], "both vehicles report the same prim"
    separation = abs(lead_z - wing_z)
    assert abs(separation - 600.0) < ALTITUDE_TOLERANCE_M, f"commanded 600 m of separation, measured {separation:.2f} m"


def test_a_swarm_refuses_single_vehicle_only_commands(swarm_session: Any) -> None:
    # Silently aiming the first vehicle is worse than refusing, which is what it used to do.
    with pytest.raises(Exception, match="single vehicle|single-vehicle|supports a single"):
        swarm_session.set_gimbal(pitch_deg=-20.0)


def test_each_vehicle_gets_its_own_rtsp_port(swarm_session: Any) -> None:
    # Both vehicles asked for 8554 after a config dump/reload, so the second server never bound.
    config = swarm_session.config.get()
    vehicles = list(config["vehicles"])
    assert len(vehicles) == 2, f"expected two vehicles, got {vehicles}"
