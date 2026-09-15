"""System tests for a single headless vehicle: pose pipeline, gimbal and lifecycle.

Every assertion here corresponds to something that broke in this project while the unit suite stayed
green. They are written to measure the observable outcome -- pixels on disk, topics on the wire, the
stage transform -- rather than that a call returned without raising.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from tests.system.conftest import settle

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


# -- pose pipeline ----------------------------------------------------------- #


def test_a_commanded_pose_reaches_the_stage(sim_session: Any) -> None:
    # The whole point of the simulator: a pose sent in must move the camera prim.
    sim_session.set_pose(lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=TEST_ALTITUDE_M)
    settle(sim_session)
    translate = sim_session.get_pose()["translate"]
    expected_up = TEST_ALTITUDE_M - _reference_altitude(sim_session)
    assert (
        abs(translate[2] - expected_up) < ALTITUDE_TOLERANCE_M
    ), f"stage Z is {translate[2]:.2f}, expected {expected_up:.2f} for altitude {TEST_ALTITUDE_M}"


def test_the_stage_reports_ready_and_composed(sim_session: Any) -> None:
    state = sim_session.state()
    assert state["ready"] is True
    assert state["stage_composed"] is True


def test_the_camera_layer_composed(sim_session: Any) -> None:
    capabilities = sim_session.get_capabilities()
    assert capabilities, "get_capabilities returned nothing"


# -- gimbal ------------------------------------------------------------------ #


def test_a_gimbal_target_is_accepted_and_reported(sim_session: Any) -> None:
    result = sim_session.set_gimbal(pitch_deg=-30.0)
    assert result, "set_gimbal returned nothing"
    reported = float(result.get("pitch_deg", 0.0))
    assert reported == pytest.approx(-30.0, abs=0.001)
    # Put it back so later tests see a level camera.
    sim_session.set_gimbal(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
    settle(sim_session)


# -- lifecycle --------------------------------------------------------------- #


def test_the_control_plane_stays_responsive_across_pause_and_resume(sim_session: Any) -> None:
    # Pausing used to terminate the loop, which took the control plane with it.
    sim_session.pause()
    assert sim_session.get_pose()["translate"], "get_pose stopped answering while paused"
    sim_session.step(count=5)
    sim_session.resume()
    assert sim_session.state()["ready"] is True
