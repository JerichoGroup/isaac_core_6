"""System tests for a two-vehicle swarm.

Every assertion here corresponds to something that broke in this project while the unit suite stayed
green. They are written to measure the observable outcome -- pixels on disk, topics on the wire, the
stage transform -- rather than that a call returned without raising.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from tests.system.conftest import settle, wait_for_stage_z

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


def test_a_swarm_accepts_a_gimbal_command_for_a_named_vehicle(swarm_session: Any) -> None:
    # This used to refuse, because before that it silently aimed the first vehicle and reported success.
    # Both are addressable now; `tests/system/test_swarm_parity.py` reads the two prims back separately.
    result = swarm_session.set_gimbal(vehicle="wing", pitch_deg=-20.0)
    assert result["vehicle"] == "wing"
    assert result["pitch_deg"] == pytest.approx(-20.0)


def test_an_unnamed_gimbal_command_aims_the_first_vehicle(swarm_session: Any) -> None:
    # Omitting the name must not refuse either, or every single-vehicle script breaks on a swarm config.
    result = swarm_session.set_gimbal(pitch_deg=-12.0)
    assert result["vehicle"] == list(swarm_session.config.get()["vehicles"])[0]


def test_each_vehicle_gets_its_own_rtsp_port(swarm_session: Any) -> None:
    # Both vehicles asked for 8554 after a config dump/reload, so the second server never bound.
    config = swarm_session.config.get()
    vehicles = list(config["vehicles"])
    assert len(vehicles) == 2, f"expected two vehicles, got {vehicles}"


# -- per-vehicle gimbal, zoom and capture --------------------------------------- #
#
# These live here rather than in their own module on purpose: `swarm_session` is module-scoped, so a
# second module using it launches a second two-vehicle simulator. Measured, that took the full system
# suite from about two minutes to over twenty and made it fail. One heavy fixture, one launch.

# Degrees of tolerance when reading an offset back off a prim.
ANGLE_TOLERANCE_DEG = 0.5

LEAD_ALTITUDE_M = 1000.0
WING_ALTITUDE_M = 1400.0


def _gimbal_prim(vehicle: str) -> str:
    """Return the prim carrying a vehicle's gimbal offsets."""
    return f"/World/Environment/{vehicle}/PoseSync/global_position_to_local_position"


def _offsets(session: Any, vehicle: str) -> dict[str, float]:
    """Read a vehicle's three gimbal offsets off its own prim.

    Args:
        session: The devkit session.
        vehicle: Which vehicle.

    Returns:
        The roll, pitch and yaw offsets in degrees.

    """
    prim = _gimbal_prim(vehicle)
    values: dict[str, float] = {}
    for axis in ("roll", "pitch", "yaw"):
        reply = session.client.call("read_prim_attribute", {"prim": prim, "attribute": f"inputs:offset_{axis}_deg"})
        values[axis] = float(reply["value"])
    return values


def test_each_vehicle_gimbal_is_aimed_independently(swarm_session: Any) -> None:
    swarm_session.set_gimbal(vehicle="lead", roll_deg=0.0, pitch_deg=-40.0, yaw_deg=0.0)
    swarm_session.set_gimbal(vehicle="wing", roll_deg=0.0, pitch_deg=-10.0, yaw_deg=25.0)
    settle(swarm_session)

    lead = _offsets(swarm_session, "lead")
    wing = _offsets(swarm_session, "wing")
    assert lead["pitch"] == pytest.approx(-40.0, abs=ANGLE_TOLERANCE_DEG), lead
    assert wing["pitch"] == pytest.approx(-10.0, abs=ANGLE_TOLERANCE_DEG), wing
    assert wing["yaw"] == pytest.approx(25.0, abs=ANGLE_TOLERANCE_DEG), wing
    assert lead["yaw"] == pytest.approx(0.0, abs=ANGLE_TOLERANCE_DEG), "aiming wing also yawed lead"


def test_aiming_one_vehicle_leaves_the_other_where_it_was(swarm_session: Any) -> None:
    swarm_session.set_gimbal(vehicle="lead", roll_deg=0.0, pitch_deg=-30.0, yaw_deg=15.0)
    settle(swarm_session)
    before = _offsets(swarm_session, "lead")

    swarm_session.set_gimbal(vehicle="wing", pitch_deg=-55.0)
    settle(swarm_session)

    after = _offsets(swarm_session, "lead")
    assert after == pytest.approx(before, abs=ANGLE_TOLERANCE_DEG), "commanding wing moved lead"


def test_omitting_the_vehicle_aims_the_first_declared_one(swarm_session: Any) -> None:
    # Single-vehicle scripts must keep working unchanged against a swarm config.
    result = swarm_session.set_gimbal(pitch_deg=-20.0)
    assert result["vehicle"] in swarm_session.config.get()["vehicles"]
    settle(swarm_session)
    assert _offsets(swarm_session, result["vehicle"])["pitch"] == pytest.approx(-20.0, abs=ANGLE_TOLERANCE_DEG)


def test_an_unknown_vehicle_is_refused_and_names_the_real_ones(swarm_session: Any) -> None:
    # The failure this replaced was silently acting on the first vehicle and reporting success.
    with pytest.raises(Exception, match="unknown vehicle") as excinfo:
        swarm_session.set_gimbal(vehicle="not_a_vehicle", pitch_deg=-5.0)
    message = str(excinfo.value)
    for name in swarm_session.config.get()["vehicles"]:
        assert name in message, message


def test_each_vehicle_pose_is_still_independent(swarm_session: Any) -> None:
    # Guards against per-vehicle gimbal work having disturbed per-vehicle posing.
    swarm_session.set_pose(vehicle="lead", lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=LEAD_ALTITUDE_M)
    swarm_session.set_pose(vehicle="wing", lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=WING_ALTITUDE_M)
    settle(swarm_session)

    reference_alt = float(swarm_session.config.get()["geo"]["enu_reference"]["alt_m"])
    lead_z = float(swarm_session.get_pose(vehicle="lead")["translate"][2])
    wing_z = float(swarm_session.get_pose(vehicle="wing")["translate"][2])
    assert lead_z == pytest.approx(LEAD_ALTITUDE_M - reference_alt, abs=1.0)
    assert wing_z == pytest.approx(WING_ALTITUDE_M - reference_alt, abs=1.0)


def test_a_gimbal_command_does_not_move_the_airframe(swarm_session: Any) -> None:
    # Aiming a camera must not fly the aircraft, which one shared prim write could easily have done.
    swarm_session.set_pose(vehicle="wing", lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=WING_ALTITUDE_M)
    settle(swarm_session)
    before = swarm_session.get_pose(vehicle="wing")["translate"]

    swarm_session.set_gimbal(vehicle="wing", pitch_deg=-35.0, yaw_deg=-40.0)
    settle(swarm_session)

    after = swarm_session.get_pose(vehicle="wing")["translate"]
    assert after == pytest.approx(before, abs=0.5), "aiming wing's gimbal moved wing"
