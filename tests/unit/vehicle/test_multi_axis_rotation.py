"""Rotation tests for attitudes with two or more non-zero angles.

Every rotation test before this one turned a single axis from rest, and single-axis attitudes decompose
identically in the world and body frames. That is why a real bug survived a green suite: composing a
world yaw onto a pitched airframe and decomposing in the body frame reported roll 30, pitch 0 where the
truth was roll 0, pitch -30, and a user watching the horizon tip called it "a hot mess".

Two separate defects are pinned here.

*Expression.* The wire carries Euler angles and the simulator rebuilds them with the vehicle's
configured ``rotation_frame``, so every conversion in the motion stack has to use that same frame. It
used to use the library default regardless.

*Intent.* In the world convention the angles compose as yaw about world up, then pitch, then roll, so
"pitch by 30" means moving the pitch term. Rotating about the fixed world Y axis instead is a different
operation: while yawed 90 degrees it rolls the airframe, which is geometrically true and never what the
caller asked for.

*Path.* Endpoints being right is not enough. SLERP takes the shortest great-circle path in rotation
space, which swung roll up to 45 degrees mid-turn and back, so the horizon visibly tipped while every
endpoint assertion passed.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.pose import Lla
from isaac_core.devkit import PoseBot
from isaac_core.devkit.transport import FakePoseTransport
from isaac_core.geo.rotations import euler_to_matrix, matrix_to_euler
from isaac_core.vehicle import VehicleState

_START = Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1000.0)


def _bot() -> tuple[PoseBot, FakePoseTransport]:
    """Return a bot that records instead of sending and never sleeps."""
    transport = FakePoseTransport()
    return PoseBot(transport=transport, start=_START, sleep=lambda _seconds: None), transport


def _degrees(bot: PoseBot) -> tuple[float, float, float]:
    """Return the bot's attitude in degrees."""
    orientation = bot.pose.orientation
    return (
        math.degrees(orientation.roll_r),
        math.degrees(orientation.pitch_r),
        math.degrees(orientation.yaw_r),
    )


# -- expression: a state reads back the angles it was built from ---------------- #


@pytest.mark.parametrize("frame", [RotationFrame.WORLD, RotationFrame.BODY])
@pytest.mark.parametrize(
    ("roll_deg", "pitch_deg", "yaw_deg"),
    [(0.0, -30.0, 90.0), (20.0, -30.0, 0.0), (20.0, 0.0, 90.0), (15.0, -25.0, 135.0)],
)
def test_a_state_round_trips_a_multi_axis_attitude(
    frame: RotationFrame, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> None:
    state = VehicleState.from_angles(
        0.0, 0.0, 0.0, math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg), frame
    )
    orientation = state.to_pose().orientation
    assert math.degrees(orientation.roll_r) == pytest.approx(roll_deg, abs=1e-9)
    assert math.degrees(orientation.pitch_r) == pytest.approx(pitch_deg, abs=1e-9)
    assert math.degrees(orientation.yaw_r) == pytest.approx(yaw_deg, abs=1e-9)


def test_the_two_frames_disagree_on_a_multi_axis_attitude() -> None:
    # The reason the frame has to be carried rather than assumed. If these agreed, none of this would
    # matter and the original bug would have been harmless.
    matrix = euler_to_matrix(0.0, math.radians(-30.0), math.radians(90.0), RotationFrame.WORLD)
    as_world = matrix_to_euler(matrix, RotationFrame.WORLD)
    as_body = matrix_to_euler(matrix, RotationFrame.BODY)
    assert as_world != as_body
    assert math.degrees(as_world[1]) == pytest.approx(-30.0, abs=1e-9), "world reads the pitch we built"
    assert math.degrees(as_body[0]) == pytest.approx(30.0, abs=1e-9), "body reads it as roll instead"


def test_a_single_axis_attitude_agrees_in_both_frames() -> None:
    # Why the bug survived: every earlier test turned one axis from rest.
    matrix = euler_to_matrix(0.0, math.radians(-30.0), 0.0, RotationFrame.WORLD)
    assert matrix_to_euler(matrix, RotationFrame.WORLD) == pytest.approx(matrix_to_euler(matrix, RotationFrame.BODY))


# -- intent: each turn moves its own axis and nothing else --------------------- #


def test_yawing_a_pitched_airframe_holds_the_pitch() -> None:
    # The reported case: pitch down 30, then circle. The horizon must stay where the pitch put it.
    bot, _ = _bot()
    bot.turn_pitch(-30.0, duration_s=1.0)
    for expected_yaw in (90.0, 180.0, -90.0, 0.0):
        bot.turn_yaw(90.0, duration_s=1.0)
        roll_deg, pitch_deg, yaw_deg = _degrees(bot)
        assert roll_deg == pytest.approx(0.0, abs=1e-6), f"yaw leaked into roll: {roll_deg}"
        assert pitch_deg == pytest.approx(-30.0, abs=1e-6), f"pitch drifted: {pitch_deg}"
        # Compared as an angle: 180 and -180 are the same heading, and normalisation may return either.
        assert math.cos(math.radians(yaw_deg - expected_yaw)) == pytest.approx(
            1.0, abs=1e-9
        ), f"yaw {yaw_deg} is not {expected_yaw}"


def test_pitching_a_yawed_airframe_holds_the_yaw_and_does_not_roll() -> None:
    # Rotating about the fixed world Y axis here would put -30 into roll, which is what it did.
    bot, _ = _bot()
    bot.turn_yaw(90.0, duration_s=1.0)
    bot.turn_pitch(-30.0, duration_s=1.0)
    roll_deg, pitch_deg, yaw_deg = _degrees(bot)
    assert roll_deg == pytest.approx(0.0, abs=1e-6), f"pitch leaked into roll: {roll_deg}"
    assert pitch_deg == pytest.approx(-30.0, abs=1e-6)
    assert yaw_deg == pytest.approx(90.0, abs=1e-6)


def test_rolling_a_yawed_and_pitched_airframe_holds_both() -> None:
    bot, _ = _bot()
    bot.turn_yaw(90.0, duration_s=1.0)
    bot.turn_pitch(-30.0, duration_s=1.0)
    bot.turn_roll(20.0, duration_s=1.0)
    roll_deg, pitch_deg, yaw_deg = _degrees(bot)
    assert roll_deg == pytest.approx(20.0, abs=1e-6)
    assert pitch_deg == pytest.approx(-30.0, abs=1e-6)
    assert yaw_deg == pytest.approx(90.0, abs=1e-6)


@pytest.mark.parametrize("order", ["rpy", "ryp", "pry", "pyr", "yrp", "ypr"])
def test_the_three_turns_commute_in_the_world_frame(order: str) -> None:
    # Independent axes must compose to the same attitude whatever order they are applied in. This is
    # the property that makes the world frame usable, and it did not hold.
    moves = {"r": (20.0, "turn_roll"), "p": (-30.0, "turn_pitch"), "y": (90.0, "turn_yaw")}
    bot, _ = _bot()
    for key in order:
        delta, method = moves[key]
        getattr(bot, method)(delta, duration_s=1.0)
    roll_deg, pitch_deg, yaw_deg = _degrees(bot)
    assert roll_deg == pytest.approx(20.0, abs=1e-6), f"order {order} gave roll {roll_deg}"
    assert pitch_deg == pytest.approx(-30.0, abs=1e-6), f"order {order} gave pitch {pitch_deg}"
    assert yaw_deg == pytest.approx(90.0, abs=1e-6), f"order {order} gave yaw {yaw_deg}"


# -- path: the axes not asked for stay still for the whole turn ---------------- #


def _spans(transport: FakePoseTransport) -> dict[str, tuple[float, float]]:
    """Return the min and max of each angle across a recorded turn, in degrees."""
    return {
        "roll": (
            min(math.degrees(p.orientation.roll_r) for p in transport.history),
            max(math.degrees(p.orientation.roll_r) for p in transport.history),
        ),
        "pitch": (
            min(math.degrees(p.orientation.pitch_r) for p in transport.history),
            max(math.degrees(p.orientation.pitch_r) for p in transport.history),
        ),
    }


def test_yawing_never_tips_the_horizon_mid_turn() -> None:
    bot, transport = _bot()
    bot.turn_pitch(-30.0, duration_s=1.0)
    transport.history.clear()
    bot.turn_yaw(180.0, duration_s=2.0)
    spans = _spans(transport)
    assert spans["roll"] == pytest.approx((0.0, 0.0), abs=1e-6), f"roll moved during a yaw: {spans['roll']}"
    assert spans["pitch"] == pytest.approx((-30.0, -30.0), abs=1e-6), f"pitch moved: {spans['pitch']}"


def test_turn_to_point_holds_roll_for_the_whole_turn() -> None:
    # It held roll at the endpoints while swinging it up to 45 degrees on the way, so the camera rolled
    # and came back and every endpoint assertion passed.
    bot, transport = _bot()
    bot.turn_roll(20.0, duration_s=1.0)
    transport.history.clear()
    bot.turn_to_point(32.2247, 35.2563, 900.0, duration_s=2.0)
    low, high = _spans(transport)["roll"]
    assert low == pytest.approx(20.0, abs=1e-6), f"roll dipped to {low}"
    assert high == pytest.approx(20.0, abs=1e-6), f"roll rose to {high}"


def test_turn_to_point_after_a_multi_axis_attitude_still_holds_roll() -> None:
    bot, transport = _bot()
    bot.turn_roll(20.0, duration_s=1.0)
    bot.turn_pitch(-30.0, duration_s=1.0)
    bot.turn_yaw(45.0, duration_s=1.0)
    transport.history.clear()
    bot.turn_to_point(32.2247, 35.2563, 900.0, duration_s=2.0)
    low, high = _spans(transport)["roll"]
    assert (low, high) == pytest.approx((20.0, 20.0), abs=1e-6)


def test_yaw_takes_the_short_way_round() -> None:
    # Interpolating 170 to -170 the long way sweeps 340 degrees, which looks like a full spin.
    bot, transport = _bot()
    bot.turn_yaw(170.0, duration_s=1.0)
    transport.history.clear()
    bot.turn_yaw(20.0, duration_s=1.0)
    yaws = [math.degrees(p.orientation.yaw_r) for p in transport.history]
    steps = [abs(later - earlier) for earlier, later in zip(yaws, yaws[1:], strict=False)]
    # Every step is small except the single wrap from +180 to -180, which is a representation change.
    big = [step for step in steps if step > 90.0]
    assert len(big) <= 1, f"the yaw swept the long way: {yaws[:6]}"
