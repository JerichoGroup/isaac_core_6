"""Tests for PoseBot: commanding a vehicle rather than computing a pose stream.

`sleep` is injected as a no-op everywhere, so these run at full speed while still exercising the real
pacing arithmetic. The geometry itself belongs to `isaac_core.vehicle` and is tested there; what is
tested here is that a bot delegates to the right primitive, advances its held state from the last pose
actually sent, and enforces the limits it was given.
"""

from __future__ import annotations

import math
from typing import Final

import pytest

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.bot import DEFAULT_START, PoseBot
from isaac_core.devkit.transport import FakePoseTransport
from isaac_core.vehicle import MotionLimits

_EARTH_RADIUS_M: Final = 6371000.0


def _bot(**kwargs: object) -> tuple[PoseBot, FakePoseTransport]:
    """Return a bot wired to a recording transport, with sleeping disabled."""
    transport = FakePoseTransport()
    bot = PoseBot(transport=transport, sleep=lambda _seconds: None, **kwargs)  # type: ignore[arg-type]
    return bot, transport


def _ground_distance_m(start: Lla, end: Lla) -> float:
    """Return the flat-earth ground distance between two positions."""
    north = math.radians(end.lat_deg - start.lat_deg) * _EARTH_RADIUS_M
    east = math.radians(end.lon_deg - start.lon_deg) * _EARTH_RADIUS_M * math.cos(math.radians(start.lat_deg))
    return math.hypot(north, east)


# -- state -------------------------------------------------------------------- #


def test_a_new_bot_starts_over_the_scene_reference_point() -> None:
    # Defaulting to lat 0, lon 0 would put the vehicle in the Atlantic, off the terrain entirely.
    bot, _ = _bot()
    assert bot.pose.position == DEFAULT_START


def test_a_start_position_is_honoured() -> None:
    start = Lla(lat_deg=10.0, lon_deg=20.0, alt_m=500.0)
    bot, _ = _bot(start=start)
    assert bot.pose.position == start


def test_a_zero_or_negative_rate_is_refused() -> None:
    for rate in (0.0, -30.0):
        with pytest.raises(ValueError, match="rate_hz"):
            PoseBot(transport=FakePoseTransport(), rate_hz=rate)


def test_commands_compose_because_state_advances() -> None:
    # The whole point of holding state: a second command starts where the first ended.
    bot, _ = _bot()
    bot.move_to_point(32.3, 35.3, 1500.0, duration_s=1.0)
    assert bot.pose.position.lat_deg == pytest.approx(32.3)
    bot.move_up_down(-500.0, duration_s=1.0)
    assert bot.pose.position.alt_m == pytest.approx(1000.0, abs=1.0)


def test_every_pose_reaches_the_transport() -> None:
    bot, transport = _bot()
    sent = bot.move_to_point(32.23, 35.26, 1100.0, duration_s=2.0)
    assert sent == len(transport.history)
    assert bot.sent == sent


# -- position ----------------------------------------------------------------- #


def test_move_to_point_arrives_at_the_point() -> None:
    bot, _ = _bot()
    bot.move_to_point(32.30000, 35.30000, 1234.0, duration_s=2.0)
    assert bot.pose.position.lat_deg == pytest.approx(32.30000)
    assert bot.pose.position.lon_deg == pytest.approx(35.30000)
    assert bot.pose.position.alt_m == pytest.approx(1234.0)


@pytest.mark.parametrize("distance_m", [100.0, -100.0, 250.0])
def test_move_forward_backward_travels_the_requested_distance(distance_m: float) -> None:
    bot, _ = _bot()
    start = bot.pose.position
    bot.move_forward_backward(distance_m, duration_s=1.0)
    assert _ground_distance_m(start, bot.pose.position) == pytest.approx(abs(distance_m), rel=0.01)


def test_forward_and_backward_return_to_the_start() -> None:
    bot, _ = _bot()
    start = bot.pose.position
    bot.move_forward_backward(100.0, duration_s=1.0)
    bot.move_forward_backward(-100.0, duration_s=1.0)
    assert _ground_distance_m(start, bot.pose.position) == pytest.approx(0.0, abs=0.5)


def test_move_right_left_is_perpendicular_to_forward() -> None:
    bot, _ = _bot()
    start = bot.pose.position
    bot.move_right_left(100.0, duration_s=1.0)
    assert _ground_distance_m(start, bot.pose.position) == pytest.approx(100.0, rel=0.01)


def test_move_up_down_changes_only_altitude() -> None:
    bot, _ = _bot()
    start = bot.pose.position
    bot.move_up_down(-250.0, duration_s=1.0)
    assert bot.pose.position.alt_m == pytest.approx(start.alt_m - 250.0, abs=1.0)
    assert _ground_distance_m(start, bot.pose.position) == pytest.approx(0.0, abs=1.0)


# -- attitude ----------------------------------------------------------------- #


def test_turn_yaw_takes_degrees_not_radians() -> None:
    # The primitives take radians; the bot is the user-facing layer and takes degrees, which is what
    # every other user-facing surface in this project uses.
    bot, _ = _bot()
    bot.turn_yaw(90.0, duration_s=1.0)
    assert math.degrees(bot.pose.orientation.yaw_r) == pytest.approx(90.0, abs=0.5)


def test_turn_pitch_raises_the_nose() -> None:
    bot, _ = _bot()
    bot.turn_pitch(30.0, duration_s=1.0)
    assert math.degrees(bot.pose.orientation.pitch_r) == pytest.approx(30.0, abs=0.5)


def test_turn_roll_rolls() -> None:
    bot, _ = _bot()
    bot.turn_roll(45.0, duration_s=1.0)
    assert math.degrees(bot.pose.orientation.roll_r) == pytest.approx(45.0, abs=0.5)


def test_turn_to_point_leaves_roll_alone() -> None:
    # The 2023 implementation rolled the airframe as a side effect of aiming, which is wrong: aiming
    # is a yaw and pitch operation.
    bot, _ = _bot()
    bot.turn_roll(20.0, duration_s=1.0)
    roll_before = bot.pose.orientation.roll_r
    bot.turn_to_point(32.20, 35.20, 500.0, duration_s=1.0)
    assert bot.pose.orientation.roll_r == pytest.approx(roll_before, abs=1e-6)


def test_turn_to_point_pitches_down_towards_something_below() -> None:
    bot, _ = _bot(start=Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=2000.0))
    bot.turn_to_point(32.22481, 35.30000, 500.0, duration_s=1.0)
    assert bot.pose.orientation.pitch_r < 0.0


def test_turn_to_point_does_not_move_the_vehicle() -> None:
    bot, _ = _bot()
    start = bot.pose.position
    bot.turn_to_point(32.10, 35.10, 100.0, duration_s=1.0)
    assert bot.pose.position == start


def test_a_body_frame_turn_is_accepted() -> None:
    bot, _ = _bot()
    bot.turn_yaw(30.0, duration_s=1.0, frame=RotationFrame.BODY)
    assert bot.sent > 0


# -- combined ----------------------------------------------------------------- #


def test_steer_moves_and_turns_together() -> None:
    bot, _ = _bot()
    start = bot.pose
    bot.steer(turn_radius_m=100.0, speed_mps=50.0, duration_s=1.0)
    assert _ground_distance_m(start.position, bot.pose.position) > 1.0
    assert bot.pose.orientation.yaw_r != start.orientation.yaw_r


def test_orbit_holds_its_radius_from_the_centre() -> None:
    centre = Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1000.0)
    bot, transport = _bot()
    bot.orbit(centre.lat_deg, centre.lon_deg, radius_m=800.0, speed_mps=30.0, duration_s=2.0)
    radii = [_ground_distance_m(centre, pose.position) for pose in transport.history]
    assert radii, "the orbit sent nothing"
    for radius in radii:
        assert radius == pytest.approx(800.0, rel=0.02)


def test_orbit_defaults_to_the_bots_current_altitude() -> None:
    bot, transport = _bot(start=Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1750.0))
    bot.orbit(32.22481, 35.25621, radius_m=500.0, speed_mps=30.0, duration_s=1.0)
    assert transport.history[0].position.alt_m == pytest.approx(1750.0)


def test_fly_path_arrives_at_the_last_waypoint() -> None:
    end = Lla(lat_deg=32.24, lon_deg=35.27, alt_m=1200.0)
    bot, _ = _bot()
    bot.fly_path([Lla(32.22, 35.25, 900.0), Lla(32.23, 35.26, 1000.0), end], speed_mps=150.0)
    assert bot.pose.position.lat_deg == pytest.approx(end.lat_deg, abs=1e-9)
    assert bot.pose.position.alt_m == pytest.approx(end.alt_m, abs=1e-6)


def test_fly_path_faces_the_direction_of_travel() -> None:
    # Flying north-east, so the heading should be about 45 degrees.
    bot, transport = _bot()
    bot.fly_path([Lla(32.22, 35.25, 1000.0), Lla(32.23, 35.26, 1000.0)], speed_mps=200.0)
    heading_deg = math.degrees(transport.history[len(transport.history) // 2].orientation.yaw_r)
    assert heading_deg == pytest.approx(40.0, abs=10.0)


def test_fly_path_can_hold_attitude_instead() -> None:
    bot, transport = _bot()
    bot.turn_yaw(90.0, duration_s=1.0)
    held = bot.pose.orientation.yaw_r
    transport.history.clear()
    bot.fly_path([Lla(32.22, 35.25, 1000.0), Lla(32.23, 35.26, 1000.0)], speed_mps=200.0, face_travel=False)
    for pose in transport.history:
        assert pose.orientation.yaw_r == pytest.approx(held)


def test_fly_path_needs_two_waypoints() -> None:
    bot, _ = _bot()
    with pytest.raises(ValueError, match="at least two waypoints"):
        bot.fly_path([Lla(32.22, 35.25, 1000.0)], speed_mps=50.0)


# -- limits ------------------------------------------------------------------- #


def test_limits_are_enforced_rather_than_accepted_and_ignored() -> None:
    # MotionLimits was coded and tested for a whole release while nothing applied it at runtime. A bot
    # carries limits so one declaration bounds a whole flight.
    limits = MotionLimits(max_speed_mps=10.0, max_accel_mps2=None, max_turn_rate_deg_s=None, max_climb_rate_mps=None)
    limited, _ = _bot(limits=limits)
    unlimited, _ = _bot()
    assert limited.limits is limits

    start = limited.pose.position
    limited.move_forward_backward(1000.0, duration_s=1.0)
    limited_distance = _ground_distance_m(start, limited.pose.position)

    unlimited.move_forward_backward(1000.0, duration_s=1.0)
    unlimited_distance = _ground_distance_m(start, unlimited.pose.position)

    assert limited_distance < unlimited_distance, "the speed limit did not restrain the move"
    assert limited_distance == pytest.approx(10.0, rel=0.2), f"expected about 10 m in 1 s, got {limited_distance}"


# -- plumbing ----------------------------------------------------------------- #


def test_send_puts_one_pose_on_the_wire_and_adopts_it() -> None:
    bot, transport = _bot()
    pose = GeodeticPose(
        position=Lla(lat_deg=1.0, lon_deg=2.0, alt_m=3.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
    )
    bot.send(pose)
    assert transport.history == [pose]
    assert bot.pose.position == pose.position


def test_a_bot_that_opened_its_transport_closes_it() -> None:
    bot = PoseBot(port=39999, sleep=lambda _seconds: None)
    bot.close()
    # Sending after close must fail rather than silently going nowhere.
    with pytest.raises(RuntimeError, match="closed"):
        bot.send(
            GeodeticPose(
                position=Lla(lat_deg=1.0, lon_deg=2.0, alt_m=3.0),
                orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
            )
        )


def test_an_injected_transport_is_left_to_its_owner() -> None:
    # Same convention as Sim.attach not stopping a simulator it did not start.
    bot, transport = _bot()
    bot.close()
    assert transport.closed is False


def test_the_context_manager_closes_an_owned_transport() -> None:
    with PoseBot(port=39998, sleep=lambda _seconds: None) as bot:
        # Entering sends the starting pose, so the counter has already moved: a bot constructed with a
        # start used to transmit nothing until some command ran.
        assert bot.sent > 0
    with pytest.raises(RuntimeError, match="closed"):
        bot.send(
            GeodeticPose(
                position=Lla(lat_deg=1.0, lon_deg=2.0, alt_m=3.0),
                orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
            )
        )


def test_the_context_manager_closes_on_an_exception() -> None:
    bot = PoseBot(port=39997, sleep=lambda _seconds: None)
    with pytest.raises(RuntimeError), bot:
        msg = "deliberate"
        raise RuntimeError(msg)
    with pytest.raises(RuntimeError, match="closed"):
        bot.send(
            GeodeticPose(
                position=Lla(lat_deg=1.0, lon_deg=2.0, alt_m=3.0),
                orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0),
            )
        )


def test_pacing_targets_absolute_deadlines_not_fixed_intervals() -> None:
    # sleep(1 / rate) adds the time spent sending to every tick, so a long flight drifts late by a
    # growing margin. An accumulating deadline instead aims at absolute times.
    #
    # The signature, with a sleep that consumes no clock: each requested wait is one interval longer
    # than the last, because the deadlines march on while the clock does not. A fixed-interval
    # implementation would request the same number every time.
    requested: list[float] = []
    transport = FakePoseTransport()
    bot = PoseBot(transport=transport, rate_hz=10.0, sleep=requested.append)
    bot.move_to_point(32.23, 35.26, 1100.0, duration_s=1.0)

    assert len(requested) > 2, "nothing was paced"
    steps = [later - earlier for earlier, later in zip(requested, requested[1:], strict=False)]
    interval = 1.0 / 10.0
    for step in steps:
        assert step == pytest.approx(interval, abs=0.02), f"deadlines are not one interval apart: {steps[:5]}"


def test_there_is_no_hold_method() -> None:
    # A "hold" would be a sleep wearing a method's clothes: the simulator holds the last good pose by
    # design, so nothing needs to be streamed to stay put.
    assert not hasattr(PoseBot, "hold")


# -- entering the context puts the vehicle somewhere --------------------------- #


def test_entering_the_context_sends_the_starting_pose() -> None:
    # `with PoseBot(start=...) as bot: pass` used to send nothing at all, so the vehicle stayed
    # wherever it already was and the start argument looked broken.
    start = Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=900.0)
    transport = FakePoseTransport()
    with PoseBot(transport=transport, start=start, sleep=lambda _seconds: None):
        pass
    assert transport.history, "entering the context sent nothing"
    assert transport.history[0].position == start
    assert transport.history[-1].position == start


def test_the_starting_pose_is_streamed_not_sent_once() -> None:
    # A single datagram is lost if the simulator's node has not bound its port yet, which is why the
    # shipped senders stream. One packet would be a coin toss.
    transport = FakePoseTransport()
    with PoseBot(transport=transport, sleep=lambda _seconds: None):
        pass
    assert len(transport.history) > 1, f"only {len(transport.history)} packet(s) sent"
