"""Target tracking and follow-me on `PoseBot`.

`turn_to_point` aims once at a fixed point. Tracking re-reads the target every tick, so the test that
matters is against a target that **moves**: aiming correctly at a stationary point proves nothing that
`turn_to_point` did not already prove, and is exactly the shape of test that would pass while tracking was
broken.

Everything here is pure — a fake transport records what would have been sent and nothing sleeps.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.pose import Lla
from isaac_core.devkit.bot import PoseBot
from isaac_core.devkit.transport import FakePoseTransport
from isaac_core.geo.distance import geodesic_distance_m
from isaac_core.vehicle import MotionLimits

# The scene's reference point, so these are the coordinates a real flight uses.
START = Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1000.0)

# Fast enough to be obvious over a short run, slow enough to stay local.
TRACK_RATE_HZ = 10.0


def _bot(**kwargs: object) -> PoseBot:
    """Return a bot with a recording transport that never sleeps."""
    return PoseBot(
        transport=FakePoseTransport(),
        start=START,
        rate_hz=TRACK_RATE_HZ,
        sleep=lambda _seconds: None,
        **kwargs,  # type: ignore[arg-type]
    )


def _bearing_to(observer: Lla, target: Lla) -> float:
    """Return the compass bearing from an observer to a target, in degrees."""
    north = math.radians(target.lat_deg - observer.lat_deg)
    east = math.radians(target.lon_deg - observer.lon_deg) * math.cos(math.radians(observer.lat_deg))
    return math.degrees(math.atan2(east, north)) % 360.0


def _yaw_deg(pose: object) -> float:
    """Return a pose's yaw in degrees, normalised to [0, 360)."""
    return math.degrees(pose.orientation.yaw_r) % 360.0  # type: ignore[attr-defined]


# -- tracking a stationary target ------------------------------------------------ #


def test_tracking_holds_position() -> None:
    # Tracking aims; it does not fly. A tracking call that moved the vehicle would be a `follow`.
    bot = _bot()
    target = Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0)
    bot.track_point(target, duration_s=1.0)
    history = bot._transport.history  # type: ignore[attr-defined]
    assert len(history) == 10
    for pose in history:
        assert pose.position.lat_deg == pytest.approx(START.lat_deg)
        assert pose.position.lon_deg == pytest.approx(START.lon_deg)
        assert pose.position.alt_m == pytest.approx(START.alt_m)


def test_tracking_a_stationary_target_aims_at_it() -> None:
    bot = _bot()
    target = Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0)
    bot.track_point(target, duration_s=0.5)
    sent = bot._transport.history[-1]  # type: ignore[attr-defined]
    assert _yaw_deg(sent) == pytest.approx(_bearing_to(START, target), abs=0.5)


def test_tracking_a_target_below_aims_down() -> None:
    bot = _bot()
    bot.track_point(Lla(lat_deg=32.23, lon_deg=35.25621, alt_m=0.0), duration_s=0.5)
    assert bot._transport.history[-1].orientation.pitch_r < 0.0  # type: ignore[attr-defined]


# -- tracking a MOVING target, which is the whole point -------------------------- #


def test_the_aim_follows_a_target_that_moves() -> None:
    # The defect this guards: reading the target once and reusing it. That passes every stationary test
    # and leaves the camera pointing where the target used to be.
    positions = [
        Lla(lat_deg=32.230, lon_deg=35.256, alt_m=500.0),
        Lla(lat_deg=32.230, lon_deg=35.276, alt_m=500.0),
        Lla(lat_deg=32.230, lon_deg=35.296, alt_m=500.0),
        Lla(lat_deg=32.230, lon_deg=35.316, alt_m=500.0),
        Lla(lat_deg=32.230, lon_deg=35.336, alt_m=500.0),
    ]
    step = iter(positions)
    bot = _bot()
    bot.track_point(lambda: next(step), duration_s=0.5)

    # Signed bearings, because the first target is just west of due north and comparing 358 against 72
    # across the 0/360 wrap would fail on correct output.
    yaws = [(_yaw_deg(pose) + 180.0) % 360.0 - 180.0 for pose in bot._transport.history]  # type: ignore[attr-defined]
    assert len(yaws) == 5
    assert len({round(value, 3) for value in yaws}) == 5, f"the aim did not change: {yaws}"
    # The target moves east, so the bearing to it must increase.
    assert yaws == sorted(yaws), yaws


def test_each_tick_aims_at_that_tick_s_target() -> None:
    positions = [
        Lla(lat_deg=32.240, lon_deg=35.256, alt_m=500.0),
        Lla(lat_deg=32.220, lon_deg=35.276, alt_m=500.0),
        Lla(lat_deg=32.210, lon_deg=35.236, alt_m=500.0),
    ]
    step = iter(positions)
    bot = _bot()
    bot.track_point(lambda: next(step), duration_s=0.3)
    for pose, expected in zip(bot._transport.history, positions, strict=True):  # type: ignore[attr-defined]
        assert _yaw_deg(pose) == pytest.approx(_bearing_to(START, expected), abs=0.5)


def test_the_target_is_read_once_per_tick() -> None:
    # Reading it twice per tick would double-consume a live feed or a generator.
    calls = 0

    def moving() -> Lla:
        nonlocal calls
        calls += 1
        return Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0)

    _bot().track_point(moving, duration_s=1.0)
    assert calls == 10


def test_a_fixed_point_needs_no_lambda() -> None:
    # Both forms must work, or every stationary caller has to write a closure.
    bot = _bot()
    assert bot.track_point(Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0), duration_s=0.2) == 2


def test_roll_is_preserved_while_tracking() -> None:
    # Aiming must not roll the airframe, the same defect that was fixed in turn_to_point.
    bot = _bot()
    bot.turn_roll(20.0, duration_s=0.2)
    rolled = bot.pose.orientation.roll_r
    bot.track_point(Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0), duration_s=0.5)
    for pose in bot._transport.history[-5:]:  # type: ignore[attr-defined]
        assert pose.orientation.roll_r == pytest.approx(rolled, abs=1e-9)


def test_tracking_a_target_directly_below_does_not_snap_north() -> None:
    # atan2(0, 0) is zero, which pointed the camera due north on the frame a vehicle arrived overhead.
    bot = _bot()
    bot.turn_yaw(123.0, duration_s=0.2)
    held = _yaw_deg(bot.pose)
    bot.track_point(Lla(lat_deg=START.lat_deg, lon_deg=START.lon_deg, alt_m=0.0), duration_s=0.4)
    assert _yaw_deg(bot._transport.history[-1]) == pytest.approx(held, abs=0.5)  # type: ignore[attr-defined]


@pytest.mark.parametrize("duration", [0.0, -1.0])
def test_a_nonpositive_duration_is_refused(duration: float) -> None:
    with pytest.raises(ValueError, match="duration_s must be positive"):
        _bot().track_point(Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0), duration_s=duration)


# -- follow-me -------------------------------------------------------------------- #


def test_following_closes_the_distance_to_the_standoff() -> None:
    bot = _bot()
    target = Lla(lat_deg=32.235, lon_deg=35.25621, alt_m=900.0)
    bot.follow_point(target, distance_m=200.0, speed_mps=100.0, duration_s=20.0)
    final = bot.pose.position
    assert geodesic_distance_m(final, target) == pytest.approx(200.0, rel=0.1)


def test_following_holds_the_height_offset() -> None:
    bot = _bot()
    target = Lla(lat_deg=32.235, lon_deg=35.25621, alt_m=900.0)
    bot.follow_point(target, distance_m=100.0, height_m=150.0, speed_mps=100.0, duration_s=20.0)
    assert bot.pose.position.alt_m == pytest.approx(1050.0, abs=5.0)


def test_following_stays_aimed_at_the_target() -> None:
    bot = _bot()
    target = Lla(lat_deg=32.235, lon_deg=35.26, alt_m=500.0)
    bot.follow_point(target, distance_m=150.0, speed_mps=80.0, duration_s=4.0)
    last = bot._transport.history[-1]  # type: ignore[attr-defined]
    assert _yaw_deg(last) == pytest.approx(_bearing_to(last.position, target), abs=1.0)


def test_following_is_speed_limited_rather_than_teleporting() -> None:
    # One tick at 10 Hz and 50 m/s can cover 5 m, so the first pose must be about 5 m from the start.
    bot = _bot()
    far = Lla(lat_deg=32.30, lon_deg=35.40, alt_m=1000.0)
    bot.follow_point(far, distance_m=0.0, speed_mps=50.0, duration_s=0.2)
    first = bot._transport.history[0]  # type: ignore[attr-defined]
    assert geodesic_distance_m(START, first.position) == pytest.approx(5.0, rel=0.2)


def test_a_motion_limit_caps_the_follow_speed() -> None:
    # One declaration of limits must bound every command, including this one.
    bot = _bot(limits=MotionLimits(max_speed_mps=10.0))
    far = Lla(lat_deg=32.30, lon_deg=35.40, alt_m=1000.0)
    bot.follow_point(far, distance_m=0.0, speed_mps=500.0, duration_s=0.2)
    first = bot._transport.history[0]  # type: ignore[attr-defined]
    assert geodesic_distance_m(START, first.position) == pytest.approx(1.0, rel=0.3)


def test_following_a_moving_target_keeps_chasing_it() -> None:
    # A follower that read the target once would stop where the target started.
    lon = 35.25621

    def drifting() -> Lla:
        nonlocal lon
        lon += 0.0004
        return Lla(lat_deg=32.235, lon_deg=lon, alt_m=900.0)

    bot = _bot()
    bot.follow_point(drifting, distance_m=100.0, speed_mps=120.0, duration_s=5.0)
    assert bot.pose.position.lon_deg > 35.257, "the follower did not move east after the target"


def test_a_zero_standoff_flies_to_the_target_itself() -> None:
    bot = _bot()
    target = Lla(lat_deg=32.2255, lon_deg=35.2565, alt_m=1000.0)
    bot.follow_point(target, distance_m=0.0, speed_mps=100.0, duration_s=5.0)
    assert geodesic_distance_m(bot.pose.position, target) < 1.0


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"distance_m": 100.0, "speed_mps": 0.0}, "speed_mps must be positive"),
        ({"distance_m": 100.0, "speed_mps": -5.0}, "speed_mps must be positive"),
        ({"distance_m": -1.0, "speed_mps": 10.0}, "distance_m must not be negative"),
    ],
)
def test_invalid_follow_arguments_are_refused(kwargs: dict[str, float], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _bot().follow_point(Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0), duration_s=1.0, **kwargs)


def test_a_nonpositive_follow_duration_is_refused() -> None:
    with pytest.raises(ValueError, match="duration_s must be positive"):
        _bot().follow_point(
            Lla(lat_deg=32.23, lon_deg=35.26, alt_m=500.0), distance_m=100.0, speed_mps=10.0, duration_s=0.0
        )


def test_the_bot_adopts_where_it_ended_up() -> None:
    # A following command must leave the bot where it arrived, so a later command starts from there.
    bot = _bot()
    target = Lla(lat_deg=32.235, lon_deg=35.25621, alt_m=900.0)
    bot.follow_point(target, distance_m=100.0, speed_mps=100.0, duration_s=4.0)
    assert bot.pose.position.lat_deg == pytest.approx(bot._transport.history[-1].position.lat_deg)  # type: ignore[attr-defined]


# -- the deadband bug this file found ------------------------------------------- #


def test_flying_over_a_target_does_not_snap_the_aim_north() -> None:
    # The aim deadband measured distance in THREE dimensions, so a target 1000 m directly below looked
    # far away, the deadband never engaged, atan2(0, 0) returned zero and the airframe snapped due north
    # the moment it passed over its target. Yaw depends only on horizontal separation, so the deadband
    # has to as well. This affected move_to_point and turn_to_point too, not only tracking.
    bot = _bot()
    bot.turn_yaw(90.0, duration_s=0.2)
    held = _yaw_deg(bot.pose)
    directly_below = Lla(lat_deg=START.lat_deg, lon_deg=START.lon_deg, alt_m=0.0)
    bot.track_point(directly_below, duration_s=0.5)
    for pose in bot._transport.history[-5:]:  # type: ignore[attr-defined]
        assert _yaw_deg(pose) == pytest.approx(held, abs=0.5), "the aim snapped away from where it was held"


def test_turning_to_a_point_directly_below_also_holds_its_heading() -> None:
    # Same defect, reached through the older aiming path rather than tracking. Asserted on what is SENT,
    # because that is what reaches the simulator -- see the gimbal-lock test below for why the bot's own
    # readback cannot be used here.
    bot = _bot()
    bot.turn_yaw(45.0, duration_s=0.2)
    held = _yaw_deg(bot.pose)
    before = len(bot._transport.history)  # type: ignore[attr-defined]
    bot.turn_to_point(START.lat_deg, START.lon_deg, 0.0, duration_s=0.5)
    aimed = bot._transport.history[before:]  # type: ignore[attr-defined]
    assert aimed, "turn_to_point sent nothing"
    for pose in aimed:
        assert _yaw_deg(pose) == pytest.approx(held, abs=0.5), "the heading was not held over the target"


def test_aiming_exactly_straight_down_cannot_report_its_own_yaw() -> None:
    # Not a bug, and worth pinning so it is not mistaken for one. At pitch exactly -90 the Euler
    # decomposition is singular: yaw and roll describe the same rotation, so a matrix round-trip moves
    # yaw into roll. Measured: yaw 45 / pitch -90 comes back as yaw 0 / roll 45, while pitch -89.9 comes
    # back exactly. The poses on the wire still carry the right yaw, which is what the simulator reads;
    # only `bot.pose`, which stores a rotation matrix, loses it.
    bot = _bot()
    bot.turn_yaw(45.0, duration_s=0.2)
    bot.turn_to_point(START.lat_deg, START.lon_deg, 0.0, duration_s=0.3)
    sent = bot._transport.history[-1]  # type: ignore[attr-defined]
    assert _yaw_deg(sent) == pytest.approx(45.0, abs=0.5), "the wire lost the heading, which would be a bug"
    assert math.degrees(sent.orientation.pitch_r) == pytest.approx(-90.0, abs=0.5)
    # And a hair off the singularity round-trips perfectly, which is what makes it a singularity.
    bot2 = _bot()
    bot2.turn_yaw(45.0, duration_s=0.2)
    bot2.turn_to_point(START.lat_deg + 0.00002, START.lon_deg, 0.0, duration_s=0.3)
    assert _yaw_deg(bot2.pose) == pytest.approx(0.0, abs=1.0), "just off the singularity should aim north"
