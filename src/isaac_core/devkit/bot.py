"""A stateful pose sender: command a vehicle, do not compute a stream of poses.

The pure generators in :mod:`isaac_core.vehicle` are the substance -- they compute the geodesy and
the interpolation -- but using them directly means holding a :class:`~isaac_core.vehicle.VehicleState`
by hand, threading a rate into every call, and pacing the output yourself:

    state = VehicleState.from_pose(pose)
    poses = move_forward_backward(state, 100.0, duration_s=5.0, rate_hz=30.0)
    pace(poses, transport, rate_hz=30.0)
    state = VehicleState.from_pose(last_pose_you_remembered_to_keep)

:class:`PoseBot` holds the state and the transport so a script reads as commands:

    with PoseBot(port=33333) as bot:
        bot.move_to_point(32.22481, 35.25621, 900.0, duration_s=5.0)
        bot.move_forward_backward(100.0)
        bot.turn_to_point(32.22481, 35.25621, 1000.0)

Each call streams in real time and leaves the bot where it arrived, so calls compose. Nothing here
computes geometry; every method delegates to the primitive of the same name and then advances the
held state from the last pose actually sent.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator, Sequence
import math
import time
from typing import TYPE_CHECKING, Final

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import PoseTransport, UdpPoseTransport
from isaac_core.geo.distance import (
    EARTH_RADIUS_M as _EARTH_RADIUS_M,
    look_at_angles,
    meters_to_latlon_offset,
)
from isaac_core.vehicle import (
    MotionLimits,
    OrbitTrajectory,
    PathTrajectory,
    VehicleState,
    move_forward_backward,
    move_right_left,
    move_to,
    move_up_down,
    steer,
    turn_pitch,
    turn_roll,
    turn_to_point,
    turn_yaw,
)

if TYPE_CHECKING:
    from types import TracebackType

# Matches the camera layers' publish rate and what the shipped senders use, so a bot's output looks
# like a real feed rather than a slideshow.
DEFAULT_RATE_HZ: Final = 30.0

# A single command with no duration given. Long enough to see, short enough not to feel stuck.
DEFAULT_DURATION_S: Final = 1.0

# The scene's reference point, so a bot constructed with no starting pose is already over terrain
# rather than at lat 0, lon 0 in the Atlantic.
DEFAULT_START: Final = Lla(lat_deg=32.22481, lon_deg=35.25621, alt_m=1000.0)

# A path needs a start and an end before it has a leg to fly.
_MIN_WAYPOINTS: Final = 2

# Closer than this to the target there is no meaningful direction to look in, so the last real
# bearing is held rather than collapsing to zero.
_AIM_DEADBAND_M: Final = 1.0

# Below this ground separation there is no bearing from a target to preserve, so a standoff direction
# would be picked from floating-point noise and the follower would jitter around the target.
_STANDOFF_DEGENERATE_M = 0.5

# How long the starting pose is streamed on entering the context. A single datagram is lost if the
# simulator's node has not bound its port yet, which is why real senders stream.
_START_SETTLE_S: Final = 0.5


def _horizontal_distance_m(a: Lla, b: Lla) -> float:
    """Return the ground distance between two positions, in metres.

    Deliberately ignores altitude, because this feeds the aim deadband and yaw depends only on
    horizontal separation. Measuring in three dimensions made a target directly below look far away --
    1000 m of altitude difference with no ground separation -- so the deadband did not engage,
    ``atan2(0, 0)`` returned zero and the airframe snapped due north the moment it flew over its target.

    Args:
        a: One position.
        b: The other.

    Returns:
        Ground distance in metres, using the same flat-earth approximation as the aiming maths.

    """
    cos_lat = math.cos(math.radians(a.lat_deg))
    north_m = math.radians(b.lat_deg - a.lat_deg) * _EARTH_RADIUS_M
    east_m = math.radians(b.lon_deg - a.lon_deg) * _EARTH_RADIUS_M * cos_lat
    return math.hypot(north_m, east_m)


class PoseBot:
    """Command a simulated vehicle by sending poses, holding its state between calls.

    Args:
        host: Target host for the default UDP transport.
        port: Target UDP port for the default transport.
        transport: An explicit transport, which takes precedence over *host* and *port*. Supply a
            :class:`~isaac_core.devkit.transport.FakePoseTransport` in a test, or a ROS 2 transport
            to drive a vehicle whose ``pose_source`` is ``"ros"``.
        start: Where the vehicle begins. Defaults to the scene's reference point at 1000 m.
        rate_hz: Send rate for every command.
        limits: Motion limits applied to every command. Unlike the primitives, which accept limits
            per call, a bot carries them so a whole flight is bounded by one declaration.
        sleep: Injectable sleep, for tests that must not take real time.

    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 33333,
        *,
        transport: PoseTransport | None = None,
        start: Lla | None = None,
        rate_hz: float = DEFAULT_RATE_HZ,
        limits: MotionLimits | None = None,
        sleep: Callable[[float], None] | None = None,
    ) -> None:
        """Initialise a bot, opening a UDP transport unless one is supplied."""
        if rate_hz <= 0.0:
            msg = f"rate_hz must be greater than zero, got {rate_hz}"
            raise ValueError(msg)
        self._transport: PoseTransport = transport if transport is not None else UdpPoseTransport(host, port)
        self._owns_transport = transport is None
        self._rate_hz = rate_hz
        self._limits = limits
        self._sleep = sleep if sleep is not None else time.sleep
        self._sent = 0
        origin = start if start is not None else DEFAULT_START
        self._state = VehicleState.from_pose(
            GeodeticPose(position=origin, orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0))
        )

    # -- state ---------------------------------------------------------------- #

    @property
    def state(self) -> VehicleState:
        """Return where the bot believes the vehicle is."""
        return self._state

    @property
    def pose(self) -> GeodeticPose:
        """Return the bot's current pose, the one a further command starts from."""
        return self._state.to_pose()

    @property
    def sent(self) -> int:
        """Return how many pose packets this bot has sent."""
        return self._sent

    @property
    def limits(self) -> MotionLimits | None:
        """Return the motion limits applied to every command."""
        return self._limits

    # -- position ------------------------------------------------------------- #

    def move_to_point(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        *,
        duration_s: float = 5.0,
        face_target: bool = True,
    ) -> int:
        """Fly to a geodetic point, by default looking at it the whole way.

        Args:
            lat_deg: Target latitude in degrees.
            lon_deg: Target longitude in degrees.
            alt_m: Target altitude in metres.
            duration_s: How long the move takes.
            face_target: Aim at the target while flying, which is what a camera operator wants and
                what "go and look at that" means. Roll is held throughout. Pass ``False`` to keep the
                current attitude instead.

        Returns:
            The number of poses sent.

        """
        poses = move_to(self._state, lat_deg, lon_deg, alt_m, duration_s, self._rate_hz, self._limits)
        if not face_target:
            return self._stream(poses)
        return self._stream(self._aimed_at(poses, lat_deg, lon_deg, alt_m))

    def _aimed_at(
        self,
        poses: Iterable[GeodeticPose],
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
    ) -> Iterator[GeodeticPose]:
        """Re-aim each pose in a stream at a fixed target, holding its roll.

        Recomputed per pose rather than once at the start, because the bearing to the target changes
        as the vehicle travels: aiming once and then flying would leave the camera pointing at where
        the target used to be relative to the aircraft.

        Args:
            poses: The position stream to re-aim.
            lat_deg: Target latitude in degrees.
            lon_deg: Target longitude in degrees.
            alt_m: Target altitude in metres.

        Yields:
            The same positions with look-at orientation.

        """
        target = Lla(lat_deg=lat_deg, lon_deg=lon_deg, alt_m=alt_m)
        last_aim: tuple[float, float] | None = None
        for pose in poses:
            # On arrival there is no direction left to look in and atan2(0, 0) is zero, which snapped
            # the camera to due north on the final frame of every move. Holding the last real bearing
            # is what an operator sees when a vehicle settles over its target.
            if _horizontal_distance_m(pose.position, target) > _AIM_DEADBAND_M:
                last_aim = look_at_angles(pose.position, target)
            aim = last_aim
            if aim is None:
                yield pose
                continue
            yield GeodeticPose(
                position=pose.position,
                orientation=Rpy(
                    roll_r=pose.orientation.roll_r, pitch_r=aim[1], yaw_r=aim[0], frame=pose.orientation.frame
                ),
            )

    def move_forward_backward(self, distance_m: float, *, duration_s: float = DEFAULT_DURATION_S) -> int:
        """Move along the vehicle's own forward axis; negative goes backwards.

        Args:
            distance_m: Distance in metres, negative for backwards.
            duration_s: How long the move takes.

        Returns:
            The number of poses sent.

        """
        return self._stream(move_forward_backward(self._state, distance_m, duration_s, self._rate_hz, self._limits))

    def move_right_left(self, distance_m: float, *, duration_s: float = DEFAULT_DURATION_S) -> int:
        """Move along the vehicle's own right axis; negative goes left.

        Args:
            distance_m: Distance in metres, negative for left.
            duration_s: How long the move takes.

        Returns:
            The number of poses sent.

        """
        return self._stream(move_right_left(self._state, distance_m, duration_s, self._rate_hz, self._limits))

    def move_up_down(self, distance_m: float, *, duration_s: float = DEFAULT_DURATION_S) -> int:
        """Move along the vehicle's own up axis; negative goes down.

        Args:
            distance_m: Distance in metres, negative for down.
            duration_s: How long the move takes.

        Returns:
            The number of poses sent.

        """
        return self._stream(move_up_down(self._state, distance_m, duration_s, self._rate_hz, self._limits))

    # -- attitude ------------------------------------------------------------- #

    def turn_roll(
        self,
        delta_deg: float,
        *,
        duration_s: float = DEFAULT_DURATION_S,
        frame: RotationFrame = RotationFrame.WORLD,
    ) -> int:
        """Roll by a delta in degrees.

        Args:
            delta_deg: Change in roll, degrees.
            duration_s: How long the turn takes.
            frame: Whether the rotation composes in the world or body frame.

        Returns:
            The number of poses sent.

        """
        return self._stream(
            turn_roll(self._state, math.radians(delta_deg), duration_s, self._rate_hz, frame, self._limits)
        )

    def turn_pitch(
        self,
        delta_deg: float,
        *,
        duration_s: float = DEFAULT_DURATION_S,
        frame: RotationFrame = RotationFrame.WORLD,
    ) -> int:
        """Pitch by a delta in degrees; positive raises the nose.

        Args:
            delta_deg: Change in pitch, degrees.
            duration_s: How long the turn takes.
            frame: Whether the rotation composes in the world or body frame.

        Returns:
            The number of poses sent.

        """
        return self._stream(
            turn_pitch(self._state, math.radians(delta_deg), duration_s, self._rate_hz, frame, self._limits)
        )

    def turn_yaw(
        self,
        delta_deg: float,
        *,
        duration_s: float = DEFAULT_DURATION_S,
        frame: RotationFrame = RotationFrame.WORLD,
    ) -> int:
        """Yaw by a delta in degrees; positive turns right.

        Args:
            delta_deg: Change in yaw, degrees.
            duration_s: How long the turn takes.
            frame: Whether the rotation composes in the world or body frame.

        Returns:
            The number of poses sent.

        """
        return self._stream(
            turn_yaw(self._state, math.radians(delta_deg), duration_s, self._rate_hz, frame, self._limits)
        )

    def turn_to_point(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        *,
        duration_s: float = DEFAULT_DURATION_S,
    ) -> int:
        """Aim the vehicle at a geodetic point without moving it.

        Yaw and pitch come from look-at geometry and roll is preserved, which is what aiming means:
        an earlier implementation of this rolled the airframe as a side effect.

        Args:
            lat_deg: Target latitude in degrees.
            lon_deg: Target longitude in degrees.
            alt_m: Target altitude in metres.
            duration_s: How long the turn takes.

        Returns:
            The number of poses sent.

        """
        return self._stream(
            turn_to_point(self._state, lat_deg, lon_deg, alt_m, duration_s, self._rate_hz, self._limits)
        )

    # -- combined ------------------------------------------------------------- #

    def track_point(
        self,
        target: Lla | Callable[[], Lla],
        *,
        duration_s: float = DEFAULT_DURATION_S,
    ) -> int:
        """Keep the vehicle aimed at a target while holding position.

        `turn_to_point` aims once at a fixed point and finishes. This re-reads the target every tick, so
        a moving one stays in frame -- which is the difference between aiming and tracking.

        Pass a callable for a target that moves. It is called once per tick, so it can read a live feed,
        another simulated vehicle, or a scripted path.

        Args:
            target: A fixed point, or a callable returning the current one.
            duration_s: How long to track for.

        Returns:
            The number of poses sent.

        Raises:
            ValueError: If ``duration_s`` is not positive.

        """
        if duration_s <= 0.0:
            raise ValueError(f"duration_s must be positive, got {duration_s}")
        return self._stream(self._tracking_poses(_as_target(target), duration_s))

    def _tracking_poses(self, target: Callable[[], Lla], duration_s: float) -> Iterator[GeodeticPose]:
        """Yield held-position poses re-aimed at a moving target each tick.

        Args:
            target: Called once per tick for the current target position.
            duration_s: How long to track for.

        Yields:
            Poses at the current position, aimed at the target.

        """
        start = self._state.to_pose()
        position = start.position
        last_aim: tuple[float, float] | None = None
        for _ in range(max(1, int(round(duration_s * self._rate_hz)))):
            current = target()
            # Same deadband as `_aimed_at`: directly over the target there is no bearing, and
            # atan2(0, 0) would snap the camera due north.
            if _horizontal_distance_m(position, current) > _AIM_DEADBAND_M:
                last_aim = look_at_angles(position, current)
            if last_aim is None:
                yield GeodeticPose(position=position, orientation=start.orientation)
                continue
            yield GeodeticPose(
                position=position,
                orientation=Rpy(
                    roll_r=start.orientation.roll_r,
                    pitch_r=last_aim[1],
                    yaw_r=last_aim[0],
                    frame=start.orientation.frame,
                ),
            )

    def follow_point(
        self,
        target: Lla | Callable[[], Lla],
        *,
        distance_m: float,
        height_m: float = 0.0,
        speed_mps: float,
        duration_s: float = DEFAULT_DURATION_S,
    ) -> int:
        """Chase a target, holding a standoff distance and staying aimed at it.

        Follow-me: each tick reads the target, works out where the vehicle should be to sit
        ``distance_m`` away from it at ``height_m`` above it, and moves toward that at up to
        ``speed_mps``. Speed-limited rather than teleporting, so the vehicle lags a fast target the way a
        real one would instead of pretending to be attached to it.

        Args:
            target: A fixed point, or a callable returning the current one.
            distance_m: Standoff distance to hold, in metres. Zero means fly to the target itself.
            height_m: How far above the target to sit, in metres.
            speed_mps: Maximum ground speed while chasing.
            duration_s: How long to follow for.

        Returns:
            The number of poses sent.

        Raises:
            ValueError: If ``speed_mps`` or ``duration_s`` is not positive, or ``distance_m`` is negative.

        """
        if speed_mps <= 0.0:
            raise ValueError(f"speed_mps must be positive, got {speed_mps}")
        if duration_s <= 0.0:
            raise ValueError(f"duration_s must be positive, got {duration_s}")
        if distance_m < 0.0:
            raise ValueError(f"distance_m must not be negative, got {distance_m}")
        # A limits object may leave any single bound unset, so the speed cap has to be checked rather
        # than assumed present.
        ceiling = None if self._limits is None else self._limits.max_speed_mps
        capped = speed_mps if ceiling is None else min(speed_mps, ceiling)
        return self._stream(self._following_poses(_as_target(target), distance_m, height_m, capped, duration_s))

    def _following_poses(
        self,
        target: Callable[[], Lla],
        distance_m: float,
        height_m: float,
        speed_mps: float,
        duration_s: float,
    ) -> Iterator[GeodeticPose]:
        """Yield poses chasing a moving target at a standoff, aimed at it.

        Args:
            target: Called once per tick for the current target position.
            distance_m: Standoff distance in metres.
            height_m: Height above the target in metres.
            speed_mps: Maximum ground speed.
            duration_s: How long to follow for.

        Yields:
            Poses moving toward the standoff point and aimed at the target.

        """
        start = self._state.to_pose()
        position = start.position
        roll_r = start.orientation.roll_r
        step_m = speed_mps / self._rate_hz
        last_aim: tuple[float, float] | None = None
        for _ in range(max(1, int(round(duration_s * self._rate_hz)))):
            current = target()
            goal = _standoff_point(position, current, distance_m, height_m)
            position = _advance_toward(position, goal, step_m)
            if _horizontal_distance_m(position, current) > _AIM_DEADBAND_M:
                last_aim = look_at_angles(position, current)
            orientation = (
                Rpy(roll_r=roll_r, pitch_r=last_aim[1], yaw_r=last_aim[0], frame=start.orientation.frame)
                if last_aim is not None
                else start.orientation
            )
            yield GeodeticPose(position=position, orientation=orientation)

    def steer(self, *, turn_radius_m: float, speed_mps: float, duration_s: float = DEFAULT_DURATION_S) -> int:
        """Fly a constant-radius arc, turning while moving.

        Args:
            turn_radius_m: Arc radius in metres. Negative turns the other way.
            speed_mps: Ground speed along the arc.
            duration_s: How long to fly the arc.

        Returns:
            The number of poses sent.

        """
        return self._stream(steer(self._state, turn_radius_m, speed_mps, duration_s, self._rate_hz, self._limits))

    def orbit(
        self,
        centre_lat_deg: float,
        centre_lon_deg: float,
        *,
        radius_m: float,
        speed_mps: float,
        duration_s: float,
        height_m: float | None = None,
    ) -> int:
        """Circle a point, facing along the direction of travel.

        Args:
            centre_lat_deg: Centre latitude in degrees.
            centre_lon_deg: Centre longitude in degrees.
            radius_m: Orbit radius in metres.
            speed_mps: Ground speed along the circle.
            duration_s: How long to orbit for.
            height_m: Altitude to orbit at. Defaults to the bot's current altitude.

        Returns:
            The number of poses sent.

        """
        trajectory = OrbitTrajectory(
            center_lat_deg=centre_lat_deg,
            center_lon_deg=centre_lon_deg,
            radius_m=radius_m,
            height_m=height_m if height_m is not None else self._state.alt_m,
            speed_mps=speed_mps,
            orbit_duration_s=duration_s,
        )
        return self._stream(trajectory.poses(rate_hz=self._rate_hz))

    def fly_path(self, waypoints: Sequence[Lla], *, speed_mps: float, face_travel: bool = True) -> int:
        """Fly through waypoints in order at a constant ground speed.

        Args:
            waypoints: Points to fly through, in order. At least two are needed for a leg.
            speed_mps: Ground speed along the path.
            face_travel: Point the airframe along the direction of travel, which is what a flight
                looks like. Set ``False`` to hold the current attitude instead.

        Returns:
            The number of poses sent.

        Raises:
            ValueError: If fewer than two waypoints are given.

        """
        if len(waypoints) < _MIN_WAYPOINTS:
            msg = f"a path needs at least two waypoints, got {len(waypoints)}"
            raise ValueError(msg)
        trajectory = PathTrajectory(waypoints=tuple(waypoints), speed_mps=speed_mps)
        poses = trajectory.poses(rate_hz=self._rate_hz)
        if face_travel:
            return self._stream(poses)
        held = self._state.to_pose().orientation
        return self._stream(GeodeticPose(position=pose.position, orientation=held) for pose in poses)

    # -- plumbing ------------------------------------------------------------- #

    def send(self, pose: GeodeticPose) -> None:
        """Send one pose and adopt it as the current state.

        Args:
            pose: The pose to send.

        """
        self._transport.send(pose)
        self._sent += 1
        self._state = VehicleState.from_pose(pose)

    def _stream(self, poses: Iterable[GeodeticPose]) -> int:
        """Send poses at ``rate_hz`` in real time, then adopt the last one.

        An accumulating deadline rather than ``sleep(1 / rate)``: sleeping a fixed interval adds the
        time spent sending to every tick, so a long flight arrives late by a growing margin.

        Args:
            poses: The poses to send, in order.

        Returns:
            How many were sent.

        """
        interval = 1.0 / self._rate_hz
        deadline = time.monotonic()
        count = 0
        last: GeodeticPose | None = None
        for pose in poses:
            self._transport.send(pose)
            count += 1
            last = pose
            deadline += interval
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                self._sleep(remaining)
        self._sent += count
        if last is not None:
            self._state = VehicleState.from_pose(last)
        return count

    def close(self) -> None:
        """Close the transport, if this bot opened it."""
        if self._owns_transport:
            self._transport.close()

    def __enter__(self) -> PoseBot:
        """Send the starting pose, then return self.

        Constructing a bot with a ``start`` used to transmit nothing: the position only reached the
        simulator once some command ran, so ``with PoseBot(start=...) as bot: pass`` left the vehicle
        wherever it already was. Streamed briefly rather than sent once, because a single datagram is
        lost if the simulator's node has not bound its port yet.

        Returns:
            This bot.

        """
        self.hold_start()
        return self

    def hold_start(self) -> int:
        """Stream the current pose for long enough to be sure it arrived.

        Returns:
            The number of poses sent.

        """
        pose = self._state.to_pose()
        return self._stream(pose for _ in range(max(1, int(_START_SETTLE_S * self._rate_hz))))

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the transport on the way out, including on an exception."""
        self.close()


__all__ = ["DEFAULT_DURATION_S", "DEFAULT_RATE_HZ", "DEFAULT_START", "PoseBot"]


def _as_target(target: Lla | Callable[[], Lla]) -> Callable[[], Lla]:
    """Return a callable giving the target's current position.

    Accepting both a point and a callable keeps a stationary target from needing a lambda, while a
    moving one needs no separate method.

    Args:
        target: A fixed point, or a callable returning the current one.

    Returns:
        A zero-argument callable.

    """
    if callable(target):
        return target
    return lambda: target


def _standoff_point(observer: Lla, target: Lla, distance_m: float, height_m: float) -> Lla:
    """Return where to sit to hold a standoff from a target.

    The standoff is taken along the observer's current bearing from the target, so a follower keeps the
    side it is already on rather than swinging around to a fixed compass offset.

    Args:
        observer: Where the vehicle is now.
        target: Where the target is now.
        distance_m: Standoff distance in metres. Zero means the target's own position.
        height_m: Height above the target in metres.

    Returns:
        The position to aim for.

    """
    goal_alt = target.alt_m + height_m
    if distance_m <= 0.0:
        return Lla(lat_deg=target.lat_deg, lon_deg=target.lon_deg, alt_m=goal_alt)
    north_m, east_m = _north_east_offset_m(target, observer)
    span = math.hypot(north_m, east_m)
    if span < _STANDOFF_DEGENERATE_M:
        # Directly overhead: no bearing to preserve, so hold due north of the target rather than
        # picking a direction from noise.
        north_m, east_m, span = 1.0, 0.0, 1.0
    scale = distance_m / span
    lat_off, lon_off = meters_to_latlon_offset(north_m * scale, east_m * scale, target.lat_deg)
    return Lla(lat_deg=target.lat_deg + lat_off, lon_deg=target.lon_deg + lon_off, alt_m=goal_alt)


def _advance_toward(position: Lla, goal: Lla, step_m: float) -> Lla:
    """Move a position toward a goal by at most one step.

    Args:
        position: Where the vehicle is.
        goal: Where it is heading.
        step_m: Maximum ground distance to cover, in metres.

    Returns:
        The new position, which is ``goal`` once within a step of it.

    """
    north_m, east_m = _north_east_offset_m(position, goal)
    span = math.hypot(north_m, east_m)
    climb_m = goal.alt_m - position.alt_m
    if span <= step_m:
        return Lla(lat_deg=goal.lat_deg, lon_deg=goal.lon_deg, alt_m=goal.alt_m)
    scale = step_m / span
    lat_off, lon_off = meters_to_latlon_offset(north_m * scale, east_m * scale, position.lat_deg)
    # Altitude is closed at the same fraction as the ground track, so the vehicle arrives level rather
    # than climbing to the final height on the last tick.
    return Lla(
        lat_deg=position.lat_deg + lat_off,
        lon_deg=position.lon_deg + lon_off,
        alt_m=position.alt_m + climb_m * scale,
    )


def _north_east_offset_m(origin: Lla, point: Lla) -> tuple[float, float]:
    """Return the north and east offset in metres from one position to another.

    Args:
        origin: The reference position.
        point: The position to measure to.

    Returns:
        ``(north_m, east_m)``.

    """
    north_m = math.radians(point.lat_deg - origin.lat_deg) * _EARTH_RADIUS_M
    east_m = math.radians(point.lon_deg - origin.lon_deg) * _EARTH_RADIUS_M * math.cos(math.radians(origin.lat_deg))
    return north_m, east_m
