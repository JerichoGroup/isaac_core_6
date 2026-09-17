"""Pure motion primitives operating on a :class:`~isaac_core.vehicle.state.VehicleState`.

Every function in this module is a generator that yields
:class:`~isaac_core.contracts.pose.GeodeticPose` instances. There is **no I/O**:
no sockets, no threads, no ``time.sleep``, no wall-clock reads. The caller is
responsible for pacing (sleeping between sends) and transport (shipping bytes).

Rotation primitives delegate to :func:`~isaac_core.geo.compose_rotation` and
accept a :class:`~isaac_core.contracts.frames.RotationFrame` parameter,
implementing decision D14 -- the same angular command produces observably
different results under ``WORLD`` vs ``BODY`` frame.

Orientation interpolation uses :func:`~isaac_core.geo.slerp` throughout.
"""

from __future__ import annotations

from collections.abc import Iterator
import math

import numpy as np
from numpy.typing import NDArray
from transforms3d.quaternions import mat2quat, quat2mat

from isaac_core.contracts.angles import normalize_angle
from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.geo.distance import (
    geodesic_distance_m,
    look_at_angles,
    meters_to_latlon_offset,
)
from isaac_core.geo.rotations import (
    compose_rotation,
    euler_to_matrix,
    matrix_to_euler,
    slerp,
)
from isaac_core.vehicle.limits import MotionLimits
from isaac_core.vehicle.state import VehicleState


def _state_to_pose(state: VehicleState) -> GeodeticPose:
    """Convert a VehicleState to a GeodeticPose (NED)."""
    roll_r, pitch_r, yaw_r = matrix_to_euler(state.rotation, state.frame)
    return GeodeticPose(
        position=Lla(lat_deg=state.lat_deg, lon_deg=state.lon_deg, alt_m=state.alt_m),
        orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
    )


def _mat_to_quat(mat: NDArray[np.float64]) -> tuple[float, float, float, float]:
    """Convert a 3×3 rotation matrix to a (w, x, y, z) quaternion tuple."""
    q: NDArray[np.float64] = mat2quat(mat)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _quat_to_mat(q: tuple[float, float, float, float]) -> NDArray[np.float64]:
    """Convert a (w, x, y, z) quaternion tuple to a 3×3 rotation matrix."""
    result: NDArray[np.float64] = quat2mat(q)
    return result


def move_to(
    state: VehicleState,
    target_lat_deg: float,
    target_lon_deg: float,
    target_alt_m: float,
    duration_s: float,
    rate_hz: float,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that move the vehicle from its current position to a target.

    Linear interpolation in LLA space, preserving current orientation throughout.
    The number of yielded samples is exactly ``int(duration_s * rate_hz)``.

    Args:
        state: Current vehicle state.
        target_lat_deg: Target latitude in degrees.
        target_lon_deg: Target longitude in degrees.
        target_alt_m: Target altitude in metres.
        duration_s: Duration of the move in seconds.
        rate_hz: Output rate in Hz.
        limits: Optional motion limits. The implied speed is ``distance / duration``; if that
            exceeds ``max_speed_mps`` the duration is extended so the move respects the limit
            rather than teleporting.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    if limits is not None:
        # This parameter was accepted and ignored, so a caller passing a 1 m/s limit still got the
        # full-distance move. Every sibling honours limits, so silently not honouring it here was the
        # API lying rather than a documented exception.
        distance_m = geodesic_distance_m(
            Lla(lat_deg=state.lat_deg, lon_deg=state.lon_deg, alt_m=state.alt_m),
            Lla(lat_deg=target_lat_deg, lon_deg=target_lon_deg, alt_m=target_alt_m),
        )
        if duration_s > 0.0:
            implied_speed = distance_m / duration_s
            clamped = abs(limits.clamp_speed(implied_speed))
            if clamped > 0.0 and clamped < implied_speed:
                duration_s = distance_m / clamped

    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    start_lat = state.lat_deg
    start_lon = state.lon_deg
    start_alt = state.alt_m

    for step in range(1, total_steps + 1):
        alpha = step / total_steps
        lat = start_lat + alpha * (target_lat_deg - start_lat)
        lon = start_lon + alpha * (target_lon_deg - start_lon)
        alt = start_alt + alpha * (target_alt_m - start_alt)

        roll_r, pitch_r, yaw_r = matrix_to_euler(state.rotation, state.frame)
        yield GeodeticPose(
            position=Lla(lat_deg=lat, lon_deg=lon, alt_m=alt),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )


def move_forward_backward(
    state: VehicleState,
    distance_m: float,
    duration_s: float,
    rate_hz: float,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that move the vehicle along its current heading.

    Positive distance moves forward; negative moves backward. The heading
    (yaw) determines the direction of travel in the horizontal plane. This
    moves along the body-forward axis.

    Args:
        state: Current vehicle state.
        distance_m: Distance in metres (positive=forward, negative=backward).
        duration_s: Duration of the move in seconds.
        rate_hz: Output rate in Hz.
        limits: Optional motion limits.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    effective_distance = distance_m
    if limits is not None:
        speed = abs(distance_m) / duration_s
        clamped_speed = abs(limits.clamp_speed(speed))
        effective_distance = math.copysign(clamped_speed * duration_s, distance_m)

    yaw_r = state.heading_r

    # Forward direction in NED: dx (east) = sin(yaw), dy (north) = cos(yaw)
    dx_total = effective_distance * math.sin(yaw_r)
    dy_total = effective_distance * math.cos(yaw_r)

    start_lat = state.lat_deg
    start_lon = state.lon_deg
    roll_r, pitch_r, _ = matrix_to_euler(state.rotation, state.frame)

    for step in range(1, total_steps + 1):
        alpha = step / total_steps
        dx = alpha * dx_total
        dy = alpha * dy_total

        d_lat, d_lon = meters_to_latlon_offset(dy, dx, start_lat)

        yield GeodeticPose(
            position=Lla(lat_deg=start_lat + d_lat, lon_deg=start_lon + d_lon, alt_m=state.alt_m),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )


def move_right_left(
    state: VehicleState,
    distance_m: float,
    duration_s: float,
    rate_hz: float,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that move the vehicle laterally relative to its heading.

    Positive distance moves right; negative moves left. This matches the
    the body-right axis.

    Args:
        state: Current vehicle state.
        distance_m: Distance in metres (positive=right, negative=left).
        duration_s: Duration of the move in seconds.
        rate_hz: Output rate in Hz.
        limits: Optional motion limits.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    effective_distance = distance_m
    if limits is not None:
        speed = abs(distance_m) / duration_s
        clamped_speed = abs(limits.clamp_speed(speed))
        effective_distance = math.copysign(clamped_speed * duration_s, distance_m)

    yaw_r = state.heading_r

    # Right direction: perpendicular to forward, rotated 90 degrees clockwise
    dx_total = effective_distance * math.cos(yaw_r)
    dy_total = -effective_distance * math.sin(yaw_r)

    start_lat = state.lat_deg
    start_lon = state.lon_deg
    roll_r, pitch_r, _ = matrix_to_euler(state.rotation, state.frame)

    for step in range(1, total_steps + 1):
        alpha = step / total_steps
        dx = alpha * dx_total
        dy = alpha * dy_total

        d_lat, d_lon = meters_to_latlon_offset(dy, dx, start_lat)

        yield GeodeticPose(
            position=Lla(lat_deg=start_lat + d_lat, lon_deg=start_lon + d_lon, alt_m=state.alt_m),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )


def move_up_down(
    state: VehicleState,
    distance_m: float,
    duration_s: float,
    rate_hz: float,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that move the vehicle vertically.

    Positive distance moves up; negative moves down.

    Args:
        state: Current vehicle state.
        distance_m: Distance in metres (positive=up, negative=down).
        duration_s: Duration of the move in seconds.
        rate_hz: Output rate in Hz.
        limits: Optional motion limits.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    effective_distance = distance_m
    if limits is not None:
        climb_rate = distance_m / duration_s
        clamped_rate = limits.clamp_climb_rate(climb_rate)
        effective_distance = clamped_rate * duration_s

    start_alt = state.alt_m
    roll_r, pitch_r, yaw_r = matrix_to_euler(state.rotation, state.frame)

    for step in range(1, total_steps + 1):
        alpha = step / total_steps
        alt = start_alt + alpha * effective_distance

        yield GeodeticPose(
            position=Lla(lat_deg=state.lat_deg, lon_deg=state.lon_deg, alt_m=alt),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )


def _turn_interpolated(
    state: VehicleState,
    target_matrix: NDArray[np.float64],
    duration_s: float,
    rate_hz: float,
) -> Iterator[GeodeticPose]:
    """Yield poses that interpolate orientation from current to target using SLERP.

    Position is held constant throughout.

    Args:
        state: Current vehicle state.
        target_matrix: Target 3×3 rotation matrix.
        duration_s: Duration of the rotation.
        rate_hz: Output rate in Hz.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    q_from = _mat_to_quat(state.rotation)
    q_to = _mat_to_quat(target_matrix)

    for step in range(1, total_steps + 1):
        alpha = step / total_steps
        q_interp = slerp(q_from, q_to, alpha)
        mat_interp = _quat_to_mat(q_interp)
        roll_r, pitch_r, yaw_r = matrix_to_euler(mat_interp, state.frame)

        yield GeodeticPose(
            position=Lla(lat_deg=state.lat_deg, lon_deg=state.lon_deg, alt_m=state.alt_m),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )


def _interpolate_to_angles(
    state: VehicleState,
    target_roll_r: float,
    target_pitch_r: float,
    target_yaw_r: float,
    duration_s: float,
    rate_hz: float,
) -> Iterator[GeodeticPose]:
    """Yield poses walking each Euler angle to its target, holding position.

    Used instead of SLERP for world-frame turns, because SLERP takes the shortest great-circle path
    in rotation space and that path does not hold the other angles: turning to face a point swung
    roll up to 45 degrees on the way and back again, so the horizon visibly tipped mid-turn even
    though the endpoints were right. Interpolating the angles themselves keeps the axes the user did
    not ask to move exactly where they were.

    Yaw takes the short way round, so 170 to -170 sweeps 20 degrees rather than 340.

    Args:
        state: Current vehicle state.
        target_roll_r: Target roll in radians.
        target_pitch_r: Target pitch in radians.
        target_yaw_r: Target yaw in radians.
        duration_s: Duration of the rotation.
        rate_hz: Output rate in Hz.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    start_roll_r, start_pitch_r, start_yaw_r = matrix_to_euler(state.rotation, state.frame)
    delta_roll_r = normalize_angle(target_roll_r - start_roll_r)
    delta_pitch_r = normalize_angle(target_pitch_r - start_pitch_r)
    delta_yaw_r = normalize_angle(target_yaw_r - start_yaw_r)

    for step in range(1, total_steps + 1):
        fraction = step / total_steps
        yield GeodeticPose(
            position=Lla(lat_deg=state.lat_deg, lon_deg=state.lon_deg, alt_m=state.alt_m),
            orientation=Rpy(
                roll_r=normalize_angle(start_roll_r + fraction * delta_roll_r),
                pitch_r=normalize_angle(start_pitch_r + fraction * delta_pitch_r),
                yaw_r=normalize_angle(start_yaw_r + fraction * delta_yaw_r),
                frame=Frame.NED,
            ),
        )


def turn_roll(
    state: VehicleState,
    delta_r: float,
    duration_s: float,
    rate_hz: float,
    frame: RotationFrame = RotationFrame.WORLD,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that rotate the vehicle about the roll axis.

    Args:
        state: Current vehicle state.
        delta_r: Roll delta in radians.
        duration_s: Duration of the rotation.
        rate_hz: Output rate in Hz.
        frame: Whether the delta is applied in world or body frame.
        limits: Optional motion limits (turn rate clamping).

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    if limits is not None:
        rate_rad_s = delta_r / duration_s if duration_s > 0 else delta_r
        clamped_rate = limits.clamp_turn_rate_r(rate_rad_s)
        delta_r = clamped_rate * duration_s

    if frame is RotationFrame.WORLD:
        # In the world convention the angles compose as yaw about world up, then pitch, then roll, so
        # "roll by delta" means moving the roll term and leaving the other two alone. Rotating about
        # the fixed world axis instead is a different operation: pitching about world Y while yawed 90
        # degrees rolls the airframe, which is geometrically true and never what the caller asked for.
        current_roll_r, current_pitch_r, current_yaw_r = matrix_to_euler(state.rotation, state.frame)
        yield from _interpolate_to_angles(
            state, current_roll_r + delta_r, current_pitch_r, current_yaw_r, duration_s, rate_hz
        )
        return

    delta_matrix = euler_to_matrix(delta_r, 0.0, 0.0, state.frame)
    target_matrix = compose_rotation(state.rotation, delta_matrix, frame)
    yield from _turn_interpolated(state, target_matrix, duration_s, rate_hz)


def turn_pitch(
    state: VehicleState,
    delta_r: float,
    duration_s: float,
    rate_hz: float,
    frame: RotationFrame = RotationFrame.WORLD,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that rotate the vehicle about the pitch axis.

    Args:
        state: Current vehicle state.
        delta_r: Pitch delta in radians.
        duration_s: Duration of the rotation.
        rate_hz: Output rate in Hz.
        frame: Whether the delta is applied in world or body frame.
        limits: Optional motion limits (turn rate clamping).

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    if limits is not None:
        rate_rad_s = delta_r / duration_s if duration_s > 0 else delta_r
        clamped_rate = limits.clamp_turn_rate_r(rate_rad_s)
        delta_r = clamped_rate * duration_s

    if frame is RotationFrame.WORLD:
        # In the world convention the angles compose as yaw about world up, then pitch, then roll, so
        # "pitch by delta" means moving the pitch term and leaving the other two alone. Rotating about
        # the fixed world axis instead is a different operation: pitching about world Y while yawed 90
        # degrees rolls the airframe, which is geometrically true and never what the caller asked for.
        current_roll_r, current_pitch_r, current_yaw_r = matrix_to_euler(state.rotation, state.frame)
        yield from _interpolate_to_angles(
            state, current_roll_r, current_pitch_r + delta_r, current_yaw_r, duration_s, rate_hz
        )
        return

    delta_matrix = euler_to_matrix(0.0, delta_r, 0.0, state.frame)
    target_matrix = compose_rotation(state.rotation, delta_matrix, frame)
    yield from _turn_interpolated(state, target_matrix, duration_s, rate_hz)


def turn_yaw(
    state: VehicleState,
    delta_r: float,
    duration_s: float,
    rate_hz: float,
    frame: RotationFrame = RotationFrame.WORLD,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that rotate the vehicle about the yaw axis.

    Under ``WORLD`` frame, +yaw always turns about the world vertical,
    whatever the current attitude. Under ``BODY`` frame, +yaw turns about the
    vehicle's own vertical axis.

    Args:
        state: Current vehicle state.
        delta_r: Yaw delta in radians.
        duration_s: Duration of the rotation.
        rate_hz: Output rate in Hz.
        frame: Whether the delta is applied in world or body frame.
        limits: Optional motion limits (turn rate clamping).

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    if limits is not None:
        rate_rad_s = delta_r / duration_s if duration_s > 0 else delta_r
        clamped_rate = limits.clamp_turn_rate_r(rate_rad_s)
        delta_r = clamped_rate * duration_s

    if frame is RotationFrame.WORLD:
        # In the world convention the angles compose as yaw about world up, then pitch, then roll, so
        # "yaw by delta" means moving the yaw term and leaving the other two alone. Rotating about
        # the fixed world axis instead is a different operation: pitching about world Y while yawed 90
        # degrees rolls the airframe, which is geometrically true and never what the caller asked for.
        current_roll_r, current_pitch_r, current_yaw_r = matrix_to_euler(state.rotation, state.frame)
        yield from _interpolate_to_angles(
            state, current_roll_r, current_pitch_r, current_yaw_r + delta_r, duration_s, rate_hz
        )
        return

    delta_matrix = euler_to_matrix(0.0, 0.0, delta_r, state.frame)
    target_matrix = compose_rotation(state.rotation, delta_matrix, frame)
    yield from _turn_interpolated(state, target_matrix, duration_s, rate_hz)


def turn_to_point(
    state: VehicleState,
    target_lat_deg: float,
    target_lon_deg: float,
    target_alt_m: float,
    duration_s: float,
    rate_hz: float,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that rotate the vehicle to face a target point.

    Compute the yaw and pitch required to look at the target from the current
    position, then SLERP to that orientation. Roll is preserved. This matches
    look-at geometry.

    Args:
        state: Current vehicle state.
        target_lat_deg: Target latitude in degrees.
        target_lon_deg: Target longitude in degrees.
        target_alt_m: Target altitude in metres.
        duration_s: Duration of the rotation.
        rate_hz: Output rate in Hz.
        limits: Optional motion limits (turn rate clamping).

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    yaw_r, pitch_r = look_at_angles(
        Lla(lat_deg=state.lat_deg, lon_deg=state.lon_deg, alt_m=state.alt_m),
        Lla(lat_deg=target_lat_deg, lon_deg=target_lon_deg, alt_m=target_alt_m),
    )

    # Preserve current roll
    roll_r, _pitch_r, _yaw_r = matrix_to_euler(state.rotation, state.frame)

    if limits is not None:
        # Accepted and ignored before: a turn-rate limit had no effect on turn_to_point, while
        # turn_yaw/turn_roll/turn_pitch all honour it.
        _, _, current_yaw_r = matrix_to_euler(state.rotation, state.frame)
        sweep_r = abs(normalize_angle(yaw_r - current_yaw_r))
        if duration_s > 0.0 and sweep_r > 0.0:
            implied_rate = sweep_r / duration_s
            clamped_rate = abs(limits.clamp_turn_rate_r(implied_rate))
            if clamped_rate > 0.0 and clamped_rate < implied_rate:
                duration_s = sweep_r / clamped_rate

    # Angle interpolation, not SLERP: aiming is a yaw and pitch operation, and the shortest path in
    # rotation space swung roll up to 45 degrees on the way and back. The endpoints were right, so the
    # horizon tipped visibly mid-turn while every endpoint assertion passed.
    yield from _interpolate_to_angles(state, roll_r, pitch_r, yaw_r, duration_s, rate_hz)


def steer(
    state: VehicleState,
    turn_radius_m: float,
    speed_mps: float,
    duration_s: float,
    rate_hz: float,
    limits: MotionLimits | None = None,
) -> Iterator[GeodeticPose]:
    """Yield poses that drive in a circular arc (constant radius and speed).

    Positive radius = turning right; negative radius = turning left;
    zero radius = straight line. This is the coordinated-turn
    geometry: per-step Euler integration of position and yaw.

    Args:
        state: Current vehicle state.
        turn_radius_m: Turn radius in metres (positive=right, negative=left, 0=straight).
        speed_mps: Forward speed in m/s.
        duration_s: Duration of the steer manoeuvre.
        rate_hz: Output rate in Hz.
        limits: Optional motion limits.

    Yields:
        :class:`~isaac_core.contracts.pose.GeodeticPose` instances.

    """
    total_steps = int(duration_s * rate_hz)
    if total_steps == 0:
        return

    dt = 1.0 / rate_hz

    effective_speed = speed_mps
    if limits is not None:
        effective_speed = limits.clamp_speed(speed_mps)

    yaw_rate = effective_speed / turn_radius_m if turn_radius_m != 0.0 else 0.0
    if limits is not None:
        yaw_rate = limits.clamp_turn_rate_r(yaw_rate)

    current_lat = state.lat_deg
    current_lon = state.lon_deg
    current_rotation = state.rotation.copy()

    for _step in range(total_steps):
        _roll_r, _pitch_r, current_yaw = matrix_to_euler(current_rotation, state.frame)

        # Position update: step forward along current heading
        dist_step = effective_speed * dt
        dx = dist_step * math.sin(current_yaw)
        dy = dist_step * math.cos(current_yaw)

        d_lat, d_lon = meters_to_latlon_offset(dy, dx, current_lat)
        current_lat += d_lat
        current_lon += d_lon

        # Yaw update: apply yaw rate
        if yaw_rate != 0.0:
            delta_yaw_r = yaw_rate * dt
            delta_rotation = euler_to_matrix(0.0, 0.0, delta_yaw_r, state.frame)
            current_rotation = compose_rotation(current_rotation, delta_rotation, RotationFrame.WORLD)

        roll_r, pitch_r, yaw_r = matrix_to_euler(current_rotation, state.frame)
        yield GeodeticPose(
            position=Lla(lat_deg=current_lat, lon_deg=current_lon, alt_m=state.alt_m),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )
