"""Trajectory generators: deterministic, pure iterators that yield geodetic poses.

A :class:`Trajectory` is a protocol with a ``poses(rate_hz)`` method returning
an :class:`~collections.abc.Iterator` of
:class:`~isaac_core.contracts.pose.GeodeticPose`. Each implementation is pure,
synchronous, and I/O-free -- the caller is responsible for pacing and transport.
Identical inputs always produce identical sequences.

Implementations:

:class:`HoldTrajectory`
    Yield a single fixed pose forever (unbounded duration).
:class:`OrbitTrajectory`
    Circular orbit with yaw tangent to the path.
:class:`PathTrajectory`
    Constant-speed arc-length interpolation over LLA waypoints.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import math
from typing import Protocol, runtime_checkable

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.geo.distance import geodesic_distance_m, meters_to_latlon_offset


@runtime_checkable
class Trajectory(Protocol):
    """Protocol for a deterministic trajectory generator.

    A trajectory has a known duration and yields exactly
    ``int(duration_s * rate_hz)`` poses when iterated. Implementations must be
    pure: no I/O, no wall-clock reads, no randomness.
    """

    @property
    def duration_s(self) -> float:
        """Return the total trajectory duration in seconds."""
        ...  # pragma: no cover

    def poses(self, rate_hz: float) -> Iterator[GeodeticPose]:
        """Yield poses at the given rate.

        The number of yielded poses is exactly ``int(duration_s * rate_hz)``.

        Args:
            rate_hz: Output rate in Hz (samples per second).

        Yields:
            :class:`~isaac_core.contracts.pose.GeodeticPose` instances in NED frame.

        """
        ...  # pragma: no cover


@dataclass(frozen=True, slots=True)
class HoldTrajectory:
    """Yield a single fixed pose indefinitely.

    Replace the old ``OnePointSender``'s payload role. Since the duration is
    infinite, ``poses()`` yields ``int(duration_s * rate_hz)`` samples where
    ``duration_s`` is ``math.inf`` -- in practice, the caller stops iteration.

    Args:
        pose: The pose to hold.

    """

    pose: GeodeticPose

    @property
    def duration_s(self) -> float:
        """Return infinite duration."""
        return math.inf

    def poses(self, rate_hz: float) -> Iterator[GeodeticPose]:
        """Yield the held pose forever.

        Args:
            rate_hz: Output rate in Hz (ignored for hold; included for protocol).

        Yields:
            The same :class:`~isaac_core.contracts.pose.GeodeticPose` repeatedly.

        """
        while True:
            yield self.pose


@dataclass(frozen=True, slots=True)
class OrbitTrajectory:
    """Circular orbit around a centre point with yaw tangent to the path.

    Match the geometry of the old ``OrbitSender``: the orbit lies in a
    horizontal plane at ``height_m`` MSL, the vehicle travels at constant
    ground speed, and the yaw always points tangent to the circle (computed
    from the finite difference to the next sample).

    Args:
        center_lat_deg: Centre latitude in degrees.
        center_lon_deg: Centre longitude in degrees.
        radius_m: Orbit radius in metres (must be positive).
        height_m: Altitude in metres MSL for the orbit plane.
        speed_mps: Ground speed in m/s (must be positive).
        orbit_duration_s: Total orbit duration in seconds.
        roll_r: Fixed roll angle in radians (default 0).
        pitch_r: Fixed pitch angle in radians (default 0).

    """

    center_lat_deg: float
    center_lon_deg: float
    radius_m: float
    height_m: float
    speed_mps: float
    orbit_duration_s: float
    roll_r: float = 0.0
    pitch_r: float = 0.0

    def __post_init__(self) -> None:
        """Validate orbit parameters."""
        if self.radius_m <= 0.0:
            msg = f"radius_m must be positive, got {self.radius_m}"
            raise ValueError(msg)
        if self.speed_mps <= 0.0:
            msg = f"speed_mps must be positive, got {self.speed_mps}"
            raise ValueError(msg)
        if self.orbit_duration_s <= 0.0:
            msg = f"orbit_duration_s must be positive, got {self.orbit_duration_s}"
            raise ValueError(msg)

    @property
    def duration_s(self) -> float:
        """Return the orbit duration."""
        return self.orbit_duration_s

    def poses(self, rate_hz: float) -> Iterator[GeodeticPose]:
        """Yield orbit poses at the given rate.

        Exactly ``int(duration_s * rate_hz)`` samples are yielded. The yaw at
        each sample is the tangent direction derived from the finite difference
        to the next sample.

        Args:
            rate_hz: Output rate in Hz.

        Yields:
            :class:`~isaac_core.contracts.pose.GeodeticPose` with NED orientation.

        """
        total_steps = int(self.orbit_duration_s * rate_hz)
        if total_steps == 0:
            return

        circumference = 2.0 * math.pi * self.radius_m
        num_cycles = (self.speed_mps * self.orbit_duration_s) / circumference

        for step in range(total_steps):
            # Angle for current position
            t = 2.0 * math.pi * num_cycles * (step / total_steps)
            x = self.radius_m * math.cos(t)
            y = self.radius_m * math.sin(t)

            d_lat, d_lon = meters_to_latlon_offset(x, y, self.center_lat_deg)
            lat = self.center_lat_deg + d_lat
            lon = self.center_lon_deg + d_lon

            # Yaw from the finite difference to the next step
            t_next = 2.0 * math.pi * num_cycles * ((step + 1) / total_steps)
            x_next = self.radius_m * math.cos(t_next)
            y_next = self.radius_m * math.sin(t_next)
            dx = x_next - x
            dy = y_next - y
            yaw = math.atan2(dy, dx)

            yield GeodeticPose(
                position=Lla(lat_deg=lat, lon_deg=lon, alt_m=self.height_m),
                orientation=Rpy(roll_r=self.roll_r, pitch_r=self.pitch_r, yaw_r=yaw, frame=Frame.NED),
            )


# Minimum number of waypoints for a valid path.
_MIN_WAYPOINTS = 2


@dataclass(frozen=True, slots=True)
class PathTrajectory:
    """Constant-speed arc-length interpolation over LLA waypoints.

    Match the old ``PathSender``'s cumulative-distance approach: the path is
    parameterised by arc length, the vehicle travels at constant ground speed,
    and the yaw at each sample points along the current segment. The duration
    is determined by ``total_distance / speed_mps``.

    Args:
        waypoints: A sequence of at least 2
            :class:`~isaac_core.contracts.pose.Lla` positions.
        speed_mps: Ground speed in m/s (must be positive).
        roll_r: Fixed roll angle in radians (default 0).
        pitch_r: Fixed pitch angle in radians (default 0).

    Raises:
        ValueError: If fewer than 2 waypoints are provided, or speed is
            non-positive.

    """

    waypoints: tuple[Lla, ...]
    speed_mps: float
    roll_r: float = 0.0
    pitch_r: float = 0.0

    def __post_init__(self) -> None:
        """Validate path parameters."""
        if len(self.waypoints) < _MIN_WAYPOINTS:
            msg = f"PathTrajectory requires at least {_MIN_WAYPOINTS} waypoints, got {len(self.waypoints)}"
            raise ValueError(msg)
        if self.speed_mps <= 0.0:
            msg = f"speed_mps must be positive, got {self.speed_mps}"
            raise ValueError(msg)

    @property
    def duration_s(self) -> float:
        """Return the total path duration based on distance and speed.

        Duration is ``total_distance / speed_mps``.

        """
        return self._total_distance / self.speed_mps

    @property
    def _cumulative_distances(self) -> list[float]:
        """Compute the cumulative distance array along the waypoint sequence."""
        distances = [0.0]
        for i in range(1, len(self.waypoints)):
            d = geodesic_distance_m(self.waypoints[i - 1], self.waypoints[i])
            distances.append(distances[-1] + d)
        return distances

    @property
    def _total_distance(self) -> float:
        """Return total path length in metres."""
        return self._cumulative_distances[-1]

    def poses(self, rate_hz: float) -> Iterator[GeodeticPose]:
        """Yield path poses at the given rate.

        Exactly ``int(duration_s * rate_hz)`` samples are yielded. The position
        is interpolated linearly in LLA between waypoints based on cumulative
        arc-length distance. Yaw points along the current segment.

        Args:
            rate_hz: Output rate in Hz.

        Yields:
            :class:`~isaac_core.contracts.pose.GeodeticPose` with NED orientation.

        """
        cumulative = self._cumulative_distances
        total_dist = cumulative[-1]
        total_duration = total_dist / self.speed_mps
        total_steps = int(total_duration * rate_hz)
        if total_steps == 0:
            return

        for step in range(total_steps):
            target_dist = step * self.speed_mps / rate_hz

            # Find the segment containing target_dist
            seg_idx = 1
            while seg_idx < len(cumulative) and cumulative[seg_idx] < target_dist:
                seg_idx += 1

            if seg_idx >= len(cumulative):
                seg_idx = len(cumulative) - 1

            d_start = cumulative[seg_idx - 1]
            d_end = cumulative[seg_idx]
            seg_frac = (target_dist - d_start) / (d_end - d_start) if d_end != d_start else 0.0

            p1 = self.waypoints[seg_idx - 1]
            p2 = self.waypoints[seg_idx]

            lat = p1.lat_deg + seg_frac * (p2.lat_deg - p1.lat_deg)
            lon = p1.lon_deg + seg_frac * (p2.lon_deg - p1.lon_deg)
            alt = p1.alt_m + seg_frac * (p2.alt_m - p1.alt_m)

            # Yaw points along the segment direction
            d_lat = p2.lat_deg - p1.lat_deg
            d_lon = p2.lon_deg - p1.lon_deg
            yaw = math.atan2(d_lon, d_lat) if (d_lat != 0.0 or d_lon != 0.0) else 0.0

            yield GeodeticPose(
                position=Lla(lat_deg=lat, lon_deg=lon, alt_m=alt),
                orientation=Rpy(roll_r=self.roll_r, pitch_r=self.pitch_r, yaw_r=yaw, frame=Frame.NED),
            )
