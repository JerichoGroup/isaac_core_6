"""
Motion limits: optional caps on speed, acceleration, turn rate, and climb rate.

``MotionLimits`` is a frozen dataclass whose fields are all ``None | float``.
``None`` means unlimited -- reproducing the old constant-speed, no-dynamics
behaviour exactly so existing scenarios stay reproducible. When values are
supplied, the pure clamping helpers restrict commanded motion without
implementing a full dynamics model: this is the seam where acceleration-limited
motion gets added later while the pure-kinematic path stays unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class MotionLimits:
    """
    Optional kinematic constraints on vehicle motion.

    Each field is ``None`` (unlimited) or a positive ``float``. The clamping
    helpers are pure -- they return a bounded value without any side effects.

    Args:
        max_speed_mps: Maximum speed in metres per second.
        max_accel_mps2: Maximum acceleration in m/s².
        max_turn_rate_deg_s: Maximum turn rate in degrees per second.
        max_climb_rate_mps: Maximum vertical speed in metres per second.

    """

    max_speed_mps: float | None = None
    max_accel_mps2: float | None = None
    max_turn_rate_deg_s: float | None = None
    max_climb_rate_mps: float | None = None

    def __post_init__(self) -> None:
        """Validate that all non-None limits are positive."""
        for field_name in ("max_speed_mps", "max_accel_mps2", "max_turn_rate_deg_s", "max_climb_rate_mps"):
            value = getattr(self, field_name)
            if value is not None and value <= 0.0:
                msg = f"{field_name} must be positive, got {value}"
                raise ValueError(msg)

    def clamp_speed(self, speed_mps: float) -> float:
        """
        Clamp a speed value to the configured maximum.

        The sign of ``speed_mps`` is preserved so negative (backward) motion
        is still possible; only the magnitude is capped.

        Args:
            speed_mps: Commanded speed in m/s (may be negative for reverse).

        Returns:
            The clamped speed.

        """
        if self.max_speed_mps is None:
            return speed_mps
        return math.copysign(min(abs(speed_mps), self.max_speed_mps), speed_mps)

    def clamp_turn_rate_r(self, rate_rad_s: float) -> float:
        """
        Clamp an angular rate (radians/s) to the configured maximum.

        The sign is preserved (positive = clockwise in NED yaw convention).

        Args:
            rate_rad_s: Commanded turn rate in radians per second.

        Returns:
            The clamped rate in radians per second.

        """
        if self.max_turn_rate_deg_s is None:
            return rate_rad_s
        max_rad_s = math.radians(self.max_turn_rate_deg_s)
        return math.copysign(min(abs(rate_rad_s), max_rad_s), rate_rad_s)

    def clamp_climb_rate(self, climb_mps: float) -> float:
        """
        Clamp a vertical speed to the configured maximum.

        The sign is preserved (positive = ascending).

        Args:
            climb_mps: Commanded vertical speed in m/s.

        Returns:
            The clamped vertical speed.

        """
        if self.max_climb_rate_mps is None:
            return climb_mps
        return math.copysign(min(abs(climb_mps), self.max_climb_rate_mps), climb_mps)

    def effective_speed(self, desired_mps: float, dt: float, current_speed_mps: float) -> float:
        """
        Compute the effective speed after applying both speed and acceleration limits.

        If acceleration is unlimited, returns the clamped desired speed directly.
        Otherwise, the speed change per time step is limited by
        ``max_accel_mps2 * dt``.

        Args:
            desired_mps: The speed the caller wants.
            dt: Time step in seconds.
            current_speed_mps: Current speed in m/s.

        Returns:
            The achievable speed for this time step.

        """
        target = self.clamp_speed(desired_mps)
        if self.max_accel_mps2 is None:
            return target
        max_change = self.max_accel_mps2 * dt
        delta = target - current_speed_mps
        clamped_delta = math.copysign(min(abs(delta), max_change), delta)
        return current_speed_mps + clamped_delta

    @classmethod
    def unlimited(cls) -> MotionLimits:
        """
        Return a limits instance with no constraints.

        This reproduces the old constant-speed, no-dynamics behaviour exactly.

        Returns:
            A :class:`MotionLimits` with all fields ``None``.

        """
        return cls()
