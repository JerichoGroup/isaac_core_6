"""
Gimbal angle state and slew-rate limiting, independent of any physics engine.

The OmniGraph node owns writing the resulting orientation onto the gimbal prim; everything
about *how far the gimbal is allowed to move this tick* lives here so it can be tested without
Isaac Sim. The interesting cases are the boundary ones -- wrapping across the +/-pi seam,
a rate of zero meaning "no limit", a non-positive timestep -- and each is easy to get subtly
wrong, so they are pinned here rather than in node code.

Config expresses gimbal start angles and limits in degrees; this kernel works in radians.
Cross the boundary explicitly with :meth:`GimbalAngles.from_degrees` and
:meth:`GimbalAngles.to_degrees` rather than letting the two units mix.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees, radians

from isaac_core.contracts.angles import normalize_angle


@dataclass(frozen=True, slots=True)
class GimbalAngles:
    """
    A gimbal orientation as roll, pitch and yaw in radians.

    Radians are used internally because that is what every rotation utility in the stack
    expects. Config and user-facing APIs speak degrees; convert at the edges with
    :meth:`from_degrees` and :meth:`to_degrees` rather than carrying degrees around.

    Args:
        roll_r: Roll in radians.
        pitch_r: Pitch in radians.
        yaw_r: Yaw in radians.

    """

    roll_r: float
    pitch_r: float
    yaw_r: float

    @classmethod
    def from_degrees(cls, roll_deg: float, pitch_deg: float, yaw_deg: float) -> GimbalAngles:
        """Build from degrees, the unit used in config and user-facing APIs."""
        return cls(radians(roll_deg), radians(pitch_deg), radians(yaw_deg))

    def to_degrees(self) -> tuple[float, float, float]:
        """Return ``(roll_deg, pitch_deg, yaw_deg)`` for display or user-facing APIs."""
        return (degrees(self.roll_r), degrees(self.pitch_r), degrees(self.yaw_r))

    def as_tuple(self) -> tuple[float, float, float]:
        """Return ``(roll_r, pitch_r, yaw_r)`` in radians."""
        return (self.roll_r, self.pitch_r, self.yaw_r)

    def normalized(self) -> GimbalAngles:
        """Return a copy with every axis wrapped to ``[-pi, pi]``."""
        return GimbalAngles(
            normalize_angle(self.roll_r),
            normalize_angle(self.pitch_r),
            normalize_angle(self.yaw_r),
        )


# Per-axis mechanical stops in radians. ``None`` for either bound leaves that side
# unconstrained. Ordering is ``(roll, pitch, yaw)`` to match GimbalAngles.
AxisLimit = tuple[float | None, float | None]
GimbalLimits = tuple[AxisLimit, AxisLimit, AxisLimit]


def _step_axis(current_r: float, target_r: float, max_step_r: float | None) -> float:
    """
    Move one axis toward its target along the shortest angular path.

    Args:
        current_r: Current angle in radians.
        target_r: Target angle in radians.
        max_step_r: Largest permitted move this tick in radians, or ``None`` for an
            unlimited (instant) move to the target.

    Returns:
        The new angle in radians, normalised to ``[-pi, pi]``.

    """
    # Shortest path: the wrapped delta is the signed short-arc distance, so 179 -> -179 is
    # a +2 degree move rather than -358 degrees.
    delta_r = normalize_angle(target_r - current_r)

    if max_step_r is None:
        return normalize_angle(target_r)

    # No-overshoot: when the remaining move is within one step, land exactly on the target.
    if abs(delta_r) <= max_step_r:
        return normalize_angle(target_r)

    step_r = max_step_r if delta_r > 0.0 else -max_step_r
    return normalize_angle(current_r + step_r)


def slew_towards(
    current: GimbalAngles,
    target: GimbalAngles,
    max_rate_r_s: float,
    dt_s: float,
) -> GimbalAngles:
    """
    Move a gimbal toward a target orientation, rate-limited per axis.

    Each axis is limited independently and travels the shortest angular path, so crossing the
    +/-pi seam takes the short way round. Outputs are normalised to ``[-pi, pi]``.

    A ``max_rate_r_s`` of zero or negative means UNLIMITED: the gimbal snaps straight to the
    target in a single call. This is surprising but deliberate -- it matches the config default
    of ``0`` meaning "no slew limit", so an unset limit behaves as an instant follow rather than
    freezing the gimbal in place.

    A ``dt_s`` of zero or negative returns ``current`` unchanged rather than dividing by or
    extrapolating over a non-positive interval.

    Args:
        current: Current gimbal orientation.
        target: Desired gimbal orientation.
        max_rate_r_s: Maximum angular rate per axis in radians per second. Zero or negative
            means no limit.
        dt_s: Elapsed time since the last update in seconds.

    Returns:
        The new gimbal orientation, normalised to ``[-pi, pi]``.

    """
    if dt_s <= 0.0:
        return current.normalized()

    max_step_r: float | None = max_rate_r_s * dt_s if max_rate_r_s > 0.0 else None

    return GimbalAngles(
        _step_axis(current.roll_r, target.roll_r, max_step_r),
        _step_axis(current.pitch_r, target.pitch_r, max_step_r),
        _step_axis(current.yaw_r, target.yaw_r, max_step_r),
    )


def _clamp_axis(value_r: float, limit: AxisLimit) -> float:
    """
    Clamp one axis to its mechanical stops.

    Args:
        value_r: Angle in radians.
        limit: ``(min_r, max_r)`` bounds in radians; ``None`` on either side leaves that
            direction unconstrained.

    Returns:
        The clamped angle in radians, normalised to ``[-pi, pi]``.

    Raises:
        ValueError: If both bounds are present and ``max_r < min_r``, an empty interval.

    """
    min_r, max_r = limit
    if min_r is not None and max_r is not None and max_r < min_r:
        message = f"axis limit max must not be below min, got [{min_r}, {max_r}]"
        raise ValueError(message)

    clamped = value_r
    if min_r is not None and clamped < min_r:
        clamped = min_r
    if max_r is not None and clamped > max_r:
        clamped = max_r
    return normalize_angle(clamped)


def clamp_angles(angles: GimbalAngles, limits: GimbalLimits) -> GimbalAngles:
    """
    Clamp gimbal angles to per-axis mechanical stops.

    Each axis carries an optional ``(min_r, max_r)`` pair; ``None`` on either side leaves that
    direction unconstrained, which models a gimbal free to spin one way but stopped the other.
    Outputs are normalised to ``[-pi, pi]``.

    Args:
        angles: Gimbal orientation to clamp.
        limits: ``(roll_limit, pitch_limit, yaw_limit)``, each an ``(min_r, max_r)`` pair with
            ``None`` for an unconstrained bound.

    Returns:
        The clamped gimbal orientation, normalised to ``[-pi, pi]``.

    Raises:
        ValueError: If any axis has both bounds present with ``max_r < min_r``.

    """
    roll_limit, pitch_limit, yaw_limit = limits
    return GimbalAngles(
        _clamp_axis(angles.roll_r, roll_limit),
        _clamp_axis(angles.pitch_r, pitch_limit),
        _clamp_axis(angles.yaw_r, yaw_limit),
    )


__all__ = ["GimbalAngles", "GimbalLimits", "clamp_angles", "slew_towards"]
