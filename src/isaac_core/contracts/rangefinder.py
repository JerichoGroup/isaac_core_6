"""
Range-sensor value handling, independent of any physics engine.

The OmniGraph node owns the raycast call; everything about *interpreting* the result lives
here so it can be tested without Isaac Sim. That split matters because the interesting cases
are the boundary ones -- nothing hit, a hit beyond the sensor's rated range, a hit closer than
its minimum -- and `sensor_msgs/Range` has explicit conventions for each.

`sensor_msgs/Range` semantics, which this module implements:

- A reading outside ``[min_range, max_range]`` should be reported as an out-of-range value
  rather than silently clamped, so a consumer can tell "nothing there" from "something at
  exactly max range".
- ROS convention for "no detection" on a rangefinder is ``+inf``; for "too close to measure"
  it is ``-inf``.
"""

from __future__ import annotations

import math

# Reported when the ray hits nothing within its rated maximum. ROS convention for a
# rangefinder that detected no obstacle.
NO_DETECTION: float = math.inf

# Reported when a hit is closer than the sensor's rated minimum, which real hardware cannot
# measure. ROS convention is negative infinity for this case.
TOO_CLOSE: float = -math.inf


def resolve_range(
    hit: bool,
    distance_m: float,
    min_range_m: float,
    max_range_m: float,
) -> float:
    """
    Turn a raw raycast result into a ``sensor_msgs/Range`` value.

    Args:
        hit: Whether the ray struck anything at all.
        distance_m: Distance to the hit, in metres. Ignored when ``hit`` is false.
        min_range_m: Sensor's rated minimum measurable distance.
        max_range_m: Sensor's rated maximum measurable distance.

    Returns:
        The distance in metres when it lies inside the rated band, :data:`NO_DETECTION` when
        nothing was hit or the hit is beyond ``max_range_m``, or :data:`TOO_CLOSE` when the
        hit is nearer than ``min_range_m``.

    Raises:
        ValueError: If the rated band is not a positive interval, since every reading would
            otherwise be meaningless.

    """
    if not math.isfinite(min_range_m) or not math.isfinite(max_range_m):
        message = f"range bounds must be finite, got [{min_range_m}, {max_range_m}]"
        raise ValueError(message)
    if min_range_m < 0.0:
        message = f"min_range must not be negative, got {min_range_m}"
        raise ValueError(message)
    if max_range_m <= min_range_m:
        message = f"max_range must exceed min_range, got [{min_range_m}, {max_range_m}]"
        raise ValueError(message)

    if not hit:
        return NO_DETECTION
    if not math.isfinite(distance_m):
        return NO_DETECTION
    if distance_m < min_range_m:
        return TOO_CLOSE
    if distance_m > max_range_m:
        return NO_DETECTION
    return float(distance_m)


def is_valid_reading(range_m: float) -> bool:
    """
    Report whether a range value is an actual measurement.

    Convenience for consumers, so they do not each re-derive that both infinities mean
    "no usable measurement".

    Args:
        range_m: A value produced by :func:`resolve_range`.

    Returns:
        ``True`` when the value is a real distance rather than a sentinel.

    """
    return math.isfinite(range_m)
