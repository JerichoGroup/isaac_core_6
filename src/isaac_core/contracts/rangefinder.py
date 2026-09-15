"""Interpretation of a raw raycast hit as a ``sensor_msgs/Range`` reading.

Pure logic with no Isaac Sim dependency, so the boundary rules -- nothing detected, closer than
the sensor can measure, beyond its reach -- are testable in isolation and live in exactly one
place. The OmniGraph node performs the raycast and delegates every judgement here.

**Out-of-band readings saturate at the rated limits rather than reporting infinity.** ROS's own
convention for ``sensor_msgs/Range`` is +inf for "nothing detected" and -inf for "too close", and
that was the original implementation. It was changed deliberately: ``float("inf")`` cannot be cast
to ``int`` without raising, so a perfectly reasonable consumer doing ``int(msg.range)`` crashes
exactly when the sensor sees nothing -- which is the common case for a downward rangefinder in
level flight. Saturating keeps every published reading a finite number, and a value parked at the
limit is itself the signal that the target is out of range.

The cost, stated plainly: "exactly at the limit" and "beyond the limit" are no longer
distinguishable. For a rangefinder whose rated limit is approximate anyway, that is a good trade.
Use :func:`is_saturated` when a consumer needs to know it is looking at a clamped value.
"""

import math

__all__ = [
    "boresight_direction",
    "clamp_to_band",
    "is_saturated",
    "is_valid_reading",
    "resolve_range",
]


def resolve_range(
    hit: bool,
    distance_m: float,
    min_range_m: float,
    max_range_m: float,
) -> float:
    """Turn a raw raycast result into a publishable range in metres.

    Args:
        hit: Whether the ray struck anything.
        distance_m: Distance to the hit in metres; ignored when ``hit`` is ``False``.
        min_range_m: Closest distance the sensor can report.
        max_range_m: Furthest distance the sensor can report.

    Returns:
        A finite distance in metres, clamped into ``[min_range_m, max_range_m]``. Nothing detected
        reports ``max_range_m``; a hit nearer than the rated minimum reports ``min_range_m``.

    Raises:
        ValueError: If the rated band is not a positive, finite interval, since every reading
            would then be meaningless.

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

    # No hit, and a non-finite distance from a misbehaving query, both mean "saw nothing".
    if not hit or not math.isfinite(distance_m):
        return float(max_range_m)
    return clamp_to_band(distance_m, min_range_m, max_range_m)


def clamp_to_band(distance_m: float, min_range_m: float, max_range_m: float) -> float:
    """Clamp a measured distance into the sensor's rated band.

    Args:
        distance_m: Measured distance in metres.
        min_range_m: Closest reportable distance.
        max_range_m: Furthest reportable distance.

    Returns:
        The distance, or the nearer band edge when it falls outside.

    """
    return float(min(max(distance_m, min_range_m), max_range_m))


def is_saturated(range_m: float, min_range_m: float, max_range_m: float) -> bool:
    """Report whether a reading is sitting on a band edge.

    A reading at an edge is either a genuine measurement at that exact distance or a clamped
    out-of-band one; they are indistinguishable by design. Consumers that must not act on a
    clamped value should treat both the same way.

    Args:
        range_m: The published range.
        min_range_m: Closest reportable distance.
        max_range_m: Furthest reportable distance.

    Returns:
        ``True`` when the reading equals either band edge.

    """
    return range_m <= min_range_m or range_m >= max_range_m


def is_valid_reading(range_m: float, min_range_m: float, max_range_m: float) -> bool:
    """Report whether a reading is strictly inside the rated band.

    Args:
        range_m: The published range.
        min_range_m: Closest reportable distance.
        max_range_m: Furthest reportable distance.

    Returns:
        ``True`` when the value is a measurement not sitting on either limit.

    """
    return math.isfinite(range_m) and not is_saturated(range_m, min_range_m, max_range_m)


def boresight_direction(
    rotation_rows: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
) -> tuple[float, float, float]:
    """Return the direction a USD camera looks, given its world rotation matrix rows.

    A USD camera looks along its own local **-Z**, so the direction is the negated third row of its
    world rotation. Deriving it from the *camera's* transform rather than a sibling prim's is what
    makes a boresighted sensor structurally aligned: a plain Xform is identity while the camera
    carries its own local orientation, so casting along the Xform's -Z was permanently 90 degrees off
    the view.

    Args:
        rotation_rows: The three rows of the camera's world rotation matrix.

    Returns:
        A direction vector along the camera's view axis. Not normalised: a rotation matrix's rows are
        already unit length.

    """
    _, _, third = rotation_rows
    return (-third[0], -third[1], -third[2])
