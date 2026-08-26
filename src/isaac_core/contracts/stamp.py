"""
ROS 2 message timestamp representation.

A ROS 2 ``builtin_interfaces/msg/Time`` is a pair of integers, not a float: ``sec`` is a
signed 32-bit count of seconds and ``nanosec`` is an unsigned 32-bit remainder that must
stay inside ``[0, 1e9)``. Isaac Sim, meanwhile, reports simulation time as a single
``double`` in seconds (``isaacsim.core.nodes.IsaacReadSimulationTime`` emits
``outputs:simulationTime``).

Bridging those two shapes is the whole reason this module exists. Without it a
``GeoPoseStamped`` publishes ``stamp: {sec: 0, nanosec: 0}``, which looks like a valid
message but carries no usable time, so a consumer cannot order or correlate samples.

Kept here, pure and dependency-free, rather than inside the OmniGraph node so the
arithmetic -- which has three easy-to-miss edge cases -- can be tested without Isaac Sim.
"""

from __future__ import annotations

import math

# Nanoseconds in one second. The ROS 2 `nanosec` field must stay strictly below this.
NANOSECONDS_PER_SECOND: int = 1_000_000_000

# Inclusive bounds of the ROS 2 `sec` field, which is int32.
#
# Simulation time will not realistically reach these, but a graph can feed an
# uninitialised or garbage double into the node, and silently wrapping a timestamp is far
# worse than refusing to produce one.
SEC_MIN: int = -(2**31)
SEC_MAX: int = 2**31 - 1


def seconds_to_ros_stamp(seconds: float) -> tuple[int, int]:
    """
    Split a floating-point seconds value into a ROS 2 ``(sec, nanosec)`` pair.

    Uses floor semantics, so the remainder is always non-negative and ``nanosec`` is
    always a valid unsigned value even for negative inputs -- ``-0.25`` becomes
    ``(-1, 750000000)``, which is the same instant, not ``(0, -250000000)``, which
    ``builtin_interfaces/Time`` cannot represent.

    Args:
        seconds: Time in seconds, typically Isaac's ``outputs:simulationTime``.

    Returns:
        ``(sec, nanosec)`` with ``0 <= nanosec < 1e9``.

    Raises:
        ValueError: If ``seconds`` is NaN or infinite, or if the result would not fit in
            the int32 ``sec`` field.

    """
    if not math.isfinite(seconds):
        message = f"simulation time must be finite, got {seconds!r}"
        raise ValueError(message)

    sec = math.floor(seconds)
    nanosec = round((seconds - sec) * NANOSECONDS_PER_SECOND)

    # Rounding the remainder can land exactly on one full second (for example
    # 1.9999999999 rounds to 1e9 nanoseconds). Carrying keeps nanosec in range instead of
    # emitting a value that ROS treats as malformed.
    if nanosec >= NANOSECONDS_PER_SECOND:
        sec += 1
        nanosec -= NANOSECONDS_PER_SECOND

    if not SEC_MIN <= sec <= SEC_MAX:
        message = f"seconds value {seconds!r} does not fit in the int32 ROS 2 sec field"
        raise ValueError(message)

    return sec, nanosec
