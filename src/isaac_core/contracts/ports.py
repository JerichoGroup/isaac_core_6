"""
Default network ports and per-vehicle port allocation.

One UDP port per vehicle, allocated as ``base + index``. That keeps the wire
format unchanged from the previous generation while still supporting a swarm: a
single aircraft lands on the familiar 33333, and additional aircraft follow
sequentially unless a port is set explicitly in config.
"""

from typing import Final

# Base port for inbound pose packets. Vehicle *n* defaults to ``base + n``.
DEFAULT_POSE_UDP_PORT: Final = 33333

# Port for the JSON-RPC control plane. Bound to localhost unless configured otherwise.
DEFAULT_CONTROL_PLANE_PORT: Final = 8760

DEFAULT_RTP_VIDEO_PORT: Final = 5004
DEFAULT_RTP_META_PORT: Final = 5005

# Lowest port we will allocate; below this requires privileges we should not need.
MIN_PORT: Final = 1024

MAX_PORT: Final = 65535


def validate_port(port: int) -> int:
    """
    Check that a port is in the unprivileged range.

    Args:
        port: Port number to check.

    Returns:
        The port unchanged, for convenient inline use.

    Raises:
        ValueError: If the port is outside ``[MIN_PORT, MAX_PORT]``.

    """
    if not MIN_PORT <= port <= MAX_PORT:
        msg = f"port must be in [{MIN_PORT}, {MAX_PORT}], got {port}"
        raise ValueError(msg)
    return port


def pose_port_for_index(index: int, base: int = DEFAULT_POSE_UDP_PORT) -> int:
    """
    Return the default pose port for the vehicle at ``index``.

    Args:
        index: Zero-based vehicle index.
        base: Base port to offset from.

    Returns:
        ``base + index``, validated.

    Raises:
        ValueError: If ``index`` is negative, or the result is out of range.

    """
    if index < 0:
        msg = f"vehicle index must be non-negative, got {index}"
        raise ValueError(msg)
    return validate_port(base + index)


__all__ = [
    "DEFAULT_CONTROL_PLANE_PORT",
    "DEFAULT_POSE_UDP_PORT",
    "DEFAULT_RTP_META_PORT",
    "DEFAULT_RTP_VIDEO_PORT",
    "MAX_PORT",
    "MIN_PORT",
    "pose_port_for_index",
    "validate_port",
]
