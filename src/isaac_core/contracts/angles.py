"""Angle utilities with no dependencies beyond the standard library."""

import math


def normalize_angle(radians: float) -> float:
    """
    Normalise an angle in radians to the range ``[-pi, pi]``.

    Args:
        radians: Angle in radians, unbounded.

    Returns:
        The equivalent angle in ``[-pi, pi]``.

    """
    return (radians + math.pi) % (2.0 * math.pi) - math.pi


__all__ = ["normalize_angle"]
