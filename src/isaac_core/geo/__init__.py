"""
Geodesy and rotation maths layer.

Pure-Python functions for geographic coordinate conversions, frame rotations,
SLERP, and distance computations. This package depends only on ``numpy``,
``pyproj``, ``transforms3d``, and :mod:`isaac_core.contracts` -- it never imports
omni, carb, pxr, rclpy, or any other isaac_core subpackage.

Modules:

:mod:`~isaac_core.geo.enu`
    LLA ↔ ENU conversions anchored at a reference point.
:mod:`~isaac_core.geo.rotations`
    NED ↔ ENU frame conversions, Euler/matrix/quaternion utilities, SLERP, and
    configurable rotation composition.
:mod:`~isaac_core.geo.distance`
    Haversine geodesic distance and metre-to-degree offsets.
"""

from isaac_core.geo.distance import geodesic_distance_m, meters_to_latlon_offset
from isaac_core.geo.enu import EnuConverter
from isaac_core.geo.rotations import (
    compose_rotation,
    enu_to_ned,
    euler_to_matrix,
    euler_to_quaternion,
    matrix_to_euler,
    ned_to_enu,
    normalize_angle,
    quaternion_to_euler,
    slerp,
)

__all__ = [
    "EnuConverter",
    "compose_rotation",
    "enu_to_ned",
    "euler_to_matrix",
    "euler_to_quaternion",
    "geodesic_distance_m",
    "matrix_to_euler",
    "meters_to_latlon_offset",
    "ned_to_enu",
    "normalize_angle",
    "quaternion_to_euler",
    "slerp",
]
