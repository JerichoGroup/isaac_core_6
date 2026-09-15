"""Geodesic distance and metre-to-degree offset utilities.

Haversine-based great-circle distance (R = 6 371 000 m). The metre-to-latlon offset function uses unambiguous
parameter names (``north_m``, ``east_m``) to fix the confusing ``(dx, dy)``
signature, where ``dx`` mapped to latitude and ``dy`` to longitude.
"""

import math

from isaac_core.contracts.pose import Lla

# Mean Earth radius in metres.
EARTH_RADIUS_M: float = 6_371_000.0


def geodesic_distance_m(point_a: Lla, point_b: Lla) -> float:
    """Compute the great-circle distance between two points using the haversine formula.

    Uses a spherical Earth model with radius :data:`EARTH_RADIUS_M`.

    Args:
        point_a: First WGS84 geodetic position (altitude is ignored).
        point_b: Second WGS84 geodetic position (altitude is ignored).

    Returns:
        Distance in metres along the surface.

    """
    lat1 = math.radians(point_a.lat_deg)
    lon1 = math.radians(point_a.lon_deg)
    lat2 = math.radians(point_b.lat_deg)
    lon2 = math.radians(point_b.lon_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2

    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def meters_to_latlon_offset(north_m: float, east_m: float, ref_lat_deg: float) -> tuple[float, float]:
    """Convert north/east metre offsets to latitude/longitude degree offsets.

    Uses a spherical approximation, with unambiguous
    parameter names. The old signature was ``(dx, dy, ref_lat)`` where ``dx``
    mapped to latitude and ``dy`` to longitude -- a naming that caused confusion
    and bugs. Here: ``north_m`` → latitude, ``east_m`` → longitude.

    Args:
        north_m: Northward offset in metres (positive = increasing latitude).
        east_m: Eastward offset in metres (positive = increasing longitude).
        ref_lat_deg: Reference latitude in degrees, used to scale the longitude
            offset by ``cos(lat)``.

    Returns:
        A tuple ``(d_lat_deg, d_lon_deg)``.

    """
    d_lat_rad = north_m / EARTH_RADIUS_M
    d_lon_rad = east_m / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat_deg)))

    return (math.degrees(d_lat_rad), math.degrees(d_lon_rad))
