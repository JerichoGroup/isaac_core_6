"""LLA ↔ ENU coordinate conversion.

Provide an immutable :class:`EnuConverter` that converts between WGS84 geodetic
coordinates (LLA) and a local East-North-Up tangent plane anchored at a
reference point, using ECEF as the intermediate representation.

pyproj transforms LLA to ECEF via EPSG:4979 → EPSG:4978, then the standard closed-form
ECEF→ENU rotation matrix projects the ECEF delta into the tangent plane. The inverse
(ENU→LLA) inverts the rotation, adds the reference ECEF, and transforms back.
"""

import math

import numpy as np
from numpy.typing import NDArray
from pyproj import Transformer

from isaac_core.contracts.pose import Lla


def _build_enu_rotation_matrix(lat_rad: float, lon_rad: float) -> NDArray[np.float64]:
    """Build the ECEF→ENU rotation matrix for a given reference latitude and longitude.

    Args:
        lat_rad: Reference latitude in radians.
        lon_rad: Reference longitude in radians.

    Returns:
        A 3×3 rotation matrix.

    """
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)

    return np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )


class EnuConverter:
    """Convert between LLA and a local ENU tangent plane.

    Immutable after construction. The pyproj :class:`~pyproj.Transformer` is
    built once and reused for every call, avoiding the cost of CRS resolution on
    each conversion.

    Args:
        ref_lat_deg: Reference latitude in degrees.
        ref_lon_deg: Reference longitude in degrees.
        ref_alt_m: Reference altitude in metres above the WGS84 ellipsoid.

    """

    __slots__ = ("_ecef_to_lla", "_lla_to_ecef", "_ref_ecef", "_rotation", "_rotation_inv")

    def __init__(self, ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float) -> None:
        """Construct the converter from a geodetic reference point."""
        self._lla_to_ecef: Transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        self._ecef_to_lla: Transformer = Transformer.from_crs("EPSG:4978", "EPSG:4979", always_xy=True)

        x0, y0, z0 = self._lla_to_ecef.transform(ref_lon_deg, ref_lat_deg, ref_alt_m)
        self._ref_ecef: NDArray[np.float64] = np.array([x0, y0, z0], dtype=np.float64)

        lat_rad = math.radians(ref_lat_deg)
        lon_rad = math.radians(ref_lon_deg)
        self._rotation: NDArray[np.float64] = _build_enu_rotation_matrix(lat_rad, lon_rad)
        self._rotation_inv: NDArray[np.float64] = self._rotation.T

    def lla_to_enu(self, position: Lla) -> tuple[float, float, float]:
        """Convert an LLA position to ENU metres relative to the reference point.

        Args:
            position: A WGS84 geodetic position.

        Returns:
            A tuple ``(east, north, up)`` in metres.

        """
        x, y, z = self._lla_to_ecef.transform(position.lon_deg, position.lat_deg, position.alt_m)
        delta = np.array([x, y, z], dtype=np.float64) - self._ref_ecef
        enu = self._rotation @ delta
        return (float(enu[0]), float(enu[1]), float(enu[2]))

    def enu_to_lla(self, east: float, north: float, up: float) -> Lla:
        """Convert ENU metres back to an LLA position.

        This is the inverse of :meth:`lla_to_enu`, which many pipelines
        never provided.

        Args:
            east: Eastward offset in metres.
            north: Northward offset in metres.
            up: Upward offset in metres.

        Returns:
            The WGS84 geodetic position corresponding to the given ENU offset.

        """
        enu = np.array([east, north, up], dtype=np.float64)
        ecef = self._rotation_inv @ enu + self._ref_ecef
        lon, lat, alt = self._ecef_to_lla.transform(float(ecef[0]), float(ecef[1]), float(ecef[2]))
        return Lla(lat_deg=lat, lon_deg=lon, alt_m=alt)
