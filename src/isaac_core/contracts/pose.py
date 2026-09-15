"""Frame-tagged pose value types.

These are immutable, hashable, dependency-free carriers used across the whole
codebase, from the UDP codec up to the vehicle model. Angles are tagged with the
frame they are expressed in, so a NED value cannot be silently passed where an ENU
value is expected, which is the usual cause of orientation bugs.
"""

from dataclasses import dataclass, replace
from math import degrees, radians

from isaac_core.contracts.frames import Frame

MIN_LAT_DEG = -90.0
MAX_LAT_DEG = 90.0
MIN_LON_DEG = -180.0
MAX_LON_DEG = 180.0


@dataclass(frozen=True, slots=True)
class Lla:
    """A WGS84 geodetic position.

    Args:
        lat_deg: Latitude in degrees, in ``[-90, 90]``.
        lon_deg: Longitude in degrees, in ``[-180, 180]``.
        alt_m: Altitude in metres, in the same datum as ``geo.enu_reference.alt_m``. The ENU
            conversion sends both through an ECEF transform that treats height as ellipsoidal, but
            because the position and the reference share a datum, a consistent mean-sea-level input
            still yields a correct local ENU to within the geoid variation across the scene. What
            matters is that the sender and the reference agree; the wire format documents metres
            with sea level at 0, which is what a GPS or MAVLink source provides.

    """

    lat_deg: float
    lon_deg: float
    alt_m: float

    def __post_init__(self) -> None:
        """Reject positions that cannot exist on the ellipsoid."""
        if not MIN_LAT_DEG <= self.lat_deg <= MAX_LAT_DEG:
            msg = f"lat_deg must be in [{MIN_LAT_DEG}, {MAX_LAT_DEG}], got {self.lat_deg}"
            raise ValueError(msg)
        if not MIN_LON_DEG <= self.lon_deg <= MAX_LON_DEG:
            msg = f"lon_deg must be in [{MIN_LON_DEG}, {MAX_LON_DEG}], got {self.lon_deg}"
            raise ValueError(msg)

    def as_tuple(self) -> tuple[float, float, float]:
        """Return ``(lat_deg, lon_deg, alt_m)``, the ordering used on the wire."""
        return (self.lat_deg, self.lon_deg, self.alt_m)


@dataclass(frozen=True, slots=True)
class Rpy:
    """An attitude as intrinsic-XYZ Euler angles in radians, tagged with its frame.

    Radians are used because that is what the wire format and every rotation
    library in the stack expect. Convert at the edges with :meth:`from_degrees` and
    :meth:`to_degrees` rather than carrying degrees around internally.

    Args:
        roll_r: Roll in radians.
        pitch_r: Pitch in radians.
        yaw_r: Yaw in radians.
        frame: Which geographic frame these angles are expressed in.

    """

    roll_r: float
    pitch_r: float
    yaw_r: float
    frame: Frame = Frame.NED

    @classmethod
    def from_degrees(
        cls,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        frame: Frame = Frame.NED,
    ) -> "Rpy":
        """Build from degrees, the unit used in user-facing APIs and config."""
        return cls(radians(roll_deg), radians(pitch_deg), radians(yaw_deg), frame)

    def to_degrees(self) -> tuple[float, float, float]:
        """Return ``(roll_deg, pitch_deg, yaw_deg)`` for display or user-facing APIs."""
        return (degrees(self.roll_r), degrees(self.pitch_r), degrees(self.yaw_r))

    def as_tuple(self) -> tuple[float, float, float]:
        """Return ``(roll_r, pitch_r, yaw_r)``, the ordering used on the wire."""
        return (self.roll_r, self.pitch_r, self.yaw_r)

    def tagged(self, frame: Frame) -> "Rpy":
        """Return a copy relabelled as ``frame``, without converting the values.

        This is a relabelling only. Use it when the numbers are already correct for
        ``frame`` and the tag needs to catch up -- never to "convert" between
        frames, which is :mod:`isaac_core.geo`'s job.
        """
        return replace(self, frame=frame)


@dataclass(frozen=True, slots=True)
class GeodeticPose:
    """A complete pose: where a body is, and how it is oriented.

    Args:
        position: WGS84 geodetic position.
        orientation: Attitude, tagged with its frame.

    """

    position: Lla
    orientation: Rpy

    @property
    def frame(self) -> Frame:
        """The frame this pose's orientation is expressed in."""
        return self.orientation.frame


__all__ = ["GeodeticPose", "Lla", "Rpy"]
