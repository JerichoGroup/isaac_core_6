"""Immutable vehicle state: geodetic position, a rotation matrix, and the frame it is read in.

A ``VehicleState`` is never mutated -- every transition returns a **new** state.
The rotation matrix is the authoritative orientation; Euler angles and direction
vectors are derived properties. Conversions to/from
:class:`~isaac_core.contracts.pose.GeodeticPose` are provided for interop with
the protocol and trajectory layers.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from isaac_core.contracts.frames import Frame, RotationFrame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.geo.rotations import euler_to_matrix, matrix_to_euler


@dataclass(frozen=True, slots=True)
class VehicleState:
    """Immutable snapshot of a vehicle's kinematic state.

    Carries a WGS84 position, a 3×3 rotation matrix, and the frame its Euler angles are expressed
    in. All transitions produce a new instance rather than mutating this one. Direction vectors and
    heading are derived from the matrix on the fly.

    The frame is part of the state because the wire carries Euler angles, not a matrix, and the
    simulator rebuilds them using the vehicle's configured ``rotation_frame``. Converting with a
    different frame than the consumer will use is silently wrong for any attitude with two non-zero
    angles: composing a world yaw onto a pitched airframe and then decomposing in the body frame
    reported roll 30, pitch 0 where the truth was roll 0, pitch -30. Single-axis attitudes agree in
    both frames, which is why it went unnoticed.

    Args:
        lat_deg: Latitude in degrees.
        lon_deg: Longitude in degrees.
        alt_m: Altitude in metres above the WGS84 ellipsoid.
        rotation: A 3×3 rotation matrix.
        frame: The frame this state's Euler angles are expressed in. Defaults to ``WORLD``, matching
            the schema default for ``vehicles.<id>.rotation_frame``.

    """

    lat_deg: float
    lon_deg: float
    alt_m: float
    rotation: NDArray[np.float64]
    frame: RotationFrame = RotationFrame.WORLD

    @classmethod
    def from_angles(
        cls,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        roll_r: float = 0.0,
        pitch_r: float = 0.0,
        yaw_r: float = 0.0,
        frame: RotationFrame = RotationFrame.WORLD,
    ) -> VehicleState:
        """Build a state from angles, with the matrix and the frame guaranteed to agree.

        Constructing the matrix separately and passing it to ``VehicleState(...)`` makes it easy to
        build one whose matrix was composed in a different frame than the state declares, which reads
        back the wrong angles for any attitude with two non-zero components. This is the path that
        cannot be got wrong.

        Args:
            lat_deg: Latitude in degrees.
            lon_deg: Longitude in degrees.
            alt_m: Altitude in metres.
            roll_r: Roll in radians.
            pitch_r: Pitch in radians.
            yaw_r: Yaw in radians.
            frame: The frame these angles are expressed in.

        Returns:
            A consistent :class:`VehicleState`.

        """
        return cls(
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            alt_m=alt_m,
            rotation=euler_to_matrix(roll_r, pitch_r, yaw_r, frame),
            frame=frame,
        )

    @classmethod
    def from_pose(cls, pose: GeodeticPose, frame: RotationFrame = RotationFrame.WORLD) -> VehicleState:
        """Construct a state from a :class:`~isaac_core.contracts.pose.GeodeticPose`.

        The pose must be in NED frame (the wire/user-facing convention). The
        resulting rotation matrix is built from the NED Euler angles.

        Args:
            pose: A geodetic pose tagged NED.
            frame: The frame the pose's Euler angles are expressed in.

        Returns:
            A new :class:`VehicleState`.

        Raises:
            ValueError: If the pose orientation is not in NED frame.

        """
        if pose.frame is not Frame.NED:
            msg = f"VehicleState.from_pose requires NED orientation, got {pose.frame.value}"
            raise ValueError(msg)
        rpy = pose.orientation
        mat = euler_to_matrix(rpy.roll_r, rpy.pitch_r, rpy.yaw_r, frame)
        return cls(
            lat_deg=pose.position.lat_deg,
            lon_deg=pose.position.lon_deg,
            alt_m=pose.position.alt_m,
            rotation=mat,
        )

    def to_pose(self) -> GeodeticPose:
        """Convert this state to a :class:`~isaac_core.contracts.pose.GeodeticPose`.

        The returned pose is tagged NED (the wire/user-facing convention).

        Returns:
            A geodetic pose with NED orientation.

        """
        roll_r, pitch_r, yaw_r = matrix_to_euler(self.rotation, self.frame)
        return GeodeticPose(
            position=Lla(lat_deg=self.lat_deg, lon_deg=self.lon_deg, alt_m=self.alt_m),
            orientation=Rpy(roll_r=roll_r, pitch_r=pitch_r, yaw_r=yaw_r, frame=Frame.NED),
        )

    @property
    def heading_r(self) -> float:
        """Return the yaw angle in radians, normalised to ``[-pi, pi]``.

        This is the compass heading in the NED intrinsic-XYZ convention.

        """
        _roll_r, _pitch_r, yaw_r = matrix_to_euler(self.rotation, self.frame)
        return float(yaw_r)

    @property
    def forward_vector(self) -> NDArray[np.float64]:
        """Return the unit vector pointing in the vehicle's forward (heading) direction.

        In the navigation convention, heading ``h`` maps to world-frame
        direction ``[sin(h), cos(h), 0]`` where X=east, Y=north, Z=up.
        This is the direction that :func:`~isaac_core.vehicle.motion.move_forward_backward`
        travels along.

        """
        h = self.heading_r
        return np.array([math.sin(h), math.cos(h), 0.0], dtype=np.float64)

    @property
    def right_vector(self) -> NDArray[np.float64]:
        """Return the unit vector pointing to the vehicle's right.

        Perpendicular to :attr:`forward_vector`, rotated 90° clockwise in
        the horizontal plane: ``[cos(h), -sin(h), 0]``.

        """
        h = self.heading_r
        return np.array([math.cos(h), -math.sin(h), 0.0], dtype=np.float64)

    @property
    def up_vector(self) -> NDArray[np.float64]:
        """Return the unit vector pointing upward from the vehicle.

        For a level vehicle this is ``[0, 0, 1]``. For a tilted vehicle,
        this is column 2 of the rotation matrix (body Z-axis in world space).

        """
        vec: NDArray[np.float64] = self.rotation[:, 2].copy()
        return vec

    def with_position(self, lat_deg: float, lon_deg: float, alt_m: float) -> VehicleState:
        """Return a new state with the given position and the same orientation.

        Args:
            lat_deg: New latitude.
            lon_deg: New longitude.
            alt_m: New altitude.

        Returns:
            A new :class:`VehicleState`.

        """
        return VehicleState(lat_deg=lat_deg, lon_deg=lon_deg, alt_m=alt_m, rotation=self.rotation, frame=self.frame)

    def with_rotation(self, rotation: NDArray[np.float64]) -> VehicleState:
        """Return a new state with the given rotation and the same position.

        Args:
            rotation: A 3×3 rotation matrix.

        Returns:
            A new :class:`VehicleState`.

        """
        return VehicleState(
            lat_deg=self.lat_deg, lon_deg=self.lon_deg, alt_m=self.alt_m, rotation=rotation, frame=self.frame
        )

    def with_heading_r(self, yaw_r: float) -> VehicleState:
        """Return a new state with the given yaw, preserving roll and pitch.

        Args:
            yaw_r: Desired yaw in radians.

        Returns:
            A new :class:`VehicleState`.

        """
        roll_r, pitch_r, _yaw_r = matrix_to_euler(self.rotation, self.frame)
        new_mat = euler_to_matrix(roll_r, pitch_r, yaw_r, self.frame)
        return self.with_rotation(new_mat)

    def distance_to(self, other_lat_deg: float, other_lon_deg: float, other_alt_m: float) -> float:
        """Compute the Euclidean 3D distance to another LLA point using flat-earth approximation.

        This uses a spherical offset approach for
        consistency in motion computations (haversine for horizontal, direct
        subtraction for vertical).

        Args:
            other_lat_deg: Target latitude.
            other_lon_deg: Target longitude.
            other_alt_m: Target altitude.

        Returns:
            Approximate distance in metres.

        """
        from isaac_core.geo.distance import EARTH_RADIUS_M

        dlat = math.radians(other_lat_deg - self.lat_deg)
        dlon = math.radians(other_lon_deg - self.lon_deg)
        cos_lat = math.cos(math.radians(self.lat_deg))

        north_m = dlat * EARTH_RADIUS_M
        east_m = dlon * EARTH_RADIUS_M * cos_lat
        up_m = other_alt_m - self.alt_m

        return math.sqrt(north_m**2 + east_m**2 + up_m**2)
