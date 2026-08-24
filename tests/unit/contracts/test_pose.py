"""Tests for the frame conventions and pose value types."""

from math import isclose, pi

import pytest

from isaac_core.contracts.frames import EULER_AXES, Frame, PoseSource, RotationFrame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy

# --------------------------------------------------------------------------- #
# conventions
# --------------------------------------------------------------------------- #


def test_euler_axes_is_intrinsic_xyz() -> None:
    # 'r' prefix means rotating (intrinsic) frame in transforms3d's notation.
    assert EULER_AXES == "rxyz"


def test_rotation_frame_offers_both_world_and_body() -> None:
    assert {member.value for member in RotationFrame} == {"world", "body"}


def test_pose_source_covers_every_planned_backend() -> None:
    assert {member.value for member in PoseSource} == {
        "udp",
        "ros",
        "script",
        "replay",
        "mavlink",
    }


def test_enums_compare_as_strings_so_toml_config_round_trips() -> None:
    # Config arrives as strings from TOML; these must match without conversion.
    assert RotationFrame("body") is RotationFrame.BODY
    assert PoseSource("udp") == "udp"


# --------------------------------------------------------------------------- #
# Lla
# --------------------------------------------------------------------------- #


def test_lla_as_tuple_matches_wire_ordering() -> None:
    assert Lla(32.5, 35.25, 1000.0).as_tuple() == (32.5, 35.25, 1000.0)


def test_lla_is_frozen() -> None:
    position = Lla(32.5, 35.25, 1000.0)
    with pytest.raises(AttributeError):
        position.lat_deg = 0.0  # type: ignore[misc]


def test_lla_is_hashable_and_compares_by_value() -> None:
    assert Lla(32.5, 35.25, 1000.0) == Lla(32.5, 35.25, 1000.0)
    assert len({Lla(1.0, 2.0, 3.0), Lla(1.0, 2.0, 3.0)}) == 1


@pytest.mark.parametrize("lat", [-90.0, 0.0, 90.0])
def test_lla_accepts_latitude_at_and_within_the_poles(lat: float) -> None:
    assert Lla(lat, 0.0, 0.0).lat_deg == lat


@pytest.mark.parametrize("lat", [-90.1, 90.1, 1000.0])
def test_lla_rejects_impossible_latitude(lat: float) -> None:
    with pytest.raises(ValueError, match="lat_deg"):
        Lla(lat, 0.0, 0.0)


@pytest.mark.parametrize("lon", [-180.1, 180.1])
def test_lla_rejects_impossible_longitude(lon: float) -> None:
    with pytest.raises(ValueError, match="lon_deg"):
        Lla(0.0, lon, 0.0)


def test_lla_altitude_is_unbounded_because_negative_is_legitimate() -> None:
    # Below-ellipsoid altitudes occur over the Dead Sea, which is in region.
    assert Lla(31.5, 35.5, -430.0).alt_m == -430.0


# --------------------------------------------------------------------------- #
# Rpy
# --------------------------------------------------------------------------- #


def test_rpy_defaults_to_ned_because_that_is_the_wire_convention() -> None:
    assert Rpy(0.0, 0.0, 0.0).frame is Frame.NED


def test_rpy_from_degrees_converts_to_radians() -> None:
    attitude = Rpy.from_degrees(0.0, 90.0, 180.0)
    assert isclose(attitude.pitch_r, pi / 2)
    assert isclose(attitude.yaw_r, pi)


def test_rpy_degrees_round_trip() -> None:
    roll_deg, pitch_deg, yaw_deg = Rpy.from_degrees(10.0, -20.0, 30.0).to_degrees()
    assert isclose(roll_deg, 10.0)
    assert isclose(pitch_deg, -20.0)
    assert isclose(yaw_deg, 30.0)


def test_rpy_from_degrees_preserves_frame() -> None:
    assert Rpy.from_degrees(0.0, 0.0, 0.0, Frame.ENU).frame is Frame.ENU


def test_rpy_tagged_relabels_without_touching_values() -> None:
    original = Rpy(0.1, 0.2, 0.3, Frame.NED)
    relabelled = original.tagged(Frame.ENU)
    assert relabelled.frame is Frame.ENU
    assert relabelled.as_tuple() == original.as_tuple()
    assert original.frame is Frame.NED


def test_rpy_frame_participates_in_equality() -> None:
    # The whole point of tagging: NED and ENU values must not compare equal.
    assert Rpy(0.1, 0.2, 0.3, Frame.NED) != Rpy(0.1, 0.2, 0.3, Frame.ENU)


# --------------------------------------------------------------------------- #
# GeodeticPose
# --------------------------------------------------------------------------- #


def test_geodetic_pose_exposes_its_orientation_frame() -> None:
    pose = GeodeticPose(Lla(32.5, 35.25, 900.0), Rpy(0.0, 0.0, 0.0, Frame.ENU))
    assert pose.frame is Frame.ENU
