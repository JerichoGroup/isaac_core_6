"""Tests for the pose pipeline kernel extracted from the camera-pose OmniGraph node.

These cover the branches that previously lived inside ``compute()`` and so could only have been
exercised by launching Isaac Sim.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.pose import Lla
from isaac_core.geo.enu import EnuConverter
from isaac_core.geo.pose_pipeline import (
    compose_local_pose,
    has_position_fix,
    quaternion_to_isaac_order,
    rotation_frame_from_text,
)

REFERENCE = (32.22481, 35.25621, 516.7)


@pytest.fixture()
def converter() -> EnuConverter:
    """Return a converter for the shipped scene's reference point."""
    return EnuConverter(*REFERENCE)


# -- the no-data guard ------------------------------------------------------- #


def test_an_all_zero_position_is_not_a_fix() -> None:
    # Guards the branch that stops a pre-data tick teleporting the camera to the ENU origin.
    # (0, 0, 0) is a real place in the Gulf of Guinea, but on this wire it means "nothing yet".
    assert has_position_fix((0.0, 0.0, 0.0)) is False


@pytest.mark.parametrize(
    "position",
    [
        (32.22481, 35.25621, 516.7),
        (0.0, 0.0, 1000.0),
        (0.0, 35.0, 0.0),
        (32.0, 0.0, 0.0),
        (-0.0001, 0.0, 0.0),
    ],
)
def test_any_nonzero_component_counts_as_a_fix(position: tuple[float, float, float]) -> None:
    assert has_position_fix(position) is True


# -- rotation frame parsing -------------------------------------------------- #


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("world", RotationFrame.WORLD),
        ("WORLD", RotationFrame.WORLD),
        ("  World  ", RotationFrame.WORLD),
        ("body", RotationFrame.BODY),
        ("BODY", RotationFrame.BODY),
        ("", RotationFrame.BODY),
        ("nonsense", RotationFrame.BODY),
    ],
)
def test_rotation_frame_parsing_defaults_to_body(text: str, expected: RotationFrame) -> None:
    # BODY is the safe default: it is what a gimbal physically is, and an unparseable value should
    # not silently switch the composition to world axes.
    assert rotation_frame_from_text(text) is expected


# -- quaternion ordering ----------------------------------------------------- #


def test_quaternion_is_reordered_into_isaac_storage_order() -> None:
    # OmniGraph quatd[4] stores (x, y, z, w) while the GUI shows scalar-first. Getting this
    # backwards is a 180 degree error that looks like a coordinate bug somewhere else entirely.
    assert quaternion_to_isaac_order(1.0, 2.0, 3.0, 4.0) == (2.0, 3.0, 4.0, 1.0)


def test_identity_quaternion_survives_the_reorder() -> None:
    assert quaternion_to_isaac_order(1.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)


# -- composition ------------------------------------------------------------- #


def test_a_pose_at_the_reference_sits_at_the_stage_origin(converter: EnuConverter) -> None:
    pose = compose_local_pose(
        converter=converter,
        position=Lla(lat_deg=REFERENCE[0], lon_deg=REFERENCE[1], alt_m=REFERENCE[2]),
        attitude_r=(0.0, 0.0, 0.0),
        rotation_frame=RotationFrame.BODY,
        gimbal_offset_deg=(0.0, 0.0, 0.0),
    )
    east, north, up = pose.local_position
    assert east == pytest.approx(0.0, abs=1e-6)
    assert north == pytest.approx(0.0, abs=1e-6)
    assert up == pytest.approx(0.0, abs=1e-6)


def test_altitude_above_the_reference_becomes_stage_up(converter: EnuConverter) -> None:
    # The relationship the README documents: stage Z is altitude minus the reference altitude.
    pose = compose_local_pose(
        converter=converter,
        position=Lla(lat_deg=REFERENCE[0], lon_deg=REFERENCE[1], alt_m=REFERENCE[2] + 483.3),
        attitude_r=(0.0, 0.0, 0.0),
        rotation_frame=RotationFrame.BODY,
        gimbal_offset_deg=(0.0, 0.0, 0.0),
    )
    assert pose.local_position[2] == pytest.approx(483.3, abs=1e-3)


def test_a_zero_gimbal_offset_leaves_the_airframe_attitude_untouched(converter: EnuConverter) -> None:
    attitude = (0.05, -0.1, 1.2)
    pose = compose_local_pose(
        converter=converter,
        position=Lla(lat_deg=32.3, lon_deg=35.3, alt_m=1500.0),
        attitude_r=attitude,
        rotation_frame=RotationFrame.WORLD,
        gimbal_offset_deg=(0.0, 0.0, 0.0),
    )
    assert np.allclose(pose.attitude_r, attitude, atol=1e-9)


def test_the_returned_quaternion_is_normalised(converter: EnuConverter) -> None:
    pose = compose_local_pose(
        converter=converter,
        position=Lla(lat_deg=32.3, lon_deg=35.3, alt_m=1500.0),
        attitude_r=(0.2, -0.3, 0.4),
        rotation_frame=RotationFrame.BODY,
        gimbal_offset_deg=(10.0, -20.0, 30.0),
    )
    assert math.sqrt(sum(component * component for component in pose.quaternion_xyzw)) == pytest.approx(1.0)


def test_the_gimbal_offset_is_applied_in_the_body_frame_whatever_the_vehicle_frame(
    converter: EnuConverter,
) -> None:
    # The bug this guards: composing the offset in the vehicle's frame (WORLD by default) made a
    # commanded pitch land on the roll axis, because a level north-bound aircraft already carries
    # ENU yaw of +90 degrees. The offset must behave identically regardless of the airframe's frame
    # when the airframe itself is not rotated.
    kwargs = {
        "converter": converter,
        "position": Lla(lat_deg=32.3, lon_deg=35.3, alt_m=1500.0),
        "attitude_r": (0.0, 0.0, 0.0),
        "gimbal_offset_deg": (0.0, -30.0, 0.0),
    }
    in_body = compose_local_pose(rotation_frame=RotationFrame.BODY, **kwargs)  # type: ignore[arg-type]
    in_world = compose_local_pose(rotation_frame=RotationFrame.WORLD, **kwargs)  # type: ignore[arg-type]
    assert np.allclose(in_body.attitude_r, in_world.attitude_r, atol=1e-9)


def test_pitch_and_yaw_offsets_carry_the_ned_sign_flips(converter: EnuConverter) -> None:
    # A commanded +pitch must not produce +pitch in the stage: the offset carries the same NED->ENU
    # sign flips as the airframe, or the same command means opposite things on the UDP and ROS paths.
    pose = compose_local_pose(
        converter=converter,
        position=Lla(lat_deg=32.3, lon_deg=35.3, alt_m=1500.0),
        attitude_r=(0.0, 0.0, 0.0),
        rotation_frame=RotationFrame.BODY,
        gimbal_offset_deg=(0.0, 30.0, 0.0),
    )
    assert pose.attitude_r[1] == pytest.approx(math.radians(-30.0), abs=1e-9)
