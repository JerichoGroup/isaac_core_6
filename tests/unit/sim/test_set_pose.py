"""Tests for the set_pose control method's packet encoding."""

import math
import struct

import pytest

from isaac_core.contracts import packet as packet_spec
from isaac_core.protocol import decode
from isaac_core.sim.runtime import _encode_pose_packet


def _pose(**overrides: float) -> dict[str, float]:
    """Build a pose dict in degrees with sensible defaults."""
    base = {
        "lat_deg": 32.22481,
        "lon_deg": 35.25621,
        "alt_m": 1200.0,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": 0.0,
    }
    base.update(overrides)
    return base


def test_encoded_packet_matches_the_wire_contract() -> None:
    """Produce a packet of the declared size with the declared header and checksum."""
    data = _encode_pose_packet(_pose())

    assert len(data) == packet_spec.PACKET_SIZE
    assert tuple(data[: packet_spec.HEADER_SIZE]) == packet_spec.HEADER
    payload = data[packet_spec.PAYLOAD_OFFSET : packet_spec.CHECKSUM_OFFSET]
    assert data[packet_spec.CHECKSUM_OFFSET] == packet_spec.checksum(payload)


def test_the_real_decoder_reads_back_what_was_encoded() -> None:
    # The strongest guarantee available: round-trip through the shipped decoder rather than
    # re-deriving the layout in the test, so a layout change cannot pass silently.
    data = _encode_pose_packet(_pose(alt_m=1234.0, yaw_deg=90.0))
    pose = decode(data)

    assert math.isclose(pose.position.lat_deg, 32.22481)
    assert math.isclose(pose.position.alt_m, 1234.0)
    assert math.isclose(pose.orientation.yaw_r, math.radians(90.0))


@pytest.mark.parametrize(("field", "degrees"), [("roll_deg", 30.0), ("pitch_deg", -15.0), ("yaw_deg", 180.0)])
def test_angles_are_converted_from_degrees_to_radians(field: str, degrees: float) -> None:
    # The wire carries radians while this API takes degrees. Sending degrees raw would place the
    # aircraft at a wildly wrong attitude while looking superficially plausible.
    data = _encode_pose_packet(_pose(**{field: degrees}))
    values = struct.unpack(
        packet_spec.PAYLOAD_FORMAT,
        data[packet_spec.PAYLOAD_OFFSET : packet_spec.CHECKSUM_OFFSET],
    )
    index = packet_spec.FIELD_ORDER.index(field.replace("_deg", "_r"))
    assert math.isclose(values[index], math.radians(degrees))
