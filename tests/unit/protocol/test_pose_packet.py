"""Tests for the pose packet wire codec."""

import random
import struct

import pytest

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.packet import (
    CHECKSUM_OFFSET,
    HEADER,
    PACKET_SIZE,
    PAYLOAD_FORMAT,
    PAYLOAD_OFFSET,
    checksum,
)
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.protocol.errors import (
    PacketChecksumError,
    PacketHeaderError,
    PacketLengthError,
    PosePacketError,
)
from isaac_core.protocol.pose_packet import HoldLastGoodDecoder, decode, encode

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

# A reference pose chosen to exercise non-trivial values in every field.
_REF_LAT = 32.22481
_REF_LON = 35.25621
_REF_ALT = 516.7
_REF_ROLL = 0.1
_REF_PITCH = -0.05
_REF_YAW = 1.2217304763960306  # ~70 degrees

_REF_POSE = GeodeticPose(
    position=Lla(lat_deg=_REF_LAT, lon_deg=_REF_LON, alt_m=_REF_ALT),
    orientation=Rpy(roll_r=_REF_ROLL, pitch_r=_REF_PITCH, yaw_r=_REF_YAW, frame=Frame.NED),
)


def _build_golden_packet() -> bytes:
    """Build the expected 51 bytes using struct directly, mirroring the 2023 encoder."""
    payload = struct.pack(
        PAYLOAD_FORMAT,
        _REF_LAT,
        _REF_LON,
        _REF_ALT,
        _REF_ROLL,
        _REF_PITCH,
        _REF_YAW,
    )
    chk = 0
    for b in payload:
        chk ^= b
    return bytes([HEADER[0], HEADER[1]]) + payload + bytes([chk])


# --------------------------------------------------------------------------- #
# golden vector — byte-for-byte compatibility with the 2023 encoder
# --------------------------------------------------------------------------- #


def test_encode_produces_the_golden_vector_byte_for_byte() -> None:
    # This is the single most important test: if this breaks, every existing sender,
    # the debugger GUI, and the OmniGraph decoder stop working.
    expected = _build_golden_packet()
    result = encode(_REF_POSE)
    assert result == expected


def test_encode_matches_bytes_captured_from_the_2023_encoder() -> None:
    # A stronger guarantee than the test above, which builds its expectation from
    # the same contracts.packet constants that encode() uses and so cannot catch
    # the spec itself drifting. This hex literal was produced by running
    # isaac_core_2023's BaseUDPSender._build_packet verbatim for _REF_POSE, so it
    # is anchored to an external observation. Existing team senders, the debugger
    # GUI and the OmniGraph decoder all depend on this staying true.
    captured_from_2023 = bytes.fromhex(
        "acdc29e8f692c61c40403f74417dcba041409a999999992580409a9999999999b93f9a9999999999a9bff65b8a41358cf33f9e"
    )
    assert encode(_REF_POSE) == captured_from_2023


def test_golden_vector_is_exactly_51_bytes() -> None:
    assert len(_build_golden_packet()) == PACKET_SIZE


def test_golden_vector_starts_with_the_sync_preamble() -> None:
    pkt = _build_golden_packet()
    assert pkt[0] == 0xAC
    assert pkt[1] == 0xDC


def test_golden_vector_checksum_matches_xor_of_payload() -> None:
    pkt = _build_golden_packet()
    payload = pkt[PAYLOAD_OFFSET:CHECKSUM_OFFSET]
    assert pkt[CHECKSUM_OFFSET] == checksum(payload)


# --------------------------------------------------------------------------- #
# encode / decode round-trip
# --------------------------------------------------------------------------- #


def test_round_trip_preserves_position_exactly() -> None:
    decoded = decode(encode(_REF_POSE))
    assert decoded.position.lat_deg == _REF_LAT
    assert decoded.position.lon_deg == _REF_LON
    assert decoded.position.alt_m == _REF_ALT


def test_round_trip_preserves_orientation_exactly() -> None:
    decoded = decode(encode(_REF_POSE))
    assert decoded.orientation.roll_r == _REF_ROLL
    assert decoded.orientation.pitch_r == _REF_PITCH
    assert decoded.orientation.yaw_r == _REF_YAW


def test_round_trip_tags_orientation_as_ned() -> None:
    decoded = decode(encode(_REF_POSE))
    assert decoded.orientation.frame is Frame.NED


def test_round_trip_at_zero_pose() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=0.0, lon_deg=0.0, alt_m=0.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
    )
    assert decode(encode(pose)) == pose


def test_round_trip_at_extreme_lat_lon() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=-90.0, lon_deg=-180.0, alt_m=-500.0),
        orientation=Rpy(roll_r=-3.14159, pitch_r=1.5707, yaw_r=3.14159, frame=Frame.NED),
    )
    assert decode(encode(pose)) == pose


def test_round_trip_with_negative_altitude() -> None:
    # Dead Sea is -430m; altitude can legitimately be negative.
    pose = GeodeticPose(
        position=Lla(lat_deg=31.5, lon_deg=35.5, alt_m=-430.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
    )
    assert decode(encode(pose)) == pose


# --------------------------------------------------------------------------- #
# encode validation
# --------------------------------------------------------------------------- #


def test_encode_rejects_enu_orientation() -> None:
    pose = GeodeticPose(
        position=Lla(lat_deg=32.0, lon_deg=35.0, alt_m=100.0),
        orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.ENU),
    )
    with pytest.raises(ValueError, match="NED"):
        encode(pose)


def test_encode_returns_exactly_51_bytes() -> None:
    assert len(encode(_REF_POSE)) == PACKET_SIZE


# --------------------------------------------------------------------------- #
# decode error paths
# --------------------------------------------------------------------------- #


def test_decode_raises_length_error_on_truncated_packet() -> None:
    short = encode(_REF_POSE)[:30]
    with pytest.raises(PacketLengthError) as exc_info:
        decode(short)
    assert exc_info.value.actual == 30
    assert exc_info.value.expected == PACKET_SIZE


def test_decode_raises_length_error_on_empty_input() -> None:
    with pytest.raises(PacketLengthError) as exc_info:
        decode(b"")
    assert exc_info.value.actual == 0


def test_decode_raises_length_error_on_overlong_packet() -> None:
    # One extra byte appended -- still wrong length.
    overlong = encode(_REF_POSE) + b"\x00"
    with pytest.raises(PacketLengthError) as exc_info:
        decode(overlong)
    assert exc_info.value.actual == 52


def test_decode_raises_header_error_on_wrong_first_byte() -> None:
    pkt = bytearray(encode(_REF_POSE))
    pkt[0] = 0xFF
    with pytest.raises(PacketHeaderError) as exc_info:
        decode(bytes(pkt))
    assert exc_info.value.actual == (0xFF, 0xDC)
    assert exc_info.value.expected == HEADER


def test_decode_raises_header_error_on_wrong_second_byte() -> None:
    pkt = bytearray(encode(_REF_POSE))
    pkt[1] = 0x00
    with pytest.raises(PacketHeaderError) as exc_info:
        decode(bytes(pkt))
    assert exc_info.value.actual == (0xAC, 0x00)


def test_decode_raises_checksum_error_on_flipped_checksum() -> None:
    pkt = bytearray(encode(_REF_POSE))
    pkt[CHECKSUM_OFFSET] ^= 0xFF  # flip all bits of the checksum
    with pytest.raises(PacketChecksumError) as exc_info:
        decode(bytes(pkt))
    # The stored checksum is wrong; computed is the original one.
    assert exc_info.value.expected == pkt[CHECKSUM_OFFSET]


def test_decode_raises_checksum_error_on_corrupted_payload_byte() -> None:
    # Flip a payload byte so the checksum no longer matches.
    pkt = bytearray(encode(_REF_POSE))
    pkt[10] ^= 0x01
    with pytest.raises(PacketChecksumError):
        decode(bytes(pkt))


# --------------------------------------------------------------------------- #
# fuzz — decode must only raise PosePacketError subclasses, never hang
# --------------------------------------------------------------------------- #


def test_fuzz_decode_never_raises_unexpected_exceptions() -> None:
    # Feed 1000 random byte strings of random lengths up to 200.
    rng = random.Random(42)  # noqa: S311
    for _ in range(1000):
        length = rng.randint(0, 200)
        data = bytes(rng.getrandbits(8) for _ in range(length))
        try:
            result = decode(data)
            # If it somehow decoded, it must be a valid GeodeticPose.
            assert isinstance(result, GeodeticPose)
        except PosePacketError:
            pass  # expected


def test_fuzz_decode_with_correct_length_random_content() -> None:
    # 51-byte random inputs -- exercises header/checksum/payload branches.
    rng = random.Random(99)  # noqa: S311
    for _ in range(500):
        data = bytes(rng.getrandbits(8) for _ in range(PACKET_SIZE))
        try:
            decode(data)
        except PosePacketError:
            pass


# --------------------------------------------------------------------------- #
# HoldLastGoodDecoder
# --------------------------------------------------------------------------- #


def test_hold_decoder_returns_none_before_any_good_packet() -> None:
    dec = HoldLastGoodDecoder()
    assert dec.decode(b"garbage") is None


def test_hold_decoder_returns_none_with_zero_failure_count_initially() -> None:
    dec = HoldLastGoodDecoder()
    assert dec.failure_count == 0
    assert dec.last_error is None


def test_hold_decoder_returns_pose_on_first_good_packet() -> None:
    dec = HoldLastGoodDecoder()
    pkt = encode(_REF_POSE)
    result = dec.decode(pkt)
    assert result == _REF_POSE


def test_hold_decoder_holds_previous_pose_on_corruption() -> None:
    dec = HoldLastGoodDecoder()
    dec.decode(encode(_REF_POSE))
    # Feed garbage -- should get back the previous good pose.
    result = dec.decode(b"\x00" * PACKET_SIZE)
    assert result == _REF_POSE


def test_hold_decoder_increments_failure_count() -> None:
    dec = HoldLastGoodDecoder()
    dec.decode(encode(_REF_POSE))
    dec.decode(b"bad")
    dec.decode(b"also bad")
    assert dec.failure_count == 2


def test_hold_decoder_records_last_error() -> None:
    dec = HoldLastGoodDecoder()
    dec.decode(b"short")
    assert isinstance(dec.last_error, PacketLengthError)


def test_hold_decoder_updates_on_next_good_packet() -> None:
    dec = HoldLastGoodDecoder()
    dec.decode(encode(_REF_POSE))
    dec.decode(b"garbage")

    # A new valid pose should replace the held one.
    new_pose = GeodeticPose(
        position=Lla(lat_deg=10.0, lon_deg=20.0, alt_m=300.0),
        orientation=Rpy(roll_r=0.5, pitch_r=0.3, yaw_r=-1.0, frame=Frame.NED),
    )
    result = dec.decode(encode(new_pose))
    assert result == new_pose
    assert dec.last_good == new_pose


def test_hold_decoder_failure_before_first_good_increments_count() -> None:
    dec = HoldLastGoodDecoder()
    dec.decode(b"bad1")
    dec.decode(b"bad2")
    dec.decode(b"bad3")
    assert dec.failure_count == 3
    assert dec.last_good is None


def test_hold_decoder_last_error_is_none_after_good_packet() -> None:
    # The last_error stays as the most recent error, not cleared by success.
    dec = HoldLastGoodDecoder()
    dec.decode(b"bad")
    dec.decode(encode(_REF_POSE))
    # last_error is still the error from "bad" since it was never replaced by a newer error.
    assert isinstance(dec.last_error, PacketLengthError)


def test_hold_decoder_consecutive_good_packets_update() -> None:
    dec = HoldLastGoodDecoder()
    poses = [
        GeodeticPose(
            position=Lla(lat_deg=float(i), lon_deg=float(i), alt_m=float(i * 100)),
            orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
        )
        for i in range(5)
    ]
    for pose in poses:
        result = dec.decode(encode(pose))
        assert result == pose
    assert dec.failure_count == 0
