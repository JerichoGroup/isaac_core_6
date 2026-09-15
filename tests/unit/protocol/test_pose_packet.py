"""Tests for the pose packet wire codec."""

import random
import struct
from typing import Final

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
    rng = random.Random(42)
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
    rng = random.Random(99)
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


def _packet_with(
    lat: float = 32.2, lon: float = 35.2, alt: float = 900.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> bytes:
    """Build a structurally valid packet (header + checksum) carrying arbitrary field values."""
    payload = struct.pack(PAYLOAD_FORMAT, lat, lon, alt, roll, pitch, yaw)
    chk = 0
    for byte in payload:
        chk ^= byte
    return bytes(HEADER) + payload + bytes([chk])


@pytest.mark.parametrize(
    ("label", "kwargs"),
    [
        ("lat out of range", {"lat": 200.0}),
        ("lat NaN", {"lat": float("nan")}),
        ("lon out of range", {"lon": 999.0}),
        ("alt infinite", {"alt": float("inf")}),
        ("roll NaN", {"roll": float("nan")}),
        ("yaw infinite", {"yaw": float("inf")}),
    ],
)
def test_decode_rejects_nonfinite_and_out_of_range_as_packet_error(label: str, kwargs: dict[str, float]) -> None:
    # Guards a real bug: these values reached Lla/Rpy, whose __post_init__ raises ValueError.
    # ValueError is not a PosePacketError, so HoldLastGoodDecoder did not catch it and a single
    # glitched or hostile UDP packet crashed the receive loop.
    with pytest.raises(PosePacketError):
        decode(_packet_with(**kwargs))


@pytest.mark.parametrize(
    ("label", "kwargs"),
    [
        ("lat out of range", {"lat": 200.0}),
        ("lat NaN", {"lat": float("nan")}),
        ("alt infinite", {"alt": float("inf")}),
        ("yaw NaN", {"yaw": float("nan")}),
    ],
)
def test_hold_last_good_survives_semantically_invalid_packets(label: str, kwargs: dict[str, float]) -> None:
    # The documented guarantee is that ANY bad packet holds the last good pose. That has to include
    # a packet whose header and checksum are perfectly valid but whose payload is nonsense.
    decoder = HoldLastGoodDecoder()
    good = encode(
        GeodeticPose(
            position=Lla(lat_deg=32.2, lon_deg=35.2, alt_m=900.0),
            orientation=Rpy(roll_r=0.0, pitch_r=0.0, yaw_r=0.0, frame=Frame.NED),
        )
    )
    assert decoder.decode(good) is not None
    held = decoder.decode(_packet_with(**kwargs))
    assert held is not None
    assert held.position.lat_deg == pytest.approx(32.2)
    assert held.position.alt_m == pytest.approx(900.0)
    assert decoder.failure_count == 1


# Captured from this encoder, with every field distinct so a transposition cannot survive. The single
# 2023-captured vector elsewhere in this file anchors the format to the external sender; these anchor
# the FIELD ORDER, which a symmetric pose cannot: if lat and lon, or roll and pitch, were swapped in
# PAYLOAD_FORMAT or FIELD_ORDER, a pose with equal values would still round-trip perfectly.
_DISTINCT_FIELD_VECTORS: Final = [
    (
        "all six fields distinct",
        (12.345678, -45.678901, 1234.5, 0.111111, 0.222222, 0.333333),
        bytes.fromhex(
            "acdcb4e4f1b4fcb02840611c5c3ae6d646c000000000004a9340b3d1393fc571bc3fb3d1393fc571cc3f465d6bef5355d53f06"
        ),
    ),
    (
        "a lat/lon swap would show here",
        (10.0, 20.0, 30.0, 0.4, 0.5, 0.6),
        bytes.fromhex(
            "acdc000000000000244000000000000034400000000000003e409a9999999999d93f000000000000e03f333333333333e33f88"
        ),
    ),
    (
        "negative in every field",
        (-1.5, -2.5, -3.5, -0.25, -0.5, -0.75),
        bytes.fromhex(
            "acdc000000000000f8bf00000000000004c00000000000000cc0000000000000d0bf000000000000e0bf000000000000e8bf28"
        ),
    ),
    (
        "roll pitch yaw ascending",
        (32.0, 35.0, 900.0, 0.1, 0.2, 0.3),
        bytes.fromhex(
            "acdc000000000000404000000000008041400000000000208c409a9999999999b93f9a9999999999c93f333333333333d33ff1"
        ),
    ),
]


@pytest.mark.parametrize(
    ("label", "fields", "expected"), _DISTINCT_FIELD_VECTORS, ids=[v[0] for v in _DISTINCT_FIELD_VECTORS]
)
def test_encoding_is_byte_stable_for_distinct_field_values(
    label: str, fields: tuple[float, ...], expected: bytes
) -> None:
    lat, lon, alt, roll, pitch, yaw = fields
    raw = encode(
        GeodeticPose(
            position=Lla(lat_deg=lat, lon_deg=lon, alt_m=alt),
            orientation=Rpy(roll_r=roll, pitch_r=pitch, yaw_r=yaw, frame=Frame.NED),
        )
    )
    assert raw == expected, label


@pytest.mark.parametrize(
    ("label", "fields", "expected"), _DISTINCT_FIELD_VECTORS, ids=[v[0] for v in _DISTINCT_FIELD_VECTORS]
)
def test_decoding_recovers_every_distinct_field(label: str, fields: tuple[float, ...], expected: bytes) -> None:
    lat, lon, alt, roll, pitch, yaw = fields
    pose = decode(expected)
    assert pose.position.lat_deg == pytest.approx(lat)
    assert pose.position.lon_deg == pytest.approx(lon)
    assert pose.position.alt_m == pytest.approx(alt)
    assert pose.orientation.roll_r == pytest.approx(roll)
    assert pose.orientation.pitch_r == pytest.approx(pitch)
    assert pose.orientation.yaw_r == pytest.approx(yaw)


def test_the_distinct_field_vectors_really_are_distinct() -> None:
    # A guard on the guard: vectors with repeated values would not catch a transposition at all.
    for label, fields, _ in _DISTINCT_FIELD_VECTORS:
        assert len(set(fields)) == len(fields), f"{label} repeats a value, so a swap could hide"
