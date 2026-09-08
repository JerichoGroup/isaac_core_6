"""Tests for the UDP packet specification and port allocation."""

import struct

import pytest

from isaac_core.contracts import packet, ports

# --------------------------------------------------------------------------- #
# packet specification
# --------------------------------------------------------------------------- #


def test_packet_total_size_is_the_51_bytes_the_previous_generation_used() -> None:
    # Compatibility with existing senders and the debugger GUI depends on this.
    assert packet.PACKET_SIZE == 51


def test_packet_sizes_are_internally_consistent() -> None:
    assert packet.HEADER_SIZE + packet.PAYLOAD_SIZE + packet.CHECKSUM_SIZE == packet.PACKET_SIZE


def test_payload_format_matches_declared_payload_size() -> None:
    assert struct.calcsize(packet.PAYLOAD_FORMAT) == packet.PAYLOAD_SIZE


def test_field_order_matches_payload_format_arity() -> None:
    assert len(packet.FIELD_ORDER) == 6


def test_packet_offsets_land_where_the_documented_layout_says() -> None:
    assert packet.PAYLOAD_OFFSET == 2
    assert packet.CHECKSUM_OFFSET == 50


def test_packet_header_is_the_expected_sync_preamble() -> None:
    assert packet.HEADER == (0xAC, 0xDC)


def test_packet_angle_fields_are_named_as_radians() -> None:
    # The previous generation's debugger displayed "deg" while sending radians.
    # Encoding the unit in the field names makes that mistake unwriteable.
    assert packet.FIELD_ORDER[3:] == ("roll_r", "pitch_r", "yaw_r")


# --------------------------------------------------------------------------- #
# checksum
# --------------------------------------------------------------------------- #


def test_checksum_is_xor_over_all_payload_bytes() -> None:
    assert packet.checksum(bytes([0x01, 0x02, 0x03])) == 0x01 ^ 0x02 ^ 0x03


def test_checksum_of_empty_payload_is_zero() -> None:
    assert packet.checksum(b"") == 0


def test_checksum_is_in_single_byte_range_for_a_full_payload() -> None:
    payload = struct.pack(packet.PAYLOAD_FORMAT, 32.5, 35.25, 1000.0, 0.1, 0.2, 0.3)
    assert 0 <= packet.checksum(payload) <= 0xFF


def test_checksum_self_cancels_so_duplicated_payloads_sum_to_zero() -> None:
    payload = bytes(range(48))
    assert packet.checksum(payload + payload) == 0


# --------------------------------------------------------------------------- #
# port allocation
# --------------------------------------------------------------------------- #


def test_single_vehicle_lands_on_the_familiar_default_port() -> None:
    assert ports.pose_port_for_index(0) == 33333


def test_additional_vehicles_get_sequential_ports() -> None:
    assert [ports.pose_port_for_index(i) for i in range(3)] == [33333, 33334, 33335]


def test_pose_port_base_is_overridable() -> None:
    assert ports.pose_port_for_index(2, base=40000) == 40002


def test_pose_port_rejects_negative_index() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        ports.pose_port_for_index(-1)


def test_pose_port_rejects_a_swarm_that_would_overflow_the_port_range() -> None:
    with pytest.raises(ValueError, match="port must be in"):
        ports.pose_port_for_index(1, base=ports.MAX_PORT)


@pytest.mark.parametrize("port", [1024, 8760, 33333, 65535])
def test_validate_port_accepts_unprivileged_ports(port: int) -> None:
    assert ports.validate_port(port) == port


@pytest.mark.parametrize("port", [0, 80, 1023, 65536])
def test_validate_port_rejects_privileged_and_out_of_range_ports(port: int) -> None:
    with pytest.raises(ValueError, match="port must be in"):
        ports.validate_port(port)


def test_default_ports_are_themselves_valid() -> None:
    for port in (
        ports.DEFAULT_POSE_UDP_PORT,
        ports.DEFAULT_CONTROL_PLANE_PORT,
    ):
        assert ports.validate_port(port) == port
