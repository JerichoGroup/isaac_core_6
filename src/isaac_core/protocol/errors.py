"""Exception hierarchy for pose packet codec failures.

Logging every failure kind identically makes it impossible for a caller to distinguish a
length error from a checksum corruption. These typed exceptions let the transport layer
and the hold-last-good decoder expose *why* a packet was rejected while still
preserving the hold behaviour.
"""


class PosePacketError(Exception):
    """Base for all pose packet codec failures."""


class PacketLengthError(PosePacketError):
    """Raise when a packet is not the expected 51 bytes.

    Args:
        actual: Length of the received data.
        expected: The fixed length required (51).

    """

    def __init__(self, actual: int, expected: int) -> None:
        """Construct with the actual and expected lengths."""
        self.actual = actual
        self.expected = expected
        super().__init__(f"Packet length {actual} != expected {expected}")


class PacketHeaderError(PosePacketError):
    """Raise when the two-byte sync preamble does not match.

    Args:
        actual: The header bytes received, as a tuple.
        expected: The correct header bytes, as a tuple.

    """

    def __init__(self, actual: tuple[int, ...], expected: tuple[int, ...]) -> None:
        """Construct with the actual and expected header bytes."""
        self.actual = actual
        self.expected = expected
        super().__init__(
            f"Header mismatch: got {tuple(hex(b) for b in actual)}, " f"expected {tuple(hex(b) for b in expected)}"
        )


class PacketChecksumError(PosePacketError):
    """Raise when the trailing XOR checksum does not match the payload.

    Args:
        actual: The checksum computed from the payload.
        expected: The checksum byte read from the packet.

    """

    def __init__(self, actual: int, expected: int) -> None:
        """Construct with the computed and received checksums."""
        self.actual = actual
        self.expected = expected
        super().__init__(f"Checksum mismatch: computed 0x{actual:02X}, packet has 0x{expected:02X}")


class PacketPayloadError(PosePacketError):
    """Raise when the payload cannot be unpacked into six float64 fields.

    Args:
        reason: A human-readable description of the struct.unpack failure.

    """

    def __init__(self, reason: str) -> None:
        """Construct with the reason the unpack failed."""
        self.reason = reason
        super().__init__(f"Payload unpack failed: {reason}")


__all__ = [
    "PacketChecksumError",
    "PacketHeaderError",
    "PacketLengthError",
    "PacketPayloadError",
    "PosePacketError",
]
