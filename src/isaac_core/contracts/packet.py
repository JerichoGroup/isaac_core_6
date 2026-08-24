"""
Wire specification for the UDP pose packet.

Byte-for-byte identical to the format used by the previous generation of this
tooling, so existing senders, the debugger GUI and any team scripts built against
it remain valid. A vehicle id and a version byte were considered and deliberately
rejected as speculative: one UDP port per vehicle is simpler to debug (point a
capture at a single port and you see exactly one aircraft), and gives complete
isolation, since a malformed stream can only ever disturb the vehicle it belongs
to.

Layout, 51 bytes total, little-endian::

    offset  size  field       type     notes
    0       1     header[0]   uint8    0xAC
    1       1     header[1]   uint8    0xDC
    2       8     latitude    float64  degrees
    10      8     longitude   float64  degrees
    18      8     altitude    float64  metres, sea level = 0
    26      8     roll        float64  RADIANS, NED
    34      8     pitch       float64  RADIANS, NED
    42      8     yaw         float64  RADIANS, NED
    50      1     checksum    uint8    XOR of bytes [2, 50)

Note the angles are **radians**, despite the previous generation's debugger GUI
displaying a table that claimed degrees while transmitting radians. That
discrepancy is why every angle in this codebase carries its unit in the name.

The codec itself lives in :mod:`isaac_core.protocol`; this module is the
specification only, so it stays importable with no dependencies at all.
"""

from typing import Final

# Fixed two-byte sync preamble.
HEADER: Final = (0xAC, 0xDC)

HEADER_SIZE: Final = 2
PAYLOAD_SIZE: Final = 48
CHECKSUM_SIZE: Final = 1
PACKET_SIZE: Final = HEADER_SIZE + PAYLOAD_SIZE + CHECKSUM_SIZE

PAYLOAD_OFFSET: Final = HEADER_SIZE
CHECKSUM_OFFSET: Final = HEADER_SIZE + PAYLOAD_SIZE

# struct format for the payload: six little-endian float64 fields.
PAYLOAD_FORMAT: Final = "<6d"

# Payload field order, matching PAYLOAD_FORMAT.
FIELD_ORDER: Final = ("lat_deg", "lon_deg", "alt_m", "roll_r", "pitch_r", "yaw_r")


def checksum(payload: bytes) -> int:
    """
    Compute the trailing checksum for a payload.

    A plain XOR over every payload byte, matching the previous generation's
    sender and receiver exactly.

    Args:
        payload: The 48 payload bytes, excluding header and checksum.

    Returns:
        The checksum byte, in ``[0, 255]``.

    """
    result = 0
    for byte in payload:
        result ^= byte
    return result


__all__ = [
    "CHECKSUM_OFFSET",
    "CHECKSUM_SIZE",
    "FIELD_ORDER",
    "HEADER",
    "HEADER_SIZE",
    "PACKET_SIZE",
    "PAYLOAD_FORMAT",
    "PAYLOAD_OFFSET",
    "PAYLOAD_SIZE",
    "checksum",
]
