"""Encode and decode the 51-byte UDP pose packet.

This is the wire codec only -- no sockets, no I/O. Transport belongs to a later
layer. Byte compatibility is mandatory: existing senders produce and consume exactly these bytes.

The specification lives in :mod:`isaac_core.contracts.packet`; this module is the
implementation that honours it.
"""

import math
import struct

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.packet import (
    CHECKSUM_OFFSET,
    FIELD_ORDER,
    HEADER,
    PACKET_SIZE,
    PAYLOAD_FORMAT,
    PAYLOAD_OFFSET,
    checksum,
)
from isaac_core.contracts.pose import (
    MAX_LAT_DEG,
    MAX_LON_DEG,
    MIN_LAT_DEG,
    MIN_LON_DEG,
    GeodeticPose,
    Lla,
    Rpy,
)
from isaac_core.protocol.errors import (
    PacketChecksumError,
    PacketHeaderError,
    PacketLengthError,
    PacketPayloadError,
    PosePacketError,
)


def encode(pose: GeodeticPose) -> bytes:
    """Serialize a geodetic pose into the 51-byte wire format.

    Angles go on the wire in radians. The orientation must be tagged
    :attr:`Frame.NED` -- silently sending ENU is exactly the class of bug this
    codebase is designed to prevent.

    Args:
        pose: The pose to encode. Its orientation frame must be ``Frame.NED``.

    Returns:
        Exactly 51 bytes: 2-byte header, 48-byte payload, 1-byte XOR checksum.

    Raises:
        ValueError: If the orientation is not in the NED frame.

    """
    if pose.orientation.frame is not Frame.NED:
        msg = (
            f"Wire format requires NED orientation, got {pose.orientation.frame.value!r}. " f"Convert before encoding."
        )
        raise ValueError(msg)

    payload = struct.pack(
        PAYLOAD_FORMAT,
        pose.position.lat_deg,
        pose.position.lon_deg,
        pose.position.alt_m,
        pose.orientation.roll_r,
        pose.orientation.pitch_r,
        pose.orientation.yaw_r,
    )

    chk = checksum(payload)
    return bytes(HEADER) + payload + bytes([chk])


def decode(data: bytes) -> GeodeticPose:
    """Deserialize 51 raw bytes into a geodetic pose tagged ``Frame.NED``.

    Validates length, header, checksum, and payload structure, raising a specific
    :class:`~isaac_core.protocol.errors.PosePacketError` subclass for each failure
    mode.

    Args:
        data: Raw bytes received from the network.

    Returns:
        The decoded pose with orientation in ``Frame.NED``.

    Raises:
        PacketLengthError: If ``len(data)`` is not 51.
        PacketHeaderError: If the first two bytes are not ``0xAC 0xDC``.
        PacketChecksumError: If the XOR checksum does not match.
        PacketPayloadError: If the payload cannot be unpacked, or carries a non-finite or
            out-of-range value.

    """
    if len(data) != PACKET_SIZE:
        raise PacketLengthError(actual=len(data), expected=PACKET_SIZE)

    h0, h1 = data[0], data[1]
    if (h0, h1) != HEADER:
        raise PacketHeaderError(actual=(h0, h1), expected=HEADER)

    payload = data[PAYLOAD_OFFSET:CHECKSUM_OFFSET]
    expected_chk = data[CHECKSUM_OFFSET]
    actual_chk = checksum(payload)

    if actual_chk != expected_chk:
        raise PacketChecksumError(actual=actual_chk, expected=expected_chk)

    try:
        fields = struct.unpack(PAYLOAD_FORMAT, payload)
    except struct.error as exc:
        raise PacketPayloadError(reason=str(exc)) from exc

    # Converted here because Lla raises ValueError, which HoldLastGoodDecoder does not catch -- one
    # hostile packet crashed the receive loop. Non-finite values are rejected too: inf and NaN
    # propagate silently into the stage and fail far from the cause.
    for name, value in zip(FIELD_ORDER, fields, strict=True):
        if not math.isfinite(value):
            raise PacketPayloadError(reason=f"{name} is not finite: {value!r}")
    if not MIN_LAT_DEG <= fields[0] <= MAX_LAT_DEG:
        raise PacketPayloadError(reason=f"lat_deg must be in [{MIN_LAT_DEG}, {MAX_LAT_DEG}], got {fields[0]!r}")
    if not MIN_LON_DEG <= fields[1] <= MAX_LON_DEG:
        raise PacketPayloadError(reason=f"lon_deg must be in [{MIN_LON_DEG}, {MAX_LON_DEG}], got {fields[1]!r}")

    return GeodeticPose(
        position=Lla(lat_deg=fields[0], lon_deg=fields[1], alt_m=fields[2]),
        orientation=Rpy(roll_r=fields[3], pitch_r=fields[4], yaw_r=fields[5], frame=Frame.NED),
    )


class HoldLastGoodDecoder:
    """Stateful decoder implementing the hold-last-good contract.

    On any malformed packet, the last successfully decoded pose is returned instead
    of raising or dropping to zero. This is deliberate and valuable for a real-time
    camera rig: a single corrupted datagram should freeze the view, not teleport it
    to null island.

    The failure count and last error are exposed rather than only logged, so monitoring can
    detect sustained corruption.
    """

    def __init__(self) -> None:
        """Initialize with no good packet and zero failures."""
        self._last_good: GeodeticPose | None = None
        self._failure_count: int = 0
        self._last_error: PosePacketError | None = None

    @property
    def last_good(self) -> GeodeticPose | None:
        """The most recently decoded valid pose, or ``None`` before any good packet."""
        return self._last_good

    @property
    def failure_count(self) -> int:
        """Total number of failed decode attempts since construction."""
        return self._failure_count

    @property
    def last_error(self) -> PosePacketError | None:
        """The exception from the most recent failed decode, or ``None`` if none yet."""
        return self._last_error

    def decode(self, data: bytes) -> GeodeticPose | None:
        """Attempt to decode a packet, falling back to the last good pose on failure.

        Args:
            data: Raw bytes received from the network.

        Returns:
            The decoded pose if valid, the previous good pose if invalid, or
            ``None`` if no valid packet has ever been received.

        """
        try:
            pose = decode(data)
        except PosePacketError as exc:
            self._failure_count += 1
            self._last_error = exc
            return self._last_good

        self._last_good = pose
        return pose


__all__ = [
    "HoldLastGoodDecoder",
    "decode",
    "encode",
]
