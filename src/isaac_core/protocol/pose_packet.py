"""
Encode and decode the 51-byte UDP pose packet.

This is the wire codec only -- no sockets, no I/O. Transport belongs to a later
layer. Byte compatibility with the previous generation is mandatory: the existing
debugger GUI and senders produce and consume exactly these bytes.

The specification lives in :mod:`isaac_core.contracts.packet`; this module is the
implementation that honours it.
"""

import struct

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
    PacketPayloadError,
    PosePacketError,
)


def encode(pose: GeodeticPose) -> bytes:
    """
    Serialize a geodetic pose into the 51-byte wire format.

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
    """
    Deserialize 51 raw bytes into a geodetic pose tagged ``Frame.NED``.

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
        PacketPayloadError: If the payload cannot be unpacked.

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

    return GeodeticPose(
        position=Lla(lat_deg=fields[0], lon_deg=fields[1], alt_m=fields[2]),
        orientation=Rpy(roll_r=fields[3], pitch_r=fields[4], yaw_r=fields[5], frame=Frame.NED),
    )


class HoldLastGoodDecoder:
    """
    Stateful decoder preserving the previous generation's hold-last-good behaviour.

    On any malformed packet, the last successfully decoded pose is returned instead
    of raising or dropping to zero. This is deliberate and valuable for a real-time
    camera rig: a single corrupted datagram should freeze the view, not teleport it
    to null island.

    Unlike the previous generation, which only logged failures silently, this
    decoder exposes the failure count and the last error so monitoring can detect
    sustained corruption.
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
        """
        Attempt to decode a packet, falling back to the last good pose on failure.

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
