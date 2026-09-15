"""Wire codecs for the pose UDP protocol.

This package contains the encoder/decoder for the 51-byte pose packet that
drives camera placement in Isaac Sim. It is pure Python with no I/O: encoding
bytes and decoding bytes, nothing more. The transport layer (sockets, threading,
rate control) belongs elsewhere.

Contents:

:mod:`~isaac_core.protocol.errors`
    Typed exception hierarchy for codec failures.
:mod:`~isaac_core.protocol.pose_packet`
    Encode, decode, and hold-last-good stateful decoder.
"""

from isaac_core.protocol.errors import (
    PacketChecksumError,
    PacketHeaderError,
    PacketLengthError,
    PacketPayloadError,
    PosePacketError,
)
from isaac_core.protocol.pose_packet import HoldLastGoodDecoder, decode, encode

__all__ = [
    "HoldLastGoodDecoder",
    "PacketChecksumError",
    "PacketHeaderError",
    "PacketLengthError",
    "PacketPayloadError",
    "PosePacketError",
    "decode",
    "encode",
]
