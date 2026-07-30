"""Bounded authenticated UDP v1 envelope."""

from __future__ import annotations

import hashlib
import hmac
import struct
from typing import Final

from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    AuthenticatedDatagram,
    BootId,
    MessageType,
    OutboundFrame,
    SenderId,
    Sequence,
    SourceMillis,
)


MAGIC: Final = 0x4454
VERSION: Final = 1
MAX_PAYLOAD_BYTES: Final = 64
HMAC_TAG_BYTES: Final = 8
MINIMUM_KEY_BYTES: Final = 32
HEADER: Final = struct.Struct("<HBBHIIII")
CRC: Final = struct.Struct("<H")
MINIMUM_DATAGRAM_BYTES: Final = HEADER.size + CRC.size + HMAC_TAG_BYTES
MAXIMUM_DATAGRAM_BYTES: Final = MINIMUM_DATAGRAM_BYTES + MAX_PAYLOAD_BYTES


def crc16_ccitt(data: bytes) -> int:
    """Return CRC16-CCITT-FALSE for explicit cross-language golden vectors."""
    checksum = 0xFFFF
    for byte in data:
        checksum ^= byte << 8
        for _ in range(8):
            checksum = (
                ((checksum << 1) ^ 0x1021) & 0xFFFF
                if checksum & 0x8000
                else (checksum << 1) & 0xFFFF
            )
    return checksum


def encode_datagram(frame: OutboundFrame, key: bytes) -> bytes:
    _require_key(key)
    _require_uint32(frame.sender_id, ProtocolErrorCode.BAD_SENDER_ID, "sender ID")
    _require_uint32(frame.boot_id, ProtocolErrorCode.BAD_BOOT_EPOCH, "boot ID")
    if len(frame.payload) > MAX_PAYLOAD_BYTES:
        raise ProtocolError(ProtocolErrorCode.DATAGRAM_TOO_LARGE, "payload exceeds 64 bytes")
    _require_uint32(frame.sequence, ProtocolErrorCode.BAD_PAYLOAD, "sequence")
    _require_uint32(frame.source_millis, ProtocolErrorCode.BAD_PAYLOAD, "source millis")

    header = HEADER.pack(
        MAGIC,
        VERSION,
        int(frame.message_type),
        len(frame.payload),
        frame.sender_id,
        frame.boot_id,
        frame.sequence,
        frame.source_millis,
    )
    authenticated_body = header + frame.payload + CRC.pack(crc16_ccitt(header + frame.payload))
    tag = hmac.new(key, authenticated_body, hashlib.sha256).digest()[:HMAC_TAG_BYTES]
    return authenticated_body + tag


def decode_datagram(data: bytes, key: bytes) -> AuthenticatedDatagram:
    _require_key(key)
    if len(data) < MINIMUM_DATAGRAM_BYTES:
        raise ProtocolError(ProtocolErrorCode.DATAGRAM_TOO_SHORT, "datagram is shorter than v1 envelope")
    if len(data) > MAXIMUM_DATAGRAM_BYTES:
        raise ProtocolError(ProtocolErrorCode.DATAGRAM_TOO_LARGE, "datagram exceeds v1 bound")

    magic, version, raw_type, payload_length, sender_id, boot_id, sequence, source_millis = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ProtocolError(ProtocolErrorCode.BAD_MAGIC, "unexpected UDP magic")
    if version != VERSION:
        raise ProtocolError(ProtocolErrorCode.BAD_VERSION, f"unsupported version {version}")
    try:
        message_type = MessageType(raw_type)
    except ValueError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_MESSAGE_TYPE, f"unknown message type {raw_type}") from error
    expected_length = MINIMUM_DATAGRAM_BYTES + payload_length
    if payload_length > MAX_PAYLOAD_BYTES or len(data) != expected_length:
        raise ProtocolError(ProtocolErrorCode.BAD_LENGTH, "payload length does not match datagram")

    authenticated_body = data[:-HMAC_TAG_BYTES]
    expected_tag = hmac.new(key, authenticated_body, hashlib.sha256).digest()[:HMAC_TAG_BYTES]
    if not hmac.compare_digest(data[-HMAC_TAG_BYTES:], expected_tag):
        raise ProtocolError(ProtocolErrorCode.BAD_HMAC, "authentication tag mismatch")
    payload_end = HEADER.size + payload_length
    expected_crc = CRC.unpack_from(data, payload_end)[0]
    actual_crc = crc16_ccitt(data[:payload_end])
    if actual_crc != expected_crc:
        raise ProtocolError(ProtocolErrorCode.BAD_CRC, "CRC16 mismatch")

    frame = OutboundFrame(
        message_type=message_type,
        sender_id=SenderId(sender_id),
        boot_id=BootId(boot_id),
        sequence=Sequence(sequence),
        source_millis=SourceMillis(source_millis),
        payload=data[HEADER.size:payload_end],
    )
    return AuthenticatedDatagram(frame=frame, checksum_crc16=expected_crc)


def _require_key(key: bytes) -> None:
    if len(key) < MINIMUM_KEY_BYTES:
        raise ProtocolError(ProtocolErrorCode.KEY_TOO_SHORT, "HMAC key must be at least 32 bytes")


def _require_uint32(value: int, code: ProtocolErrorCode, field: str) -> None:
    if not 0 <= value <= 0xFFFFFFFF:
        raise ProtocolError(code, f"{field} must be uint32")
