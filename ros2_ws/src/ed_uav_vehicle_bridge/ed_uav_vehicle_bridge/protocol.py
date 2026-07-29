"""Bounded authenticated UDP v1 envelope."""

from __future__ import annotations

import hashlib
import hmac
import struct
from typing import Final

from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    AuthenticatedDatagram,
    BootEpoch,
    MessageType,
    OutboundFrame,
    Sequence,
    SourceMillis,
)


MAGIC: Final = b"EDU1"
VERSION: Final = 1
MAX_PAYLOAD_BYTES: Final = 256
HMAC_TAG_BYTES: Final = 16
MINIMUM_KEY_BYTES: Final = 32
HEADER: Final = struct.Struct(">4sBBH8sQII")
CRC: Final = struct.Struct(">H")
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
    try:
        sender = frame.sender_id.encode("ascii")
    except UnicodeEncodeError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_SENDER_ID, "sender ID must be ASCII") from error
    if not 1 <= len(sender) <= 8 or b"\x00" in sender:
        raise ProtocolError(ProtocolErrorCode.BAD_SENDER_ID, "sender ID must contain 1-8 non-NUL bytes")
    if not 1 <= frame.boot_epoch <= 0xFFFFFFFFFFFFFFFF:
        raise ProtocolError(ProtocolErrorCode.BAD_BOOT_EPOCH, "boot epoch must be a nonzero uint64")
    if len(frame.payload) > MAX_PAYLOAD_BYTES:
        raise ProtocolError(ProtocolErrorCode.DATAGRAM_TOO_LARGE, "payload exceeds 256 bytes")
    if not 0 <= frame.sequence <= 0xFFFFFFFF or not 0 <= frame.source_millis <= 0xFFFFFFFF:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "sequence and source millis must be uint32")

    header = HEADER.pack(
        MAGIC,
        VERSION,
        int(frame.message_type),
        len(frame.payload),
        sender.ljust(8, b"\x00"),
        frame.boot_epoch,
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

    magic, version, raw_type, payload_length, raw_sender, epoch, sequence, source_millis = HEADER.unpack_from(data)
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
    sender = _decode_sender(raw_sender)
    if epoch == 0:
        raise ProtocolError(ProtocolErrorCode.BAD_BOOT_EPOCH, "boot epoch must be nonzero")

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
        sender_id=sender,
        boot_epoch=BootEpoch(epoch),
        sequence=Sequence(sequence),
        source_millis=SourceMillis(source_millis),
        payload=data[HEADER.size:payload_end],
    )
    return AuthenticatedDatagram(frame=frame, checksum_crc16=expected_crc)


def _require_key(key: bytes) -> None:
    if len(key) < MINIMUM_KEY_BYTES:
        raise ProtocolError(ProtocolErrorCode.KEY_TOO_SHORT, "HMAC key must be at least 32 bytes")


def _decode_sender(raw_sender: bytes) -> str:
    sender_bytes, separator, padding = raw_sender.partition(b"\x00")
    if not sender_bytes or (separator and any(padding)):
        raise ProtocolError(ProtocolErrorCode.BAD_SENDER_ID, "invalid sender padding")
    try:
        return sender_bytes.decode("ascii")
    except UnicodeDecodeError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_SENDER_ID, "sender ID must be ASCII") from error
