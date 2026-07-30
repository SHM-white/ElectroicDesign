from dataclasses import replace
import hashlib
import hmac

import pytest

from ed_uav_vehicle_bridge.errors import ProtocolError, ProtocolErrorCode
from ed_uav_vehicle_bridge.models import BootId, MessageType, OutboundFrame, SenderId, Sequence, SourceMillis
from ed_uav_vehicle_bridge.protocol import (
    HMAC_TAG_BYTES,
    MAX_PAYLOAD_BYTES,
    decode_datagram,
    encode_datagram,
)


KEY = bytes(range(32))
FRAME = OutboundFrame(
    message_type=MessageType.CAR_TELEMETRY,
    sender_id=SenderId(0x43415231),
    boot_id=BootId(0x10203040),
    sequence=Sequence(0xFFFFFFFE),
    source_millis=SourceMillis(0x01020304),
    payload=bytes.fromhex("010102070003002efbffff410183ff0000"),
)
GOLDEN_HEX = "5444010211003152414340302010feffffff04030201010102070003002efbffff410183ff000050ee78e8287ab844854e"


def test_wire_golden_vector_is_stable() -> None:
    # Given: delegated fixed fields, telemetry payload, and a 32-byte key.
    # When: the v1 datagram is serialized.
    encoded = encode_datagram(FRAME, KEY)

    # Then: every wire byte matches the independently generated vector.
    assert encoded.hex() == GOLDEN_HEX
    assert len(encoded) == 49
    assert decode_datagram(encoded, KEY).frame == FRAME


@pytest.mark.parametrize(
    ("mutate", "expected"),
    [
        (lambda packet: b"\x00\x00" + packet[2:], ProtocolErrorCode.BAD_MAGIC),
        (lambda packet: packet[:2] + b"\x02" + packet[3:], ProtocolErrorCode.BAD_VERSION),
        (lambda packet: packet[:3] + b"\xff" + packet[4:], ProtocolErrorCode.BAD_MESSAGE_TYPE),
        (lambda packet: packet[:4] + b"\x01\x00" + packet[6:], ProtocolErrorCode.BAD_LENGTH),
        (lambda packet: packet[:-1] + bytes([packet[-1] ^ 1]), ProtocolErrorCode.BAD_HMAC),
    ],
)
def test_malformed_envelope_is_rejected(mutate, expected: ProtocolErrorCode) -> None:
    # Given: a valid authenticated datagram.
    packet = encode_datagram(FRAME, KEY)

    # When: one bounded envelope invariant is corrupted.
    with pytest.raises(ProtocolError) as raised:
        decode_datagram(mutate(packet), KEY)

    # Then: parsing fails with a typed boundary reason.
    assert raised.value.code is expected


def test_crc_failure_is_distinct_after_valid_authentication() -> None:
    # Given: a packet whose CRC is changed and whose HMAC is recomputed by a key holder.
    packet = bytearray(encode_datagram(FRAME, KEY))
    packet[-HMAC_TAG_BYTES - 2] ^= 1
    packet[-HMAC_TAG_BYTES:] = hmac.new(KEY, packet[:-HMAC_TAG_BYTES], hashlib.sha256).digest()[:HMAC_TAG_BYTES]

    # When: the packet crosses the parser.
    with pytest.raises(ProtocolError) as raised:
        decode_datagram(bytes(packet), KEY)

    # Then: the checksum failure is reported after authentication succeeds.
    assert raised.value.code is ProtocolErrorCode.BAD_CRC


def test_sender_and_payload_bounds_are_enforced() -> None:
    # Given: a valid frame, a maximum-size payload, and an oversized payload.
    maximum_payload = replace(FRAME, payload=b"x" * MAX_PAYLOAD_BYTES)
    oversized_payload = replace(FRAME, payload=b"x" * (MAX_PAYLOAD_BYTES + 1))

    # When/Then: the maximum is encoded and the next byte is rejected.
    assert len(encode_datagram(maximum_payload, KEY)) == 96
    with pytest.raises(ProtocolError) as payload_error:
        encode_datagram(oversized_payload, KEY)
    assert payload_error.value.code is ProtocolErrorCode.DATAGRAM_TOO_LARGE


def test_short_key_is_rejected() -> None:
    with pytest.raises(ProtocolError) as raised:
        encode_datagram(FRAME, b"development-key")
    assert raised.value.code is ProtocolErrorCode.KEY_TOO_SHORT
