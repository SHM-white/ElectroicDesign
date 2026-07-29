from dataclasses import replace
import hashlib
import hmac

import pytest

from ed_uav_vehicle_bridge.errors import ProtocolError, ProtocolErrorCode
from ed_uav_vehicle_bridge.models import (
    BootEpoch,
    MessageType,
    OutboundFrame,
    Sequence,
    SourceMillis,
)
from ed_uav_vehicle_bridge.protocol import decode_datagram, encode_datagram


KEY = bytes(range(32))
FRAME = OutboundFrame(
    message_type=MessageType.CAR_TELEMETRY,
    sender_id="CAR-01",
    boot_epoch=BootEpoch(0x0102030405060708),
    sequence=Sequence(0xFFFFFFFE),
    source_millis=SourceMillis(0x10203040),
    payload=bytes.fromhex("010203"),
)
GOLDEN_HEX = (
    "45445531010100034341522d303100000102030405060708fffffffe10203040"
    "0102034450affe1d99aa17475115930f8a10f67f52"
)


def test_wire_golden_vector_is_stable() -> None:
    # Given: fixed fields, payload, and a 32-byte provisioning key.
    # When: the v1 datagram is serialized.
    encoded = encode_datagram(FRAME, KEY)

    # Then: every wire byte matches the independently generated vector.
    assert encoded.hex() == GOLDEN_HEX
    assert decode_datagram(encoded, KEY).frame == FRAME


@pytest.mark.parametrize(
    ("mutate", "expected"),
    [
        (lambda packet: b"BAD!" + packet[4:], ProtocolErrorCode.BAD_MAGIC),
        (lambda packet: packet[:4] + b"\x02" + packet[5:], ProtocolErrorCode.BAD_VERSION),
        (lambda packet: packet[:5] + b"\xff" + packet[6:], ProtocolErrorCode.BAD_MESSAGE_TYPE),
        (lambda packet: packet[:6] + b"\x01\x00" + packet[8:], ProtocolErrorCode.BAD_LENGTH),
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
    packet[-18] ^= 1
    packet[-16:] = hmac.new(KEY, packet[:-16], hashlib.sha256).digest()[:16]

    # When: the packet crosses the parser.
    with pytest.raises(ProtocolError) as raised:
        decode_datagram(bytes(packet), KEY)

    # Then: the checksum failure is reported after authentication succeeds.
    assert raised.value.code is ProtocolErrorCode.BAD_CRC


def test_sender_and_payload_bounds_are_enforced() -> None:
    # Given: values outside the fixed sender and payload bounds.
    oversized_sender = replace(FRAME, sender_id="CAR-SENDER")
    oversized_payload = replace(FRAME, payload=b"x" * 257)

    # When/Then: serialization rejects both before socket I/O.
    with pytest.raises(ProtocolError) as sender_error:
        encode_datagram(oversized_sender, KEY)
    with pytest.raises(ProtocolError) as payload_error:
        encode_datagram(oversized_payload, KEY)
    assert sender_error.value.code is ProtocolErrorCode.BAD_SENDER_ID
    assert payload_error.value.code is ProtocolErrorCode.DATAGRAM_TOO_LARGE


def test_short_key_is_rejected() -> None:
    with pytest.raises(ProtocolError) as raised:
        encode_datagram(FRAME, b"development-key")
    assert raised.value.code is ProtocolErrorCode.KEY_TOO_SHORT
