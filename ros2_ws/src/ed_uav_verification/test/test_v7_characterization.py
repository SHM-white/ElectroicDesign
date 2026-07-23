from __future__ import annotations

from ed_uav_verification.v7 import V7Frame, decode_v7_frame, encode_v7_frame


def test_v7_codec_matches_characterized_legacy_samples() -> None:
    """Given legacy vectors, when encoded and decoded, then V7 bytes stay exact."""
    vectors = (
        (V7Frame(address=0xFF, frame_id=0x08, payload=bytes.fromhex("d2040000c9fdffff")), "aaff0808d2040000c9fdffff5349"),
        (V7Frame(address=0xFF, frame_id=0x06, payload=bytes.fromhex("0300000000")), "aaff06050300000000b749"),
        (V7Frame(address=0xFF, frame_id=0x05, payload=bytes.fromhex("960000000000000001")), "aaff05099600000000000000014e6e"),
    )

    for expected, hex_frame in vectors:
        encoded = encode_v7_frame(expected)
        decoded = decode_v7_frame(encoded)

        assert encoded.hex() == hex_frame
        assert decoded == expected


def test_v7_codec_rejects_length_and_checksum_corruption() -> None:
    """Given malformed V7 bytes, when decoded, then no stale frame can be accepted."""
    valid = bytes.fromhex("aaff0808d2040000c9fdffff5349")
    corrupt = valid[:-1] + bytes([valid[-1] ^ 0xFF])

    assert decode_v7_frame(corrupt) is None
    assert decode_v7_frame(valid[:-1]) is None
