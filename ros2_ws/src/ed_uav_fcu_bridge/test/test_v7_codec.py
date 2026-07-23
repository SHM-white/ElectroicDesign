from __future__ import annotations

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.v7_codec import (  # noqa: E402
    FrameDecodeError,
    V7StreamDecoder,
    build_frame,
    cmd_hover,
    cmd_land,
    cmd_lock,
    cmd_mode,
    cmd_move,
    cmd_takeoff,
    cmd_unlock,
    decode_frame,
)


def test_verified_legacy_command_vectors_are_byte_identical() -> None:
    # Given: the verified legacy high-level command vectors.
    expected = (
        (cmd_unlock(), "AAFFE00B1000010000000000000000A585"),
        (cmd_lock(), "AAFFE00B1000020000000000000000A68E"),
        (cmd_mode(3), "AAFFE00B01010103000000000000009A02"),
        (cmd_takeoff(150), "AAFFE00B10000596000000000000003F59"),
        (cmd_land(), "AAFFE00B1000060000000000000000AAB2"),
        (cmd_move(100, 30, 90), "AAFFE00B10020364001E005A00000085E7"),
    )

    # When: the native V7 builders encode each command.
    actual = tuple((frame, frame.hex().upper()) for frame, _ in expected)

    # Then: every verified byte, including both checksums, remains unchanged.
    assert actual == expected


def test_hover_uses_the_manual_one_key_hover_command() -> None:
    # Given: the V7 manual's CID=0x10, CMD0=0x00, CMD1=0x04 hover command.
    expected = build_frame(0xFF, 0xE0, bytes((0x10, 0x00, 0x04)) + bytes(8))

    # When: the bridge builds hover.
    actual = cmd_hover()

    # Then: it is a dedicated one-key hover command, not an invalid zero-speed climb.
    assert actual == expected
    assert actual[4:7] == bytes((0x10, 0x00, 0x04))


def test_decoder_round_trips_fragmented_frame_without_partial_delivery() -> None:
    # Given: a valid V7 frame split at arbitrary serial boundaries.
    raw = cmd_move(100, 30, 90)
    decoder = V7StreamDecoder()

    # When: its fragments arrive incrementally.
    outputs = tuple(
        frame
        for chunk in (raw[:1], raw[1:6], raw[6:-1], raw[-1:])
        for frame in decoder.feed(chunk)
    )

    # Then: exactly one intact native frame is emitted.
    assert len(outputs) == 1
    assert outputs[0].raw == raw
    assert decoder.rejected_frames == 0


def test_decoder_rejects_checksum_and_declared_length_corruption() -> None:
    # Given: valid frames corrupted independently in checksum and declared length.
    checksum_bad = bytearray(cmd_unlock())
    checksum_bad[-1] ^= 0xFF
    length_bad = bytearray(cmd_unlock())
    length_bad[3] += 1

    # When / Then: direct decoding rejects both corrupt forms.
    with pytest.raises(FrameDecodeError):
        decode_frame(bytes(checksum_bad))
    with pytest.raises(FrameDecodeError):
        decode_frame(bytes(length_bad))


def test_decoder_resynchronizes_after_malformed_input() -> None:
    # Given: garbage and a checksum-invalid frame before a good frame.
    invalid = bytearray(cmd_lock())
    invalid[-2] ^= 0x01
    decoder = V7StreamDecoder()

    # When: all bytes arrive as one serial chunk.
    frames = decoder.feed(b"garbage" + bytes(invalid) + cmd_land())

    # Then: malformed bytes do not block the subsequent valid command-sized frame.
    assert tuple(frame.raw for frame in frames) == (cmd_land(),)
    assert decoder.rejected_frames == 1
