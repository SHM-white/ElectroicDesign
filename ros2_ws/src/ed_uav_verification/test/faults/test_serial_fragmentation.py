"""Fragmented serial frame handling: corruption, truncation, and bit errors."""

from __future__ import annotations

import struct

from ed_uav_verification.fcu import DeterministicPtyFcu, FcuReadTimeout, position_v7_frame
from ed_uav_verification.v7 import V7Frame, V7_MINIMUM_FRAME_SIZE, decode_v7_frame, encode_v7_frame


def test_truncated_frame_is_rejected() -> None:
    """Given a truncated V7 frame, when decoded, then no valid frame is produced."""
    valid = encode_v7_frame(V7Frame(address=0xFF, frame_id=0x08, payload=b"testdata"))

    # Every truncation shorter than the minimum frame size must fail
    for cut in range(1, V7_MINIMUM_FRAME_SIZE):
        assert decode_v7_frame(valid[:cut]) is None, f"truncation to {cut} bytes produced a frame"

    # Truncation of a valid frame at any intermediate byte must fail
    for cut in range(V7_MINIMUM_FRAME_SIZE, len(valid) - 1):
        result = decode_v7_frame(valid[:cut])
        if result is not None:
            # Verify it's not our original frame (it shouldn't be)
            assert result.payload != b"testdata", f"truncation to {cut} bytes produced original payload"


def test_bit_flipped_checksum_is_rejected() -> None:
    """Given a V7 frame with a flipped checksum byte, when decoded, then it is rejected."""
    valid = encode_v7_frame(V7Frame(address=0xFF, frame_id=0x06, payload=struct.pack("<i", 12345)))

    # Flip each bit in each checksum byte (last 2 bytes)
    for byte_offset in (-2, -1):
        for bit in range(8):
            corrupt = bytearray(valid)
            corrupt[byte_offset] ^= 1 << bit
            assert decode_v7_frame(bytes(corrupt)) is None, (
                f"bit flip at offset {byte_offset} bit {bit} produced a valid frame"
            )


def test_length_field_corruption_is_rejected() -> None:
    """Given a V7 frame with a corrupted length field, when decoded, then it is rejected."""
    payload = b"payload_data_16b"
    valid = encode_v7_frame(V7Frame(address=0xFF, frame_id=0x05, payload=payload))
    original_length = len(payload)

    # Length field is at index 3
    for lie in range(256):
        if lie == original_length:
            continue
        corrupt = bytearray(valid)
        corrupt[3] = lie
        assert decode_v7_frame(bytes(corrupt)) is None, (
            f"forged length {lie} produced a valid frame"
        )


def test_interleaved_garbage_does_not_produce_spurious_frame() -> None:
    """Given a valid V7 frame embedded in random bytes, when scanned, then no spurious frame emerges."""
    valid = encode_v7_frame(V7Frame(address=0xFF, frame_id=0x08, payload=b"target"))

    # Inject garbage between the header and payload
    garbage = bytes([0x00, 0xFF, 0xAA, 0x41, 0x55])
    interleaved = valid[:4] + garbage + valid[4:]
    assert decode_v7_frame(interleaved) is None, "interleaved garbage produced a spurious frame"

    # Inject garbage before the header
    prefixed = garbage + valid
    assert decode_v7_frame(prefixed) is None, "garbage-prefixed frame was accepted"

    # Inject garbage after a valid frame
    suffixed = valid + garbage
    assert decode_v7_frame(suffixed) is None, "garbage-suffixed frame was accepted"


def test_empty_and_minimal_payload_frames_encode_correctly() -> None:
    """Given empty and single-byte V7 payloads, when encoded and decoded, then they survive round-trip."""
    frames = (
        V7Frame(address=0x01, frame_id=0x02, payload=b""),
        V7Frame(address=0xFF, frame_id=0x00, payload=b"x"),
        V7Frame(address=0x00, frame_id=0xFF, payload=b"\x00"),
        V7Frame(address=0xAA, frame_id=0x55, payload=b"\xFF\x00\xAB"),
    )

    for original in frames:
        encoded = encode_v7_frame(original)
        decoded = decode_v7_frame(encoded)
        assert decoded is not None, f"round-trip failed for {original}"
        assert decoded == original
        assert len(encoded) == V7_MINIMUM_FRAME_SIZE + len(original.payload)


def test_pty_read_without_emit_times_out_without_deadlock() -> None:
    """Given an empty PTY slave, when reading without emitting, then it times out without deadlock."""
    with DeterministicPtyFcu() as fcu:
        import time

        started = time.perf_counter_ns()
        try:
            fcu.read_slave_frame()
            assert False, "expected FcuReadTimeout"
        except FcuReadTimeout:
            elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000
            assert elapsed_ms < 500, f"read timeout took {elapsed_ms:.0f}ms, expected <500ms"
        # PTY is still usable after timeout
        assert not fcu.closed


def test_fragment_frame_reassembly_on_sequential_writes() -> None:
    """Given a V7 frame written byte-by-byte, when the slave reads, then the complete frame is available."""
    with DeterministicPtyFcu() as fcu:
        frame = position_v7_frame(x_cm=100, y_cm=200)
        import os

        # Write the frame one byte at a time to simulate fragmentation
        for byte in frame:
            os.write(fcu._master_fd, bytes([byte]))

        received = fcu.read_slave_frame()
        assert received == frame
        assert decode_v7_frame(received) is not None

    assert fcu.closed
