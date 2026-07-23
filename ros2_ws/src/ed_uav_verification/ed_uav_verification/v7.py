"""Minimal native V7 framing shared by the deterministic PTY fake."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final


V7_HEADER: Final = 0xAA
V7_MINIMUM_FRAME_SIZE: Final = 6


@dataclass(frozen=True, slots=True)
class V7Frame:
    """A decoded native V7 frame without its transport checksums."""

    address: int
    frame_id: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class V7FrameEncodeError(Exception):
    """Raised when a caller cannot encode fields representable by native V7."""

    reason: str

    def __str__(self) -> str:
        return self.reason


def encode_v7_frame(frame: V7Frame) -> bytes:
    """Encode a native V7 frame using the legacy SC and AC checksum algorithm."""
    payload_length = len(frame.payload)
    if not 0 <= frame.address <= 0xFF or not 0 <= frame.frame_id <= 0xFF:
        raise V7FrameEncodeError("V7 address and frame id must fit in one byte")
    if payload_length > 0xFF:
        raise V7FrameEncodeError("V7 payload exceeds one-byte length")
    prefix = bytes((V7_HEADER, frame.address, frame.frame_id, payload_length)) + frame.payload
    sum_check = 0
    add_check = 0
    for value in prefix:
        sum_check = (sum_check + value) & 0xFF
        add_check = (add_check + sum_check) & 0xFF
    return prefix + bytes((sum_check, add_check))


def decode_v7_frame(raw: bytes) -> V7Frame | None:
    """Decode a complete valid V7 frame, rejecting length and checksum corruption."""
    if len(raw) < V7_MINIMUM_FRAME_SIZE or raw[0] != V7_HEADER:
        return None
    expected_length = raw[3] + V7_MINIMUM_FRAME_SIZE
    if len(raw) != expected_length:
        return None
    encoded = encode_v7_frame(V7Frame(address=raw[1], frame_id=raw[2], payload=raw[4:-2]))
    if encoded != raw:
        return None
    return V7Frame(address=raw[1], frame_id=raw[2], payload=raw[4:-2])
