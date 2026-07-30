"""Pure encoder and incremental decoder for native Lingxiao V7 frames."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Final

FRAME_HEADER: Final = 0xAA
MAX_PAYLOAD_BYTES: Final = 255
PROGRAMMABLE_FRAME_IDS: Final = frozenset((0xE0,))
REALTIME_FRAME_IDS: Final = frozenset((0x41,))


class FrameDecodeError(ValueError):
    """Raised when raw bytes are not one valid native V7 frame."""


@dataclass(frozen=True, slots=True)
class V7Frame:
    """A checksum-verified native V7 frame."""

    address: int
    frame_id: int
    data: bytes
    sum_check: int
    add_check: int
    raw: bytes


@dataclass(frozen=True, slots=True)
class RealtimeControlFields:
    """The seven signed int16 fields in one Lingxiao V7 realtime frame."""

    roll: int
    pitch: int
    thr: int
    yaw_dps: int
    spd_x: int
    spd_y: int
    spd_z: int


def _checksums(prefix: bytes) -> tuple[int, int]:
    sum_check = 0
    add_check = 0
    for value in prefix:
        sum_check = (sum_check + value) & 0xFF
        add_check = (add_check + sum_check) & 0xFF
    return sum_check, add_check


def build_frame(address: int, frame_id: int, data: bytes) -> bytes:
    """Encode one native V7 frame including its Fletcher-style checksums."""
    if not 0 <= address <= 0xFF or not 0 <= frame_id <= 0xFF:
        raise ValueError("V7 address and frame id must fit in one byte")
    if len(data) > MAX_PAYLOAD_BYTES:
        raise ValueError("V7 payload exceeds the one-byte length field")
    prefix = bytes((FRAME_HEADER, address, frame_id, len(data))) + data
    sum_check, add_check = _checksums(prefix)
    return prefix + bytes((sum_check, add_check))


def cmd_realtime_control(fields: RealtimeControlFields) -> bytes:
    """Encode ID 0x41 with seven little-endian signed int16 control fields."""
    values = (
        fields.roll,
        fields.pitch,
        fields.thr,
        fields.yaw_dps,
        fields.spd_x,
        fields.spd_y,
        fields.spd_z,
    )
    if any(value < -0x8000 or value > 0x7FFF for value in values):
        raise ValueError("V7 realtime-control fields must fit signed int16")
    return build_frame(0xFF, 0x41, struct.pack("<7h", *values))


def decode_frame(raw: bytes) -> V7Frame:
    """Decode exactly one complete native V7 frame or raise FrameDecodeError."""
    if len(raw) < 6:
        raise FrameDecodeError("V7 frame is shorter than its fixed fields")
    if raw[0] != FRAME_HEADER:
        raise FrameDecodeError("V7 frame header is not 0xAA")
    if len(raw) != raw[3] + 6:
        raise FrameDecodeError("V7 frame length does not match its declared payload")
    sum_check, add_check = _checksums(raw[:-2])
    if (sum_check, add_check) != (raw[-2], raw[-1]):
        raise FrameDecodeError("V7 frame checksum mismatch")
    return V7Frame(
        address=raw[1],
        frame_id=raw[2],
        data=raw[4:-2],
        sum_check=raw[-2],
        add_check=raw[-1],
        raw=raw,
    )


class V7StreamDecoder:
    """Mutable incremental serial decoder which resynchronizes after bad frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.rejected_frames = 0

    def feed(self, chunk: bytes) -> tuple[V7Frame, ...]:
        """Consume serial bytes and return complete checksum-verified frames."""
        self._buffer.extend(chunk)
        frames: list[V7Frame] = []
        while len(self._buffer) >= 4:
            try:
                start = self._buffer.index(FRAME_HEADER)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break
            total_length = self._buffer[3] + 6
            if len(self._buffer) < total_length:
                break
            raw = bytes(self._buffer[:total_length])
            try:
                frame = decode_frame(raw)
            except FrameDecodeError:
                self.rejected_frames += 1
                del self._buffer[0]
                continue
            del self._buffer[:total_length]
            frames.append(frame)
        return tuple(frames)


def _command(payload: bytes) -> bytes:
    return build_frame(0xFF, 0xE0, payload + bytes(11 - len(payload)))


def cmd_mode(mode: int) -> bytes:
    """Build the legacy-compatible V7 mode selection command."""
    if mode not in (0, 1, 2, 3):
        raise ValueError("V7 mode must be in the inclusive range 0..3")
    return _command(bytes((0x01, 0x01, 0x01, mode)))


def cmd_unlock() -> bytes:
    """Build the V7 one-key motor unlock command."""
    return _command(bytes((0x10, 0x00, 0x01)))


def cmd_lock() -> bytes:
    """Build the V7 one-key motor lock command."""
    return _command(bytes((0x10, 0x00, 0x02)))


def cmd_hover() -> bytes:
    """Build the manual-defined V7 one-key hover command."""
    return _command(bytes((0x10, 0x00, 0x04)))


def cmd_takeoff(height_cm: int) -> bytes:
    """Build the V7 one-key takeoff command with a little-endian height."""
    return _command(bytes((0x10, 0x00, 0x05)) + height_cm.to_bytes(2, "little"))


def cmd_land() -> bytes:
    """Build the V7 one-key land command."""
    return _command(bytes((0x10, 0x00, 0x06)))


def cmd_move(distance_cm: int, speed_cmps: int, direction_deg: int) -> bytes:
    """Build the legacy-compatible V7 body-relative move command."""
    if not 0 <= distance_cm <= 10000:
        raise ValueError("V7 move distance must be in the inclusive range 0..10000 cm")
    if not 10 <= speed_cmps <= 300:
        raise ValueError("V7 move speed must be in the inclusive range 10..300 cm/s")
    if not 0 <= direction_deg <= 359:
        raise ValueError("V7 move direction must be in the inclusive range 0..359 degrees")
    return _command(
        bytes((0x10, 0x02, 0x03))
        + distance_cm.to_bytes(2, "little")
        + speed_cmps.to_bytes(2, "little")
        + direction_deg.to_bytes(2, "little")
    )


def _signed_cm(value_cm: int) -> bytes:
    if not -100000 <= value_cm <= 100000:
        raise ValueError("V7 programmable distance must be in the inclusive range -100000..100000 cm")
    return value_cm.to_bytes(4, "little", signed=True)


def _vertical_distance_speed(distance_cm: int, speed_cmps: int) -> bytes:
    if not 0 <= distance_cm <= 10000:
        raise ValueError("V7 vertical distance must be in the inclusive range 0..10000 cm")
    if not 10 <= speed_cmps <= 300:
        raise ValueError("V7 vertical speed must be in the inclusive range 10..300 cm/s")
    return distance_cm.to_bytes(2, "little") + speed_cmps.to_bytes(2, "little")


def cmd_target_position(first_cm: int, second_cm: int) -> bytes:
    """Build documented target-position fields; the manual does not name their axes."""
    return _command(bytes((0x10, 0x01, 0x01)) + _signed_cm(first_cm) + _signed_cm(second_cm))


def cmd_target_height(height_cm: int) -> bytes:
    """Build the documented signed target-ground-height command."""
    return _command(bytes((0x10, 0x01, 0x02)) + _signed_cm(height_cm))


def cmd_ascend(distance_cm: int, speed_cmps: int) -> bytes:
    """Build the documented relative ascend command."""
    return _command(bytes((0x10, 0x02, 0x01)) + _vertical_distance_speed(distance_cm, speed_cmps))


def cmd_descend(distance_cm: int, speed_cmps: int) -> bytes:
    """Build the documented relative descend command."""
    return _command(bytes((0x10, 0x02, 0x02)) + _vertical_distance_speed(distance_cm, speed_cmps))
