"""OpenMV recognition-result serial backend.

Wire format (ASCII, one frame per line)::

    $OMV1,<sequence>,<green_per_mille>,<digit>*<xor>\r\n

The XOR is calculated over the bytes between ``$`` and ``*``. ``digit`` is
``-1`` when no block number was recognized. No image data is transferred.
"""

import logging
import time
from typing import Callable, Optional

try:
    from .vision_result import VisionResult
except ImportError:
    from vision_result import VisionResult

logger = logging.getLogger('drone.openmv')

PROTOCOL_PREFIX = 'OMV1'
MAX_LINE_LENGTH = 96


def _xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def build_openmv_frame(sequence: int, green_ratio: float,
                       digit: Optional[int] = None) -> bytes:
    """Build a protocol frame, primarily for tests and serial diagnostics."""
    if not 0 <= sequence <= 65535:
        raise ValueError('sequence must be in range 0..65535')
    if not 0.0 <= green_ratio <= 1.0:
        raise ValueError('green_ratio must be in range 0.0..1.0')
    if digit is not None and not 1 <= digit <= 28:
        raise ValueError('digit must be None or in range 1..28')

    green_per_mille = int(round(green_ratio * 1000.0))
    digit_value = -1 if digit is None else digit
    body = f'{PROTOCOL_PREFIX},{sequence},{green_per_mille},{digit_value}'.encode('ascii')
    return b'$' + body + f'*{_xor_checksum(body):02X}\r\n'.encode('ascii')


def parse_openmv_frame(line: bytes) -> Optional[VisionResult]:
    """Parse and validate one OpenMV result frame.

    Invalid or corrupted frames return ``None`` so callers can keep using the
    last recent valid observation.
    """
    try:
        raw = line.strip()
        if len(raw) > MAX_LINE_LENGTH or not raw.startswith(b'$'):
            return None

        body, checksum_text = raw[1:].rsplit(b'*', 1)
        if len(checksum_text) != 2:
            return None
        if int(checksum_text, 16) != _xor_checksum(body):
            return None

        fields = body.decode('ascii').split(',')
        if len(fields) != 4 or fields[0] != PROTOCOL_PREFIX:
            return None

        sequence = int(fields[1])
        green_per_mille = int(fields[2])
        digit_value = int(fields[3])
        if not 0 <= sequence <= 65535:
            return None
        if not 0 <= green_per_mille <= 1000:
            return None
        if digit_value != -1 and not 1 <= digit_value <= 28:
            return None

        return VisionResult(
            green_ratio=green_per_mille / 1000.0,
            digit=None if digit_value == -1 else digit_value,
            sequence=sequence,
        )
    except (UnicodeDecodeError, ValueError):
        return None


class OpenMVVision:
    """Non-blocking serial receiver for recognition results from OpenMV."""

    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 115200,
                 stale_timeout_s: float = 0.5, serial_instance=None,
                 monotonic: Callable[[], float] = time.monotonic):
        self.port = port
        self.baudrate = baudrate
        self.stale_timeout_s = stale_timeout_s
        self._serial = serial_instance
        self._owns_serial = serial_instance is None
        self._monotonic = monotonic
        self._rx_buffer = bytearray()
        self._last_result: Optional[VisionResult] = None
        self._last_result_time = 0.0
        self._invalid_frames = 0

    def open(self) -> bool:
        """Open the OpenMV result serial port."""
        if self._serial is not None:
            return True
        try:
            import serial
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0.1,
            )
            reset_input = getattr(self._serial, 'reset_input_buffer', None)
            if callable(reset_input):
                reset_input()
            logger.info("OpenMV connected: %s @ %d", self.port, self.baudrate)
            return True
        except Exception as exc:
            logger.error("Cannot open OpenMV serial port %s: %s", self.port, exc)
            self._serial = None
            return False

    def _poll(self) -> None:
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._rx_buffer.extend(self._serial.read(waiting))
        except Exception as exc:
            logger.warning("OpenMV serial read error: %s", exc)
            return

        while b'\n' in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            frame_start = raw_line.rfind(b'$')
            if frame_start >= 0:
                raw_line = raw_line[frame_start:]
            result = parse_openmv_frame(raw_line)
            if result is None:
                self._invalid_frames += 1
                if self._invalid_frames <= 3 or self._invalid_frames % 100 == 0:
                    logger.warning("Ignored invalid OpenMV frame (%d total)",
                                   self._invalid_frames)
                continue
            self._last_result = result
            self._last_result_time = self._monotonic()

        if len(self._rx_buffer) > MAX_LINE_LENGTH * 2:
            logger.warning("Discarding oversized OpenMV receive buffer")
            self._rx_buffer.clear()

    def read_result(self) -> Optional[VisionResult]:
        """Return the newest result while it is fresh enough for control use."""
        self._poll()
        if self._last_result is None:
            return None
        if self._monotonic() - self._last_result_time > self.stale_timeout_s:
            return None
        return self._last_result

    def release(self) -> None:
        """Close the serial port if this backend opened it."""
        if self._serial is not None and self._owns_serial:
            try:
                self._serial.close()
            except Exception as exc:
                logger.warning("OpenMV serial close error: %s", exc)
        self._serial = None
        self._rx_buffer.clear()
        logger.info("OpenMV released")

    @property
    def invalid_frames(self) -> int:
        return self._invalid_frames
