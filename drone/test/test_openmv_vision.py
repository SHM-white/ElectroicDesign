"""Tests for the OpenMV result protocol and serial backend."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from openmv_vision import (  # noqa: E402
    OpenMVVision,
    build_openmv_frame,
    parse_openmv_frame,
)


class FakeSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, size=1):
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def inject(self, data):
        self.buffer.extend(data)

    def close(self):
        self.closed = True


class TestOpenMVProtocol(unittest.TestCase):
    def test_round_trip(self):
        result = parse_openmv_frame(build_openmv_frame(42, 0.731, 21))
        self.assertIsNotNone(result)
        self.assertEqual(result.sequence, 42)
        self.assertAlmostEqual(result.green_ratio, 0.731)
        self.assertEqual(result.digit, 21)
        self.assertIsNone(result.frame)

    def test_no_digit_round_trip(self):
        result = parse_openmv_frame(build_openmv_frame(7, 0.0, None))
        self.assertIsNotNone(result)
        self.assertIsNone(result.digit)

    def test_rejects_bad_checksum(self):
        frame = bytearray(build_openmv_frame(1, 0.5, 3))
        frame[8] = ord('9')
        self.assertIsNone(parse_openmv_frame(bytes(frame)))

    def test_rejects_out_of_range_values(self):
        # Checksum is valid, but green_per_mille=1001 is not.
        body = b'OMV1,1,1001,-1'
        checksum = 0
        for value in body:
            checksum ^= value
        frame = b'$' + body + ('*%02X\n' % checksum).encode('ascii')
        self.assertIsNone(parse_openmv_frame(frame))

    def test_builder_validates_values(self):
        with self.assertRaises(ValueError):
            build_openmv_frame(1, 1.1, None)
        with self.assertRaises(ValueError):
            build_openmv_frame(1, 0.5, 29)


class TestOpenMVVision(unittest.TestCase):
    def test_partial_frame_and_cached_result(self):
        serial = FakeSerial()
        now = [10.0]
        backend = OpenMVVision(
            serial_instance=serial,
            stale_timeout_s=0.5,
            monotonic=lambda: now[0],
        )
        frame = build_openmv_frame(5, 0.625, 12)
        serial.inject(frame[:8])
        self.assertIsNone(backend.read_result())

        serial.inject(frame[8:])
        result = backend.read_result()
        self.assertEqual(result.sequence, 5)
        self.assertEqual(result.digit, 12)

        now[0] += 0.4
        self.assertIs(backend.read_result(), result)
        now[0] += 0.2
        self.assertIsNone(backend.read_result())

    def test_uses_latest_valid_frame(self):
        serial = FakeSerial()
        backend = OpenMVVision(serial_instance=serial)
        serial.inject(b'noise\n')
        serial.inject(build_openmv_frame(10, 0.2, None))
        serial.inject(build_openmv_frame(11, 0.8, 28))

        result = backend.read_result()
        self.assertEqual(result.sequence, 11)
        self.assertAlmostEqual(result.green_ratio, 0.8)
        self.assertEqual(result.digit, 28)
        self.assertEqual(backend.invalid_frames, 1)

    def test_recovers_frame_after_same_line_noise(self):
        serial = FakeSerial()
        backend = OpenMVVision(serial_instance=serial)
        serial.inject(b'boot message: ' + build_openmv_frame(3, 0.4, 8))
        result = backend.read_result()
        self.assertIsNotNone(result)
        self.assertEqual(result.sequence, 3)
        self.assertEqual(result.digit, 8)


if __name__ == '__main__':
    unittest.main(verbosity=2)
