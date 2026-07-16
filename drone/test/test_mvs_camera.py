"""Tests for Hikrobot MVS frame orientation."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mvs_camera import _orient_mvs_rgb_frame


class TestMvsFrameOrientation(unittest.TestCase):
    def test_converts_rgb_to_bgr_and_rotates_180_degrees(self):
        rgb = np.array([
            [[255, 0, 0], [0, 255, 0]],
            [[0, 0, 255], [255, 255, 255]],
        ], dtype=np.uint8)

        actual = _orient_mvs_rgb_frame(rgb)

        expected = np.array([
            [[255, 255, 255], [255, 0, 0]],
            [[0, 255, 0], [0, 0, 255]],
        ], dtype=np.uint8)
        np.testing.assert_array_equal(actual, expected)


if __name__ == '__main__':
    unittest.main(verbosity=2)