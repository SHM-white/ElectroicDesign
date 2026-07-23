"""灰色数字中心检测回归测试。"""

from pathlib import Path
import sys
import unittest

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gray_marker import GrayMarkerDetector


SAMPLE_ROOT = Path(__file__).resolve().parents[2]


class TestGrayMarkerDetector(unittest.TestCase):
    def setUp(self):
        self.detector = GrayMarkerDetector()

    @pytest.mark.field_data
    def test_detects_bright_saved_sample_center(self):
        image = cv2.imread(str(
            SAMPLE_ROOT / 'Vision Test - [h] help_screenshot_15.07.2026.png'
        ))
        self.assertIsNotNone(image)
        marker = self.detector.detect(image).best_marker
        self.assertIsNotNone(marker)
        self.assertAlmostEqual(marker.center[0], 917.0, delta=25.0)
        self.assertAlmostEqual(marker.center[1], 761.5, delta=25.0)

    @pytest.mark.field_data
    def test_detects_dark_saved_sample_center(self):
        image = cv2.imread(str(SAMPLE_ROOT / 'mission_vision_343598988361.png'))
        self.assertIsNotNone(image)
        result = self.detector.detect(image)
        self.assertTrue(result.dark_scene)
        marker = result.best_marker
        self.assertIsNotNone(marker)
        self.assertAlmostEqual(marker.center[0], 753.0, delta=20.0)
        self.assertAlmostEqual(marker.center[1], 525.0, delta=20.0)

    def test_detects_synthetic_two_character_center(self):
        image = np.full((480, 640, 3), (100, 220, 100), dtype=np.uint8)
        cv2.putText(
            image, '21', (235, 300), cv2.FONT_HERSHEY_DUPLEX,
            3.5, (200, 200, 200), 8, cv2.LINE_AA,
        )
        marker = self.detector.detect(image).best_marker
        self.assertIsNotNone(marker)
        self.assertGreaterEqual(len(marker.characters), 1)
        self.assertLess(abs(marker.center[0] - 320), 80)
        self.assertLess(abs(marker.center[1] - 240), 100)

    def test_empty_image_is_rejected(self):
        with self.assertRaises(ValueError):
            self.detector.detect(np.empty((0, 0, 3), dtype=np.uint8))


if __name__ == '__main__':
    unittest.main()
