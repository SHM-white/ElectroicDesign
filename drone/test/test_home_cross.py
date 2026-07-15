"""起降十字检测与视觉结果回归测试。"""

import os
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vision import HomeCrossDetector


SAMPLE_ROOT = Path(__file__).resolve().parents[2]
HOME_SAMPLE_NAMES = (
    'mission_vision_402151711011.png',
    'mission_vision_402741364456.png',
    'mission_vision_402853639001.png',
    'mission_vision_402964927359.png',
    'mission_vision_403072737295.png',
    'mission_vision_403712551477.png',
    'mission_vision_403827566089.png',
    'mission_vision_403925567301.png',
    'mission_vision_404026620771.png',
    'mission_vision_404645304801.png',
    'mission_vision_404775083782.png',
    'mission_vision_404865113743.png',
    'mission_vision_404963121431.png',
    'mission_vision_414888071266.png',
    'mission_vision_415038856974.png',
    'mission_vision_415124878070.png',
    'mission_vision_415212646144.png',
    'mission_vision_415312878597.png',
    'mission_vision_415416002860.png',
    'mission_vision_416010570452.png',
    'mission_vision_416123816725.png',
    'mission_vision_416235453123.png',
)


class TestHomeCrossDetector(unittest.TestCase):
    def setUp(self):
        self.detector = HomeCrossDetector(min_confidence=0.45)

    def test_detects_black_cross_center(self):
        image = np.full((480, 640, 3), 235, dtype=np.uint8)
        cv2.rectangle(image, (300, 130), (340, 350), (0, 0, 0), -1)
        cv2.rectangle(image, (210, 220), (430, 260), (0, 0, 0), -1)

        center, confidence = self.detector.detect(image)

        self.assertIsNotNone(center)
        self.assertAlmostEqual(center[0], 320, delta=3)
        self.assertAlmostEqual(center[1], 240, delta=3)
        self.assertGreaterEqual(confidence, 0.45)

    def test_rejects_single_black_line(self):
        image = np.full((480, 640, 3), 235, dtype=np.uint8)
        cv2.rectangle(image, (300, 100), (340, 380), (0, 0, 0), -1)

        center, _ = self.detector.detect(image)

        self.assertIsNone(center)

    def test_rejects_l_shape(self):
        image = np.full((480, 640, 3), 235, dtype=np.uint8)
        cv2.rectangle(image, (220, 120), (250, 350), (0, 0, 0), -1)
        cv2.rectangle(image, (220, 320), (430, 350), (0, 0, 0), -1)

        center, _ = self.detector.detect(image)

        self.assertIsNone(center)

    def test_detects_complete_saved_home_sample(self):
        image = cv2.imread(str(SAMPLE_ROOT / 'mission_vision_415312878597.png'))
        self.assertIsNotNone(image)

        center, confidence = HomeCrossDetector().detect(image)

        self.assertIsNotNone(center)
        self.assertGreaterEqual(confidence, 0.58)

    def test_detects_saved_home_sequence(self):
        detector = HomeCrossDetector()
        detected = 0
        for name in HOME_SAMPLE_NAMES:
            with self.subTest(name=name):
                image = cv2.imread(str(SAMPLE_ROOT / name))
                self.assertIsNotNone(image)
                center, confidence = detector.detect(image)
                if center is not None and confidence >= detector.min_confidence:
                    detected += 1

        # 部分照片裁切不完整；要求序列多数帧可用于时序确认。
        self.assertGreaterEqual(detected, 16)


if __name__ == '__main__':
    unittest.main(verbosity=2)
