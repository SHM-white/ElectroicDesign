"""Regression tests for industrial-camera OCR and preview responsiveness."""

from __future__ import annotations

import os
import sys
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import test_vision  # noqa: E402
from vision import BlockDetector, DigitReader  # noqa: E402


SAMPLE_ROOT = Path(__file__).resolve().parents[2]
SAMPLE_21 = SAMPLE_ROOT / "Vision Test - [h] help_screenshot_15.07.2026.png"
SAMPLE_21_2 = SAMPLE_ROOT / "Vision Test - [h] help_screenshot_15.07.2026-2.png"
SAMPLE_28 = SAMPLE_ROOT / "Vision Test - [h] help_screenshot_15.07.2026-3.png"


class TestMvsSampleRecognition(unittest.TestCase):
    def setUp(self) -> None:
        self.detector = BlockDetector()
        self.reader = DigitReader()

    def _read(self, path: Path) -> np.ndarray:
        frame = cv2.imread(str(path))
        self.assertIsNotNone(frame, f"Missing MVS sample: {path}")
        return frame

    def test_recognizes_block_28_from_mvs_sample(self) -> None:
        frame = self._read(SAMPLE_28)

        digit = self.reader.extract_digits(frame, detector=self.detector)

        self.assertEqual(digit, 28, f"Expected block 28, got {digit}")

    def test_recognizes_block_21_from_both_mvs_samples(self) -> None:
        for path in (SAMPLE_21, SAMPLE_21_2):
            with self.subTest(path=path.name):
                frame = self._read(path)

                digit = self.reader.extract_digits(frame, detector=self.detector)

                self.assertEqual(digit, 21, f"Expected block 21, got {digit}")

    def test_finds_a_marker_in_both_block_21_samples(self) -> None:
        for path in (SAMPLE_21, SAMPLE_21_2):
            with self.subTest(path=path.name):
                frame = self._read(path)

                marker = self.reader.find_a_marker(frame)

                self.assertIsNotNone(marker, f"A marker missing in {path.name}")


class _BlockingDigitReader:
    def __init__(self, started: threading.Event, release: threading.Event) -> None:
        self._started = started
        self._release = release

    def extract_digits(
        self,
        _frame: np.ndarray,
        *,
        detector: BlockDetector,
    ) -> int:
        del detector
        self._started.set()
        if not self._release.wait(timeout=5.0):
            raise TimeoutError("test did not release blocking OCR")
        return 21


class _FrameSequence:
    def __init__(self, second_frame_read: threading.Event) -> None:
        self._second_frame_read = second_frame_read
        self._reads = 0
        self.released = False

    def read(self) -> tuple[bool, np.ndarray | None]:
        self._reads += 1
        if self._reads == 2:
            self._second_frame_read.set()
        if self._reads <= 2:
            return True, np.zeros((32, 32, 3), dtype=np.uint8)
        return False, None

    def release(self) -> None:
        self.released = True


class TestPreviewResponsiveness(unittest.TestCase):
    def test_live_preview_reads_next_frame_while_ocr_is_running(self) -> None:
        ocr_started = threading.Event()
        ocr_release = threading.Event()
        second_frame_read = threading.Event()
        capture = _FrameSequence(second_frame_read)
        reader = _BlockingDigitReader(ocr_started, ocr_release)

        def wait_key(_delay_ms: int) -> int:
            return ord("q") if second_frame_read.is_set() else -1

        with (
            patch.object(test_vision, "DigitReader", return_value=reader),
            patch.object(test_vision.cv2, "imshow"),
            patch.object(test_vision.cv2, "waitKey", side_effect=wait_key),
            patch.object(test_vision.cv2, "destroyAllWindows"),
            ThreadPoolExecutor(max_workers=1) as executor,
        ):
            loop = executor.submit(test_vision._recognition_loop, capture, False, False)
            self.assertTrue(ocr_started.wait(timeout=2.0), "OCR never started")
            try:
                read_while_ocr_running = second_frame_read.wait(timeout=0.25)
            finally:
                ocr_release.set()
            loop.result(timeout=2.0)

        self.assertTrue(
            read_while_ocr_running,
            "preview did not read a second frame while OCR was in flight",
        )
        self.assertTrue(capture.released)


if __name__ == "__main__":
    unittest.main(verbosity=2)
