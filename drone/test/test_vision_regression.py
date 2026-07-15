"""Regression tests for industrial-camera OCR and preview responsiveness."""

from __future__ import annotations

import os
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import test_vision  # noqa: E402
from vision import (  # noqa: E402
    BlockDetector, Camera, DigitReader, draw_mission_overlay,
)


SAMPLE_ROOT = Path(__file__).resolve().parents[2]
SAMPLE_21 = SAMPLE_ROOT / "Vision Test - [h] help_screenshot_15.07.2026.png"
SAMPLE_21_2 = SAMPLE_ROOT / "Vision Test - [h] help_screenshot_15.07.2026-2.png"
SAMPLE_28 = SAMPLE_ROOT / "Vision Test - [h] help_screenshot_15.07.2026-3.png"
PHONE_SAMPLE_21 = SAMPLE_ROOT / "IMG_20260715_175823.jpg"
LOW_LIGHT_START_SAMPLE = SAMPLE_ROOT / "mission_vision_156657515933.png"
LOW_LIGHT_FALSE_DIGIT_SAMPLE = SAMPLE_ROOT / "mission_vision_194047772428.png"
LOW_LIGHT_DIGIT_SAMPLES = {
    19: SAMPLE_ROOT / "mission_vision_216786120831.png",
    20: SAMPLE_ROOT / "mission_vision_294206335845.png",
    22: SAMPLE_ROOT / "mission_vision_295544256805.png",
    25: SAMPLE_ROOT / "mission_vision_343598988361.png",
}


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

    def test_recognizes_phone_sample_block_21_and_a_marker(self) -> None:
        frame = self._read(PHONE_SAMPLE_21)

        self.assertEqual(
            self.reader.extract_digits(frame, detector=self.detector), 21,
        )
        self.assertIsNotNone(self.reader.find_a_marker(frame))

    def test_finds_a_marker_in_low_light_start_sample(self) -> None:
        frame = self._read(LOW_LIGHT_START_SAMPLE)

        self.assertIsNotNone(self.reader.find_a_marker(frame))

    def test_expected_digit_rejects_low_light_noise_ocr(self) -> None:
        frame = self._read(LOW_LIGHT_FALSE_DIGIT_SAMPLE)

        self.assertIsNone(
            self.reader.extract_digits(
                frame, detector=self.detector, expected_digit=21,
            )
        )

    def test_recognizes_saved_low_light_two_digit_samples(self) -> None:
        for expected, path in LOW_LIGHT_DIGIT_SAMPLES.items():
            with self.subTest(path=path.name):
                frame = self._read(path)
                self.assertEqual(
                    self.reader.extract_digits(
                        frame, detector=self.detector,
                        expected_digit=expected,
                    ),
                    expected,
                )

    def test_recognizes_brightness_reduced_mvs_samples(self) -> None:
        for path, expected in ((SAMPLE_21, 21), (SAMPLE_28, 28)):
            with self.subTest(path=path.name):
                frame = self._read(path)
                dark = np.clip(
                    frame.astype(np.float32) * 0.25, 0, 255,
                ).astype(np.uint8)
                self.assertEqual(
                    self.reader.extract_digits(
                        dark, detector=self.detector,
                        expected_digit=expected,
                    ),
                    expected,
                )

    def test_component_pipeline_supports_single_digit_blocks(self) -> None:
        mask = np.zeros((200, 200), dtype=np.uint8)
        cv2.rectangle(mask, (82, 70), (112, 120), 255, thickness=-1)

        with patch.object(self.reader, '_ocr_single', return_value=7) as ocr:
            digit = self.reader._ocr_component_mask(
                mask,
                '-c tessedit_char_whitelist=0123456789',
                expected_digit=7,
            )

        self.assertEqual(digit, 7)
        self.assertIn('--psm 10', ocr.call_args.args[1])

    def test_preprocessing_includes_local_light_candidate(self) -> None:
        height, width = 120, 180
        gradient = np.tile(
            np.linspace(25, 180, width, dtype=np.uint8), (height, 1),
        )
        frame = cv2.cvtColor(gradient, cv2.COLOR_GRAY2BGR)

        from vision import _preprocess_ocr
        candidates = _preprocess_ocr(gradient, frame)

        self.assertGreaterEqual(len(candidates), 6)
        self.assertTrue(all(image.shape == gradient.shape for image in candidates))
        self.assertTrue(
            set(np.unique(candidates[-2])).issubset({0, 255}),
        )

    def test_all_saved_field_frames_are_readable(self) -> None:
        samples = sorted(SAMPLE_ROOT.glob("mission_vision_*.png"))
        self.assertGreater(len(samples), 0, "No saved field frames found")
        for path in samples:
            with self.subTest(path=path.name):
                frame = cv2.imread(str(path))
                self.assertIsNotNone(frame)
                self.assertEqual(frame.ndim, 3)
                self.assertEqual(frame.shape[2], 3)


class TestImageDirectoryReplay(unittest.TestCase):
    def test_discovers_supported_images_in_name_order(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('b.jpg', 'a.png', 'ignored.txt'):
                (root / name).touch()

            paths = test_vision.discover_images(directory)

        self.assertEqual([path.name for path in paths], ['a.png', 'b.jpg'])

    def test_replay_processes_each_image_and_reports_damage(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cv2.imwrite(str(root / '01.png'), np.zeros((64, 96, 3), np.uint8))
            cv2.imwrite(str(root / '02.jpg'), np.zeros((64, 96, 3), np.uint8))
            (root / '03.png').write_bytes(b'not a png')
            with (
                patch.object(test_vision.DigitReader, 'extract_digits', return_value=21),
                patch.object(test_vision.DigitReader, 'find_a_marker', return_value=None),
                patch.object(test_vision.cv2, 'imshow') as imshow,
                patch.object(test_vision.cv2, 'waitKey', return_value=-1),
                patch.object(test_vision.cv2, 'destroyAllWindows'),
            ):
                summary = test_vision.run_image_directory(directory, 0.001)

        self.assertEqual(summary['total'], 3)
        self.assertEqual(summary['readable'], 2)
        self.assertEqual(summary['recognized'], 2)
        self.assertEqual(summary['unreadable'], 1)
        self.assertGreaterEqual(imshow.call_count, 2)


class _BlockingDigitReader:
    def __init__(self, started: threading.Event, release: threading.Event) -> None:
        self._started = started
        self._release = release

    def extract_digits(
        self,
        _frame: np.ndarray,
        *,
        detector: BlockDetector,
        expected_digit: int | None = None,
    ) -> int:
        del detector, expected_digit
        self._started.set()
        if not self._release.wait(timeout=5.0):
            raise TimeoutError("test did not release blocking OCR")
        return 21

    def find_a_marker(self, _frame: np.ndarray):
        return None


class _ImmediateDigitReader:
    def extract_digits(self, _frame: np.ndarray, **_kwargs) -> int:
        return 21

    def find_a_marker(self, _frame: np.ndarray):
        return None


class _ConstantFrame:
    def read(self) -> tuple[bool, np.ndarray]:
        return True, np.zeros((32, 32, 3), dtype=np.uint8)

    def release(self) -> None:
        pass


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
    def test_mission_overlay_draws_recognition_ui(self) -> None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        display = draw_mission_overlay(
            frame,
            state_label='FIND_START',
            green_ratio=0.42,
            green_blocks=[(320, 260, 240, 180, 40000.0)],
            digit=21,
            ocr_enabled=True,
            start_marker=(300, 240),
            home_cross=(360.0, 280.0),
            home_confidence=0.91,
        )

        self.assertEqual(display.shape, frame.shape)
        self.assertFalse(np.array_equal(display, frame))
        self.assertTrue(np.array_equal(frame, np.zeros_like(frame)))
        self.assertGreater(np.count_nonzero(display), 1000)

    def test_processing_modes_store_state_and_clear_old_start_marker(self) -> None:
        camera = Camera(preview=False)
        camera._last_start_marker = (100, 100)

        camera.set_processing_modes(
            ocr=True, start_marker=False, home_cross=False,
            expected_digit=20,
            state_label='NAVIGATE',
        )

        self.assertEqual(camera._state_label, 'NAVIGATE')
        self.assertEqual(camera._expected_digit, 20)
        self.assertIsNone(camera._last_start_marker)
        camera.release()

    def test_production_camera_does_not_wait_for_ocr(self) -> None:
        ocr_started = threading.Event()
        ocr_release = threading.Event()
        second_frame_read = threading.Event()
        capture = _FrameSequence(second_frame_read)
        reader = _BlockingDigitReader(ocr_started, ocr_release)
        camera = Camera(preview=True, ocr_interval_s=0.0)
        camera.cap = capture
        camera.detector = BlockDetector()
        camera.digit_reader = reader
        camera.set_processing_modes(ocr=True, home_cross=False)

        with (
            patch('vision.cv2.imshow'),
            patch('vision.cv2.waitKey', return_value=-1),
            patch('vision.cv2.destroyAllWindows'),
        ):
            camera.read_result()
            self.assertTrue(ocr_started.wait(timeout=1.0), "OCR never started")
            started = time.monotonic()
            try:
                camera.read_result()
                elapsed = time.monotonic() - started
            finally:
                ocr_release.set()
                camera.release()

        self.assertTrue(second_frame_read.is_set())
        self.assertLess(elapsed, 0.25, "production preview waited for OCR")

    def test_camera_requires_two_matching_ocr_observations(self) -> None:
        camera = Camera(preview=False, ocr_interval_s=0.0)
        camera.cap = _ConstantFrame()
        camera.detector = BlockDetector()
        camera.digit_reader = _ImmediateDigitReader()
        camera.set_processing_modes(
            ocr=True, home_cross=False, expected_digit=21,
        )
        try:
            first = camera.read_result()
            self.assertIsNone(first.digit)
            camera._ocr_future.result(timeout=1.0)
            second = camera.read_result()
            self.assertIsNone(second.digit)
            camera._ocr_future.result(timeout=1.0)
            third = camera.read_result()
            self.assertEqual(third.digit, 21)
        finally:
            camera.release()

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
