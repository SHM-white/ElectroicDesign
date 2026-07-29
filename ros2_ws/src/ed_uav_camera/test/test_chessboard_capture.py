"""Direct V4L2 chessboard capture configuration tests."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.calibration_models import (
    BoardSpec,
    CalibrationBootstrapError,
    CalibrationSelection,
    CapturePolicy,
    CaptureSession,
    Observation,
)
from ed_uav_camera.chessboard_capture import capture_observations, _open_capture
from ed_uav_camera.model import CameraRole


class FakeCapture:
    """Record mutable OpenCV capture configuration for one test."""

    def __init__(self) -> None:
        self.settings: list[tuple[int, float]] = []

    def isOpened(self) -> bool:
        return True

    def set(self, property_id: int, value: float) -> bool:
        self.settings.append((property_id, value))
        return True

    def release(self) -> None:
        return None


class FrameCapture(FakeCapture):
    """Return a fixed sequence of frames for preview tests."""

    def __init__(self, frames: list[np.ndarray]) -> None:
        super().__init__()
        self.frames = iter(frames)
        self.released = False

    def read(self) -> tuple[bool, np.ndarray]:
        try:
            return True, next(self.frames)
        except StopIteration:
            return False, np.empty((0, 0, 3), dtype=np.uint8)

    def release(self) -> None:
        self.released = True


class RejectingCapture(FrameCapture):
    """Reject the first requested V4L2 setting."""

    def set(self, property_id: int, value: float) -> bool:
        self.settings.append((property_id, value))
        return False


@pytest.fixture(autouse=True)
def accessible_fake_capture_source(monkeypatch) -> None:
    """Treat synthetic capture paths as accessible unless a test denies them."""
    monkeypatch.setattr(os, "access", lambda _path, _mode: True)


def test_direct_v4l2_capture_requests_mjpg_at_30_fps(monkeypatch) -> None:
    # Given: a direct V4L2 calibration session and an observable capture adapter.
    capture = FakeCapture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda _source, _api: capture)
    selection = CalibrationSelection(
        CameraRole.NARROW,
        "usb-revision:0ac8:3460:0122",
        "usb-revision:0ac8:3460:0122",
        "/dev/v4l/by-id/camera-video-index0",
        1280,
        720,
    )
    session = CaptureSession(
        "/dev/v4l/by-id/camera-video-index0",
        selection,
        BoardSpec.parse("10x7", 15.0),
        CapturePolicy(),
        True,
    )

    # When: the direct capture is opened.
    _open_capture(session)

    # Then: compressed 30-fps capture is requested before the target raster.
    assert capture.settings == [
        (cv2.CAP_PROP_FOURCC, float(cv2.VideoWriter_fourcc(*"MJPG"))),
        (cv2.CAP_PROP_FRAME_WIDTH, 1280.0),
        (cv2.CAP_PROP_FRAME_HEIGHT, 720.0),
        (cv2.CAP_PROP_FPS, 30.0),
        (cv2.CAP_PROP_AUTOFOCUS, 1.0),
    ]


def test_direct_v4l2_capture_reports_stale_video_group_permissions(monkeypatch) -> None:
    # Given: a shell that cannot access the selected V4L2 endpoint.
    session = _session(direct_v4l2=True, policy=CapturePolicy())
    monkeypatch.setattr(os, "access", lambda _path, _mode: False)
    monkeypatch.setattr(
        cv2,
        "VideoCapture",
        lambda _source, _api: pytest.fail("permission failure reached OpenCV"),
    )

    # When/Then: preflight identifies the stale group session with an actionable command.
    with pytest.raises(CalibrationBootstrapError, match="newgrp video"):
        _open_capture(session)


def test_direct_v4l2_configuration_failure_releases_capture(monkeypatch) -> None:
    # Given: an opened camera that rejects its requested MJPG configuration.
    capture = RejectingCapture([])
    monkeypatch.setattr(cv2, "VideoCapture", lambda _source, _api: capture)

    # When/Then: configuration fails explicitly and releases the device.
    with pytest.raises(CalibrationBootstrapError, match="cannot configure capture source"):
        _open_capture(_session(direct_v4l2=True, policy=CapturePolicy()))
    assert capture.released


def test_rejected_frame_computes_blur_metric_once(monkeypatch) -> None:
    # Given: one sharp frame with no detectable board.
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    capture = FrameCapture([frame])
    blur_calls = 0

    def count_blur(_gray: np.ndarray) -> float:
        nonlocal blur_calls
        blur_calls += 1
        return 100.0

    monkeypatch.setattr(cv2, "VideoCapture", lambda _source, _api: capture)
    monkeypatch.setattr("ed_uav_camera.chessboard_capture._blur_variance", count_blur)

    # When: the bounded fixture capture rejects the frame for missing corners.
    with pytest.raises(CalibrationBootstrapError, match="accepted 0"):
        capture_observations(
            _session(
                direct_v4l2=False,
                policy=CapturePolicy(minimum_observations=1, maximum_frames=1),
            )
        )

    # Then: detection and status rendering share one blur computation.
    assert blur_calls == 1


def _session(*, direct_v4l2: bool, policy: CapturePolicy) -> CaptureSession:
    selection = CalibrationSelection(
        CameraRole.NARROW,
        "usb-revision:0ac8:3460:0122",
        "usb-revision:0ac8:3460:0122",
        "/dev/v4l/by-id/camera-video-index0",
        1280,
        720,
    )
    return CaptureSession(
        "/dev/v4l/by-id/camera-video-index0",
        selection,
        BoardSpec.parse("10x7", 15.0),
        policy,
        direct_v4l2,
    )


def test_direct_v4l2_preview_draws_corners_and_closes(monkeypatch) -> None:
    # Given: one accepted observation and a headless GUI seam.
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    capture = FrameCapture([frame])
    corners = np.zeros((70, 1, 2), dtype=np.float32)
    observation = Observation(0, frame, corners, 100.0, (0.5, 0.5), 0.1)
    gui_calls: list[str] = []
    monkeypatch.setattr(cv2, "VideoCapture", lambda _source, _api: capture)
    monkeypatch.setattr(
        "ed_uav_camera.chessboard_capture._detect_observation",
        lambda _index, _frame, _session: (observation, observation.blur_variance),
    )
    monkeypatch.setattr(cv2, "namedWindow", lambda name, flags: gui_calls.append(f"named:{name}:{flags}"))
    monkeypatch.setattr(cv2, "imshow", lambda name, _frame: gui_calls.append(f"show:{name}"))
    monkeypatch.setattr(cv2, "waitKey", lambda _delay: -1)
    monkeypatch.setattr(cv2, "destroyWindow", lambda name: gui_calls.append(f"destroy:{name}"))
    monkeypatch.setattr(
        cv2,
        "drawChessboardCorners",
        lambda _frame, _pattern, _corners, _found: gui_calls.append("corners"),
    )

    # When: direct V4L2 capture completes through the preview loop.
    result = capture_observations(
        _session(
            direct_v4l2=True,
            policy=CapturePolicy(
                minimum_observations=1,
                target_observations=1,
                minimum_coverage_cells=1,
                minimum_area_span=0.0,
                autofocus_settle_seconds=0,
            ),
        )
    )

    # Then: the live frame is shown with corners and the window is closed.
    assert result == (observation,)
    assert "corners" in gui_calls
    assert any(call.startswith("named:") for call in gui_calls)
    assert any(call.startswith("show:") for call in gui_calls)
    assert any(call.startswith("destroy:") for call in gui_calls)
    assert capture.released


def test_direct_v4l2_preview_q_cancels_and_closes(monkeypatch) -> None:
    # Given: a direct capture whose operator presses q in the preview.
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    capture = FrameCapture([frame])
    destroyed: list[str] = []
    monkeypatch.setattr(cv2, "VideoCapture", lambda _source, _api: capture)
    monkeypatch.setattr(
        "ed_uav_camera.chessboard_capture._detect_observation",
        lambda _index, _frame, _session: (None, 100.0),
    )
    monkeypatch.setattr(cv2, "namedWindow", lambda _name, _flags: None)
    monkeypatch.setattr(cv2, "imshow", lambda _name, _frame: None)
    monkeypatch.setattr(cv2, "waitKey", lambda _delay: ord("q"))
    monkeypatch.setattr(cv2, "destroyWindow", lambda name: destroyed.append(name))

    # When: the operator cancels capture.
    with pytest.raises(CalibrationBootstrapError, match="capture cancelled"):
        capture_observations(
            _session(
                direct_v4l2=True,
                policy=CapturePolicy(minimum_observations=1),
            )
        )

    # Then: cancellation releases both resources and preview state.
    assert capture.released
    assert destroyed


def test_recorded_fixture_capture_remains_headless(monkeypatch) -> None:
    # Given: a recorded-video style session and GUI calls that must not run.
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    capture = FrameCapture([frame])
    corners = np.zeros((70, 1, 2), dtype=np.float32)
    observation = Observation(0, frame, corners, 100.0, (0.5, 0.5), 0.1)
    monkeypatch.setattr(cv2, "VideoCapture", lambda _source, _api: capture)
    monkeypatch.setattr(
        "ed_uav_camera.chessboard_capture._detect_observation",
        lambda _index, _frame, _session: (observation, observation.blur_variance),
    )
    for name in ("namedWindow", "imshow", "waitKey", "destroyWindow"):
        monkeypatch.setattr(cv2, name, lambda *_args: pytest.fail(f"fixture called cv2.{name}"))

    # When: the recorded fixture capture completes.
    result = capture_observations(
        _session(
            direct_v4l2=False,
            policy=CapturePolicy(
                minimum_observations=1,
                target_observations=1,
                minimum_coverage_cells=1,
                minimum_area_span=0.0,
            ),
        )
    )

    # Then: capture succeeds without touching the GUI.
    assert result == (observation,)
