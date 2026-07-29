"""Direct OpenCV frame capture and chessboard observation filtering."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
import os
import time
from types import TracebackType
from typing import Literal

import cv2
import numpy as np
from numpy.typing import NDArray

from .calibration_models import (
    CalibrationBootstrapError,
    CapturePolicy,
    CaptureSession,
    Observation,
)

_PREVIEW_WINDOW = "Chessboard calibration"


@dataclass(frozen=True, slots=True)
class _ManagedCapture:
    """Release an OpenCV capture without mutating frozen exceptions during unwind."""

    capture: cv2.VideoCapture

    def __enter__(self) -> cv2.VideoCapture:
        return self.capture

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> Literal[False]:
        self.capture.release()
        return False


def capture_observations(session: CaptureSession) -> tuple[Observation, ...]:
    """Capture sharp, unique views and require spatial and scale diversity."""
    session.selection.validate()
    observations: list[Observation] = []
    if session.direct_v4l2:
        cv2.namedWindow(_PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
    try:
        with _open_capture(session) as capture:
            # --- autofocus settle phase ---
            _autofocus_settle(capture, session)
            # --- main capture loop (unlimited frames unless capped, time-bounded) ---
            start_monotonic = time.monotonic()
            for frame_index in itertools.count():
                # frame-count guard (0 = unlimited)
                if session.policy.maximum_frames > 0 and frame_index >= session.policy.maximum_frames:
                    break
                # time guard (0 = unlimited)
                if session.policy.maximum_seconds > 0:
                    elapsed = time.monotonic() - start_monotonic
                    if elapsed >= session.policy.maximum_seconds:
                        break
                available, frame = capture.read()
                if not available:
                    break
                if frame.shape[1] != session.selection.width or frame.shape[0] != session.selection.height:
                    raise CalibrationBootstrapError(
                        f"capture raster {frame.shape[1]}x{frame.shape[0]} does not match "
                        f"selected {session.selection.width}x{session.selection.height}"
                    )
                observation, blur_variance = _detect_observation(frame_index, frame, session)
                is_duplicate = observation is not None and _is_duplicate(
                    observation, observations, session.policy
                )
                if observation is not None and not is_duplicate:
                    observations.append(observation)
                if session.direct_v4l2:
                    display = frame.copy()
                    if observation is not None:
                        cv2.drawChessboardCorners(
                            display,
                            session.board.inner_corners,
                            observation.corners,
                            True,
                        )
                    cv2.rectangle(display, (0, 0), (520, 78), (0, 0, 0), -1)
                    state = "accepted" if observation is not None and not is_duplicate else "duplicate"
                    if observation is None:
                        state = "blur" if blur_variance < session.policy.minimum_blur_variance else "not found"
                    elapsed_s = time.monotonic() - start_monotonic
                    cv2.putText(
                        display,
                        f"accepted {len(observations)}/{session.policy.target_observations}  {elapsed_s:.0f}s",
                        (16, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        display,
                        f"corners 10x7 | {state}",
                        (16, 58),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow(_PREVIEW_WINDOW, display)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        raise CalibrationBootstrapError("capture cancelled by operator")
                if len(observations) >= session.policy.target_observations:
                    break
    finally:
        if session.direct_v4l2:
            cv2.destroyWindow(_PREVIEW_WINDOW)
    _require_diversity(observations, session.policy)
    return tuple(observations)


def _autofocus_settle(capture: cv2.VideoCapture, session: CaptureSession) -> None:
    """Drain frames while camera autofocus converges; show progress in the preview."""
    settle = session.policy.autofocus_settle_seconds
    if settle <= 0 or not session.direct_v4l2:
        return
    deadline = time.monotonic() + settle
    while time.monotonic() < deadline:
        ok, frame = capture.read()
        if not ok:
            break
        remaining = max(deadline - time.monotonic(), 0.0)
        display = frame.copy()
        cv2.rectangle(display, (0, 0), (480, 48), (0, 0, 0), -1)
        cv2.putText(
            display,
            f"autofocus settling ... {remaining:.0f}s",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(_PREVIEW_WINDOW, display)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            raise CalibrationBootstrapError("capture cancelled by operator")


def _open_capture(session: CaptureSession) -> _ManagedCapture:
    if session.direct_v4l2 and not os.access(session.source, os.R_OK | os.W_OK):
        raise CalibrationBootstrapError(
            f"cannot access capture source {session.source}; run `newgrp video` "
            "or reopen the terminal after joining the video group"
        )
    api = cv2.CAP_V4L2 if session.direct_v4l2 else cv2.CAP_ANY
    capture = cv2.VideoCapture(session.source, api)
    if not capture.isOpened():
        capture.release()
        raise CalibrationBootstrapError(f"cannot open capture source {session.source}")
    if session.direct_v4l2:
        settings = (
            (cv2.CAP_PROP_FOURCC, float(cv2.VideoWriter_fourcc(*"MJPG"))),
            (cv2.CAP_PROP_FRAME_WIDTH, float(session.selection.width)),
            (cv2.CAP_PROP_FRAME_HEIGHT, float(session.selection.height)),
            (cv2.CAP_PROP_FPS, 30.0),
        )
        try:
            for property_id, value in settings:
                if not capture.set(property_id, value):
                    raise CalibrationBootstrapError(
                        f"cannot configure capture source {session.source} for MJPG "
                        f"{session.selection.width}x{session.selection.height} at 30 fps"
                    )
            # Enable camera autofocus so the lens can converge.
            capture.set(cv2.CAP_PROP_AUTOFOCUS, 1.0)
        except (CalibrationBootstrapError, cv2.error):
            capture.release()
            raise
    return _ManagedCapture(capture)


def _detect_observation(
    frame_index: int,
    frame: NDArray[np.uint8],
    session: CaptureSession,
) -> tuple[Observation | None, float]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_variance = _blur_variance(gray)
    if blur_variance < session.policy.minimum_blur_variance:
        return None, blur_variance
    found, corners = cv2.findChessboardCorners(
        gray,
        session.board.inner_corners,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK,
    )
    if not found:
        return None, blur_variance
    refined = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
    ).astype(np.float32)
    flat = refined.reshape(-1, 2)
    width = float(frame.shape[1])
    height = float(frame.shape[0])
    center = (float(flat[:, 0].mean() / width), float(flat[:, 1].mean() / height))
    hull_area = float(cv2.contourArea(cv2.convexHull(flat)))
    return (
        Observation(
            frame_index,
            frame.copy(),
            refined,
            blur_variance,
            center,
            hull_area / (width * height),
        ),
        blur_variance,
    )


def _blur_variance(gray: NDArray[np.uint8]) -> float:
    """Return the existing Laplacian sharpness metric for preview state."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _is_duplicate(
    candidate: Observation, accepted: list[Observation], policy: CapturePolicy
) -> bool:
    diagonal = math.hypot(candidate.frame.shape[1], candidate.frame.shape[0])
    candidate_points = candidate.corners.reshape(-1, 2)
    return any(
        float(np.sqrt(np.mean(np.square(candidate_points - prior.corners.reshape(-1, 2)))))
        / diagonal
        < policy.duplicate_rms_fraction
        for prior in accepted
    )


def _require_diversity(observations: list[Observation], policy: CapturePolicy) -> None:
    if len(observations) < policy.minimum_observations:
        raise CalibrationBootstrapError(
            f"minimum diverse observations is {policy.minimum_observations}; accepted {len(observations)}"
        )
    cells = {(min(int(item.center[0] * 3), 2), min(int(item.center[1] * 3), 2)) for item in observations}
    if len(cells) < policy.minimum_coverage_cells:
        raise CalibrationBootstrapError(
            f"coverage gate needs {policy.minimum_coverage_cells} image cells; observed {len(cells)}"
        )
    areas = tuple(item.area_fraction for item in observations)
    if max(areas) - min(areas) < policy.minimum_area_span:
        raise CalibrationBootstrapError("scale diversity gate failed; vary board distance")
