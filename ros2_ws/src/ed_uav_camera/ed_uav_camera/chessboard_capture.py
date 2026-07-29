"""Direct OpenCV frame capture and chessboard observation filtering."""

from __future__ import annotations

from dataclasses import dataclass
import math
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
    with _open_capture(session) as capture:
        for frame_index in range(session.policy.maximum_frames):
            available, frame = capture.read()
            if not available:
                break
            if frame.shape[1] != session.selection.width or frame.shape[0] != session.selection.height:
                raise CalibrationBootstrapError(
                    f"capture raster {frame.shape[1]}x{frame.shape[0]} does not match "
                    f"selected {session.selection.width}x{session.selection.height}"
                )
            observation = _detect_observation(frame_index, frame, session)
            if observation is not None and not _is_duplicate(observation, observations, session.policy):
                observations.append(observation)
            if len(observations) >= session.policy.target_observations:
                break
    _require_diversity(observations, session.policy)
    return tuple(observations)


def _open_capture(session: CaptureSession) -> _ManagedCapture:
    api = cv2.CAP_V4L2 if session.direct_v4l2 else cv2.CAP_ANY
    capture = cv2.VideoCapture(session.source, api)
    if not capture.isOpened():
        capture.release()
        raise CalibrationBootstrapError(f"cannot open capture source {session.source}")
    if session.direct_v4l2:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, session.selection.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, session.selection.height)
    return _ManagedCapture(capture)


def _detect_observation(
    frame_index: int,
    frame: NDArray[np.uint8],
    session: CaptureSession,
) -> Observation | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if blur_variance < session.policy.minimum_blur_variance:
        return None
    found, corners = cv2.findChessboardCorners(
        gray,
        session.board.inner_corners,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK,
    )
    if not found:
        return None
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
    return Observation(frame_index, frame.copy(), refined, blur_variance, center, hull_area / (width * height))


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
