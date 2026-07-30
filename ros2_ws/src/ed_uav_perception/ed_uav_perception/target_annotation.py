"""Render target-observation status and optical-frame pose annotations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final
from typing_extensions import assert_never

import cv2
import numpy as np

from ed_uav_perception.target_types import (
    AcceptedObservation,
    CameraModel,
    ObservationResult,
    RejectedObservation,
)

_FONT: Final = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE: Final = 0.45
_TEXT_THICKNESS: Final = 1
_LINE_HEIGHT: Final = 18
_PANEL_PADDING: Final = 8
_PANEL_COLOR: Final = (15, 15, 15)
_ACCEPTED_COLOR: Final = (60, 220, 60)
_REJECTED_COLOR: Final = (60, 60, 230)


@dataclass(frozen=True, slots=True)
class AnnotationFrame:
    """Decoded image and optional calibrated model used for frame-relative drawing."""

    image: np.ndarray
    camera: CameraModel | None = None


def _ascii_text(value: str) -> str:
    return value.encode("ascii", "replace").decode("ascii")


def annotation_lines(observation: ObservationResult) -> tuple[str, ...]:
    """Return the ASCII lines that describe one typed observation."""
    match observation:
        case AcceptedObservation(frame_id=frame_id, pose=pose):
            translation = pose.translation_m
            return (
                f"frame_id: {_ascii_text(frame_id)}",
                "optical: x right y down z forward",
                (
                    f"X: {translation[0]:+.3f} m Y: {translation[1]:+.3f} m "
                    f"Z: {translation[2]:+.3f} m"
                ),
                f"quality: {pose.quality:.3f}",
                f"reprojection RMS: {pose.reprojection_rms_px:.3f} px",
            )
        case RejectedObservation(frame_id=frame_id, reject_reason=reason):
            return (
                f"frame_id: {_ascii_text(frame_id)}",
                "status: REJECTED",
                f"reason: {reason.value}",
                "optical: x right y down z forward",
            )
        case unreachable:
            assert_never(unreachable)


def _target_pixel(
    observation: AcceptedObservation, camera: CameraModel | None
) -> tuple[int, int] | None:
    if camera is None:
        return None
    translation = observation.pose.translation_m
    if not np.all(np.isfinite(translation)) or translation[2] <= 0.0:
        return None
    matrix = camera.matrix
    x = float(matrix[0, 0] * translation[0] / translation[2] + matrix[0, 2])
    y = float(matrix[1, 1] * translation[1] / translation[2] + matrix[1, 2])
    if not np.isfinite(x) or not np.isfinite(y):
        return None
    return round(x), round(y)


def _draw_target_marker(
    image: np.ndarray,
    observation: AcceptedObservation,
    camera: CameraModel | None,
) -> None:
    target = _target_pixel(observation, camera)
    if target is None:
        return
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    bounded_target = (
        max(0, min(width - 1, target[0])),
        max(0, min(height - 1, target[1])),
    )
    cv2.arrowedLine(image, center, bounded_target, _ACCEPTED_COLOR, 2, cv2.LINE_AA)
    cv2.circle(image, bounded_target, 7, _ACCEPTED_COLOR, 2, cv2.LINE_AA)


def render_target_observation(
    frame: AnnotationFrame, observation: ObservationResult
) -> np.ndarray:
    """Return an annotated copy of one decoded BGR frame."""
    annotated = frame.image.copy()
    height = annotated.shape[0]
    panel_height = min(
        height, _PANEL_PADDING * 2 + _LINE_HEIGHT * len(annotation_lines(observation))
    )
    cv2.rectangle(
        annotated,
        (0, 0),
        (annotated.shape[1] - 1, panel_height - 1),
        _PANEL_COLOR,
        cv2.FILLED,
    )
    match observation:
        case AcceptedObservation():
            color = _ACCEPTED_COLOR
            _draw_target_marker(annotated, observation, frame.camera)
        case RejectedObservation():
            color = _REJECTED_COLOR
        case unreachable:
            assert_never(unreachable)
    for index, line in enumerate(annotation_lines(observation)):
        baseline = _PANEL_PADDING + (index + 1) * _LINE_HEIGHT
        cv2.putText(
            annotated,
            line,
            (_PANEL_PADDING, baseline),
            _FONT,
            _FONT_SCALE,
            color,
            _TEXT_THICKNESS,
            cv2.LINE_AA,
        )
    return annotated
