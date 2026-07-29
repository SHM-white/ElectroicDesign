"""Chessboard intrinsic solve, holdout evaluation, overlays, and ROS artifacts."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray

from .calibration_models import (
    BoardSpec,
    CalibrationBootstrapError,
    CalibrationSelection,
    CalibrationSolution,
    Observation,
    ReprojectionMetrics,
)


@dataclass(frozen=True, slots=True)
class _Projection:
    object_points: NDArray[np.float32]
    image_points: NDArray[np.float32]
    camera_matrix: NDArray[np.float64]
    distortion: NDArray[np.float64]
    rotation: NDArray[np.float64]
    translation: NDArray[np.float64]


def solve_calibration(
    board: BoardSpec,
    selection: CalibrationSelection,
    observations: tuple[Observation, ...],
) -> CalibrationSolution:
    """Fit on a deterministic 80% split and gate an independent 20% holdout."""
    holdout = tuple(item for index, item in enumerate(observations) if (index + 1) % 5 == 0)
    train = tuple(item for index, item in enumerate(observations) if (index + 1) % 5 != 0)
    if len(holdout) < 3 or len(train) < 8:
        raise CalibrationBootstrapError("deterministic split requires at least 8 train and 3 holdout views")
    object_points = board.object_points()
    _, camera_matrix, distortion, rotation_vectors, translation_vectors = cv2.calibrateCamera(
        [object_points for _ in train],
        [item.corners for item in train],
        (selection.width, selection.height),
        None,
        None,
    )
    train_errors = tuple(
        _view_error(
            _Projection(
                object_points,
                item.corners,
                camera_matrix,
                distortion,
                rotation,
                translation,
            )
        )
        for item, rotation, translation in zip(train, rotation_vectors, translation_vectors, strict=True)
    )
    holdout_errors = tuple(
        _holdout_error(object_points, item.corners, camera_matrix, distortion) for item in holdout
    )
    metrics = ReprojectionMetrics(
        float(np.mean(train_errors)),
        float(np.mean(holdout_errors)),
        float(max(holdout_errors)),
    )
    if metrics.holdout_mean_px > 0.5 or metrics.holdout_max_px > 1.0:
        raise CalibrationBootstrapError(
            f"holdout reprojection gate failed: mean {metrics.holdout_mean_px:.4f} px, "
            f"max {metrics.holdout_max_px:.4f} px"
        )
    return CalibrationSolution(camera_matrix, distortion, train, holdout, metrics)


def _view_error(projection: _Projection) -> float:
    projected, _ = cv2.projectPoints(
        projection.object_points,
        projection.rotation,
        projection.translation,
        projection.camera_matrix,
        projection.distortion,
    )
    differences = projection.image_points.reshape(-1, 2) - projected.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(np.square(differences), axis=1))))


def _holdout_error(
    object_points: NDArray[np.float32],
    image_points: NDArray[np.float32],
    camera_matrix: NDArray[np.float64],
    distortion: NDArray[np.float64],
) -> float:
    solved, rotation, translation = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not solved:
        raise CalibrationBootstrapError("holdout pose solve failed")
    return _view_error(
        _Projection(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            rotation,
            translation,
        )
    )

