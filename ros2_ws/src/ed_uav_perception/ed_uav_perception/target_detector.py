"""Extract versioned ring and cross correspondences from calibrated images."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

import cv2
import numpy as np

from ed_uav_perception.target_types import CorrespondenceSet, RejectReason

TARGET_REVISION: Final = "d2026-circle-cross-v1"
OUTER_RADIUS_M: Final = 0.25
INNER_RADIUS_M: Final = 0.15
EXPECTED_RADIUS_RATIO: Final = INNER_RADIUS_M / OUTER_RADIUS_M


@dataclass(frozen=True, slots=True)
class DetectionFailure:
    reason: RejectReason


def _radial_samples(gray: np.ndarray, center: tuple[float, float], radius: int) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    radii = np.arange(1, radius + 1, dtype=np.float64)
    xs = center[0] + radii[:, None] * np.cos(angles)[None, :]
    ys = center[1] + radii[:, None] * np.sin(angles)[None, :]
    return cv2.remap(
        gray,
        xs.astype(np.float32),
        ys.astype(np.float32),
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=255,
    )


def _ring_centers(profile: np.ndarray) -> list[float]:
    active = profile > max(60.0, float(profile.max()) * 0.45)
    groups: list[np.ndarray] = []
    start = 0
    for index in range(1, active.size + 1):
        if index < active.size and active[index] == active[start]:
            continue
        if active[start] and index - start >= 2:
            groups.append(np.arange(start, index))
        start = index
    return [float(np.average(group + 1, weights=profile[group])) for group in groups]


def _cross_phase(samples: np.ndarray, rings: tuple[float, float]) -> float | None:
    radial_mask = np.ones(samples.shape[0], dtype=bool)
    for radius in rings:
        radial_mask &= np.abs(np.arange(1, samples.shape[0] + 1) - radius) > 9.0
    radial_mask[: max(3, int(samples.shape[0] * 0.12))] = False
    angular = (255.0 - samples[radial_mask]).mean(axis=0)
    if float(angular.max()) < 35.0:
        return None
    index = int(np.argmax(angular))
    return index * 2.0 * np.pi / angular.size


def _points(
    center: tuple[float, float], radii_px: tuple[float, float], phase: float
) -> CorrespondenceSet:
    angles = phase + np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
    object_points = np.column_stack(
        (
            np.concatenate(
                (
                    OUTER_RADIUS_M * np.cos(angles - phase),
                    INNER_RADIUS_M * np.cos(angles - phase),
                )
            ),
            np.concatenate(
                (
                    OUTER_RADIUS_M * np.sin(angles - phase),
                    INNER_RADIUS_M * np.sin(angles - phase),
                )
            ),
            np.zeros(16),
        )
    )
    image_points = np.column_stack(
        (
            np.concatenate(
                (
                    center[0] + radii_px[0] * np.cos(angles),
                    center[0] + radii_px[1] * np.cos(angles),
                )
            ),
            np.concatenate(
                (
                    center[1] + radii_px[0] * np.sin(angles),
                    center[1] + radii_px[1] * np.sin(angles),
                )
            ),
        )
    )
    return CorrespondenceSet(object_points.astype(np.float64), image_points.astype(np.float64), 4)


def detect_target(image: np.ndarray, revision: str) -> CorrespondenceSet | DetectionFailure:
    """Detect the complete prescribed geometry or return a typed rejection."""
    if revision != TARGET_REVISION:
        return DetectionFailure(RejectReason.WRONG_REVISION)
    if image.ndim not in (2, 3) or min(image.shape[:2]) < 64:
        return DetectionFailure(RejectReason.INVALID_INPUT)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[component, cv2.CC_STAT_AREA])
    if area < 300:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    center = (float(centroids[component, 0]), float(centroids[component, 1]))
    radius_limit = int(
        min(center[0], center[1], gray.shape[1] - center[0] - 1, gray.shape[0] - center[1] - 1)
    )
    if radius_limit < 20:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    samples = _radial_samples(gray, center, radius_limit)
    darkness = (255.0 - samples).mean(axis=1)
    centers = _ring_centers(darkness)
    if len(centers) < 2:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    outer = centers[-1]
    inner = min(centers[:-1], key=lambda value: abs(value / outer - EXPECTED_RADIUS_RATIO))
    ratio = inner / outer
    if ratio < 0.30 or ratio > 0.80:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    if abs(ratio - EXPECTED_RADIUS_RATIO) > 0.09:
        return DetectionFailure(RejectReason.WRONG_REVISION)
    phase = _cross_phase(samples, (inner, outer))
    if phase is None:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    return _points(center, (outer, inner), phase)
