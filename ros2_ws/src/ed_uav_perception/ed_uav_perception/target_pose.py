"""Robust planar IPPE candidate solving and motion-prior selection."""

from __future__ import annotations

import math

import cv2
import numpy as np

from ed_uav_perception.target_types import (
    CameraModel,
    CandidateVectors,
    CorrespondenceSet,
    MotionContext,
    PoseCandidate,
    PoseEstimate,
    PoseLimits,
    PoseRejection,
    RejectReason,
)


def candidate_from_vectors(vectors: CandidateVectors) -> PoseCandidate | RejectReason:
    """Build a candidate only when every target point has positive depth."""
    rotation, _ = cv2.Rodrigues(vectors.rotation_vector)
    camera_points = (rotation @ vectors.object_points.T + vectors.translation_m.reshape(3, 1)).T
    if np.any(camera_points[:, 2] <= 1e-6):
        return RejectReason.NEGATIVE_DEPTH
    return PoseCandidate(
        vectors.rotation_vector.reshape(3).copy(),
        vectors.translation_m.reshape(3).copy(),
        vectors.reprojection_rms_px,
        vectors.object_points.shape[0],
    )


def _rms(
    points: CorrespondenceSet,
    camera: CameraModel,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(
        points.object_points, rvec, tvec, camera.matrix, camera.distortion
    )
    errors = projected.reshape(-1, 2) - points.image_points
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def _rotate_correspondences(points: CorrespondenceSet, turn: int) -> CorrespondenceSet:
    if turn == 0:
        return points
    per_ring = points.object_points.shape[0] // 2
    shift = turn * per_ring // points.symmetry_order
    images = np.concatenate(
        (
            np.roll(points.image_points[:per_ring], -shift, axis=0),
            np.roll(points.image_points[per_ring:], -shift, axis=0),
        )
    )
    return CorrespondenceSet(points.object_points, images, 1)


def _solve_variant(
    points: CorrespondenceSet, camera: CameraModel, limits: PoseLimits
) -> tuple[list[PoseCandidate], int]:
    success, _, _, inliers = cv2.solvePnPRansac(
        points.object_points,
        points.image_points,
        camera.matrix,
        camera.distortion,
        iterationsCount=150,
        reprojectionError=limits.max_reprojection_rms_px * 1.5,
        confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success or inliers is None or inliers.size < 4:
        return [], 0
    indices = inliers.reshape(-1)
    inlier_points = CorrespondenceSet(
        points.object_points[indices], points.image_points[indices], 1
    )
    if indices.size / points.object_points.shape[0] < limits.min_inlier_ratio:
        return [], int(indices.size)
    solved = cv2.solvePnPGeneric(
        inlier_points.object_points,
        inlier_points.image_points,
        camera.matrix,
        camera.distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not solved[0]:
        return [], int(indices.size)
    candidates: list[PoseCandidate] = []
    for raw_rvec, raw_tvec in zip(solved[1], solved[2]):
        rvec, tvec = cv2.solvePnPRefineLM(
            inlier_points.object_points,
            inlier_points.image_points,
            camera.matrix,
            camera.distortion,
            raw_rvec,
            raw_tvec,
        )
        rms = _rms(inlier_points, camera, rvec, tvec)
        candidate = candidate_from_vectors(
            CandidateVectors(inlier_points.object_points, rvec, tvec, rms)
        )
        if isinstance(candidate, PoseCandidate) and rms <= limits.max_reprojection_rms_px:
            candidates.append(
                PoseCandidate(
                    candidate.rotation_vector,
                    candidate.translation_m,
                    rms,
                    int(indices.size),
                )
            )
    return candidates, int(indices.size)


def _angle_delta(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def _yaw(candidate: PoseCandidate) -> float:
    rotation, _ = cv2.Rodrigues(candidate.rotation_vector)
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _select(
    candidates: list[PoseCandidate],
    motion: MotionContext,
    limits: PoseLimits,
    symmetry_order: int = 1,
) -> PoseCandidate | RejectReason:
    viable = candidates
    if motion.prior is not None:
        viable = [
            candidate
            for candidate in viable
            if np.linalg.norm(candidate.translation_m - motion.prior.translation_m)
            <= limits.max_translation_jump_m
        ]
        if not viable:
            return RejectReason.TEMPORAL_JUMP
    if motion.heading_rad is None and motion.prior is None:
        if symmetry_order > 1:
            return RejectReason.SYMMETRIC_AMBIGUOUS
        # Asymmetric target (e.g. AprilTag) without any motion prior — the
        # candidate with the lowest reprojection error is the best pose
        # (top-down field test where the tag's yaw on the ground is arbitrary).
        return min(viable, key=lambda candidate: candidate.reprojection_rms_px)
    expected_yaw = motion.heading_rad
    if expected_yaw is None and motion.prior is not None:
        prior_rotation, _ = cv2.Rodrigues(motion.prior.rotation_vector)
        expected_yaw = math.atan2(float(prior_rotation[1, 0]), float(prior_rotation[0, 0]))
    assert expected_yaw is not None
    ranked = sorted(
        viable,
        key=lambda candidate: (
            _angle_delta(_yaw(candidate), expected_yaw),
            candidate.reprojection_rms_px,
        ),
    )
    turn_allowance = min(0.5, abs(motion.yaw_rate_rad_s) * 0.2)
    heading_limit = (
        limits.max_heading_jump_rad * (1.0 + 0.5 * motion.turn_class)
        + turn_allowance
    )
    if not ranked or _angle_delta(_yaw(ranked[0]), expected_yaw) > heading_limit:
        return RejectReason.TEMPORAL_JUMP
    return ranked[0]


def estimate_target_pose(
    points: CorrespondenceSet,
    camera: CameraModel,
    motion: MotionContext,
    limits: PoseLimits,
) -> PoseEstimate | PoseRejection:
    """Solve and quality-gate all planar pose and symmetry candidates."""
    count = points.object_points.shape[0]
    if count < 4 or points.image_points.shape != (count, 2):
        return PoseRejection(RejectReason.INSUFFICIENT_POINTS)
    if np.linalg.matrix_rank(points.object_points[:, :2] - points.object_points[0, :2]) < 2:
        return PoseRejection(RejectReason.COLLINEAR_POINTS)
    candidates: list[PoseCandidate] = []
    best_inliers = 0
    for turn in range(points.symmetry_order):
        solved, inliers = _solve_variant(_rotate_correspondences(points, turn), camera, limits)
        candidates.extend(solved)
        best_inliers = max(best_inliers, inliers)
    if not candidates:
        reason = RejectReason.REPROJECTION_RMS if best_inliers >= 4 else RejectReason.PNP_FAILED
        return PoseRejection(reason)
    selected = _select(candidates, motion, limits, points.symmetry_order)
    if isinstance(selected, RejectReason):
        return PoseRejection(
            selected,
            len(candidates),
            min(item.reprojection_rms_px for item in candidates),
        )
    focal = (float(camera.matrix[0, 0]) + float(camera.matrix[1, 1])) / 2.0
    depth = float(selected.translation_m[2])
    translation_variance = max(1e-6, (selected.reprojection_rms_px * depth / focal) ** 2)
    rotation_variance = max(1e-5, (selected.reprojection_rms_px / focal) ** 2)
    covariance = [0.0] * 36
    for index in (0, 7, 14):
        covariance[index] = translation_variance
    for index in (21, 28, 35):
        covariance[index] = rotation_variance
    inlier_ratio = selected.inlier_count / count
    quality = inlier_ratio * math.exp(
        -selected.reprojection_rms_px / limits.max_reprojection_rms_px
    )
    return PoseEstimate(
        selected.rotation_vector,
        selected.translation_m,
        selected.reprojection_rms_px,
        len(candidates),
        selected.inlier_count,
        float(np.clip(quality, 0.0, 1.0)),
        tuple(covariance),
    )
