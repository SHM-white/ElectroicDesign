"""Quality-gate tests for planar IPPE/PnP target pose."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from target_fixture import render_target, ring_correspondences  # noqa: E402


def _inputs(symmetry_order: int = 1):
    from ed_uav_perception.target_types import (
        CameraModel,
        CorrespondenceSet,
        MotionContext,
        PoseLimits,
    )

    rendered = render_target()
    objects, images = ring_correspondences(rendered)
    camera = CameraModel(
        rendered.camera_matrix,
        rendered.distortion,
        640,
        480,
        "camera_optical",
        True,
    )
    points = CorrespondenceSet(objects, images, symmetry_order)
    motion = MotionContext(10.0, 100.0, 0, 0.18, 0.0, 0.6, None)
    return rendered, camera, points, motion, PoseLimits()


def test_recovers_known_target_pose_with_raw_distortion() -> None:
    # Given
    rendered, camera, points, motion, limits = _inputs()
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import PoseEstimate

    # When
    result = estimate_target_pose(points, camera, motion, limits)

    # Then
    assert isinstance(result, PoseEstimate)
    assert result.translation_m == pytest.approx(rendered.tvec.reshape(3), abs=0.004)
    assert result.reprojection_rms_px < 0.05
    assert result.candidate_count >= 1


def test_rejects_symmetric_target_without_heading_or_prior() -> None:
    # Given
    _, camera, points, motion, limits = _inputs(symmetry_order=4)
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import MotionContext, PoseRejection

    context = MotionContext(
        motion.acquisition_sec,
        motion.receipt_steady_sec,
        motion.turn_class,
        None,
        motion.yaw_rate_rad_s,
        motion.speed_m_s,
        None,
    )

    # When
    result = estimate_target_pose(points, camera, context, limits)

    # Then
    assert isinstance(result, PoseRejection)
    assert result.reason.value == "symmetric_ambiguous"


def test_ransac_rejects_single_correspondence_outlier() -> None:
    # Given
    _, camera, points, motion, limits = _inputs()
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import CorrespondenceSet, PoseEstimate

    corrupted = points.image_points.copy()
    corrupted[3] += np.array([85.0, -70.0])

    # When
    result = estimate_target_pose(
        CorrespondenceSet(points.object_points, corrupted, 1), camera, motion, limits
    )

    # Then
    assert isinstance(result, PoseEstimate)
    assert result.inlier_count == 15
    assert result.reprojection_rms_px < 0.15


def test_rejects_excessive_outlier_fraction() -> None:
    # Given
    _, camera, points, motion, limits = _inputs()
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import CorrespondenceSet, PoseRejection

    corrupted = points.image_points.copy()
    corrupted[:6] += np.array(
        [[90.0, -70.0], [-80.0, 65.0], [75.0, 80.0], [-60.0, -85.0], [95.0, 45.0], [-75.0, 90.0]]
    )

    # When
    result = estimate_target_pose(
        CorrespondenceSet(points.object_points, corrupted, 1), camera, motion, limits
    )

    # Then
    assert isinstance(result, PoseRejection)
    assert result.reason.value == "reprojection_rms"


def test_heading_disambiguates_permuted_symmetric_correspondences() -> None:
    # Given
    rendered, camera, points, motion, limits = _inputs(symmetry_order=4)
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import PoseEstimate

    # When
    result = estimate_target_pose(points, camera, motion, limits)

    # Then
    assert isinstance(result, PoseEstimate)
    assert result.translation_m == pytest.approx(rendered.tvec.reshape(3), abs=0.004)
    assert result.candidate_count >= 4


def test_prior_pose_disambiguates_without_absolute_heading() -> None:
    # Given
    rendered, camera, points, motion, limits = _inputs(symmetry_order=4)
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import MotionContext, PoseEstimate, PosePrior

    context = MotionContext(
        10.0,
        100.0,
        0,
        None,
        0.0,
        motion.speed_m_s,
        PosePrior(
            rendered.tvec.reshape(3), rendered.rvec.reshape(3), 9.95, 99.95
        ),
    )

    # When
    result = estimate_target_pose(points, camera, context, limits)

    # Then
    assert isinstance(result, PoseEstimate)
    assert result.translation_m == pytest.approx(rendered.tvec.reshape(3), abs=0.004)


def test_rejects_temporal_jump_beyond_bound() -> None:
    # Given
    _, camera, points, motion, limits = _inputs()
    from ed_uav_perception.target_pose import estimate_target_pose
    from ed_uav_perception.target_types import MotionContext, PosePrior, PoseRejection

    prior = PosePrior(np.array([1.0, 1.0, 0.4]), np.zeros(3), 9.95, 99.95)
    context = MotionContext(10.0, 100.0, 0, 0.18, 0.0, 0.6, prior)

    # When
    result = estimate_target_pose(points, camera, context, limits)

    # Then
    assert isinstance(result, PoseRejection)
    assert result.reason.value == "temporal_jump"


def test_rejects_negative_depth_candidate() -> None:
    # Given
    from ed_uav_perception.target_pose import candidate_from_vectors
    from ed_uav_perception.target_types import CandidateVectors, RejectReason

    objects = np.array(
        [[-0.25, 0.0, 0.0], [0.25, 0.0, 0.0], [0.0, -0.25, 0.0], [0.0, 0.25, 0.0]],
        dtype=np.float64,
    )

    # When
    result = candidate_from_vectors(
        CandidateVectors(
            objects,
            np.zeros((3, 1)),
            np.array([[0.0], [0.0], [-1.0]]),
            0.1,
        )
    )

    # Then
    assert result is RejectReason.NEGATIVE_DEPTH
