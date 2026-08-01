"""Freshness and calibration boundary for target observations."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from ed_uav_perception.target_detector import DetectionFailure, REVISION_APRILTAG, REVISION_CIRCLE_CROSS, detect_target
from ed_uav_perception.target_pose import estimate_target_pose
from ed_uav_perception.target_types import (
    AcceptedObservation,
    CorrespondenceSet,
    ObservationRequest,
    ObservationResult,
    PoseEstimate,
    RejectedObservation,
    RejectReason,
)


def _reject(request: ObservationRequest, reason: RejectReason) -> RejectedObservation:
    return RejectedObservation(
        request.frame.acquisition_sec,
        request.frame.source_sequence,
        request.camera.frame_id,
        request.frame.target_revision,
        reason,
    )


def observe_target(request: ObservationRequest) -> ObservationResult:
    """Return a quality-gated observation and never infer missing context."""
    if request.frame.target_revision not in (REVISION_APRILTAG, REVISION_CIRCLE_CROSS):
        return _reject(request, RejectReason.WRONG_REVISION)
    if (
        not request.camera.calibrated
        or request.camera.matrix.shape != (3, 3)
        or not np.all(np.isfinite(request.camera.matrix))
        or not np.all(np.isfinite(request.camera.distortion))
    ):
        return _reject(request, RejectReason.UNCALIBRATED)
    if request.image.shape[:2] != (request.camera.height, request.camera.width):
        return _reject(request, RejectReason.RASTER_MISMATCH)
    if not all(
        math.isfinite(value)
        for value in (
            request.frame.acquisition_sec,
            request.frame.receipt_steady_sec,
            request.frame.evaluation_steady_sec,
        )
    ):
        return _reject(request, RejectReason.INVALID_INPUT)
    image_age = request.frame.evaluation_steady_sec - request.frame.receipt_steady_sec
    if image_age < 0.0:
        return _reject(request, RejectReason.FUTURE_IMAGE)
    if image_age > request.limits.image_freshness_sec:
        return _reject(request, RejectReason.STALE_IMAGE)
    if (
        request.motion.turn_class not in (0, 1, 2)
        or not math.isfinite(request.motion.acquisition_sec)
        or not math.isfinite(request.motion.receipt_steady_sec)
        or not math.isfinite(request.motion.yaw_rate_rad_s)
        or not math.isfinite(request.motion.speed_m_s)
        or request.motion.speed_m_s < 0.0
        or (
            request.motion.heading_rad is not None
            and not math.isfinite(request.motion.heading_rad)
        )
        or (
            request.motion.prior is not None
            and (
                not math.isfinite(request.motion.prior.acquisition_sec)
                or not math.isfinite(request.motion.prior.receipt_steady_sec)
                or not np.all(np.isfinite(request.motion.prior.translation_m))
                or not np.all(np.isfinite(request.motion.prior.rotation_vector))
            )
        )
    ):
        return _reject(request, RejectReason.INVALID_VEHICLE_CONTEXT)
    if request.motion.turn_class == 0 and abs(request.motion.yaw_rate_rad_s) > 0.15:
        return _reject(request, RejectReason.INVALID_VEHICLE_CONTEXT)
    if request.motion.turn_class in (1, 2) and abs(request.motion.yaw_rate_rad_s) < 0.01:
        return _reject(request, RejectReason.INVALID_VEHICLE_CONTEXT)
    vehicle_age = (
        request.frame.evaluation_steady_sec - request.motion.receipt_steady_sec
    )
    if vehicle_age < 0.0:
        return _reject(request, RejectReason.FUTURE_VEHICLE)
    if vehicle_age > request.limits.vehicle_freshness_sec:
        return _reject(request, RejectReason.STALE_VEHICLE)
    if request.motion.prior is not None:
        prior_age = (
            request.frame.evaluation_steady_sec
            - request.motion.prior.receipt_steady_sec
        )
        if prior_age < 0.0:
            return _reject(request, RejectReason.INVALID_VEHICLE_CONTEXT)
        if prior_age > request.limits.max_prior_age_sec:
            return _reject(request, RejectReason.STALE_PRIOR)
    acquisition_delta = (
        request.frame.acquisition_sec - request.motion.acquisition_sec
    )
    if acquisition_delta < 0.0:
        return _reject(request, RejectReason.FUTURE_VEHICLE)
    motion = request.motion
    if motion.heading_rad is not None:
        predicted_heading = math.atan2(
            math.sin(motion.heading_rad + motion.yaw_rate_rad_s * acquisition_delta),
            math.cos(motion.heading_rad + motion.yaw_rate_rad_s * acquisition_delta),
        )
        motion = replace(motion, heading_rad=predicted_heading)
    detection = detect_target(request.image, request.frame.target_revision)
    if not isinstance(detection, CorrespondenceSet):
        # 正向检查: detect_target 可能返回不同模块的 DetectionFailure 类
        # (apriltag_detector / target_detector), 反向 isinstance 会漏判
        reason = getattr(detection, "reason", RejectReason.PNP_FAILED)
        return _reject(request, reason)
    pose = estimate_target_pose(detection, request.camera, motion, request.limits)
    if not isinstance(pose, PoseEstimate):
        return RejectedObservation(
            request.frame.acquisition_sec,
            request.frame.source_sequence,
            request.camera.frame_id,
            request.frame.target_revision,
            pose.reason,
            pose.candidate_count,
            pose.reprojection_rms_px,
        )
    return AcceptedObservation(
        request.frame.acquisition_sec,
        request.frame.source_sequence,
        request.camera.frame_id,
        request.frame.target_revision,
        0.020,
        pose,
    )
