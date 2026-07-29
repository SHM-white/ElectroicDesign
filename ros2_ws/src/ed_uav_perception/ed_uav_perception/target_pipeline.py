"""Freshness and calibration boundary for target observations."""

from __future__ import annotations

import math

import numpy as np

from ed_uav_perception.target_detector import DetectionFailure, TARGET_REVISION, detect_target
from ed_uav_perception.target_pose import estimate_target_pose
from ed_uav_perception.target_types import (
    AcceptedObservation,
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
    if request.frame.target_revision != TARGET_REVISION:
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
    if not math.isfinite(request.frame.now_sec) or not math.isfinite(
        request.frame.acquisition_sec
    ):
        return _reject(request, RejectReason.INVALID_INPUT)
    image_age = request.frame.now_sec - request.frame.acquisition_sec
    if image_age < 0.0:
        return _reject(request, RejectReason.FUTURE_IMAGE)
    if image_age > request.limits.freshness_sec:
        return _reject(request, RejectReason.STALE_IMAGE)
    if (
        request.motion.turn_class not in (0, 1, 2)
        or not math.isfinite(request.motion.stamp_sec)
        or not math.isfinite(request.motion.speed_m_s)
        or request.motion.speed_m_s < 0.0
        or (
            request.motion.heading_rad is not None
            and not math.isfinite(request.motion.heading_rad)
        )
        or (
            request.motion.prior is not None
            and (
                not math.isfinite(request.motion.prior.stamp_sec)
                or not np.all(np.isfinite(request.motion.prior.translation_m))
                or not np.all(np.isfinite(request.motion.prior.rotation_vector))
            )
        )
    ):
        return _reject(request, RejectReason.INVALID_VEHICLE_CONTEXT)
    vehicle_age = request.frame.now_sec - request.motion.stamp_sec
    if vehicle_age < 0.0:
        return _reject(request, RejectReason.FUTURE_VEHICLE)
    if vehicle_age > request.limits.freshness_sec:
        return _reject(request, RejectReason.STALE_VEHICLE)
    detection = detect_target(request.image, request.frame.target_revision)
    if isinstance(detection, DetectionFailure):
        return _reject(request, detection.reason)
    pose = estimate_target_pose(detection, request.camera, request.motion, request.limits)
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
        pose,
    )
