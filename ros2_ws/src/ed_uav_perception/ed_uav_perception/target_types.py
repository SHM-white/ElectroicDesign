"""Typed values for prescribed target detection and pose quality gates."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TypeAlias

import numpy as np


class RejectReason(str, Enum):
    INVALID_INPUT = "invalid_input"
    UNCALIBRATED = "uncalibrated"
    RASTER_MISMATCH = "raster_mismatch"
    WRONG_REVISION = "wrong_revision"
    PARTIAL_GEOMETRY = "partial_geometry"
    INCOMPLETE_CROSS = "incomplete_cross"
    LINE_WIDTH_OUT_OF_RANGE = "line_width_out_of_range"
    INSUFFICIENT_POINTS = "insufficient_points"
    COLLINEAR_POINTS = "collinear_points"
    PNP_FAILED = "pnp_failed"
    NEGATIVE_DEPTH = "negative_depth"
    REPROJECTION_RMS = "reprojection_rms"
    SYMMETRIC_AMBIGUOUS = "symmetric_ambiguous"
    STALE_IMAGE = "stale_image"
    STALE_VEHICLE = "stale_vehicle"
    FUTURE_IMAGE = "future_image"
    FUTURE_VEHICLE = "future_vehicle"
    INVALID_VEHICLE_CONTEXT = "invalid_vehicle_context"
    VEHICLE_CONTRACT_VERSION = "vehicle_contract_version"
    VEHICLE_HEARTBEAT_LOST = "vehicle_heartbeat_lost"
    WRONG_VEHICLE_FRAME = "wrong_vehicle_frame"
    REPLAYED_VEHICLE_SEQUENCE = "replayed_vehicle_sequence"
    VEHICLE_ACQUISITION_REGRESSION = "vehicle_acquisition_regression"
    IMAGE_ACQUISITION_REGRESSION = "image_acquisition_regression"
    CAMERA_INFO_FRAME_MISMATCH = "camera_info_frame_mismatch"
    CAMERA_INFO_RASTER_MISMATCH = "camera_info_raster_mismatch"
    CAMERA_INFO_STAMP_MISMATCH = "camera_info_stamp_mismatch"
    STALE_PRIOR = "stale_prior"
    TEMPORAL_JUMP = "temporal_jump"


@dataclass(frozen=True, slots=True)
class CameraModel:
    matrix: np.ndarray
    distortion: np.ndarray
    width: int
    height: int
    frame_id: str
    calibrated: bool


@dataclass(frozen=True, slots=True)
class CorrespondenceSet:
    object_points: np.ndarray
    image_points: np.ndarray
    symmetry_order: int
    line_width_m: float = 0.02


@dataclass(frozen=True, slots=True)
class PosePrior:
    translation_m: np.ndarray
    rotation_vector: np.ndarray
    acquisition_sec: float
    receipt_steady_sec: float


@dataclass(frozen=True, slots=True)
class MotionContext:
    acquisition_sec: float
    receipt_steady_sec: float
    turn_class: int
    heading_rad: float | None
    yaw_rate_rad_s: float
    speed_m_s: float
    prior: PosePrior | None


@dataclass(frozen=True, slots=True)
class FrameContext:
    acquisition_sec: float
    receipt_steady_sec: float
    evaluation_steady_sec: float
    source_sequence: int
    target_revision: str


@dataclass(frozen=True, slots=True)
class PoseLimits:
    image_freshness_sec: float = 0.20
    vehicle_freshness_sec: float = 0.50
    max_prior_age_sec: float = 0.20
    max_reprojection_rms_px: float = 2.0
    min_inlier_ratio: float = 0.70
    max_translation_jump_m: float = 0.35
    max_heading_jump_rad: float = 0.70


@dataclass(frozen=True, slots=True)
class ObservationRequest:
    image: np.ndarray
    camera: CameraModel
    frame: FrameContext
    motion: MotionContext
    limits: PoseLimits


@dataclass(frozen=True, slots=True)
class CandidateVectors:
    object_points: np.ndarray
    rotation_vector: np.ndarray
    translation_m: np.ndarray
    reprojection_rms_px: float


@dataclass(frozen=True, slots=True)
class PoseCandidate:
    rotation_vector: np.ndarray
    translation_m: np.ndarray
    reprojection_rms_px: float
    inlier_count: int


@dataclass(frozen=True, slots=True)
class PoseEstimate:
    rotation_vector: np.ndarray
    translation_m: np.ndarray
    reprojection_rms_px: float
    candidate_count: int
    inlier_count: int
    quality: float
    covariance: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class PoseRejection:
    reason: RejectReason
    candidate_count: int = 0
    reprojection_rms_px: float = float("inf")


@dataclass(frozen=True, slots=True)
class AcceptedObservation:
    acquisition_sec: float
    source_sequence: int
    frame_id: str
    target_revision: str
    line_width_m: float
    pose: PoseEstimate

    @property
    def candidate_count(self) -> int:
        return self.pose.candidate_count

    @property
    def reprojection_rms_px(self) -> float:
        return self.pose.reprojection_rms_px

    @property
    def quality(self) -> float:
        return self.pose.quality


@dataclass(frozen=True, slots=True)
class RejectedObservation:
    acquisition_sec: float
    source_sequence: int
    frame_id: str
    target_revision: str
    reject_reason: RejectReason
    candidate_count: int = 0
    reprojection_rms_px: float = float("inf")


ObservationResult: TypeAlias = AcceptedObservation | RejectedObservation
