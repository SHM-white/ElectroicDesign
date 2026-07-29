"""Map accepted and rejected results to the typed ROS observation contract."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion

from ed_uav_interfaces.msg import TargetObservation
from ed_uav_perception.target_types import (
    AcceptedObservation,
    ObservationResult,
)


@dataclass(frozen=True, slots=True)
class InvalidPoseMessageError(Exception):
    reason: str

    def __str__(self) -> str:
        return self.reason


def _quaternion(observation: AcceptedObservation) -> Quaternion:
    if (
        not np.all(np.isfinite(observation.pose.rotation_vector))
        or not np.all(np.isfinite(observation.pose.translation_m))
        or not np.all(np.isfinite(observation.pose.covariance))
        or not np.isfinite(observation.quality)
    ):
        raise InvalidPoseMessageError("target pose contains non-finite values")
    rotation, _ = cv2.Rodrigues(observation.pose.rotation_vector)
    trace = float(rotation.trace())
    if trace > 0.0:
        scale = 2.0 * (trace + 1.0) ** 0.5
        return Quaternion(
            x=float((rotation[2, 1] - rotation[1, 2]) / scale),
            y=float((rotation[0, 2] - rotation[2, 0]) / scale),
            z=float((rotation[1, 0] - rotation[0, 1]) / scale),
            w=float(scale / 4.0),
        )
    diagonal = [float(rotation[index, index]) for index in range(3)]
    axis = diagonal.index(max(diagonal))
    next_axis = (axis + 1) % 3
    last_axis = (axis + 2) % 3
    values = [0.0, 0.0, 0.0, 0.0]
    scale = 2.0 * (1.0 + diagonal[axis] - diagonal[next_axis] - diagonal[last_axis]) ** 0.5
    if scale <= 1e-12:
        raise InvalidPoseMessageError("target rotation cannot form a quaternion")
    values[axis] = scale / 4.0
    values[next_axis] = float((rotation[next_axis, axis] + rotation[axis, next_axis]) / scale)
    values[last_axis] = float((rotation[last_axis, axis] + rotation[axis, last_axis]) / scale)
    values[3] = float((rotation[last_axis, next_axis] - rotation[next_axis, last_axis]) / scale)
    return Quaternion(x=values[0], y=values[1], z=values[2], w=values[3])


def to_target_observation(
    observation: ObservationResult, stamp: Time
) -> TargetObservation:
    """Build one typed valid or rejected observation for every image."""
    message = TargetObservation()
    message.contract_version = TargetObservation.CONTRACT_VERSION
    message.acquisition_stamp = stamp
    message.source_sequence = observation.source_sequence
    message.observation_id = f"target-{observation.source_sequence}"
    message.target_revision = observation.target_revision
    message.frame_id = observation.frame_id
    message.candidate_count = observation.candidate_count
    rms = observation.reprojection_rms_px
    message.reprojection_rms_px = rms if math.isfinite(rms) else -1.0
    if isinstance(observation, AcceptedObservation):
        translation = observation.pose.translation_m
        message.valid = True
        message.status = TargetObservation.STATUS_VALID
        message.pose.pose = Pose(
            position=Point(
                x=float(translation[0]),
                y=float(translation[1]),
                z=float(translation[2]),
            ),
            orientation=_quaternion(observation),
        )
        message.pose.covariance = list(observation.pose.covariance)
        message.confidence = observation.quality
        message.quality = observation.quality
        message.rejection_reason = ""
        message.outer_diameter_m = 0.50
        message.inner_diameter_m = 0.30
        message.line_width_m = observation.line_width_m
    else:
        message.valid = False
        message.status = TargetObservation.STATUS_REJECTED
        message.confidence = 0.0
        message.quality = 0.0
        message.rejection_reason = observation.reject_reason.value
        message.outer_diameter_m = 0.0
        message.inner_diameter_m = 0.0
        message.line_width_m = 0.0
    return message
