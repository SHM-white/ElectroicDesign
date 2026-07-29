"""ROS contract mapping tests for accepted target observations."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def test_maps_quality_pose_to_frozen_target_observation() -> None:
    # Given
    from builtin_interfaces.msg import Time
    from ed_uav_perception.target_message import to_target_observation
    from ed_uav_perception.target_types import AcceptedObservation, PoseEstimate

    estimate = PoseEstimate(
        np.array([0.0, 0.0, 0.2]),
        np.array([0.1, -0.2, 1.4]),
        0.35,
        2,
        16,
        0.82,
        tuple([0.01] * 36),
    )
    accepted = AcceptedObservation(
        12.25, 9, "camera_optical", "d2026-circle-cross-v1", estimate
    )

    # When
    message = to_target_observation(accepted, Time(sec=12, nanosec=250_000_000))

    # Then
    assert message.contract_version == message.CONTRACT_VERSION
    assert message.acquisition_stamp.sec == 12
    assert message.frame_id == "camera_optical"
    assert message.pose.pose.position.z == 1.4
    assert message.confidence == 0.82
    assert len(message.pose.covariance) == 36


def test_rejects_nonfinite_pose_before_ros_publication() -> None:
    # Given
    from builtin_interfaces.msg import Time
    from ed_uav_perception.target_message import InvalidPoseMessageError
    from ed_uav_perception.target_message import to_target_observation
    from ed_uav_perception.target_types import AcceptedObservation, PoseEstimate

    estimate = PoseEstimate(
        np.array([np.nan, 0.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        0.2,
        2,
        16,
        0.8,
        tuple([0.01] * 36),
    )
    accepted = AcceptedObservation(
        1.0, 1, "camera_optical", "d2026-circle-cross-v1", estimate
    )

    # When / Then
    with pytest.raises(InvalidPoseMessageError):
        to_target_observation(accepted, Time(sec=1))
