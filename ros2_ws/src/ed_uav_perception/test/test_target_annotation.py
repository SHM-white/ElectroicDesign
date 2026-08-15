"""Pure rendering tests for target-observation image annotations."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def test_accepted_annotation_shows_optical_pose_quality_and_rms() -> None:
    # Given
    from ed_uav_perception.target_annotation import annotation_lines
    from ed_uav_perception.target_types import AcceptedObservation, PoseEstimate

    observation = AcceptedObservation(
        12.0,
        3,
        "camera_optical",
        "d2026-circle-cross-v1",
        0.02,
        PoseEstimate(
            np.array([0.0, 0.0, 0.0]),
            np.array([0.1, -0.2, 1.4]),
            0.35,
            8,
            16,
            0.82,
            tuple([0.01] * 36),
        ),
    )

    # When
    lines = annotation_lines(observation)

    # Then
    text = "\n".join(lines)
    assert "frame_id: camera_optical" in text
    assert "optical: x right y down z forward" in text
    assert "X: +0.100 m Y: -0.200 m Z: +1.400 m" in text
    assert "quality: 0.820" in text
    assert "reprojection RMS: 0.350 px" in text


def test_rejected_annotation_shows_reason_without_coordinates() -> None:
    # Given
    from ed_uav_perception.target_annotation import annotation_lines
    from ed_uav_perception.target_types import RejectedObservation, RejectReason

    observation = RejectedObservation(
        12.0,
        3,
        "camera_optical",
        "d2026-circle-cross-v1",
        RejectReason.STALE_VEHICLE,
    )

    # When
    text = "\n".join(annotation_lines(observation))

    # Then
    assert "frame_id: camera_optical" in text
    assert "status: REJECTED" in text
    assert "reason: stale_vehicle" in text
    assert "optical: x right y down z forward" in text
    assert "X:" not in text
    assert "Y:" not in text
    assert "Z:" not in text


def test_render_preserves_shape_and_does_not_mutate_input() -> None:
    # Given
    from ed_uav_perception.target_annotation import (
        AnnotationFrame,
        render_target_observation,
    )
    from ed_uav_perception.target_types import RejectedObservation, RejectReason

    image = np.full((24, 32, 3), 127, dtype=np.uint8)
    original = image.copy()
    observation = RejectedObservation(
        12.0,
        3,
        "camera_optical",
        "d2026-circle-cross-v1",
        RejectReason.UNCALIBRATED,
    )

    # When
    annotated = render_target_observation(AnnotationFrame(image), observation)

    # Then
    assert annotated.shape == image.shape
    assert np.array_equal(image, original)
    assert not np.array_equal(annotated, original)


def test_accepted_render_marks_target_origin_in_camera_frame() -> None:
    # Given
    from ed_uav_perception.target_annotation import (
        AnnotationFrame,
        render_target_observation,
    )
    from ed_uav_perception.target_types import (
        AcceptedObservation,
        CameraModel,
        PoseEstimate,
    )

    image = np.zeros((80, 100, 3), dtype=np.uint8)
    camera = CameraModel(
        np.array([[50.0, 0.0, 50.0], [0.0, 50.0, 40.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
        100,
        80,
        "camera_optical",
        True,
    )
    observation = AcceptedObservation(
        12.0,
        3,
        "camera_optical",
        "d2026-circle-cross-v1",
        0.02,
        PoseEstimate(
            np.zeros(3),
            np.array([0.2, -0.1, 1.0]),
            0.35,
            8,
            16,
            0.82,
            tuple([0.01] * 36),
        ),
    )

    # When
    annotated = render_target_observation(AnnotationFrame(image, camera), observation)

    # Then
    assert np.any(annotated[30:41, 55:66, 1] > 180)


def test_node_publishes_decodable_early_rejection_annotation_with_header() -> None:
    # Given
    import rclpy
    from cv_bridge import CvBridge
    from ed_uav_perception.target_observation_node import (
        ANNOTATED_IMAGE_TOPIC,
        TargetObservationNode,
    )
    from ed_uav_perception.target_annotation import annotation_lines
    from ed_uav_perception.target_types import AcceptedObservation, RejectedObservation
    from sensor_msgs.msg import Image

    class ImageCapture:
        def __init__(self) -> None:
            self.messages: list[Image] = []

        def publish(self, message: Image) -> None:
            self.messages.append(message)

    rclpy.init()
    node = TargetObservationNode()
    capture = ImageCapture()
    try:
        assert node._annotated_publisher.topic_name == ANNOTATED_IMAGE_TOPIC
        node._annotated_publisher = capture

        # Set up camera_info with mismatched dimensions (640x480) so the
        # 32x48 image fails raster binding → node returns silently.
        from ed_uav_interfaces.msg import VehicleTelemetry
        from sensor_msgs.msg import CameraInfo
        cam_info = CameraInfo()
        cam_info.header.frame_id = "camera_optical"
        cam_info.width = 640
        cam_info.height = 480
        cam_info.k = [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0]
        node._camera_info_callback(cam_info, "narrow")
        vehicle = VehicleTelemetry()
        vehicle.contract_version = vehicle.CONTRACT_VERSION
        vehicle.acquisition_stamp = node.get_clock().now().to_msg()
        vehicle.source_sequence = 1
        vehicle.heartbeat_alive = True
        vehicle.turn_class = vehicle.TURN_STRAIGHT
        vehicle.motion_kind = vehicle.MOTION_WHEEL_SPEED
        vehicle.wheel_speed_m_s = 0.0
        vehicle.heading_rad = 0.0
        vehicle.yaw_rate_rad_s = 0.0
        vehicle.frame_id = "vehicle_start"
        node._vehicle_callback(vehicle)

        source = CvBridge().cv2_to_imgmsg(
            np.full((32, 48, 3), 127, dtype=np.uint8), encoding="bgr8"
        )
        source.header.frame_id = "camera_optical"
        source.header.stamp.sec = 4
        source.header.stamp.nanosec = 5

        # When — image raster (32x48) mismatches camera_info (640x480)
        node._image_callback(source, "narrow")

        # Then — silent rejection; nothing published
        assert len(capture.messages) == 0
        assert not isinstance(node.last_result, AcceptedObservation)
    finally:
        node.destroy_node()
        rclpy.shutdown()
