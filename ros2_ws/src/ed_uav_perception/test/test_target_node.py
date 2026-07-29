"""ROS node boundary test for calibrated target observations."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from target_fixture import render_target  # noqa: E402


def test_node_consumes_typed_camera_and_vehicle_context() -> None:
    # Given
    import rclpy
    from cv_bridge import CvBridge
    from ed_uav_interfaces.msg import VehicleTelemetry
    from ed_uav_perception.target_observation_node import TargetObservationNode
    from ed_uav_perception.target_types import AcceptedObservation
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = TargetObservationNode()
    try:
        rendered = render_target()
        now = node.get_clock().now().to_msg()
        camera = CameraInfo()
        camera.header.frame_id = "camera_optical"
        camera.header.stamp = now
        camera.width = 640
        camera.height = 480
        camera.k = rendered.camera_matrix.reshape(-1).tolist()
        camera.d = rendered.distortion.tolist()
        vehicle = VehicleTelemetry()
        vehicle.contract_version = vehicle.CONTRACT_VERSION
        vehicle.acquisition_stamp = now
        vehicle.source_sequence = 1
        vehicle.heartbeat_alive = True
        vehicle.turn_class = vehicle.TURN_STRAIGHT
        vehicle.motion_kind = vehicle.MOTION_WHEEL_SPEED
        vehicle.wheel_speed_m_s = 0.6
        vehicle.heading_rad = 0.18
        vehicle.yaw_rate_rad_s = 0.0
        vehicle.frame_id = "vehicle_start"
        image = CvBridge().cv2_to_imgmsg(rendered.image, encoding="bgr8")
        image.header.frame_id = "camera_optical"
        image.header.stamp = now

        # When
        node._camera_callback(camera)
        node._vehicle_callback(vehicle)
        node._image_callback(image)

        # Then
        assert isinstance(node.last_result, AcceptedObservation)
        assert node.last_result.candidate_count == 8
        assert node.get_parameter("last_reject_reason").value == ""
        assert node.get_parameter("last_reprojection_rms_px").value < 0.5
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_publishes_rejected_dimensions_over_ros_topic() -> None:
    # Given
    import time

    import pytest
    import rclpy
    from ed_uav_interfaces.msg import TargetObservation
    from ed_uav_perception.target_observation_node import TargetObservationNode
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    producer = TargetObservationNode()
    probe = Node("rejected_target_observation_probe")
    received: list[TargetObservation] = []
    subscription = probe.create_subscription(
        TargetObservation,
        "/d_task/target_observation",
        received.append,
        qos_profile_sensor_data,
    )
    try:
        discovery_deadline = time.monotonic() + 2.0
        while (
            producer._publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(probe, timeout_sec=0.05)
            rclpy.spin_once(producer, timeout_sec=0.05)
        assert producer._publisher.get_subscription_count() == 1
        image = Image()
        image.header.frame_id = "camera_optical"
        image.header.stamp = producer.get_clock().now().to_msg()

        # When
        producer._image_callback(image)
        receipt_deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < receipt_deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)

        # Then
        assert len(received) == 1
        message = received[0]
        assert message.valid is False
        assert message.rejection_reason == "uncalibrated"
        assert message.outer_diameter_m == pytest.approx(0.50)
        assert message.inner_diameter_m == pytest.approx(0.30)
        assert message.line_width_m == pytest.approx(0.020)
    finally:
        probe.destroy_subscription(subscription)
        probe.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()
