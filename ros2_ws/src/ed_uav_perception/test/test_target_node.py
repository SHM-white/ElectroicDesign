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
