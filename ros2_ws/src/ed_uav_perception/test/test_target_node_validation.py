"""ROS boundary regressions for target-observation input validity."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from target_fixture import render_target  # noqa: E402


class _Clock:
    def __init__(self, now: float = 100.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _CapturePublisher:
    """Mutable publisher fake retaining the real ROS messages passed to it."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _case():
    import rclpy
    from cv_bridge import CvBridge
    from ed_uav_interfaces.msg import VehicleTelemetry
    from ed_uav_perception.target_observation_node import TargetObservationNode
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    clock = _Clock()
    node = TargetObservationNode(steady_clock=clock)
    capture = _CapturePublisher()
    node._publisher = capture
    rendered = render_target()
    now = node.get_clock().now().to_msg()
    camera = CameraInfo()
    camera.header.frame_id = "camera_optical"
    camera.header.stamp.sec = now.sec
    camera.header.stamp.nanosec = now.nanosec
    camera.width = 640
    camera.height = 480
    camera.k = rendered.camera_matrix.reshape(-1).tolist()
    camera.d = rendered.distortion.tolist()
    vehicle = VehicleTelemetry()
    vehicle.contract_version = vehicle.CONTRACT_VERSION
    vehicle.acquisition_stamp.sec = now.sec
    vehicle.acquisition_stamp.nanosec = now.nanosec
    vehicle.source_sequence = 7
    vehicle.heartbeat_alive = True
    vehicle.motion_kind = vehicle.MOTION_WHEEL_SPEED
    vehicle.wheel_speed_m_s = 0.6
    vehicle.turn_class = vehicle.TURN_STRAIGHT
    vehicle.heading_rad = 0.18
    vehicle.yaw_rate_rad_s = 0.0
    vehicle.frame_id = "vehicle_start"
    image = CvBridge().cv2_to_imgmsg(rendered.image, encoding="bgr8")
    image.header.frame_id = "camera_optical"
    image.header.stamp.sec = now.sec
    image.header.stamp.nanosec = now.nanosec
    return node, clock, capture, camera, vehicle, image


def _close(node) -> None:
    import rclpy

    node.destroy_node()
    rclpy.shutdown()


def test_node_uses_canonical_vehicle_topic_and_publishes_valid_status() -> None:
    # Given
    node, _, capture, camera, vehicle, image = _case()
    try:
        # When
        node._camera_callback(camera, "narrow")
        node._vehicle_callback(vehicle)
        node._image_callback(image, "narrow")

        # Then
        assert node.vehicle_topic == "/d_task/vehicle/telemetry"
        assert len(capture.messages) == 1
        assert capture.messages[0].valid is True
        assert capture.messages[0].status == capture.messages[0].STATUS_VALID
    finally:
        _close(node)


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("contract_version", 999, "vehicle_contract_version"),
        ("heartbeat_alive", False, "vehicle_heartbeat_lost"),
        ("motion_kind", 0, "invalid_vehicle_context"),
        ("turn_class", 255, "invalid_vehicle_context"),
        ("frame_id", "map", "wrong_vehicle_frame"),
    ],
)
def test_node_publishes_typed_rejection_for_invalid_vehicle(
    field: str, value, reason: str
) -> None:
    # Given
    node, _, capture, camera, vehicle, image = _case()
    try:
        setattr(vehicle, field, value)

        # When
        node._camera_callback(camera, "narrow")
        node._vehicle_callback(vehicle)
        node._image_callback(image, "narrow")

        # Then
        assert len(capture.messages) == 1
        assert capture.messages[0].valid is False
        assert capture.messages[0].rejection_reason == reason
    finally:
        _close(node)


@pytest.mark.parametrize("next_sequence", [7, 6])
def test_node_rejects_duplicate_or_replayed_vehicle_sequence(
    next_sequence: int,
) -> None:
    # Given
    node, _, capture, camera, vehicle, image = _case()
    try:
        node._vehicle_callback(vehicle)
        vehicle.source_sequence = next_sequence

        # When
        node._vehicle_callback(vehicle)
        node._camera_callback(camera, "narrow")
        node._image_callback(image, "narrow")

        # Then
        assert capture.messages[0].valid is False
        assert capture.messages[0].rejection_reason == "replayed_vehicle_sequence"
    finally:
        _close(node)


@pytest.mark.parametrize(
    ("mutation", "reason"),
    [
        ("frame", "camera_info_frame_mismatch"),
        ("raster", "camera_info_raster_mismatch"),
        ("stamp", "camera_info_stamp_mismatch"),
    ],
)
def test_node_binds_camera_info_to_image(mutation: str, reason: str) -> None:
    # Given
    node, _, capture, camera, vehicle, image = _case()
    try:
        if mutation == "frame":
            camera.header.frame_id = "other_camera"
        elif mutation == "raster":
            camera.width = 320
        else:
            shifted_ns = (
                camera.header.stamp.sec * 1_000_000_000
                + camera.header.stamp.nanosec
                + 2_000_000
            )
            camera.header.stamp.sec, camera.header.stamp.nanosec = divmod(
                shifted_ns, 1_000_000_000
            )

        # When
        node._camera_callback(camera, "narrow")
        node._vehicle_callback(vehicle)
        node._image_callback(image, "narrow")

        # Then
        assert capture.messages[0].valid is False
        assert capture.messages[0].rejection_reason == reason
    finally:
        _close(node)


def test_node_uses_steady_receipt_age_for_vehicle_freshness() -> None:
    # Given
    node, clock, capture, camera, vehicle, image = _case()
    try:
        node._camera_callback(camera, "narrow")
        node._vehicle_callback(vehicle)
        clock.now = 100.51

        # When
        node._image_callback(image, "narrow")

        # Then
        assert capture.messages[0].valid is False
        assert capture.messages[0].rejection_reason == "stale_vehicle"
    finally:
        _close(node)
