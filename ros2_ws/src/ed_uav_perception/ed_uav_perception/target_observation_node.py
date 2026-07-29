"""ROS 2 node for calibrated prescribed-geometry target observations."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.msg import TargetObservation, VehicleTelemetry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ed_uav_perception.target_message import (
    InvalidPoseMessageError,
    to_target_observation,
)
from ed_uav_perception.target_pipeline import observe_target
from ed_uav_perception.target_types import (
    AcceptedObservation,
    CameraModel,
    FrameContext,
    MotionContext,
    ObservationRequest,
    ObservationResult,
    PoseLimits,
    PosePrior,
    RejectReason,
    RejectedObservation,
)


def _seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TargetObservationNode(Node):
    """Reject-first adapter from calibrated camera and vehicle context to pose."""

    def __init__(self) -> None:
        super().__init__("target_observation_node")
        self.declare_parameter("target_revision", "d2026-circle-cross-v1")
        self.declare_parameter("initial_vehicle_heading_rad", float("nan"))
        self.declare_parameter("max_reprojection_rms_px", 2.0)
        self.declare_parameter("last_candidate_count", 0)
        self.declare_parameter("last_reprojection_rms_px", -1.0)
        self.declare_parameter("last_quality", 0.0)
        self.declare_parameter("last_reject_reason", "not_observed")
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._vehicle: VehicleTelemetry | None = None
        self._prior: PosePrior | None = None
        self._sequence = 0
        self._last_result: ObservationResult | None = None
        self.create_subscription(
            CameraInfo,
            "/camera/narrow/camera_info",
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleTelemetry,
            "/d_task/vehicle_telemetry",
            self._vehicle_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/narrow/image_raw",
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._publisher = self.create_publisher(
            TargetObservation, "/d_task/target_observation", qos_profile_sensor_data
        )

    @property
    def last_result(self) -> ObservationResult | None:
        return self._last_result

    def _camera_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _vehicle_callback(self, message: VehicleTelemetry) -> None:
        self._vehicle = message

    def _record(self, result: ObservationResult) -> None:
        self._last_result = result
        if isinstance(result, AcceptedObservation):
            values = (
                result.candidate_count,
                result.reprojection_rms_px,
                result.quality,
                "",
            )
        else:
            values = (
                result.candidate_count,
                result.reprojection_rms_px if math.isfinite(result.reprojection_rms_px) else -1.0,
                0.0,
                result.reject_reason.value,
            )
        self.set_parameters(
            [
                Parameter("last_candidate_count", value=values[0]),
                Parameter("last_reprojection_rms_px", value=values[1]),
                Parameter("last_quality", value=values[2]),
                Parameter("last_reject_reason", value=values[3]),
            ]
        )

    def _reject_without_inputs(self, message: Image, reason: RejectReason) -> None:
        revision = str(self.get_parameter("target_revision").value)
        result = RejectedObservation(
            _seconds(message.header.stamp),
            self._sequence,
            message.header.frame_id,
            revision,
            reason,
        )
        self._record(result)

    def _image_callback(self, message: Image) -> None:
        self._sequence += 1
        if self._camera_info is None:
            self._reject_without_inputs(message, RejectReason.UNCALIBRATED)
            return
        if self._vehicle is None:
            self._reject_without_inputs(message, RejectReason.STALE_VEHICLE)
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError:
            self._reject_without_inputs(message, RejectReason.INVALID_INPUT)
            return
        info = self._camera_info
        matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64)
        heading_value = float(self.get_parameter("initial_vehicle_heading_rad").value)
        heading = heading_value if math.isfinite(heading_value) else None
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        speed = float(self._vehicle.wheel_speed_m_s)
        request = ObservationRequest(
            image,
            CameraModel(
                matrix,
                distortion,
                int(info.width),
                int(info.height),
                message.header.frame_id or info.header.frame_id,
                bool(matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0),
            ),
            FrameContext(
                _seconds(message.header.stamp),
                now_sec,
                self._sequence,
                str(self.get_parameter("target_revision").value),
            ),
            MotionContext(
                _seconds(self._vehicle.acquisition_stamp),
                int(self._vehicle.turn_class),
                heading,
                speed,
                self._prior,
            ),
            PoseLimits(
                max_reprojection_rms_px=float(
                    self.get_parameter("max_reprojection_rms_px").value
                )
            ),
        )
        result = observe_target(request)
        if not isinstance(result, AcceptedObservation):
            self._record(result)
            return
        try:
            message_out = to_target_observation(result, message.header.stamp)
        except InvalidPoseMessageError:
            self._record(
                RejectedObservation(
                    result.acquisition_sec,
                    result.source_sequence,
                    result.frame_id,
                    result.target_revision,
                    RejectReason.INVALID_INPUT,
                )
            )
            return
        self._record(result)
        self._prior = PosePrior(
            result.pose.translation_m,
            result.pose.rotation_vector,
            result.acquisition_sec,
        )
        self._publisher.publish(message_out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
