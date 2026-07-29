"""ROS 2 node for calibrated prescribed-geometry target observations."""

from __future__ import annotations

import math
import time
from collections.abc import Callable

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.msg import TargetObservation, VehicleTelemetry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image

from ed_uav_perception.target_input import (
    stamp_seconds,
    validate_camera_binding,
    validate_vehicle,
)
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

VEHICLE_TOPIC = "/d_task/vehicle/telemetry"
VEHICLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
CAMERA_INFO_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TargetObservationNode(Node):
    """Reject-first adapter from calibrated camera and vehicle context to pose."""

    def __init__(
        self, *, steady_clock: Callable[[], float] = time.monotonic
    ) -> None:
        super().__init__("target_observation_node")
        self.declare_parameter("target_revision", "d2026-circle-cross-v1")
        self.declare_parameter("max_reprojection_rms_px", 2.0)
        self.declare_parameter("last_candidate_count", 0)
        self.declare_parameter("last_reprojection_rms_px", -1.0)
        self.declare_parameter("last_quality", 0.0)
        self.declare_parameter("last_reject_reason", "not_observed")
        self._steady_clock = steady_clock
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._vehicle: VehicleTelemetry | None = None
        self._vehicle_receipt_steady_sec = float("-inf")
        self._vehicle_reason = RejectReason.STALE_VEHICLE
        self._last_vehicle_sequence: int | None = None
        self._last_vehicle_acquisition_sec: float | None = None
        self._last_image_acquisition_sec: float | None = None
        self._prior: PosePrior | None = None
        self._sequence = 0
        self._last_result: ObservationResult | None = None
        self.create_subscription(
            CameraInfo,
            "/camera/narrow/camera_info",
            self._camera_callback,
            CAMERA_INFO_QOS,
        )
        self._vehicle_subscription = self.create_subscription(
            VehicleTelemetry,
            VEHICLE_TOPIC,
            self._vehicle_callback,
            VEHICLE_QOS,
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

    @property
    def vehicle_topic(self) -> str:
        return VEHICLE_TOPIC

    def _camera_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _vehicle_callback(self, message: VehicleTelemetry) -> None:
        receipt = self._steady_clock()
        reason = validate_vehicle(
            message,
            self._last_vehicle_sequence,
            self._last_vehicle_acquisition_sec,
        )
        if reason is not None:
            self._vehicle = None
            self._vehicle_reason = reason
            return
        self._vehicle = message
        self._vehicle_reason = RejectReason.STALE_VEHICLE
        self._vehicle_receipt_steady_sec = receipt
        self._last_vehicle_sequence = int(message.source_sequence)
        self._last_vehicle_acquisition_sec = stamp_seconds(
            message.acquisition_stamp
        )

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
                result.reprojection_rms_px
                if math.isfinite(result.reprojection_rms_px)
                else -1.0,
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

    def _emit(self, result: ObservationResult, image: Image) -> ObservationResult:
        final = result
        try:
            message_out = to_target_observation(final, image.header.stamp)
        except InvalidPoseMessageError:
            final = self._rejection(image, RejectReason.INVALID_INPUT)
            message_out = to_target_observation(final, image.header.stamp)
        self._record(final)
        self._publisher.publish(message_out)
        return final

    def _rejection(self, image: Image, reason: RejectReason) -> RejectedObservation:
        return RejectedObservation(
            stamp_seconds(image.header.stamp),
            self._sequence,
            image.header.frame_id or "camera_unknown",
            str(self.get_parameter("target_revision").value),
            reason,
        )

    def _reject(self, image: Image, reason: RejectReason) -> None:
        self._emit(self._rejection(image, reason), image)

    def _image_callback(self, message: Image) -> None:
        image_receipt = self._steady_clock()
        self._sequence += 1
        acquisition_sec = stamp_seconds(message.header.stamp)
        if (
            self._last_image_acquisition_sec is not None
            and acquisition_sec <= self._last_image_acquisition_sec
        ):
            self._reject(message, RejectReason.IMAGE_ACQUISITION_REGRESSION)
            return
        self._last_image_acquisition_sec = acquisition_sec
        if self._camera_info is None:
            self._reject(message, RejectReason.UNCALIBRATED)
            return
        if self._vehicle is None:
            self._reject(message, self._vehicle_reason)
            return
        binding_reason = validate_camera_binding(self._camera_info, message)
        if binding_reason is not None:
            self._reject(message, binding_reason)
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError:
            self._reject(message, RejectReason.INVALID_INPUT)
            return
        info = self._camera_info
        matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        vehicle = self._vehicle
        request = ObservationRequest(
            image,
            CameraModel(
                matrix,
                np.asarray(info.d, dtype=np.float64),
                int(info.width),
                int(info.height),
                message.header.frame_id,
                bool(matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0),
            ),
            FrameContext(
                acquisition_sec,
                image_receipt,
                self._steady_clock(),
                self._sequence,
                str(self.get_parameter("target_revision").value),
            ),
            MotionContext(
                stamp_seconds(vehicle.acquisition_stamp),
                self._vehicle_receipt_steady_sec,
                int(vehicle.turn_class),
                float(vehicle.heading_rad),
                float(vehicle.yaw_rate_rad_s),
                float(vehicle.wheel_speed_m_s),
                self._prior,
            ),
            PoseLimits(
                max_reprojection_rms_px=float(
                    self.get_parameter("max_reprojection_rms_px").value
                )
            ),
        )
        final = self._emit(observe_target(request), message)
        if isinstance(final, AcceptedObservation):
            self._prior = PosePrior(
                final.pose.translation_m,
                final.pose.rotation_vector,
                final.acquisition_sec,
                image_receipt,
            )


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
