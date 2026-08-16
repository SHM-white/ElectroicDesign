"""ROS 2 node for calibrated prescribed-geometry target observations.

Supports dual-camera fusion: narrow (primary) and wide (fallback).
When both cameras detect the target, results are quality-weighted averaged
in the body frame using known camera mounting yaw offsets.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.msg import TargetObservation, VehicleTelemetry
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image

from ed_uav_perception.target_annotation import AnnotationFrame, render_target_observation
from ed_uav_perception.target_input import (
    stamp_seconds,
    validate_camera_binding,
    validate_vehicle,
)
from ed_uav_perception.target_pipeline import observe_target
from ed_uav_perception.target_types import (
    AcceptedObservation,
    CameraModel,
    FrameContext,
    MotionContext,
    ObservationRequest,
    ObservationResult,
    PoseEstimate,
    PoseLimits,
    PosePrior,
    RejectReason,
    RejectedObservation,
)

try:
    import cv2  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

VEHICLE_TOPIC = "/d_task/vehicle/telemetry"
ANNOTATED_IMAGE_TOPIC = "/d_task/target_observation/annotated_image"
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
    durability=DurabilityPolicy.VOLATILE,
)
IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Camera mounting yaw offsets (rotation about optical Z axis).
# Standard optical frame assumes image-top = forward (nose).
#   narrow: image-top → drone right  ⇒ -π/2
#   wide:   image-top → drone left   ⇒ +π/2
_NARROW_YAW_OFFSET = -math.pi / 2
_WIDE_YAW_OFFSET = math.pi / 2


def _rotate_yaw(points: np.ndarray, yaw_rad: float) -> np.ndarray:
    """Rotate a 3-vector by *yaw_rad* about the Z axis."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return r @ points


# ---------------------------------------------------------------------------
# Dual-camera fusion
# ---------------------------------------------------------------------------

@dataclass
class _CameraState:
    """Per-camera rolling state."""
    info: CameraInfo | None = None
    last_acquisition_sec: float | None = None
    sequence: int = 0
    prior: PosePrior | None = None


from ed_uav_perception.kalman_tracker import KalmanTracker


class _DualCameraFusion:
    """Fuse narrow (primary) and wide (fallback) camera observations.

    Strategy
    --------
    1. Only one camera detects → use it directly.
    2. Both detect → transform translations to body frame (using known
       mounting yaw offsets), quality-weighted average, then transform back
       to the primary (narrow) camera frame for downstream consumers.
    3. Rotation: always use the primary camera's rotation vector (the yaw
       component is what matters for landing; both cameras see the same
       physical marker).
    4. A constant-velocity Kalman filter smooths the output, estimates
       velocity, and **predicts** the target position during frame drops.
    """

    def __init__(
        self,
        process_noise_pos: float = 0.05,
        process_noise_vel: float = 0.3,
        max_predict_age_sec: float = 0.5,
    ) -> None:
        self._tracker = KalmanTracker(
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            max_predict_age_sec=max_predict_age_sec,
        )
        self._last_obs: AcceptedObservation | None = None
        self._last_frame_id: str = ""

    def fuse(
        self,
        narrow: AcceptedObservation | None,
        wide: AcceptedObservation | None,
        now_sec: float,
    ) -> AcceptedObservation | None:
        """Return the best fused observation, or *None* when nothing detected
        and the tracker is too stale to predict.
        """
        if narrow is not None and wide is None:
            return self._update(narrow, now_sec)
        if wide is not None and narrow is None:
            return self._update(wide, now_sec)
        if narrow is not None and wide is not None:
            # Quality-weighted fusion in body frame.
            t_n = _rotate_yaw(narrow.pose.translation_m, _NARROW_YAW_OFFSET)
            t_w = _rotate_yaw(wide.pose.translation_m, _WIDE_YAW_OFFSET)

            q_n = max(narrow.quality, 1e-6)
            q_w = max(wide.quality, 1e-6)
            w_n = q_n / (q_n + q_w)

            t_fused_body = w_n * t_n + (1.0 - w_n) * t_w
            t_fused_primary = _rotate_yaw(t_fused_body, -_NARROW_YAW_OFFSET)

            fused = AcceptedObservation(
                acquisition_sec=narrow.acquisition_sec,
                source_sequence=narrow.source_sequence,
                frame_id=narrow.frame_id,
                target_revision=narrow.target_revision,
                line_width_m=narrow.line_width_m,
                pose=PoseEstimate(
                    rotation_vector=narrow.pose.rotation_vector,
                    translation_m=t_fused_primary,
                    reprojection_rms_px=min(
                        narrow.pose.reprojection_rms_px,
                        wide.pose.reprojection_rms_px,
                    ),
                    candidate_count=(
                        narrow.pose.candidate_count + wide.pose.candidate_count
                    ),
                    inlier_count=(
                        narrow.pose.inlier_count + wide.pose.inlier_count
                    ),
                    quality=(
                        w_n * narrow.pose.quality
                        + (1.0 - w_n) * wide.pose.quality
                    ),
                    covariance=narrow.pose.covariance,
                ),
            )
            return self._update(fused, now_sec)

        # Neither camera detected — try Kalman prediction.
        return self._predict(now_sec)

    # ── internals ───────────────────────────────────────────────────────

    def _update(
        self, obs: AcceptedObservation, now_sec: float
    ) -> AcceptedObservation:
        """Feed a measurement into the Kalman filter and return the result."""
        self._last_frame_id = obs.frame_id
        rms = obs.pose.reprojection_rms_px if math.isfinite(obs.pose.reprojection_rms_px) else 2.0

        filtered_pos = self._tracker.update(
            measurement=obs.pose.translation_m,
            measurement_noise_px=rms,
            now_sec=now_sec,
        )

        result = AcceptedObservation(
            acquisition_sec=obs.acquisition_sec,
            source_sequence=obs.source_sequence,
            frame_id=obs.frame_id,
            target_revision=obs.target_revision,
            line_width_m=obs.line_width_m,
            pose=PoseEstimate(
                rotation_vector=obs.pose.rotation_vector,
                translation_m=filtered_pos,
                reprojection_rms_px=obs.pose.reprojection_rms_px,
                candidate_count=obs.pose.candidate_count,
                inlier_count=obs.pose.inlier_count,
                quality=obs.pose.quality,
                covariance=obs.pose.covariance,
            ),
        )
        self._last_obs = result
        return result

    def _predict(self, now_sec: float) -> AcceptedObservation | None:
        """Extrapolate the target position from the Kalman filter state.

        Returns ``None`` when the tracker has never been initialised or
        the prediction is too stale to trust.
        """
        if not self._tracker.is_fresh:
            return None

        predicted_pos = self._tracker.predict(now_sec)
        velocity = self._tracker.velocity

        src = self._last_obs
        if src is None:
            return None

        return AcceptedObservation(
            acquisition_sec=now_sec,
            source_sequence=src.source_sequence,
            frame_id=self._last_frame_id or src.frame_id,
            target_revision=src.target_revision,
            line_width_m=src.line_width_m,
            pose=PoseEstimate(
                rotation_vector=src.pose.rotation_vector,
                translation_m=predicted_pos,
                reprojection_rms_px=src.pose.reprojection_rms_px,
                candidate_count=0,
                inlier_count=0,
                quality=max(src.quality * 0.5, 0.1),
                covariance=src.pose.covariance,
            ),
        )


class TargetObservationNode(Node):
    """Dual-camera target observation node with fusion support."""

    def __init__(
        self, *, steady_clock: Callable[[], float] = time.monotonic
    ) -> None:
        super().__init__("target_observation_node")
        self.declare_parameter("target_revision", "d2026-apriltag-v1")
        self.declare_parameter("max_reprojection_rms_px", 2.0)
        self.declare_parameter("fusion_ema_alpha", 0.6)
        self.declare_parameter("kalman_process_noise_pos", 0.05)
        self.declare_parameter("kalman_process_noise_vel", 0.3)
        self.declare_parameter("kalman_max_predict_age_sec", 0.5)
        self.declare_parameter("last_candidate_count", 0)
        self.declare_parameter("last_reprojection_rms_px", -1.0)
        self.declare_parameter("last_quality", 0.0)
        self.declare_parameter("last_reject_reason", "not_observed")
        self.declare_parameter("last_fusion_source", "none")
        self._steady_clock = steady_clock
        self._bridge = CvBridge()

        # Per-camera state
        self._cameras: dict[str, _CameraState] = {
            "narrow": _CameraState(),
            "wide": _CameraState(),
        }

        # Shared vehicle state
        self._vehicle: VehicleTelemetry | None = None
        self._vehicle_receipt_steady_sec = float("-inf")
        self._vehicle_reason = RejectReason.STALE_VEHICLE
        self._last_vehicle_sequence: int | None = None
        self._last_vehicle_acquisition_sec: float | None = None

        self._sequence = 0
        self._last_result: ObservationResult | None = None

        # Dual-camera fusion engine with Kalman filter
        self._fusion = _DualCameraFusion(
            process_noise_pos=float(self.get_parameter("kalman_process_noise_pos").value),
            process_noise_vel=float(self.get_parameter("kalman_process_noise_vel").value),
            max_predict_age_sec=float(self.get_parameter("kalman_max_predict_age_sec").value),
        )

        # ── Subscriptions ───────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            "/camera/narrow/camera_info",
            lambda msg: self._camera_info_callback(msg, "narrow"),
            CAMERA_INFO_QOS,
        )
        self.create_subscription(
            Image,
            "/camera/narrow/image_raw",
            lambda msg: self._image_callback(msg, "narrow"),
            VEHICLE_QOS,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/wide/camera_info",
            lambda msg: self._camera_info_callback(msg, "wide"),
            CAMERA_INFO_QOS,
        )
        self.create_subscription(
            Image,
            "/camera/wide/image_raw",
            lambda msg: self._image_callback(msg, "wide"),
            VEHICLE_QOS,
        )
        self.create_subscription(
            VehicleTelemetry,
            VEHICLE_TOPIC,
            self._vehicle_callback,
            VEHICLE_QOS,
        )

        # ── Publishers ──────────────────────────────────────────────────
        self._publisher = self.create_publisher(
            TargetObservation, "/d_task/target_observation", CAMERA_INFO_QOS
        )
        self._annotated_publisher = self.create_publisher(
            Image, ANNOTATED_IMAGE_TOPIC, CAMERA_INFO_QOS
        )

        self.get_logger().info(
            "TargetObservationNode started — dual-camera fusion enabled"
        )

    # ── Properties ──────────────────────────────────────────────────────

    @property
    def last_result(self) -> ObservationResult | None:
        return self._last_result

    @property
    def vehicle_topic(self) -> str:
        return VEHICLE_TOPIC

    # ── Camera info ─────────────────────────────────────────────────────

    def _camera_info_callback(self, message: CameraInfo, role: str) -> None:
        self._cameras[role].info = message

    def _camera_callback(self, message: CameraInfo, role: str = "narrow") -> None:
        """Compatibility entry point for callers using the original callback name."""
        self._camera_info_callback(message, role)

    # ── Vehicle telemetry ───────────────────────────────────────────────

    def _vehicle_callback(self, message: VehicleTelemetry) -> None:
        receipt = self._steady_clock()
        reason = validate_vehicle(
            message,
            self._last_vehicle_sequence,
            self._last_vehicle_acquisition_sec,
        )
        if reason is not None:
            if self._vehicle is not None:
                self.get_logger().warn(f"Vehicle telemetry rejected: {reason.value}")
            self._vehicle = None
            self._vehicle_reason = reason
            return
        if self._vehicle is None:
            self.get_logger().info("Vehicle telemetry accepted — now valid")
        self._vehicle = message
        self._vehicle_reason = RejectReason.STALE_VEHICLE
        self._vehicle_receipt_steady_sec = receipt
        self._last_vehicle_sequence = int(message.source_sequence)
        self._last_vehicle_acquisition_sec = stamp_seconds(
            message.acquisition_stamp
        )

    # ── Image processing (per-camera) ───────────────────────────────────

    def _image_callback(self, message: Image, role: str) -> None:
        cam = self._cameras[role]
        image_receipt = self._steady_clock()

        acquisition_sec = stamp_seconds(message.header.stamp)

        # Reject out-of-order frames *per camera*
        if (
            cam.last_acquisition_sec is not None
            and acquisition_sec <= cam.last_acquisition_sec
        ):
            self._publish_early_rejection(
                RejectReason.IMAGE_ACQUISITION_REGRESSION, message, role
            )
            return
        cam.last_acquisition_sec = acquisition_sec

        if cam.info is None:
            self.get_logger().warn(f"[{role}] No camera_info yet — rejecting frame")
            self._publish_early_rejection(RejectReason.UNCALIBRATED, message, role)
            return
        if self._vehicle is None:
            self.get_logger().warn(f"[{role}] No valid vehicle telemetry (reason={self._vehicle_reason.value}) — rejecting frame")
            self._publish_early_rejection(self._vehicle_reason, message, role)
            return

        binding_reason = validate_camera_binding(cam.info, message)
        if binding_reason is not None:
            self.get_logger().warn(
                f"[{role}] Camera binding rejected: {binding_reason.value} "
                f"(info={cam.info.width}x{cam.info.height} frame={cam.info.header.frame_id} "
                f"image={message.width}x{message.height} frame={message.header.frame_id})"
            )
            self._publish_early_rejection(binding_reason, message, role)
            return

        try:
            decoded = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError:
            self._publish_early_rejection(RejectReason.INVALID_INPUT, message, role)
            return

        self.get_logger().info(
            f"[{role}] Image decoded: {decoded.shape[1]}x{decoded.shape[0]} "
            f"min={decoded.min()} max={decoded.max()} mean={decoded.mean():.0f}"
        )

        # Build per-camera request
        cam.sequence += 1
        info = cam.info
        matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        vehicle = self._vehicle

        camera_model = CameraModel(
            matrix,
            np.asarray(info.d, dtype=np.float64),
            int(info.width),
            int(info.height),
            message.header.frame_id,
            bool(matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0),
        )

        request = ObservationRequest(
            decoded,
            camera_model,
            FrameContext(
                acquisition_sec,
                image_receipt,
                self._steady_clock(),
                cam.sequence,
                str(self.get_parameter("target_revision").value),
            ),
            MotionContext(
                stamp_seconds(vehicle.acquisition_stamp),
                self._vehicle_receipt_steady_sec,
                int(vehicle.turn_class),
                float(vehicle.heading_rad),
                float(vehicle.yaw_rate_rad_s),
                float(vehicle.wheel_speed_m_s),
                cam.prior,
            ),
            PoseLimits(
                max_reprojection_rms_px=float(
                    self.get_parameter("max_reprojection_rms_px").value
                )
            ),
        )

        result = observe_target(request)

        if isinstance(result, AcceptedObservation):
            self.get_logger().info(
                f"[{role}] TAG DETECTED! quality={result.pose.quality:.3f} "
                f"t=({result.pose.translation_m[0]:.2f},{result.pose.translation_m[1]:.2f},{result.pose.translation_m[2]:.2f})"
            )
        else:
            # Save image and run direct detection for debugging
            import cv2
            gray = decoded if decoded.ndim == 2 else cv2.cvtColor(decoded, cv2.COLOR_BGR2GRAY)
            
            # Save raw image periodically
            if cam.sequence % 30 == 1:
                import os
                save_dir = '/workspace/target_node_frames'
                os.makedirs(save_dir, exist_ok=True)
                cv2.imwrite(f'{save_dir}/{role}_{cam.sequence:04d}.png', decoded)
                self.get_logger().info(f"[{role}] Saved frame {cam.sequence} to {save_dir}")
            
            # Direct ArUco test with exact same params as camera_debug
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            p = cv2.aruco.DetectorParameters_create()
            p.adaptiveThreshWinSizeMin = 3
            p.adaptiveThreshWinSizeMax = 201
            p.adaptiveThreshWinSizeStep = 4
            p.adaptiveThreshConstant = 3
            p.minMarkerPerimeterRate = 0.03
            p.maxMarkerPerimeterRate = 4.0
            p.polygonalApproxAccuracyRate = 0.05
            p.minCornerDistanceRate = 0.05
            p.minDistanceToBorder = 0
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            corners, ids, rej = cv2.aruco.detectMarkers(gray, d, parameters=p)
            direct_found = ids is not None and len(ids) > 0
            self.get_logger().info(
                f"[{role}] Pipeline: {result.reject_reason.value} | Direct ArUco: "
                f"{'FOUND id=' + str(ids.flatten()) if direct_found else f'NOT FOUND rej={len(rej)}'} | "
                f"img={decoded.shape[1]}x{decoded.shape[0]} min={decoded.min()} max={decoded.max()} mean={decoded.mean():.0f}"
            )

        # Update per-camera prior (in camera frame, for motion-prior gating)
        if isinstance(result, AcceptedObservation):
            cam.prior = PosePrior(
                result.pose.translation_m.copy(),
                result.pose.rotation_vector.copy(),
                result.acquisition_sec,
                image_receipt,
            )

        # Attempt fusion and publish
        self._try_publish(result, role, decoded, message)

    # ── Fusion + publishing ─────────────────────────────────────────────

    def _try_publish(
        self, new_result: ObservationResult, role: str, decoded: np.ndarray,
        source_message: Image,
    ) -> None:
        """Collect the latest result from each camera and publish fused output."""
        now_sec = self._steady_clock()

        narrow_acc: AcceptedObservation | None = None
        wide_acc: AcceptedObservation | None = None

        if isinstance(new_result, AcceptedObservation):
            if role == "narrow":
                narrow_acc = new_result
                if self._cameras["wide"].prior is not None:
                    wide_acc = self._prior_to_accepted(
                        self._cameras["wide"].prior, "wide"
                    )
            else:
                wide_acc = new_result
                if self._cameras["narrow"].prior is not None:
                    narrow_acc = self._prior_to_accepted(
                        self._cameras["narrow"].prior, "narrow"
                    )
        else:
            # Current detection failed — check the other camera's prior
            if role == "narrow" and self._cameras["wide"].prior is not None:
                wide_acc = self._prior_to_accepted(
                    self._cameras["wide"].prior, "wide"
                )
            elif role == "wide" and self._cameras["narrow"].prior is not None:
                narrow_acc = self._prior_to_accepted(
                    self._cameras["narrow"].prior, "narrow"
                )

        fused = self._fusion.fuse(narrow_acc, wide_acc, now_sec)

        if fused is None:
            # Neither camera has a valid detection — publish a typed rejection
            # so downstream consumers (tests, diagnostics) can observe it.
            self._sequence += 1
            self._publish_rejection(
                (
                    new_result.reject_reason
                    if isinstance(new_result, RejectedObservation)
                    else RejectReason.PARTIAL_GEOMETRY
                ),
                role,
                source_message,
            )
            return

        # Build and publish fused TargetObservation message
        self._sequence += 1
        self._last_result = fused
        message_out = self._build_target_observation(fused)

        # Publish annotated image from the primary camera (before the
        # observation so tests that replace _annotated_publisher see it first).
        try:
            cam_info = self._cameras[role].info
            cam_model = None
            if cam_info is not None:
                matrix = np.asarray(cam_info.k, dtype=np.float64).reshape(3, 3)
                cam_model = CameraModel(
                    matrix,
                    np.asarray(cam_info.d, dtype=np.float64),
                    int(cam_info.width),
                    int(cam_info.height),
                    f"camera_{role}_optical_frame",
                    bool(matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0),
                )
            annotation = render_target_observation(
                AnnotationFrame(decoded, cam_model), fused
            )
            ann_msg = self._bridge.cv2_to_imgmsg(annotation, encoding="bgr8")
            ann_msg.header = source_message.header
            self._annotated_publisher.publish(ann_msg)
        except Exception:
            pass  # annotation is best-effort

        self._publisher.publish(message_out)

        # Record diagnostics
        import rclpy.parameter
        if fused.candidate_count == 0:
            source = "predicted"
        elif narrow_acc and wide_acc:
            source = "dual"
        else:
            source = role
        self.set_parameters([
            rclpy.parameter.Parameter(
                "last_candidate_count",
                rclpy.parameter.Parameter.Type.INTEGER,
                fused.candidate_count,
            ),
            rclpy.parameter.Parameter(
                "last_reprojection_rms_px",
                rclpy.parameter.Parameter.Type.DOUBLE,
                float(fused.pose.reprojection_rms_px),
            ),
            rclpy.parameter.Parameter(
                "last_quality",
                rclpy.parameter.Parameter.Type.DOUBLE,
                float(fused.quality),
            ),
            rclpy.parameter.Parameter(
                "last_reject_reason",
                rclpy.parameter.Parameter.Type.STRING,
                "",
            ),
            rclpy.parameter.Parameter(
                "last_fusion_source",
                rclpy.parameter.Parameter.Type.STRING,
                source,
            ),
        ])

    def _prior_to_accepted(self, prior: PosePrior, role: str) -> AcceptedObservation:
        """Build a synthetic AcceptedObservation from a stored prior.

        Allows the fusion engine to use a stale-but-valid detection from one
        camera while the other camera produced a fresh result.  Quality is
        deliberately lowered so the fresh result dominates in weighted
        averaging.
        """
        return AcceptedObservation(
            acquisition_sec=prior.acquisition_sec,
            source_sequence=0,
            frame_id=f"camera_{role}_optical_frame",
            target_revision=str(
                self.get_parameter("target_revision").value
            ),
            line_width_m=0.0,
            pose=PoseEstimate(
                rotation_vector=prior.rotation_vector.copy(),
                translation_m=prior.translation_m.copy(),
                reprojection_rms_px=2.0,
                candidate_count=0,
                inlier_count=0,
                quality=0.3,
                covariance=tuple([0.0] * 36),
            ),
        )

    def _publish_early_rejection(
        self, reason: RejectReason, source_message: Image, role: str
    ) -> None:
        self._sequence += 1
        self._publish_rejection(reason, role, source_message)

    def _publish_rejection(
        self,
        reason: RejectReason,
        role: str,
        source_message: Image | None = None,
    ) -> None:
        """Publish a typed rejection observation for diagnostics."""
        msg = TargetObservation()
        msg.contract_version = TargetObservation.CONTRACT_VERSION
        msg.acquisition_stamp = (
            source_message.header.stamp
            if source_message is not None
            else self.get_clock().now().to_msg()
        )
        msg.source_sequence = self._sequence
        msg.observation_id = f"target-fused-{self._sequence}"
        msg.target_revision = str(
            self.get_parameter("target_revision").value
        )
        msg.frame_id = (
            source_message.header.frame_id
            if source_message is not None and source_message.header.frame_id
            else f"camera_{role}_optical_frame"
        )
        msg.candidate_count = 0
        msg.reprojection_rms_px = -1.0
        msg.outer_diameter_m = 0.50
        msg.inner_diameter_m = 0.30
        msg.line_width_m = 0.020
        msg.valid = False
        msg.status = TargetObservation.STATUS_REJECTED
        msg.confidence = 0.0
        msg.quality = 0.0
        msg.rejection_reason = reason.value
        self._publisher.publish(msg)

        self._last_result = RejectedObservation(
            acquisition_sec=float(msg.acquisition_stamp.sec)
            + float(msg.acquisition_stamp.nanosec) * 1e-9,
            source_sequence=self._sequence,
            frame_id=msg.frame_id,
            target_revision=msg.target_revision,
            reject_reason=reason,
        )

        import rclpy.parameter
        self.set_parameters([
            rclpy.parameter.Parameter(
                "last_reject_reason",
                rclpy.parameter.Parameter.Type.STRING,
                reason.value,
            ),
        ])

    def _build_target_observation(
        self, obs: AcceptedObservation
    ) -> TargetObservation:
        """Build a TargetObservation ROS message from a fused observation."""
        message = TargetObservation()
        message.contract_version = TargetObservation.CONTRACT_VERSION
        message.acquisition_stamp = _float_to_time(obs.acquisition_sec)
        message.source_sequence = obs.source_sequence
        message.observation_id = f"target-fused-{self._sequence}"
        message.target_revision = obs.target_revision
        message.frame_id = obs.frame_id
        message.candidate_count = obs.candidate_count
        rms = obs.pose.reprojection_rms_px
        message.reprojection_rms_px = rms if math.isfinite(rms) else -1.0
        message.outer_diameter_m = 0.50
        message.inner_diameter_m = 0.30
        message.line_width_m = obs.line_width_m or 0.020

        message.valid = True
        message.status = TargetObservation.STATUS_VALID
        message.pose.pose.position.x = float(obs.pose.translation_m[0])
        message.pose.pose.position.y = float(obs.pose.translation_m[1])
        message.pose.pose.position.z = float(obs.pose.translation_m[2])

        if cv2 is not None:
            rotation, _ = cv2.Rodrigues(obs.pose.rotation_vector)
            message.pose.pose.orientation = _rotation_to_quaternion(rotation)

        message.pose.covariance = list(obs.pose.covariance)
        message.confidence = obs.quality
        message.quality = obs.quality
        message.rejection_reason = ""
        return message


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _float_to_time(sec: float) -> object:
    """Convert seconds to a ROS builtin_interfaces Time message."""
    from builtin_interfaces.msg import Time
    whole = int(sec)
    nanosec = int((sec - whole) * 1e9)
    return Time(sec=whole, nanosec=nanosec)


def _rotation_to_quaternion(rotation: np.ndarray) -> Quaternion:
    """Convert a 3×3 rotation matrix to a geometry_msgs Quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return Quaternion(
            x=float((rotation[2, 1] - rotation[1, 2]) * s),
            y=float((rotation[0, 2] - rotation[2, 0]) * s),
            z=float((rotation[1, 0] - rotation[0, 1]) * s),
            w=float(0.25 / s),
        )
    diagonal = [float(rotation[i, i]) for i in range(3)]
    axis = diagonal.index(max(diagonal))
    nxt = (axis + 1) % 3
    lst = (axis + 2) % 3
    s = 2.0 * math.sqrt(1.0 + diagonal[axis] - diagonal[nxt] - diagonal[lst])
    if s <= 1e-12:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    q = [0.0, 0.0, 0.0, 0.0]
    q[axis] = s / 4.0
    q[nxt] = float((rotation[nxt, axis] + rotation[axis, nxt]) / s)
    q[lst] = float((rotation[lst, axis] + rotation[axis, lst]) / s)
    q[3] = float((rotation[lst, nxt] - rotation[nxt, lst]) / s)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


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
