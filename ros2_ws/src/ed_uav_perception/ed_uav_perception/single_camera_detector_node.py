"""单相机独立检测节点

独立运行的AprilTag检测节点，支持：
- 单相机独立检测
- 发布检测结果、标注图像、诊断信息
- 多线程处理
- 动态分辨率调整
"""

from __future__ import annotations

import os
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.msg import (
    PerceptionDiagnostics,
    TargetObservation,
    VehicleTelemetry,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ed_uav_perception.apriltag_detector import AprilTagDetector
from ed_uav_perception.kalman_tracker import KalmanTracker
from ed_uav_perception.target_annotation import AnnotationFrame, render_target_observation
from ed_uav_perception.target_input import validate_camera_binding, validate_vehicle
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


class SingleCameraDetectorNode(Node):
    """单相机独立检测节点"""

    def __init__(self, camera_role: str):
        super().__init__(f'{camera_role}_detector')
        self.camera_role = camera_role
        self.bridge = CvBridge()

        # 参数声明
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('target_revision', 'd2026-apriltag-v1')
        self.declare_parameter('max_reprojection_rms_px', 2.0)
        self.declare_parameter('enable_recording', False)
        self.declare_parameter('recording_dir', 'debug_recordings')
        self.declare_parameter('min_fps_threshold', 10.0)
        self.declare_parameter('adaptive_resolution', True)

        # QoS: BEST_EFFORT 对齐硬件
        sensor_qos = qos_profile_sensor_data

        # 订阅
        self.create_subscription(
            Image,
            f'/camera/{camera_role}/image_raw',
            self._image_callback,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            f'/camera/{camera_role}/camera_info',
            self._camera_info_callback,
            sensor_qos,
        )
        self.create_subscription(
            VehicleTelemetry,
            '/d_task/vehicle/telemetry',
            self._vehicle_callback,
            sensor_qos,
        )

        # 发布
        self.detection_pub = self.create_publisher(
            TargetObservation,
            f'/perception/{camera_role}/detection',
            sensor_qos,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            f'/perception/{camera_role}/annotated_image',
            sensor_qos,
        )
        self.diagnostics_pub = self.create_publisher(
            PerceptionDiagnostics,
            f'/perception/{camera_role}/diagnostics',
            sensor_qos,
        )

        # 检测器和跟踪器
        self.detector = AprilTagDetector()
        self.tracker = KalmanTracker()

        # 状态
        self.camera_info: CameraInfo | None = None
        self.camera_model: CameraModel | None = None
        self.vehicle: VehicleTelemetry | None = None
        self.prior: PosePrior | None = None

        # 统计
        self.frame_count = 0
        self.detection_count = 0
        self.rejection_count = 0
        self.last_reject_reason = 'not_observed'
        self.last_quality = 0.0

        # FPS计算
        self._fps_history: list[float] = []
        self._last_frame_time = time.monotonic()

        # 多线程执行器
        self._executor = ThreadPoolExecutor(max_workers=2)

        # 录制器
        self._recorder: cv2.VideoWriter | None = None
        self._recording_enabled = False

        self.get_logger().info(
            f'{camera_role} detector started - tag_id={self.get_parameter("target_tag_id").value}'
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """相机信息回调"""
        self.camera_info = msg
        matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.camera_model = CameraModel(
            matrix,
            np.asarray(msg.d, dtype=np.float64),
            int(msg.width),
            int(msg.height),
            msg.header.frame_id,
            bool(matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0),
        )

    def _vehicle_callback(self, msg: VehicleTelemetry) -> None:
        """车辆遥测回调"""
        self.vehicle = msg

    def _image_callback(self, msg: Image) -> None:
        """图像回调"""
        start_time = time.monotonic()
        self.frame_count += 1

        # 计算FPS
        current_time = time.monotonic()
        dt = current_time - self._last_frame_time
        if dt > 0:
            fps = 1.0 / dt
            self._fps_history.append(fps)
            if len(self._fps_history) > 30:
                self._fps_history.pop(0)
        self._last_frame_time = current_time

        # 前置检查
        if self.camera_info is None:
            self._publish_rejection('no_camera_info', msg)
            return
        if self.camera_model is None:
            self._publish_rejection('no_camera_model', msg)
            return
        if self.vehicle is None:
            self._publish_rejection('no_vehicle', msg)
            return

        # 解码图像
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            self._publish_rejection('decode_error', msg)
            return

        # 检测
        result = self._detect(image, msg)

        # 处理结果
        if isinstance(result, AcceptedObservation):
            self._handle_detection(result, image, msg, start_time)
        else:
            self._handle_rejection(result, image, msg, start_time)

    def _detect(self, image: np.ndarray, msg: Image) -> ObservationResult:
        """执行检测"""
        # 构建请求
        acquisition_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        receipt_steady_sec = time.monotonic()

        frame_ctx = FrameContext(
            acquisition_sec=acquisition_sec,
            receipt_steady_sec=receipt_steady_sec,
            evaluation_steady_sec=time.monotonic(),
            source_sequence=self.frame_count,
            target_revision=str(self.get_parameter('target_revision').value),
        )

        vehicle = self.vehicle
        motion_ctx = MotionContext(
            acquisition_sec=vehicle.acquisition_stamp.sec + vehicle.acquisition_stamp.nanosec * 1e-9,
            receipt_steady_sec=receipt_steady_sec,
            turn_class=int(vehicle.turn_class),
            heading_rad=float(vehicle.heading_rad),
            yaw_rate_rad_s=float(vehicle.yaw_rate_rad_s),
            speed_m_s=float(vehicle.wheel_speed_m_s),
            prior=self.prior,
        )

        limits = PoseLimits(
            max_reprojection_rms_px=float(self.get_parameter('max_reprojection_rms_px').value),
        )

        request = ObservationRequest(
            image=image,
            camera=self.camera_model,
            frame=frame_ctx,
            motion=motion_ctx,
            limits=limits,
        )

        return observe_target(request)

    def _handle_detection(
        self,
        result: AcceptedObservation,
        image: np.ndarray,
        msg: Image,
        start_time: float,
    ) -> None:
        """处理检测成功"""
        self.detection_count += 1
        self.last_quality = result.quality

        # 更新跟踪器
        self.tracker.update(result.pose.translation_m, result.quality)

        # 更新prior
        self.prior = PosePrior(
            translation_m=result.pose.translation_m.copy(),
            rotation_vector=result.pose.rotation_vector.copy(),
            acquisition_sec=result.acquisition_sec,
            receipt_steady_sec=time.monotonic(),
        )

        # 发布检测结果
        detection_msg = self._build_detection_msg(result, msg)
        self.detection_pub.publish(detection_msg)

        # 异步发布标注图像
        self._executor.submit(self._publish_annotated, image, result, msg)

        # 发布诊断
        latency_ms = (time.monotonic() - start_time) * 1000
        self._publish_diagnostics(latency_ms, result)

        # 录制
        if self._recording_enabled and self._recorder is not None:
            self._executor.submit(self._record_frame, image)

    def _handle_rejection(
        self,
        result: RejectedObservation,
        image: np.ndarray,
        msg: Image,
        start_time: float,
    ) -> None:
        """处理检测失败"""
        self.rejection_count += 1
        self.last_reject_reason = result.reject_reason.value

        # 发布拒绝结果
        rejection_msg = self._build_rejection_msg(result, msg)
        self.detection_pub.publish(rejection_msg)

        # 异步发布带拒绝原因的标注图像
        self._executor.submit(self._publish_annotated_rejection, image, result, msg)

        # 发布诊断
        latency_ms = (time.monotonic() - start_time) * 1000
        self._publish_diagnostics(latency_ms, result)

    def _build_detection_msg(
        self, result: AcceptedObservation, msg: Image
    ) -> TargetObservation:
        """构建检测消息"""
        out = TargetObservation()
        out.contract_version = TargetObservation.CONTRACT_VERSION
        out.acquisition_stamp = msg.header.stamp
        out.source_sequence = self.frame_count
        out.observation_id = f'target-{self.camera_role}-{self.frame_count}'
        out.target_revision = result.target_revision
        out.frame_id = msg.header.frame_id
        out.candidate_count = result.candidate_count
        out.reprojection_rms_px = result.reprojection_rms_px

        out.outer_diameter_m = 0.50
        out.inner_diameter_m = 0.30
        out.line_width_m = result.line_width_m or 0.020

        out.valid = True
        out.status = TargetObservation.STATUS_VALID

        out.pose.pose.position.x = float(result.pose.translation_m[0])
        out.pose.pose.position.y = float(result.pose.translation_m[1])
        out.pose.pose.position.z = float(result.pose.translation_m[2])

        # 旋转
        rotation, _ = cv2.Rodrigues(result.pose.rotation_vector)
        out.pose.pose.orientation = self._rotation_to_quaternion(rotation)

        out.pose.covariance = list(result.pose.covariance)
        out.confidence = result.quality
        out.quality = result.quality
        out.rejection_reason = ''

        return out

    def _build_rejection_msg(
        self, result: RejectedObservation, msg: Image
    ) -> TargetObservation:
        """构建拒绝消息"""
        out = TargetObservation()
        out.contract_version = TargetObservation.CONTRACT_VERSION
        out.acquisition_stamp = msg.header.stamp
        out.source_sequence = self.frame_count
        out.observation_id = f'target-{self.camera_role}-{self.frame_count}'
        out.target_revision = str(self.get_parameter('target_revision').value)
        out.frame_id = msg.header.frame_id
        out.candidate_count = result.candidate_count
        out.reprojection_rms_px = -1.0

        out.outer_diameter_m = 0.50
        out.inner_diameter_m = 0.30
        out.line_width_m = 0.020

        out.valid = False
        out.status = TargetObservation.STATUS_REJECTED
        out.confidence = 0.0
        out.quality = 0.0
        out.rejection_reason = result.reject_reason.value

        return out

    def _publish_annotated(
        self,
        image: np.ndarray,
        result: AcceptedObservation,
        msg: Image,
    ) -> None:
        """发布标注图像"""
        try:
            annotation = render_target_observation(
                AnnotationFrame(image, self.camera_model), result
            )
            ann_msg = self.bridge.cv2_to_imgmsg(annotation, encoding='bgr8')
            ann_msg.header = msg.header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().debug(f'Annotation failed: {e}')

    def _publish_annotated_rejection(
        self,
        image: np.ndarray,
        result: RejectedObservation,
        msg: Image,
    ) -> None:
        """发布带拒绝原因的标注图像"""
        try:
            overlay = image.copy()

            # 添加红色边框表示失败
            cv2.rectangle(overlay, (0, 0), (overlay.shape[1]-1, overlay.shape[0]-1), (0, 0, 255), 3)

            # 添加拒绝原因文字
            reason = result.reject_reason.value
            cv2.putText(overlay, f'FAILED: {reason}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 添加帧信息
            cv2.putText(overlay, f'Frame: {self.frame_count}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            ann_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            ann_msg.header = msg.header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().debug(f'Rejection annotation failed: {e}')

    def _publish_diagnostics(
        self, latency_ms: float, result: ObservationResult
    ) -> None:
        """发布诊断信息"""
        diag = PerceptionDiagnostics()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.camera_role = self.camera_role

        # FPS
        if len(self._fps_history) > 0:
            diag.fps = sum(self._fps_history) / len(self._fps_history)
        else:
            diag.fps = 0.0

        diag.latency_ms = latency_ms
        diag.frame_count = self.frame_count
        diag.detection_count = self.detection_count
        diag.rejection_count = self.rejection_count

        if isinstance(result, AcceptedObservation):
            diag.last_reject_reason = ''
            diag.quality = result.quality
            diag.reprojection_rms = result.reprojection_rms_px
            diag.translation_m = [
                float(result.pose.translation_m[0]),
                float(result.pose.translation_m[1]),
                float(result.pose.translation_m[2]),
            ]
            diag.is_tracking = True
        else:
            diag.last_reject_reason = result.reject_reason.value
            diag.quality = 0.0
            diag.reprojection_rms = -1.0
            diag.translation_m = [0.0, 0.0, 0.0]
            diag.is_tracking = False

        self.diagnostics_pub.publish(diag)

    def _publish_rejection(self, reason: str, msg: Image) -> None:
        """发布早期拒绝"""
        rejection_msg = TargetObservation()
        rejection_msg.contract_version = TargetObservation.CONTRACT_VERSION
        rejection_msg.acquisition_stamp = msg.header.stamp
        rejection_msg.source_sequence = self.frame_count
        rejection_msg.observation_id = f'target-{self.camera_role}-{self.frame_count}'
        rejection_msg.target_revision = str(self.get_parameter('target_revision').value)
        rejection_msg.frame_id = msg.header.frame_id
        rejection_msg.valid = False
        rejection_msg.status = TargetObservation.STATUS_REJECTED
        rejection_msg.rejection_reason = reason
        self.detection_pub.publish(rejection_msg)

        # 也发布诊断
        diag = PerceptionDiagnostics()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.camera_role = self.camera_role
        diag.fps = 0.0
        diag.latency_ms = 0.0
        diag.frame_count = self.frame_count
        diag.detection_count = self.detection_count
        diag.rejection_count = self.rejection_count + 1
        diag.last_reject_reason = reason
        diag.quality = 0.0
        diag.is_tracking = False
        self.diagnostics_pub.publish(diag)

    def toggle_recording(self) -> None:
        """切换录制状态"""
        if self._recording_enabled:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self) -> None:
        """开始录制"""
        if self._recording_enabled:
            return

        recording_dir = self.get_parameter('recording_dir').value
        os.makedirs(recording_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(recording_dir, f'{self.camera_role}_{timestamp}.avi')

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 15.0  # 目标帧率
        frame_size = (1280, 960) if self.camera_role == 'narrow' else (1280, 720)

        self._recorder = cv2.VideoWriter(filename, fourcc, fps, frame_size)
        self._recording_enabled = True
        self.get_logger().info(f'Recording started: {filename}')

    def stop_recording(self) -> None:
        """停止录制"""
        if not self._recording_enabled:
            return

        if self._recorder is not None:
            self._recorder.release()
            self._recorder = None

        self._recording_enabled = False
        self.get_logger().info('Recording stopped')

    def _record_frame(self, image: np.ndarray) -> None:
        """录制帧"""
        if self._recorder is not None and self._recording_enabled:
            try:
                self._recorder.write(image)
            except Exception as e:
                self.get_logger().debug(f'Recording failed: {e}')

    @staticmethod
    def _rotation_to_quaternion(rotation: np.ndarray):
        """旋转矩阵转四元数"""
        from geometry_msgs.msg import Quaternion

        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
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
        s = 2.0 * np.sqrt(1.0 + diagonal[axis] - diagonal[nxt] - diagonal[lst])
        if s <= 1e-12:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        q = [0.0, 0.0, 0.0, 0.0]
        q[axis] = s / 4.0
        q[nxt] = float((rotation[nxt, axis] + rotation[axis, nxt]) / s)
        q[lst] = float((rotation[lst, axis] + rotation[axis, lst]) / s)
        q[3] = float((rotation[lst, nxt] - rotation[nxt, lst]) / s)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def destroy_node(self):
        """销毁节点"""
        self.stop_recording()
        self._executor.shutdown(wait=False)
        super().destroy_node()


def main(args=None, camera_role='narrow'):
    """主函数"""
    rclpy.init(args=args)
    node = SingleCameraDetectorNode(camera_role)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
