"""感知可视化调试节点

提供三个窗口显示：
- 窄相机检测结果
- 广角检测结果
- 融合状态

功能：
- 实时覆盖层：FPS、延迟、检测状态
- 录制功能：按r键开始/停止
- 截图功能：按s键保存
- 退出功能：按q键退出
"""

from __future__ import annotations

import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.msg import (
    FusionDiagnostics,
    PerceptionDiagnostics,
    TargetObservation,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DebugRecorder:
    """调试录制器"""

    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.recording = False
        self.narrow_writer: cv2.VideoWriter | None = None
        self.wide_writer: cv2.VideoWriter | None = None
        self.start_time: datetime | None = None

    def toggle_recording(self) -> bool:
        """切换录制状态"""
        if self.recording:
            self.stop_recording()
            return False
        else:
            self.start_recording()
            return True

    def start_recording(self) -> None:
        """开始录制"""
        if self.recording:
            return

        self.start_time = datetime.now()
        timestamp = self.start_time.strftime('%Y%m%d_%H%M%S')

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 15.0

        narrow_path = os.path.join(self.output_dir, f'narrow_{timestamp}.avi')
        wide_path = os.path.join(self.output_dir, f'wide_{timestamp}.avi')

        self.narrow_writer = cv2.VideoWriter(narrow_path, fourcc, fps, (640, 480))
        self.wide_writer = cv2.VideoWriter(wide_path, fourcc, fps, (640, 480))

        self.recording = True
        print(f'Recording started: {self.output_dir}')

    def stop_recording(self) -> None:
        """停止录制"""
        if not self.recording:
            return

        if self.narrow_writer is not None:
            self.narrow_writer.release()
            self.narrow_writer = None

        if self.wide_writer is not None:
            self.wide_writer.release()
            self.wide_writer = None

        self.recording = False
        duration = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        print(f'Recording stopped. Duration: {duration:.1f}s')

    def record_frames(
        self, narrow: np.ndarray | None, wide: np.ndarray | None
    ) -> None:
        """录制帧"""
        if not self.recording:
            return

        if narrow is not None and self.narrow_writer is not None:
            try:
                resized = cv2.resize(narrow, (640, 480))
                self.narrow_writer.write(resized)
            except Exception:
                pass

        if wide is not None and self.wide_writer is not None:
            try:
                resized = cv2.resize(wide, (640, 480))
                self.wide_writer.write(resized)
            except Exception:
                pass

    def save_screenshot(
        self, narrow: np.ndarray | None, wide: np.ndarray | None
    ) -> None:
        """保存截图"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        if narrow is not None:
            path = os.path.join(self.output_dir, f'narrow_{timestamp}.png')
            cv2.imwrite(path, narrow)
            print(f'Screenshot saved: {path}')

        if wide is not None:
            path = os.path.join(self.output_dir, f'wide_{timestamp}.png')
            cv2.imwrite(path, wide)
            print(f'Screenshot saved: {path}')


class PerceptionVisualizerNode(Node):
    """感知可视化调试节点"""

    def __init__(self):
        super().__init__('perception_visualizer')

        self.bridge = CvBridge()

        # 参数
        self.declare_parameter('recording_dir', 'debug_recordings')
        self.declare_parameter('window_scale', 0.75)

        recording_dir = self.get_parameter('recording_dir').value
        self.recorder = DebugRecorder(recording_dir)

        # 状态
        self.narrow_image: np.ndarray | None = None
        self.wide_image: np.ndarray | None = None
        self.narrow_diag: PerceptionDiagnostics | None = None
        self.wide_diag: PerceptionDiagnostics | None = None
        self.fusion_diag: FusionDiagnostics | None = None

        # 订阅
        self.create_subscription(
            Image,
            '/perception/narrow/annotated_image',
            self._narrow_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/perception/wide/annotated_image',
            self._wide_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PerceptionDiagnostics,
            '/perception/narrow/diagnostics',
            self._narrow_diag_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PerceptionDiagnostics,
            '/perception/wide/diagnostics',
            self._wide_diag_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FusionDiagnostics,
            '/perception/fusion/diagnostics',
            self._fusion_diag_callback,
            qos_profile_sensor_data,
        )

        # 创建窗口
        cv2.namedWindow('Narrow Camera', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Wide Camera', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Fusion Status', cv2.WINDOW_NORMAL)

        # 设置窗口大小
        scale = float(self.get_parameter('window_scale').value)
        cv2.resizeWindow('Narrow Camera', int(640 * scale), int(480 * scale))
        cv2.resizeWindow('Wide Camera', int(640 * scale), int(480 * scale))
        cv2.resizeWindow('Fusion Status', int(400 * scale), int(300 * scale))

        # 定时器用于更新显示（15fps以降低CPU负载）
        self.create_timer(0.067, self._update_display)  # ~15fps

        self.get_logger().info(
            'Perception Visualizer started\n'
            '  Press "r" to toggle recording\n'
            '  Press "s" to save screenshot\n'
            '  Press "q" to quit'
        )

    def _narrow_image_callback(self, msg: Image) -> None:
        """窄相机图像回调"""
        try:
            self.narrow_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            pass

    def _wide_image_callback(self, msg: Image) -> None:
        """广角图像回调"""
        try:
            self.wide_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            pass

    def _narrow_diag_callback(self, msg: PerceptionDiagnostics) -> None:
        """窄相机诊断回调"""
        self.narrow_diag = msg

    def _wide_diag_callback(self, msg: PerceptionDiagnostics) -> None:
        """广角诊断回调"""
        self.wide_diag = msg

    def _fusion_diag_callback(self, msg: FusionDiagnostics) -> None:
        """融合诊断回调"""
        self.fusion_diag = msg

    def _update_display(self) -> None:
        """更新显示"""
        # 显示尺寸限制（提高性能）
        display_width = 640

        # 窄相机窗口
        if self.narrow_image is not None:
            overlay = self._add_camera_overlay(
                self.narrow_image.copy(), self.narrow_diag, 'Narrow'
            )
            # 缩放以提高显示性能
            h, w = overlay.shape[:2]
            if w > display_width:
                scale = display_width / w
                overlay = cv2.resize(overlay, (display_width, int(h * scale)))
            cv2.imshow('Narrow Camera', overlay)

        # 广角窗口
        if self.wide_image is not None:
            overlay = self._add_camera_overlay(
                self.wide_image.copy(), self.wide_diag, 'Wide'
            )
            # 缩放以提高显示性能
            h, w = overlay.shape[:2]
            if w > display_width:
                scale = display_width / w
                overlay = cv2.resize(overlay, (display_width, int(h * scale)))
            cv2.imshow('Wide Camera', overlay)

        # 融合状态窗口
        status = self._create_fusion_status()
        cv2.imshow('Fusion Status', status)

        # 录制（使用原始分辨率）
        self.recorder.record_frames(self.narrow_image, self.wide_image)

        # 处理按键
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            recording = self.recorder.toggle_recording()
            self.get_logger().info(f'Recording: {"ON" if recording else "OFF"}')
        elif key == ord('s'):
            self.recorder.save_screenshot(self.narrow_image, self.wide_image)
        elif key == ord('q'):
            self.get_logger().info('Quit requested')
            rclpy.shutdown()

    def _add_camera_overlay(
        self,
        image: np.ndarray,
        diag: PerceptionDiagnostics | None,
        camera_name: str,
    ) -> np.ndarray:
        """添加相机覆盖层"""
        h, w = image.shape[:2]

        # 标题
        cv2.putText(
            image, camera_name, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
        )

        if diag is not None:
            y_offset = 60

            # FPS
            fps_color = (0, 255, 0) if diag.fps >= 10 else (0, 165, 255)
            cv2.putText(
                image, f'FPS: {diag.fps:.1f}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, fps_color, 2
            )
            y_offset += 30

            # 延迟
            latency_color = (0, 255, 0) if diag.latency_ms < 50 else (0, 0, 255)
            cv2.putText(
                image, f'Latency: {diag.latency_ms:.1f}ms', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, latency_color, 2
            )
            y_offset += 30

            # 检测状态
            if diag.is_tracking:
                cv2.putText(
                    image, f'Tracking Q:{diag.quality:.2f}', (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                y_offset += 30

                # 位置
                pos = diag.translation_m
                cv2.putText(
                    image, f'Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})', (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
                )
            else:
                reason = diag.last_reject_reason or 'unknown'
                cv2.putText(
                    image, f'LOST: {reason}', (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
                )

            # 统计
            y_offset = h - 60
            cv2.putText(
                image, f'Frames: {diag.frame_count}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
            )
            y_offset += 20
            cv2.putText(
                image, f'Detected: {diag.detection_count} | Lost: {diag.rejection_count}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
            )

        # 录制指示
        if self.recorder.recording:
            cv2.circle(image, (w - 30, 30), 10, (0, 0, 255), -1)
            cv2.putText(
                image, 'REC', (w - 60, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )

        return image

    def _create_fusion_status(self) -> np.ndarray:
        """创建融合状态窗口"""
        # 创建黑色背景
        status = np.zeros((300, 400, 3), dtype=np.uint8)

        # 标题
        cv2.putText(
            status, 'Fusion Status', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
        )

        y_offset = 60

        if self.fusion_diag is not None:
            # 融合来源
            source = self.fusion_diag.fusion_source
            source_color = {
                'dual': (0, 255, 0),
                'narrow': (0, 255, 255),
                'wide': (255, 255, 0),
                'predicted': (0, 165, 255),
            }.get(source, (200, 200, 200))

            cv2.putText(
                status, f'Source: {source}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, source_color, 2
            )
            y_offset += 30

            # 质量
            cv2.putText(
                status, f'Narrow Q: {self.fusion_diag.narrow_quality:.2f}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
            )
            y_offset += 25
            cv2.putText(
                status, f'Wide Q: {self.fusion_diag.wide_quality:.2f}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
            )
            y_offset += 25
            cv2.putText(
                status, f'Fused Q: {self.fusion_diag.fused_quality:.2f}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
            y_offset += 30

            # Kalman状态
            fresh_color = (0, 255, 0) if self.fusion_diag.is_fresh else (0, 0, 255)
            cv2.putText(
                status, f'Kalman: {"FRESH" if self.fusion_diag.is_fresh else "STALE"}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, fresh_color, 2
            )
            y_offset += 25

            # 连续未检测
            miss_color = (0, 255, 0) if self.fusion_diag.consecutive_misses < 5 else (0, 0, 255)
            cv2.putText(
                status, f'Misses: {self.fusion_diag.consecutive_misses}', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, miss_color, 1
            )
            y_offset += 25

            # 位置
            pos = self.fusion_diag.fused_position_m
            cv2.putText(
                status, f'Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
            )
        else:
            cv2.putText(
                status, 'Waiting for data...', (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 1
            )

        # 按键说明
        y_offset = 260
        cv2.putText(
            status, 'r: Record | s: Screenshot | q: Quit', (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1
        )

        return status

    def destroy_node(self) -> None:
        """销毁节点"""
        self.recorder.stop_recording()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = PerceptionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
