"""双相机融合节点

融合窄相机和广角相机的检测结果，支持：
- 跳变过滤
- Kalman滤波
- 质量加权融合
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from ed_uav_interfaces.msg import (
    FusionDiagnostics,
    TargetObservation,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ed_uav_perception.kalman_tracker import KalmanTracker


class DualCameraFusion:
    """双相机融合引擎"""

    def __init__(self):
        self.tracker = KalmanTracker(
            process_noise_pos=0.05,
            process_noise_vel=0.3,
            max_predict_age_sec=0.5,
        )
        self.last_narrow: TargetObservation | None = None
        self.last_wide: TargetObservation | None = None
        self.consecutive_misses = 0

    def update_narrow(self, msg: TargetObservation) -> None:
        """更新窄相机观测"""
        self.last_narrow = msg

    def update_wide(self, msg: TargetObservation) -> None:
        """更新广角观测"""
        self.last_wide = msg

    def fuse(
        self,
        narrow: TargetObservation | None,
        wide: TargetObservation | None,
    ) -> TargetObservation | None:
        """融合两个相机的检测结果"""
        narrow_valid = narrow is not None and narrow.valid
        wide_valid = wide is not None and wide.valid

        if narrow_valid and wide_valid:
            # 双检测：质量加权融合
            return self._fuse_dual(narrow, wide)
        elif narrow_valid:
            self.consecutive_misses = 0
            return self._fuse_single(narrow, 'narrow')
        elif wide_valid:
            self.consecutive_misses = 0
            return self._fuse_single(wide, 'wide')
        else:
            # 都失败：尝试Kalman预测
            self.consecutive_misses += 1
            return self._predict()

    def _fuse_dual(
        self, narrow: TargetObservation, wide: TargetObservation
    ) -> TargetObservation:
        """融合两个有效检测"""
        self.consecutive_misses = 0

        # 获取位置
        n_pos = np.array([
            narrow.pose.pose.position.x,
            narrow.pose.pose.position.y,
            narrow.pose.pose.position.z,
        ])
        w_pos = np.array([
            wide.pose.pose.position.x,
            wide.pose.pose.position.y,
            wide.pose.pose.position.z,
        ])

        # 质量加权
        n_q = max(narrow.quality, 1e-6)
        w_q = max(wide.quality, 1e-6)
        w_n = n_q / (n_q + w_q)

        # 融合位置
        fused_pos = w_n * n_pos + (1.0 - w_n) * w_pos

        # 更新Kalman
        self.tracker.update(fused_pos, (narrow.quality + wide.quality) / 2.0)

        # 构建融合消息（基于窄相机）
        fused = TargetObservation()
        fused.contract_version = TargetObservation.CONTRACT_VERSION
        fused.acquisition_stamp = narrow.acquisition_stamp
        fused.source_sequence = narrow.source_sequence
        fused.observation_id = f'target-fused-{narrow.source_sequence}'
        fused.target_revision = narrow.target_revision
        fused.frame_id = narrow.frame_id
        fused.candidate_count = narrow.candidate_count + wide.candidate_count
        fused.reprojection_rms_px = min(
            narrow.reprojection_rms_px, wide.reprojection_rms_px
        )

        fused.outer_diameter_m = 0.50
        fused.inner_diameter_m = 0.30
        fused.line_width_m = narrow.line_width_m

        fused.valid = True
        fused.status = TargetObservation.STATUS_VALID

        fused.pose.pose.position.x = float(fused_pos[0])
        fused.pose.pose.position.y = float(fused_pos[1])
        fused.pose.pose.position.z = float(fused_pos[2])
        fused.pose.pose.orientation = narrow.pose.pose.orientation

        fused.pose.covariance = narrow.pose.covariance
        fused.confidence = w_n * narrow.confidence + (1.0 - w_n) * wide.confidence
        fused.quality = w_n * narrow.quality + (1.0 - w_n) * wide.quality
        fused.rejection_reason = ''

        return fused

    def _fuse_single(
        self, msg: TargetObservation, source: str
    ) -> TargetObservation:
        """单相机检测"""
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

        # 更新Kalman
        self.tracker.update(pos, msg.quality)

        return msg

    def _predict(self) -> TargetObservation | None:
        """Kalman预测"""
        if not self.tracker.is_fresh:
            return None

        now_sec = time.monotonic()
        predicted_pos = self.tracker.predict(now_sec)

        # 创建预测消息
        msg = TargetObservation()
        msg.contract_version = TargetObservation.CONTRACT_VERSION
        msg.acquisition_stamp.sec = int(now_sec)
        msg.acquisition_stamp.nanosec = int((now_sec - int(now_sec)) * 1e9)
        msg.source_sequence = 0
        msg.observation_id = 'target-fused-predicted'
        msg.target_revision = 'd2026-apriltag-v1'
        msg.frame_id = 'camera_narrow_optical_frame'
        msg.candidate_count = 0
        msg.reprojection_rms_px = -1.0

        msg.outer_diameter_m = 0.50
        msg.inner_diameter_m = 0.30
        msg.line_width_m = 0.020

        msg.valid = True
        msg.status = TargetObservation.STATUS_VALID

        msg.pose.pose.position.x = float(predicted_pos[0])
        msg.pose.pose.position.y = float(predicted_pos[1])
        msg.pose.pose.position.z = float(predicted_pos[2])

        msg.confidence = 0.3  # 预测置信度较低
        msg.quality = 0.3
        msg.rejection_reason = ''

        return msg

    def get_fusion_source(
        self,
        narrow: TargetObservation | None,
        wide: TargetObservation | None,
    ) -> str:
        """获取融合来源"""
        narrow_valid = narrow is not None and narrow.valid
        wide_valid = wide is not None and wide.valid

        if narrow_valid and wide_valid:
            return 'dual'
        elif narrow_valid:
            return 'narrow'
        elif wide_valid:
            return 'wide'
        else:
            return 'predicted'


class TargetFusionNode(Node):
    """双相机融合节点"""

    def __init__(self):
        super().__init__('target_fusion')

        # 参数
        self.declare_parameter('max_position_jump', 0.5)  # 50cm
        self.declare_parameter('fusion_timeout', 0.2)  # 200ms

        sensor_qos = qos_profile_sensor_data

        # 使用普通订阅（避免message_filters的时间戳问题）
        self.create_subscription(
            TargetObservation,
            '/perception/narrow/detection',
            self._narrow_callback,
            sensor_qos,
        )
        self.create_subscription(
            TargetObservation,
            '/perception/wide/detection',
            self._wide_callback,
            sensor_qos,
        )

        # 发布
        self.fusion_pub = self.create_publisher(
            TargetObservation,
            '/d_task/target_observation',
            sensor_qos,
        )
        self.diagnostics_pub = self.create_publisher(
            FusionDiagnostics,
            '/perception/fusion/diagnostics',
            sensor_qos,
        )

        # 融合引擎
        self.fusion = DualCameraFusion()

        # 跳变过滤
        self.last_valid_position: list[float] | None = None
        self.max_position_jump = float(self.get_parameter('max_position_jump').value)
        self.fusion_timeout = float(self.get_parameter('fusion_timeout').value)

        # 最新消息缓存
        self.last_narrow: TargetObservation | None = None
        self.last_wide: TargetObservation | None = None
        self.last_narrow_time = 0.0
        self.last_wide_time = 0.0

        # 定时器用于融合
        self.create_timer(0.05, self._fusion_timer)  # 20Hz

        self.get_logger().info(
            f'Target fusion started - max_jump={self.max_position_jump}m'
        )

    def _narrow_callback(self, msg: TargetObservation) -> None:
        """窄相机回调"""
        self.last_narrow = msg
        self.last_narrow_time = time.monotonic()

    def _wide_callback(self, msg: TargetObservation) -> None:
        """广角回调"""
        self.last_wide = msg
        self.last_wide_time = time.monotonic()

    def _fusion_timer(self) -> None:
        """定时融合回调"""
        now = time.monotonic()
        
        # 检查消息是否新鲜
        narrow_fresh = (now - self.last_narrow_time) < self.fusion_timeout
        wide_fresh = (now - self.last_wide_time) < self.fusion_timeout
        
        narrow = self.last_narrow if narrow_fresh else None
        wide = self.last_wide if wide_fresh else None
        
        # 如果都没有新鲜消息，跳过
        if narrow is None and wide is None:
            return
        
        # 融合
        fused = self.fusion.fuse(narrow, wide)
        
        if fused is not None:
            self._publish_with_jump_filter(fused)
        
        # 发布诊断
        self._publish_diagnostics(narrow, wide)

    def _publish_with_jump_filter(self, msg: TargetObservation) -> None:
        """发布结果，过滤大跳变"""
        position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

        if self.last_valid_position is not None:
            jump = self._calculate_distance(position, self.last_valid_position)
            if jump > self.max_position_jump:
                self.get_logger().warn(
                    f'Large jump detected: {jump:.2f}m > {self.max_position_jump}m, '
                    f'using Kalman prediction'
                )
                # 使用Kalman预测
                predicted = self.fusion._predict()
                if predicted is not None:
                    self.fusion_pub.publish(predicted)
                    return

        # 正常发布
        self.last_valid_position = position
        self.fusion_pub.publish(msg)

    def _publish_diagnostics(
        self,
        narrow: TargetObservation | None,
        wide: TargetObservation | None,
    ) -> None:
        """发布诊断信息"""
        diag = FusionDiagnostics()
        diag.header.stamp = self.get_clock().now().to_msg()

        # 融合来源
        diag.fusion_source = self.fusion.get_fusion_source(narrow, wide)

        # 窄相机质量
        if narrow is not None and narrow.valid:
            diag.narrow_quality = narrow.quality
        else:
            diag.narrow_quality = 0.0

        # 广角质量
        if wide is not None and wide.valid:
            diag.wide_quality = wide.quality
        else:
            diag.wide_quality = 0.0

        # 融合质量
        if narrow is not None and narrow.valid and wide is not None and wide.valid:
            n_q = max(narrow.quality, 1e-6)
            w_q = max(wide.quality, 1e-6)
            diag.fused_quality = (n_q * narrow.quality + w_q * wide.quality) / (n_q + w_q)
        elif narrow is not None and narrow.valid:
            diag.fused_quality = narrow.quality
        elif wide is not None and wide.valid:
            diag.fused_quality = wide.quality
        else:
            diag.fused_quality = 0.0

        # Kalman状态
        if self.last_valid_position is not None:
            diag.fused_position_m = self.last_valid_position
        else:
            diag.fused_position_m = [0.0, 0.0, 0.0]

        diag.kalman_velocity = list(self.fusion.tracker.velocity)
        diag.kalman_uncertainty = float(self.fusion.tracker.position_uncertainty)
        diag.is_fresh = self.fusion.tracker.is_fresh
        diag.consecutive_misses = self.fusion.consecutive_misses

        self.diagnostics_pub.publish(diag)

    @staticmethod
    def _calculate_distance(p1: list[float], p2: list[float]) -> float:
        """计算两点距离"""
        return math.sqrt(
            (p1[0] - p2[0]) ** 2
            + (p1[1] - p2[1]) ** 2
            + (p1[2] - p2[2]) ** 2
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = TargetFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
