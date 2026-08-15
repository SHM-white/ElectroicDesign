"""Publish continuous odometry: FAST-LIO x/y/yaw plus simulator-owned Z."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .planar_odometry import bounded_planar_covariance, continuous_altitude, yaw_only_quaternion


class PlanarOdomFuser(Node):
    """Resolve the planar-LiDAR Z nullspace without altering LIO planar motion."""

    def __init__(self) -> None:
        super().__init__("planar_odom_fuser")
        input_topic = str(self.declare_parameter("input_topic", "/localization/lio/planar_raw").value)
        output_topic = str(self.declare_parameter("output_topic", "/localization/lio/odom").value)
        ground_truth_topic = str(self.declare_parameter("altitude_topic", "/simulation/ground_truth/odom").value)
        self._altitude_variance = float(self.declare_parameter("altitude_variance", 0.0025).value)
        self._max_altitude_age_s = float(self.declare_parameter("max_altitude_age_sec", 0.25).value)
        self._maximum_vertical_rate = float(self.declare_parameter("maximum_vertical_rate_m_s", 3.0).value)
        self._ground_truth: Odometry | None = None
        self._ground_truth_received_ns = 0
        self._last_z: float | None = None
        self._last_stamp_s: float | None = None
        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(Odometry, ground_truth_topic, self._on_ground_truth, 20)
        self.create_subscription(Odometry, input_topic, self._on_lio, 10)

        self._lio_count = 0

    def _on_ground_truth(self, message: Odometry) -> None:
        if math.isfinite(message.pose.pose.position.z):
            self._ground_truth = message
            self._ground_truth_received_ns = self.get_clock().now().nanoseconds

    def _on_lio(self, lio: Odometry) -> None:
        self._lio_count += 1
        ground_truth = self._ground_truth
        now_ns = self.get_clock().now().nanoseconds
        if ground_truth is None or (now_ns - self._ground_truth_received_ns) / 1e9 > self._max_altitude_age_s:
            if self._lio_count <= 3 or self._lio_count % 50 == 0:
                gt_age = float('inf') if ground_truth is None else (now_ns - self._ground_truth_received_ns) / 1e9
                self.get_logger().warn(f"[FUSER] skipped: gt_age={gt_age:.2f}s")
            return
        q = lio.pose.pose.orientation
        try:
            orientation = yaw_only_quaternion(q.x, q.y, q.z, q.w)
        except ValueError:
            return
        stamp_s = float(lio.header.stamp.sec) + float(lio.header.stamp.nanosec) / 1e9
        elapsed_s = 0.0 if self._last_stamp_s is None else max(0.0, stamp_s - self._last_stamp_s)
        try:
            altitude = continuous_altitude(
                self._last_z,
                float(ground_truth.pose.pose.position.z),
                elapsed_s,
                self._maximum_vertical_rate,
            )
        except ValueError:
            return

        output = Odometry()
        output.header = lio.header
        output.child_frame_id = lio.child_frame_id or "base_link"
        output.pose.pose.position.x = lio.pose.pose.position.x
        output.pose.pose.position.y = lio.pose.pose.position.y
        output.pose.pose.position.z = altitude
        output.pose.pose.orientation.x = orientation[0]
        output.pose.pose.orientation.y = orientation[1]
        output.pose.pose.orientation.z = orientation[2]
        output.pose.pose.orientation.w = orientation[3]
        output.pose.covariance = bounded_planar_covariance(
            lio.pose.covariance,
            altitude_variance=self._altitude_variance,
        )
        output.twist.twist.linear.x = lio.twist.twist.linear.x
        output.twist.twist.linear.y = lio.twist.twist.linear.y
        output.twist.twist.linear.z = ground_truth.twist.twist.linear.z
        output.twist.twist.angular.z = lio.twist.twist.angular.z
        output.twist.covariance = bounded_planar_covariance(
            lio.twist.covariance,
            altitude_variance=self._altitude_variance,
        )
        self._publisher.publish(output)
        self._last_z = altitude
        self._last_stamp_s = stamp_s
        if self._lio_count <= 3 or self._lio_count % 50 == 0:
            self.get_logger().info(
                f"[FUSER-OUT] pos=({output.pose.pose.position.x:.3f},"
                f"{output.pose.pose.position.y:.3f},"
                f"{output.pose.pose.position.z:.3f})"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlanarOdomFuser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
