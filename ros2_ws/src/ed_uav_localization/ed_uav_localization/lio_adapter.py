from __future__ import annotations

import rclpy
from pathlib import Path

from ed_uav_description.calibration import CalibrationError, load_calibration
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from ed_uav_localization.odometry import RigidTransform, normalize_odometry
from ed_uav_localization.lio_outputs import canonicalize_cloud, canonicalize_path


class LioAdapter(Node):
    def __init__(self) -> None:
        super().__init__("lio_adapter")
        self.declare_parameter("input_topic", "/fast_lio/odometry")
        self.declare_parameter("output_topic", "/localization/lio/odom")
        self.declare_parameter("cloud_input_topic", "/fast_lio/cloud_registered")
        self.declare_parameter("map_input_topic", "/fast_lio/laser_map")
        self.declare_parameter("path_input_topic", "/fast_lio/path")
        self.declare_parameter("cloud_output_topic", "/localization/lio/cloud_registered")
        self.declare_parameter("map_output_topic", "/localization/lio/map")
        self.declare_parameter("path_output_topic", "/localization/lio/path")
        self.declare_parameter("calibration_file", "")
        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value
        cloud_input_topic: str = self.get_parameter("cloud_input_topic").value
        map_input_topic: str = self.get_parameter("map_input_topic").value
        path_input_topic: str = self.get_parameter("path_input_topic").value
        cloud_output_topic: str = self.get_parameter("cloud_output_topic").value
        map_output_topic: str = self.get_parameter("map_output_topic").value
        path_output_topic: str = self.get_parameter("path_output_topic").value
        calibration_file: str = self.get_parameter("calibration_file").value
        if not calibration_file:
            raise CalibrationError("calibration_file parameter is required")
        calibration = load_calibration(Path(calibration_file))
        lidar_transform = calibration.transform_for("lidar_link")
        self._base_to_lidar = RigidTransform.from_xyz_rpy(
            xyz_m=lidar_transform.xyz_m,
            rpy_rad=lidar_transform.rpy_rad,
        )
        self._odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self._cloud_pub = self.create_publisher(PointCloud2, cloud_output_topic, 10)
        self._map_pub = self.create_publisher(PointCloud2, map_output_topic, 10)
        self._path_pub = self.create_publisher(RosPath, path_output_topic, 10)
        self._odom_sub = self.create_subscription(
            Odometry, input_topic, self._odom_callback, 10
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, cloud_input_topic, self._cloud_callback, 10
        )
        self._map_sub = self.create_subscription(
            PointCloud2, map_input_topic, self._map_callback, 10
        )
        self._path_sub = self.create_subscription(
            RosPath, path_input_topic, self._path_callback, 10
        )

        self._odom_sub_count = 0

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom_sub_count += 1
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        if self._odom_sub_count <= 3 or self._odom_sub_count % 50 == 0:
            self.get_logger().info(
                f"[LIO-RAW] pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                f"quat=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f})"
            )
        normalized = normalize_odometry(msg, self._base_to_lidar)
        if normalized is not None:
            np = normalized.pose.pose.position
            self._odom_pub.publish(normalized)
            if self._odom_sub_count <= 3 or self._odom_sub_count % 50 == 0:
                self.get_logger().info(
                    f"[LIO-NORM] pos=({np.x:.3f},{np.y:.3f},{np.z:.3f})"
                )
        else:
            self.get_logger().warn("[LIO-NORM] normalize returned None — dropped")

    def _cloud_callback(self, message: PointCloud2) -> None:
        self._cloud_pub.publish(canonicalize_cloud(message))

    def _map_callback(self, message: PointCloud2) -> None:
        self._map_pub.publish(canonicalize_cloud(message))

    def _path_callback(self, message: RosPath) -> None:
        self._path_pub.publish(canonicalize_path(message))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LioAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
