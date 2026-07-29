"""ROS runtime side branch from Livox CustomMsg to monitored PointCloud2."""

from __future__ import annotations

import struct
import time
from array import array

from .contracts import MissingPointTiming, PointTimeRegression, normalize_mid360, normalize_mid360_raw
from .health import HealthState, evaluate_health


def main() -> None:
    """Publish monitoring data while leaving the upstream FAST-LIO CustomMsg untouched."""
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    rclpy.init()
    node = Node("mid360_monitoring_adapter")
    custom_topic = node.declare_parameter("custom_topic", "/livox/lidar").value
    monitoring_topic = node.declare_parameter("monitoring_topic", "/lidar/points").value
    imu_topic = node.declare_parameter("imu_topic", "/lidar/imu").value
    deadline_ns = node.declare_parameter("health_deadline_ns", 150_000_000).value
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    point_publisher = node.create_publisher(PointCloud2, monitoring_topic, qos)
    imu_publisher = node.create_publisher(Imu, imu_topic, qos)
    started_steady_ns = time.monotonic_ns()
    last_point_steady_ns = started_steady_ns
    last_imu_steady_ns = started_steady_ns
    last_health_code = "LIDAR_STARTING"
    fields = (
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="offset_time", offset=16, datatype=PointField.UINT32, count=1),
    )

    def publish_monitor(message: CustomMsg) -> None:
        nonlocal last_point_steady_ns
        try:
            normalized = normalize_mid360(message)
        except MissingPointTiming as error:
            node.get_logger().error(f"LIDAR_POINT_TIME_MISSING: {error}")
            return
        except PointTimeRegression as error:
            node.get_logger().warn(f"LIDAR_POINT_TIME_REGRESSION: {error}")
            normalized = normalize_mid360_raw(message)
        monitored = PointCloud2()
        monitored.header = message.header
        monitored.height = 1
        monitored.width = len(message.points)
        monitored.fields = list(fields)
        monitored.is_bigendian = False
        monitored.point_step = 20
        monitored.row_step = monitored.point_step * monitored.width
        monitored.is_dense = True
        monitored.data = array(
            "B",
            b"".join(
                struct.pack(
                    "<ffffI",
                    point.x,
                    point.y,
                    point.z,
                    float(point.reflectivity),
                    offset_time_ns,
                )
                for point, offset_time_ns in zip(message.points, normalized.point_times_ns)
            ),
        )
        point_publisher.publish(monitored)
        last_point_steady_ns = time.monotonic_ns()

    def relay_imu(message: Imu) -> None:
        nonlocal last_imu_steady_ns
        imu_publisher.publish(message)
        last_imu_steady_ns = time.monotonic_ns()

    def check_health() -> None:
        nonlocal last_health_code
        now_steady_ns = time.monotonic_ns()
        report = evaluate_health(
            HealthState(
                driver_alive=True,
                last_driver_steady_ns=last_point_steady_ns,
                last_point_steady_ns=last_point_steady_ns,
                last_imu_steady_ns=last_imu_steady_ns,
            ),
            now_steady_ns=now_steady_ns,
            deadline_ns=int(deadline_ns),
        )
        if report.code != last_health_code:
            node.get_logger().info(report.code) if report.active else node.get_logger().error(report.code)
            last_health_code = report.code

    node.create_subscription(CustomMsg, custom_topic, publish_monitor, qos)
    node.create_subscription(Imu, "/livox/imu", relay_imu, qos)
    node.create_timer(0.05, check_health)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
