"""ROS 2 publisher boundary for the deterministic virtual-time harness."""

from __future__ import annotations

import argparse
import struct

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

from .cli import parse_fault
from .model import Event, EventType, ScenarioConfig, Stream
from .scenario import DeterministicScenario
from .sensors import DeterministicSensors


SENSOR_QOS = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
STATE_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)


def parse_node_config(argv: list[str] | None = None) -> ScenarioConfig:
    """Parse only virtual-time scenario controls for the ROS publisher executable."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--duration-seconds", type=int, default=60)
    parser.add_argument("--rate-hz", type=int, default=20)
    parser.add_argument("--max-ticks", type=int, default=120_000)
    parser.add_argument("--fault", action="append", default=[])
    parsed = parser.parse_args(argv)
    return ScenarioConfig(
        seed=parsed.seed,
        duration_seconds=parsed.duration_seconds,
        rate_hz=parsed.rate_hz,
        max_ticks=parsed.max_ticks,
        faults=tuple(parse_fault(raw) for raw in parsed.fault),
    )


class VerificationPublisher(Node):
    """Publishes accepted synthetic ROS messages without replaying rejected stale data."""

    def __init__(self, config: ScenarioConfig) -> None:
        super().__init__("ed_uav_verification_publisher")
        self._report = DeterministicScenario(config).run()
        self._sensors = DeterministicSensors(config.seed, config.tick_duration_ns)
        self._events = iter(self._report.events)
        self._point_publisher = self.create_publisher(PointCloud2, "/lidar/points", SENSOR_QOS)
        self._imu_publisher = self.create_publisher(Imu, "/lidar/imu", SENSOR_QOS)
        self._narrow_publisher = self.create_publisher(Image, "/camera/narrow/image_raw", SENSOR_QOS)
        self._wide_publisher = self.create_publisher(Image, "/camera/wide/image_raw", SENSOR_QOS)
        self._lio_odom_publisher = self.create_publisher(Odometry, "/localization/lio/odom", STATE_QOS)
        self._fcu_odom_publisher = self.create_publisher(Odometry, "/fcu/optical_flow/odom", STATE_QOS)
        self._timer = self.create_timer(0.001, self._publish_next)

    def _publish_next(self) -> None:
        try:
            event = next(self._events)
        except StopIteration:
            self._timer.cancel()
            self.get_logger().info("ROS SCENARIO: GREEN virtual replay completed")
            rclpy.shutdown()
            return
        if event.event_type is EventType.SAMPLE and event.accepted:
            self._publish_sample(event)

    def _publish_sample(self, event: Event) -> None:
        match event.stream:
            case Stream.FCU:
                self._fcu_odom_publisher.publish(self._odom_message(event))
            case Stream.LIDAR_POINTS:
                self._point_publisher.publish(self._point_cloud_message(event))
            case Stream.LIDAR_IMU:
                self._imu_publisher.publish(self._imu_message(event))
            case Stream.NARROW_IMAGE:
                self._narrow_publisher.publish(self._image_message(event))
            case Stream.WIDE_IMAGE:
                self._wide_publisher.publish(self._image_message(event))
            case Stream.ODOM:
                self._lio_odom_publisher.publish(self._odom_message(event))
            case Stream.GPIO | Stream.LASER:
                return
            case unreachable:
                raise AssertionError(f"unhandled stream: {unreachable}")

    def _point_cloud_message(self, event: Event) -> PointCloud2:
        fixture = self._sensors.point_cloud(event.sequence, event.acquisition_time_ns)
        message = PointCloud2()
        message.header.frame_id = fixture.frame_id
        self._set_stamp(message.header.stamp, fixture.acquisition_time_ns)
        message.height = 1
        message.width = len(fixture.points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(struct.pack("<fff", point.x_m, point.y_m, point.z_m) for point in fixture.points)
        return message

    def _imu_message(self, event: Event) -> Imu:
        fixture = self._sensors.imu(event.sequence, event.acquisition_time_ns)
        message = Imu()
        message.header.frame_id = fixture.frame_id
        self._set_stamp(message.header.stamp, fixture.acquisition_time_ns)
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = fixture.linear_acceleration_mps2
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = fixture.angular_velocity_radps
        return message

    def _image_message(self, event: Event) -> Image:
        fixture = self._sensors.image(event.stream, event.sequence, event.acquisition_time_ns)
        message = Image()
        message.header.frame_id = fixture.frame_id
        self._set_stamp(message.header.stamp, fixture.acquisition_time_ns)
        message.height = fixture.height
        message.width = fixture.width
        message.encoding = "mono8"
        message.is_bigendian = 0
        message.step = fixture.width
        message.data = fixture.data
        return message

    def _odom_message(self, event: Event) -> Odometry:
        fixture = self._sensors.odom(event.sequence, event.acquisition_time_ns)
        message = Odometry()
        message.header.frame_id = fixture.frame_id
        self._set_stamp(message.header.stamp, fixture.acquisition_time_ns)
        message.child_frame_id = fixture.child_frame_id
        message.pose.pose.position.x = fixture.x_m
        message.pose.pose.position.y = fixture.y_m
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = fixture.linear_x_mps
        message.twist.twist.linear.y = fixture.linear_y_mps
        return message

    @staticmethod
    def _set_stamp(stamp: Time, timestamp_ns: int) -> None:
        stamp.sec = timestamp_ns // 1_000_000_000
        stamp.nanosec = timestamp_ns % 1_000_000_000


def main(argv: list[str] | None = None) -> int:
    """Start and fully clean up the finite ROS publisher process."""
    config = parse_node_config(argv)
    rclpy.init(args=[])
    node = VerificationPublisher(config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
