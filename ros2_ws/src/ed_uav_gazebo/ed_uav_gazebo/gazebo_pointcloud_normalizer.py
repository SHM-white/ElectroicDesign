"""ROS boundary that normalizes Gazebo clouds for FAST-LIO type 2."""

from array import array

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from .pointcloud_normalizer import (
    PointCloudNormalizationError,
    PointFieldSpec,
    SourcePointCloud,
    normalize_gazebo_pointcloud,
)


class GazeboPointCloudNormalizer(Node):
    """Normalize only the observed Gazebo PointCloud2 schema."""

    def __init__(self) -> None:
        super().__init__("ed_uav_gazebo_pointcloud_normalizer")
        input_topic = self.declare_parameter("input_topic", "/lidar/points_raw").value
        output_topic = self.declare_parameter("output_topic", "/lidar/points").value
        self._scan_rate_hz = self.declare_parameter("scan_rate_hz", 10.0).value
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._normalize,
            qos_profile_sensor_data,
        )

    def _normalize(self, message: PointCloud2) -> None:
        """Publish one canonical cloud or report its typed input rejection."""
        source = SourcePointCloud(
            width=message.width,
            height=message.height,
            point_step=message.point_step,
            row_step=message.row_step,
            is_bigendian=message.is_bigendian,
            fields=tuple(
                PointFieldSpec(field.name, field.offset, field.datatype, field.count)
                for field in message.fields
            ),
            data=bytes(message.data),
        )
        try:
            normalized = normalize_gazebo_pointcloud(source, self._scan_rate_hz)
        except PointCloudNormalizationError as error:
            self.get_logger().error(str(error))
            return
        output = PointCloud2()
        output.header = message.header
        output.height = normalized.height
        output.width = normalized.width
        output.fields = [
            PointField(
                name=field.name,
                offset=field.offset,
                datatype=field.datatype,
                count=field.count,
            )
            for field in normalized.fields
        ]
        output.is_bigendian = normalized.is_bigendian
        output.point_step = normalized.point_step
        output.row_step = normalized.row_step
        output.data = array("B", normalized.data)
        output.is_dense = normalized.is_dense
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    """Run the Gazebo PointCloud2 normalization node."""
    rclpy.init(args=args)
    node = GazeboPointCloudNormalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
