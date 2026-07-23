"""ROS runtime for generic PointCloud2 monitoring without Livox dependencies."""

from __future__ import annotations


def main() -> None:
    """Republish a standard cloud to the frozen monitoring topic without mutation."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node("generic_lidar_monitor")
    input_topic = node.declare_parameter("input_topic", "/lidar/input/points").value
    monitoring_topic = node.declare_parameter("monitoring_topic", "/lidar/points").value
    publisher = node.create_publisher(PointCloud2, monitoring_topic, qos_profile_sensor_data)

    def relay(message: PointCloud2) -> None:
        publisher.publish(message)

    node.create_subscription(PointCloud2, input_topic, relay, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
