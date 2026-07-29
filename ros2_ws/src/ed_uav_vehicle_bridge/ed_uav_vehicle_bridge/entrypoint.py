"""Process lifecycle for the vehicle bridge ROS node."""

import rclpy
from rclpy.executors import ExternalShutdownException

from .node import VehicleBridgeNode


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VehicleBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
