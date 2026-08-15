"""Relay simulator ground truth into the existing localization input contract."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SimulatorLocalizationNode(Node):
    """Expose ground truth as the localization supervisor's LIO input."""

    def __init__(self) -> None:
        super().__init__("ed_uav_sim_localization")
        self._publisher = self.create_publisher(Odometry, "/localization/lio/odom", 10)
        self._subscription = self.create_subscription(
            Odometry,
            "/simulation/ground_truth/odom",
            self._relay_odometry,
            10,
        )
        self._count = 0

    def _relay_odometry(self, odometry: Odometry) -> None:
        """Relay ground truth without creating a second transform owner."""
        self._publisher.publish(odometry)
        self._count += 1
        if self._count <= 3 or self._count % 100 == 0:
            p = odometry.pose.pose.position
            self.get_logger().info(
                f"[GT-RELAY] pos=({p.x:.3f},{p.y:.3f},{p.z:.3f})"
            )


def main(args: list[str] | None = None) -> None:
    """Run the simulator localization relay."""
    rclpy.init(args=args)
    node = SimulatorLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
