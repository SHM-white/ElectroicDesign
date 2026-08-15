"""Drive the Gazebo D-task car and publish telemetry from observed odometry."""

from __future__ import annotations

import math

import rclpy
from ed_uav_interfaces.msg import VehicleTelemetry
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from .car_route import CapsuleRouteFollower


def _yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimCarController(Node):
    """One-lap route controller with ROS telemetry tied to physical Gazebo motion."""

    def __init__(self) -> None:
        super().__init__("sim_car_controller")
        speed_m_s = float(self.declare_parameter("speed_m_s", 0.15).value)
        self._follower = CapsuleRouteFollower(speed_m_s)
        self._odom: Odometry | None = None
        self._started = False
        self._start_ns: int | None = None
        self._sequence = 0
        self._last_command = self._follower.command(1.5, 2.0, math.pi / 2.0)
        self._command_pub = self.create_publisher(Twist, "/simulation/car/cmd_vel", 10)
        self._telemetry_pub = self.create_publisher(VehicleTelemetry, "/vehicle/telemetry", 20)
        self.create_subscription(Odometry, "/simulation/car/odom", self._on_odom, 20)
        self.create_subscription(Bool, "/simulation/competition_start", self._on_start, 10)
        self.create_timer(0.05, self._control)
        self.create_timer(0.10, self._publish_telemetry)

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _on_start(self, message: Bool) -> None:
        if message.data and not self._started:
            self._started = True
            self._start_ns = self.get_clock().now().nanoseconds
            self.get_logger().info("D-task car start event accepted; beginning one capsule lap")

    def _control(self) -> None:
        odom = self._odom
        if not self._started or odom is None:
            self._command_pub.publish(Twist())
            return
        position = odom.pose.pose.position
        self._last_command = self._follower.command(position.x, position.y, _yaw_from_odometry(odom))
        velocity = Twist()
        velocity.linear.x = self._last_command.speed_m_s
        velocity.angular.z = self._last_command.yaw_rate_rad_s
        self._command_pub.publish(velocity)

    def _publish_telemetry(self) -> None:
        now = self.get_clock().now()
        message = VehicleTelemetry()
        message.contract_version = VehicleTelemetry.CONTRACT_VERSION
        message.start_stamp = rclpy.time.Time(nanoseconds=self._start_ns or now.nanoseconds).to_msg()
        message.acquisition_stamp = now.to_msg()
        message.source_sequence = self._sequence
        self._sequence = (self._sequence + 1) % (1 << 32)
        message.checksum_crc16 = 0
        message.vehicle_id = "sim-d-task-car"
        start_age_s = 0.0 if self._start_ns is None else (now.nanoseconds - self._start_ns) / 1e9
        message.start_event = self._started and start_age_s <= 1.0
        message.heartbeat_alive = True
        message.motion_kind = VehicleTelemetry.MOTION_DISPLACEMENT
        message.displacement_m = float(self._last_command.displacement_m if self._started else 0.0)
        message.wheel_speed_m_s = float(self._last_command.speed_m_s if self._started else 0.0)
        odom = self._odom
        message.heading_rad = float(_yaw_from_odometry(odom)) if odom is not None else math.pi / 2.0
        message.yaw_rate_rad_s = float(self._last_command.yaw_rate_rad_s if self._started else 0.0)
        yaw_rate = abs(message.yaw_rate_rad_s)
        message.turn_class = (
            VehicleTelemetry.TURN_LARGE if yaw_rate > 0.7
            else VehicleTelemetry.TURN_SMALL if yaw_rate > 0.08
            else VehicleTelemetry.TURN_STRAIGHT
        )
        message.route_stage = int(self._last_command.stage if self._started else VehicleTelemetry.ROUTE_START)
        message.lap_complete = bool(self._started and self._last_command.complete)
        message.frame_id = "world"
        self._telemetry_pub.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimCarController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
