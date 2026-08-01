"""Dry-run synthetic vehicle telemetry publisher.

Simulates the wheel vehicle on ``/d_task/vehicle/telemetry`` so that the
dry-run chain (target observation → mission display) has live data without a
real car. Mirrors the message shape produced by ``to_vehicle_message``.
"""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ed_uav_interfaces.msg import VehicleTelemetry


class DryRunTelemetryNode(Node):

    def __init__(self) -> None:
        super().__init__("dry_run_telemetry")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("speed_m_s", 0.5)
        rate = float(self.get_parameter("rate_hz").value)
        self._speed = float(self.get_parameter("speed_m_s").value)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            VehicleTelemetry, "/d_task/vehicle/telemetry", qos
        )
        self._sequence = 0
        self._start_stamp = self.get_clock().now().to_msg()
        self._displacement_m = 0.0
        self.create_timer(1.0 / rate, self._publish)

    def _publish(self) -> None:
        self._sequence = (self._sequence + 1) % (1 << 32)
        now: Time = self.get_clock().now().to_msg()
        self._displacement_m += self._speed / 10.0
        message = VehicleTelemetry()
        message.contract_version = VehicleTelemetry.CONTRACT_VERSION
        message.start_stamp = self._start_stamp
        message.acquisition_stamp = now
        message.source_sequence = self._sequence
        message.checksum_crc16 = 0
        message.vehicle_id = "dry-run-sim"
        message.start_event = True
        message.heartbeat_alive = True
        message.motion_kind = VehicleTelemetry.MOTION_WHEEL_SPEED
        message.displacement_m = self._displacement_m
        message.wheel_speed_m_s = self._speed
        message.turn_class = VehicleTelemetry.TURN_STRAIGHT
        message.heading_rad = 0.0
        message.yaw_rate_rad_s = 0.0
        message.route_stage = VehicleTelemetry.ROUTE_B
        message.lap_complete = False
        message.frame_id = "vehicle_start"
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DryRunTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
