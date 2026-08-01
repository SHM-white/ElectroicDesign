"""Dry-run vehicle telemetry simulator — feeds the perception chain offline.

Without a real wheeled car the target observer refuses to fuse anything
(``_vehicle is None`` guard), so the mission display stays black. This node
publishes a monotonic simulated car on ``/d_task/vehicle/telemetry`` so the
dry-run chain — camera → AprilTag → visual tracking → display — can be
validated end to end. Never started in flight mode.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from ed_uav_interfaces.msg import VehicleTelemetry


class DryRunVehicleSim(Node):

    def __init__(self) -> None:
        super().__init__("dry_run_vehicle_sim")
        self._publisher = self.create_publisher(
            VehicleTelemetry, "/d_task/vehicle/telemetry", 10
        )
        self._sequence = 0
        self._displacement_m = 0.0
        self._timer = self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        message = VehicleTelemetry()
        message.contract_version = VehicleTelemetry.CONTRACT_VERSION
        message.start_stamp = stamp
        message.acquisition_stamp = stamp
        message.source_sequence = self._sequence
        message.checksum_crc16 = 0
        message.vehicle_id = "dry-run-sim"
        message.start_event = True
        message.heartbeat_alive = True
        message.motion_kind = VehicleTelemetry.MOTION_DISPLACEMENT
        self._displacement_m += 0.05  # 0.5 m/s @ 10 Hz
        message.displacement_m = self._displacement_m
        message.wheel_speed_m_s = 0.5
        message.turn_class = VehicleTelemetry.TURN_STRAIGHT
        message.heading_rad = 0.0
        message.yaw_rate_rad_s = 0.0
        message.route_stage = VehicleTelemetry.ROUTE_START
        message.lap_complete = False
        message.frame_id = "vehicle_start"
        self._sequence += 1
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DryRunVehicleSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
