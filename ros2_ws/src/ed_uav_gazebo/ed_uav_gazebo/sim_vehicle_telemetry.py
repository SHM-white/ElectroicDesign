"""Publish deterministic simulated VehicleTelemetry for perception pipeline testing.

In simulation there is no physical ground vehicle, so this node emits a
synthetic telemetry stream at 10 Hz.  The vehicle is reported as driving
straight at a constant speed with a valid heartbeat so the target observation
node's motion-context gates all pass.
"""

from __future__ import annotations

import time

import rclpy
from builtin_interfaces.msg import Time
from ed_uav_interfaces.msg import VehicleTelemetry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _seconds_to_stamp(sec: float) -> Time:
    whole = int(sec)
    nanosec = int((sec - whole) * 1e9)
    return Time(sec=whole, nanosec=nanosec)


class SimVehicleTelemetryNode(Node):
    """Publish synthetic VehicleTelemetry at 10 Hz for simulation."""

    def __init__(self) -> None:
        super().__init__("sim_vehicle_telemetry")
        self._publisher = self.create_publisher(
            VehicleTelemetry, "/d_task/vehicle/telemetry", 10
        )
        self._sequence = 0
        self._start_time = time.monotonic()
        self._timer = self.create_timer(0.1, self._publish)
        self.get_logger().info(
            "SimVehicleTelemetryNode started — publishing synthetic telemetry at 10 Hz"
        )

    def _publish(self) -> None:
        now = time.monotonic()
        elapsed = now - self._start_time
        self._sequence += 1

        msg = VehicleTelemetry()
        msg.contract_version = VehicleTelemetry.CONTRACT_VERSION
        msg.start_stamp = _seconds_to_stamp(self._start_time)
        msg.acquisition_stamp = _seconds_to_stamp(now)
        msg.source_sequence = self._sequence
        msg.checksum_crc16 = 0
        msg.vehicle_id = "sim-vehicle"
        msg.start_event = self._sequence == 1
        msg.heartbeat_alive = True
        msg.motion_kind = VehicleTelemetry.MOTION_DISPLACEMENT
        msg.displacement_m = elapsed * 0.3  # 0.3 m/s forward
        msg.wheel_speed_m_s = 0.3
        msg.heading_rad = 0.0
        msg.yaw_rate_rad_s = 0.0
        msg.turn_class = VehicleTelemetry.TURN_STRAIGHT
        msg.route_stage = VehicleTelemetry.ROUTE_START
        msg.lap_complete = False
        msg.frame_id = "vehicle_start"

        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimVehicleTelemetryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
