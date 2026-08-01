"""ROS 2 adapter for the source-separated native V7 bridge core."""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from ed_uav_interfaces.action import FlightCommand
from ed_uav_interfaces.msg import FcuState
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from .action_execution import FlightGoalHandle, execute_goal
from .authority import (
    FlightCommandAuthorityError,
    ProgrammableCapabilityError,
    capability_trust_from_environment,
    require_flight_command_authority,
    require_programmable_capability,
)
from .realtime_control import RealtimeControlConfig
from .ros_messages import (
    battery_message,
    diagnostic_message,
    odom_message,
    state_message,
)
from .serial_port import ExclusiveSerialPort
from .session import BridgeConfig, NativeV7Bridge
from .telemetry import FreshnessPolicy, TelemetrySnapshot

__all__ = (
    "FcuBridgeNode",
    "FlightCommandAuthorityError",
    "ProgrammableCapabilityError",
    "require_flight_command_authority",
    "require_programmable_capability",
)


class FcuBridgeNode(Node):
    """Own the FCU endpoint, native protocol, and frozen `/fcu` ROS graph surface."""

    def __init__(self) -> None:
        super().__init__("ed_uav_fcu_bridge")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_lock_dir", "/tmp")
        self.declare_parameter("baudrate", 500000)
        self.declare_parameter("position_max_age_s", 0.20)
        self.declare_parameter("aux_status_max_age_s", 0.50)
        self.declare_parameter("link_max_age_s", 0.50)
        self.declare_parameter("command_ack_timeout_s", 0.50)
        self.declare_parameter("move_speed_cmps", 30)
        self.declare_parameter("enable_realtime_control", False)
        self.declare_parameter("realtime_stream_period_s", 0.02)
        self.declare_parameter("realtime_stop_frame_count", 3)
        self.declare_parameter("realtime_position_tolerance_m", 0.05)
        self.declare_parameter("realtime_proportional_gain_cmps_per_m", 100.0)
        self.declare_parameter("enable_experimental_0x32_0x33", False)
        self.declare_parameter("enable_flight_commands", False)
        self.declare_parameter("enable_programmable_commands", False)
        self.declare_parameter("programmable_capability_report", "")
        self.declare_parameter("fcu_device_identity", "")
        commands_enabled = require_flight_command_authority(
            bool(self.get_parameter("enable_flight_commands").value),
            os.environ,
        )
        programmable_enabled = bool(
            self.get_parameter("enable_programmable_commands").value
        )
        if programmable_enabled:
            require_programmable_capability(
                True,
                capability_trust_from_environment(
                Path(str(self.get_parameter("programmable_capability_report").value)),
                str(self.get_parameter("fcu_device_identity").value),
                os.environ,
                ),
            )
        policy = FreshnessPolicy(
            float(self.get_parameter("position_max_age_s").value),
            float(self.get_parameter("aux_status_max_age_s").value),
            float(self.get_parameter("link_max_age_s").value),
        )
        self._port = ExclusiveSerialPort(
            str(self.get_parameter("serial_port").value),
            int(self.get_parameter("baudrate").value),
            lock_dir=Path(str(self.get_parameter("serial_lock_dir").value)),
        )
        self._port.open()
        self.get_logger().info(
            f"serial.open {self.get_parameter('serial_port').value}"
            f" @ {self.get_parameter('baudrate').value}"
            f" flight_commands={commands_enabled}"
            f" realtime={bool(self.get_parameter('enable_realtime_control').value)}"
        )
        realtime_config = RealtimeControlConfig(
            enable_realtime_control=bool(
                self.get_parameter("enable_realtime_control").value
            ),
            stream_period_s=float(
                self.get_parameter("realtime_stream_period_s").value
            ),
            stop_frame_count=int(
                self.get_parameter("realtime_stop_frame_count").value
            ),
            position_tolerance_m=float(
                self.get_parameter("realtime_position_tolerance_m").value
            ),
            proportional_gain_cmps_per_m=float(
                self.get_parameter("realtime_proportional_gain_cmps_per_m").value
            ),
        )
        self._bridge = NativeV7Bridge(
            self._port.write,
            BridgeConfig(freshness=policy, realtime_control=realtime_config),
        )
        self._command_result = threading.Event()
        group = ReentrantCallbackGroup()
        self._state_publisher = self.create_publisher(FcuState, "/fcu/state", 10)
        self._battery_publisher = self.create_publisher(BatteryState, "/fcu/battery", 10)
        self._odom_publisher = self.create_publisher(Odometry, "/fcu/optical_flow/odom", 10)
        self._diagnostic_publisher = self.create_publisher(DiagnosticArray, "/fcu/diagnostics", 10)
        self._timer = self.create_timer(0.02, self._poll, callback_group=group)
        if commands_enabled:
            self._action_server = ActionServer(
                self,
                FlightCommand,
                "/fcu/flight_command",
                execute_callback=self._execute,
                cancel_callback=self._cancel,
                callback_group=group,
            )
            self.get_logger().info("action.server /fcu/flight_command ready")
        # AUX / 链路边沿检测状态 (仅在变化时打 log)
        self._last_aux1_us: int | None = None
        self._last_task3_gate: bool | None = None
        self._last_aux6: bool | None = None
        self._last_emergency: bool | None = None
        self._last_link_ok: bool | None = None

    def destroy_node(self) -> bool:
        self._port.close()
        self.get_logger().info("serial.close")
        return super().destroy_node()

    def _poll(self) -> None:
        steady_now = self._steady_now()
        chunk = self._port.read()
        if chunk:
            results = self._bridge.feed(chunk, steady_now, self.get_clock().now().nanoseconds)
            if results:
                self._command_result.set()
        timeout = self._bridge.tick(steady_now)
        if timeout is not None:
            self._command_result.set()
        snapshot = self._bridge.snapshot(steady_now)
        self._log_edge_transitions(snapshot)
        self._publish(snapshot)

    def _log_edge_transitions(self, snapshot: TelemetrySnapshot) -> None:
        """Log AUX1/AUX6/硬锁/链路状态变化 (仅边沿, 避免刷屏)."""
        aux1 = snapshot.aux1_us if snapshot.aux1_valid else None
        if aux1 is not None and aux1 != self._last_aux1_us:
            self.get_logger().info(f"aux1_us={aux1}")
            self._last_aux1_us = aux1
        gate = snapshot.task3_control_allowed
        if gate != self._last_task3_gate:
            if gate:
                self.get_logger().info("aux.gate TASK3_CONTROL_ALLOWED (AUX1 1400~1600us)")
            else:
                self.get_logger().info(f"aux.gate task3_control_lost (aux1={snapshot.aux1_us}us)")
            self._last_task3_gate = gate
        aux6 = snapshot.aux is not None and snapshot.aux.valid and snapshot.aux.aux6_us > 1700
        if aux6 != self._last_aux6:
            if aux6:
                self.get_logger().info("aux6.start_switch ON (>1700us)")
            else:
                self.get_logger().info(f"aux6.start_switch OFF (aux6={snapshot.aux.aux6_us if snapshot.aux else 0}us)")
            self._last_aux6 = aux6
        if snapshot.emergency_lock_active != self._last_emergency:
            if snapshot.emergency_lock_active:
                self.get_logger().warning("emergency_lock LATCHED (AUX1>=1800us)")
            else:
                self.get_logger().warning("emergency_lock released")
            self._last_emergency = snapshot.emergency_lock_active
        if snapshot.link.valid != self._last_link_ok:
            if snapshot.link.valid:
                self.get_logger().info("link.ok (飞控遥测恢复)")
            else:
                self.get_logger().warning(f"link.lost age={snapshot.link.age_s:.2f}s")
            self._last_link_ok = snapshot.link.valid

    def _publish(self, snapshot: TelemetrySnapshot) -> None:
        stamp = self.get_clock().now().to_msg()
        self._state_publisher.publish(state_message(snapshot, stamp))
        battery = battery_message(snapshot, stamp)
        if battery is not None:
            self._battery_publisher.publish(battery)
        odom = odom_message(snapshot, stamp)
        if odom is not None:
            self._odom_publisher.publish(odom)
        diagnostic = diagnostic_message(snapshot, stamp)
        if diagnostic is not None:
            self._diagnostic_publisher.publish(diagnostic)

    def _execute(self, goal_handle: FlightGoalHandle) -> FlightCommand.Result:
        return execute_goal(self, goal_handle)

    @staticmethod
    def _cancel(goal_handle: FlightGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _steady_now() -> float:
        return time.monotonic()


def main() -> None:
    """Run the bridge with an executor that keeps serial polling alive during actions."""
    rclpy.init()
    node = FcuBridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        return
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
