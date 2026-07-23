"""ROS 2 adapter for the source-separated native V7 bridge core."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Protocol

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from ed_uav_interfaces.action import FlightCommand
from ed_uav_interfaces.msg import FcuState

from .actions import CommandKind, CommandRejectedError, CommandRequest, ResultCode
from .serial_port import ExclusiveSerialPort
from .session import BridgeConfig, NativeV7Bridge
from .telemetry import FreshnessPolicy, TelemetrySnapshot


class FlightGoalHandle(Protocol):
    """The action-handle capabilities used by this bridge's execute callback."""

    request: FlightCommand.Goal

    def abort(self) -> None: ...
    def succeed(self) -> None: ...
    def publish_feedback(self, feedback: FlightCommand.Feedback) -> None: ...


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
        self.declare_parameter("enable_experimental_0x32_0x33", False)
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
        self._bridge = NativeV7Bridge(self._port.write, BridgeConfig(freshness=policy))
        self._command_result = threading.Event()
        group = ReentrantCallbackGroup()
        self._state_publisher = self.create_publisher(FcuState, "/fcu/state", 10)
        self._battery_publisher = self.create_publisher(BatteryState, "/fcu/battery", 10)
        self._odom_publisher = self.create_publisher(Odometry, "/fcu/optical_flow/odom", 10)
        self._diagnostic_publisher = self.create_publisher(DiagnosticArray, "/fcu/diagnostics", 10)
        self._timer = self.create_timer(0.02, self._poll, callback_group=group)
        self._action_server = ActionServer(
            self,
            FlightCommand,
            "/fcu/flight_command",
            execute_callback=self._execute,
            cancel_callback=self._cancel,
            callback_group=group,
        )

    def destroy_node(self) -> bool:
        self._port.close()
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
        self._publish(self._bridge.snapshot(steady_now))

    def _publish(self, snapshot: TelemetrySnapshot) -> None:
        self._publish_state(snapshot)
        self._publish_battery(snapshot)
        self._publish_odom(snapshot)
        self._publish_diagnostic(snapshot)

    def _publish_state(self, snapshot: TelemetrySnapshot) -> None:
        message = FcuState()
        message.acquisition_stamp = self.get_clock().now().to_msg()
        message.source = FcuState.SOURCE_V7
        message.frame_id = "base_link"
        message.communication_ok = snapshot.link.valid
        message.altitude_m = snapshot.altitude_m or 0.0
        message.battery_voltage_v = snapshot.battery_voltage_v or 0.0
        if snapshot.position is not None:
            message.source_sequence = snapshot.position.source_sequence
            message.optical_flow_position_m.x = snapshot.position.right_m
            message.optical_flow_position_m.y = snapshot.position.forward_m
        else:
            message.source_sequence = snapshot.link.source_sequence
        if snapshot.status is not None:
            message.mode = snapshot.status.mode
            message.motors_armed = snapshot.status.motors_armed
        self._state_publisher.publish(message)

    def _publish_battery(self, snapshot: TelemetrySnapshot) -> None:
        if snapshot.battery_voltage_v is None:
            return
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.voltage = snapshot.battery_voltage_v
        self._battery_publisher.publish(message)

    def _publish_odom(self, snapshot: TelemetrySnapshot) -> None:
        position = snapshot.position
        if position is None or not position.valid:
            return
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = position.right_m
        message.pose.pose.position.y = position.forward_m
        message.pose.pose.orientation.w = 1.0
        self._odom_publisher.publish(message)

    def _publish_diagnostic(self, snapshot: TelemetrySnapshot) -> None:
        diagnostic = snapshot.flow_diagnostic
        if diagnostic is None:
            return
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "ed_uav_fcu_bridge/v7_0x51"
        status.hardware_id = "lingxiao-v7"
        status.level = DiagnosticStatus.OK
        status.message = "separate optical-flow diagnostic"
        status.values = [
            KeyValue(key="source", value="V7_0x51"),
            KeyValue(key="source_sequence", value=str(diagnostic.source_sequence)),
            KeyValue(key="mode", value=str(diagnostic.mode)),
            KeyValue(key="state", value=str(diagnostic.state)),
        ]
        message.status.append(status)
        self._diagnostic_publisher.publish(message)

    def _execute(self, goal_handle: FlightGoalHandle) -> FlightCommand.Result:
        request = self._request_for_goal(goal_handle.request)
        result = FlightCommand.Result()
        if request is None:
            result.result_code = FlightCommand.Result.RESULT_REJECTED
            result.reason = "goal does not map to a supported high-level V7 command"
            goal_handle.abort()
            return result
        self._command_result.clear()
        try:
            pending = self._bridge.start(request, self._steady_now(), self._goal_timeout(goal_handle.request))
        except (CommandRejectedError, ValueError) as error:
            result.result_code = FlightCommand.Result.RESULT_REJECTED
            result.reason = str(error)
            goal_handle.abort()
            return result
        feedback = FlightCommand.Feedback()
        feedback.execution_state = FlightCommand.Feedback.STATE_SENT
        feedback.correlation_id = goal_handle.request.correlation_id
        goal_handle.publish_feedback(feedback)
        self._command_result.wait(self._goal_timeout(goal_handle.request) + 0.10)
        completed = self._bridge.actions.last_result
        if completed is None or completed.command is not pending.command:
            completed = self._bridge.tick(self._steady_now())
        if completed is None:
            result.result_code = FlightCommand.Result.RESULT_TIMEOUT
            result.reason = "V7 acknowledgement deadline elapsed"
            goal_handle.abort()
            return result
        result.result_code, result.reason = self._ros_result(completed.code, completed.reason)
        result.completed_stamp = self.get_clock().now().to_msg()
        if completed.code is ResultCode.SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _request_for_goal(self, goal: FlightCommand.Goal) -> CommandRequest | None:
        match goal.command:
            case FlightCommand.Goal.COMMAND_ARM:
                return CommandRequest.unlock()
            case FlightCommand.Goal.COMMAND_DISARM:
                return CommandRequest(CommandKind.LOCK)
            case FlightCommand.Goal.COMMAND_SET_MODE:
                return CommandRequest(CommandKind.SET_MODE, mode=goal.requested_mode)
            case FlightCommand.Goal.COMMAND_TAKEOFF:
                return CommandRequest(CommandKind.TAKEOFF, height_cm=round(goal.target_pose.pose.position.z * 100.0))
            case FlightCommand.Goal.COMMAND_MOVE:
                return self._move_request(goal)
            case FlightCommand.Goal.COMMAND_HOVER:
                return CommandRequest.hover()
            case FlightCommand.Goal.COMMAND_LAND:
                return CommandRequest.land()
            case _:
                return None

    def _move_request(self, goal: FlightCommand.Goal) -> CommandRequest | None:
        position = self._bridge.snapshot(self._steady_now()).position
        if position is None or not position.valid:
            return None
        forward_m = goal.target_pose.pose.position.y - position.forward_m
        right_m = goal.target_pose.pose.position.x - position.right_m
        distance_cm = round(math.hypot(forward_m, right_m) * 100.0)
        direction_deg = round(math.degrees(math.atan2(right_m, forward_m)) % 360.0)
        requested_speed = math.hypot(goal.target_velocity.linear.x, goal.target_velocity.linear.y)
        speed_cmps = round(requested_speed * 100.0) if requested_speed > 0.0 else int(self.get_parameter("move_speed_cmps").value)
        return CommandRequest.move(distance_cm, speed_cmps, direction_deg)

    def _goal_timeout(self, goal: FlightCommand.Goal) -> float:
        return goal.timeout_sec if goal.timeout_sec > 0.0 else float(self.get_parameter("command_ack_timeout_s").value)

    @staticmethod
    def _ros_result(code: ResultCode, reason: str) -> tuple[int, str]:
        match code:
            case ResultCode.SUCCEEDED:
                return FlightCommand.Result.RESULT_SUCCEEDED, reason
            case ResultCode.TIMEOUT:
                return FlightCommand.Result.RESULT_TIMEOUT, reason
            case ResultCode.REJECTED:
                return FlightCommand.Result.RESULT_REJECTED, reason
            case ResultCode.FCU_ERROR:
                return FlightCommand.Result.RESULT_FCU_ERROR, reason
            case _:
                raise RuntimeError(f"unknown result code: {code}")

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
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
