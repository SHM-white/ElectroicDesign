"""ROS 2 adapter for the source-separated native V7 bridge core."""

from __future__ import annotations

import math
import os
import threading
import time
from pathlib import Path
from typing import Protocol

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

from .actions import CommandKind, CommandRejectedError, CommandRequest, ResultCode
from .authority import (
    FlightCommandAuthorityError,
    ProgrammableCapabilityError,
    capability_trust_from_environment,
    require_flight_command_authority,
    require_programmable_capability,
)
from .command_validation import goal_rejection_reason
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
        self._bridge = NativeV7Bridge(self._port.write, BridgeConfig(freshness=policy))
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
        rejection_reason = goal_rejection_reason(goal_handle.request)
        if rejection_reason is not None:
            return self._reject_goal(goal_handle, rejection_reason)
        try:
            request = self._request_for_goal(goal_handle.request)
        except ValueError as error:
            return self._reject_goal(goal_handle, str(error))
        if request is None:
            return self._reject_goal(
                goal_handle,
                "goal does not map to a supported high-level V7 command",
            )
        result = FlightCommand.Result()
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
    def _reject_goal(goal_handle: FlightGoalHandle, reason: str) -> FlightCommand.Result:
        result = FlightCommand.Result()
        result.result_code = FlightCommand.Result.RESULT_REJECTED
        result.reason = reason
        goal_handle.abort()
        return result

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
