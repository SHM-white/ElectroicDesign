"""Simulator FCU action/state adapter backed only by Gazebo ground truth."""

from dataclasses import dataclass
import math
import threading
import time
from typing_extensions import assert_never

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ed_uav_interfaces.action import FlightCommand
from ed_uav_interfaces.msg import FcuState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .action_semantics import (
    CommandKind,
    bounded_timeout,
    command_from_value,
    requires_armed_vehicle,
)
from .fcu_state import Position3D, touched_down
from .motion_policy import motion_command, motion_complete


@dataclass(slots=True)  # noqa: MUTABLE_OK
class SimulatorState:
    """Mutable state machine state owned by the single simulator FCU node."""

    mode: int = FcuState.MODE_STABILIZE
    motors_armed: bool = False
    source_sequence: int = 0


class SimulatorFcuNode(Node):
    """Adapt native Gazebo control and odometry to the ED FCU contracts."""

    def __init__(self) -> None:
        super().__init__("ed_uav_sim_fcu")
        self._group = ReentrantCallbackGroup()
        self._state = SimulatorState()
        self._latest_odom: Odometry | None = None
        self._lock = threading.Lock()
        self._active_goal = False
        self.declare_parameter("publish_odom_to_base_link_tf", True)
        self._command_publisher = self.create_publisher(Twist, "/simulation/cmd_vel", 10)
        self._enable_publisher = self.create_publisher(Bool, "/simulation/enable", 10)
        self._state_publisher = self.create_publisher(FcuState, "/fcu/state", 10)
        self._battery_publisher = self.create_publisher(BatteryState, "/fcu/battery", 10)
        self._flow_publisher = self.create_publisher(Odometry, "/fcu/optical_flow/odom", 10)
        self._diagnostics_publisher = self.create_publisher(DiagnosticArray, "/fcu/diagnostics", 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            "/simulation/ground_truth/odom",
            self._on_odometry,
            10,
            callback_group=self._group,
        )
        self._output_timer = self.create_timer(0.1, self._publish_outputs, callback_group=self._group)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._action_server = ActionServer(
            self,
            FlightCommand,
            "/fcu/flight_command",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._group,
        )

    def _goal_callback(self, goal: FlightCommand.Goal) -> GoalResponse:
        """Accept only known commands and one bounded action at a time."""
        command = command_from_value(goal.command)
        mode_is_valid = goal.requested_mode <= FcuState.MODE_PROGRAM
        position = goal.target_pose.pose.position
        target_is_finite = all(math.isfinite(value) for value in (position.x, position.y, position.z))
        with self._lock:
            available = not self._active_goal
            if (
                available
                and command is not None
                and math.isfinite(goal.timeout_sec)
                and goal.timeout_sec >= 0.0
                and mode_is_valid
                and target_is_finite
            ):
                self._active_goal = True
                return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def _cancel_callback(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        """Allow cancellation; the execute loop stops and publishes zero velocity."""
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> FlightCommand.Result:
        """Execute one FCU command until its physical condition is observed."""
        try:
            command = command_from_value(goal_handle.request.command)
            if command is None:
                return self._finish(
                    goal_handle,
                    FlightCommand.Result.RESULT_REJECTED,
                    "unknown command",
                )
            if requires_armed_vehicle(command) and not self._state.motors_armed:
                return self._finish(
                    goal_handle,
                    FlightCommand.Result.RESULT_REJECTED,
                    "vehicle is disarmed",
                )
            match command:
                case CommandKind.ARM:
                    return self._execute_arm(goal_handle)
                case CommandKind.DISARM:
                    return self._execute_disarm(goal_handle)
                case CommandKind.SET_MODE:
                    return self._execute_mode(goal_handle)
                case CommandKind.TAKEOFF | CommandKind.MOVE | CommandKind.HOVER | CommandKind.LAND:
                    return self._execute_motion(goal_handle, command)
                case unreachable:
                    assert_never(unreachable)
        finally:
            with self._lock:
                self._active_goal = False

    def _execute_arm(self, goal_handle: ServerGoalHandle) -> FlightCommand.Result:
        """Enable native Gazebo control and complete the arm transition."""
        self._publish_enable(True)
        self._state.motors_armed = True
        return self._finish(
            goal_handle,
            FlightCommand.Result.RESULT_SUCCEEDED,
            "simulator armed",
        )

    def _execute_disarm(self, goal_handle: ServerGoalHandle) -> FlightCommand.Result:
        """Stop native control and complete the disarm transition."""
        if self._latest_odom is None or not touched_down(self._position(self._latest_odom), 0.14):
            return self._finish(
                goal_handle,
                FlightCommand.Result.RESULT_REJECTED,
                "vehicle is not on the ground",
            )
        self._publish_zero_command()
        self._publish_enable(False)
        self._state.motors_armed = False
        return self._finish(goal_handle, FlightCommand.Result.RESULT_SUCCEEDED, "simulator disarmed")

    def _execute_mode(self, goal_handle: ServerGoalHandle) -> FlightCommand.Result:
        """Apply a simulator mode after boundary validation."""
        self._state.mode = goal_handle.request.requested_mode
        return self._finish(goal_handle, FlightCommand.Result.RESULT_SUCCEEDED, "simulator mode updated")

    def _execute_motion(self, goal_handle: ServerGoalHandle, command: CommandKind) -> FlightCommand.Result:
        """Drive native velocity control until the requested physical condition holds."""
        duration = bounded_timeout(goal_handle.request.timeout_sec)
        hold_deadline = time.monotonic() + duration if command is CommandKind.HOVER else None
        deadline = time.monotonic() + duration + (0.2 if hold_deadline is not None else 0.0)
        self._publish_enable(True)
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._publish_zero_command()
                goal_handle.canceled()
                return self._result(FlightCommand.Result.RESULT_REJECTED, "action canceled")
            odometry = self._latest_odom
            if odometry is not None:
                current = self._position(odometry)
                complete = motion_complete(command, current, odometry, goal_handle.request)
                if command is CommandKind.HOVER:
                    complete = hold_deadline is not None and time.monotonic() >= hold_deadline
                if complete:
                    self._publish_zero_command()
                    return self._finish(
                        goal_handle,
                        FlightCommand.Result.RESULT_SUCCEEDED,
                        "physical condition reached",
                    )
                self._command_publisher.publish(
                    motion_command(command, current, goal_handle.request, odometry)
                )
                self._publish_feedback(goal_handle, 0.5)
            time.sleep(0.05)
        self._publish_zero_command()
        return self._finish(goal_handle, FlightCommand.Result.RESULT_TIMEOUT, "physical condition timeout")

    def _on_odometry(self, odometry: Odometry) -> None:
        """Store ground truth, publish optical-flow odometry, and own dynamic TF."""
        self._latest_odom = odometry
        flow = Odometry()
        flow.header = odometry.header
        flow.child_frame_id = "base_link"
        flow.pose = odometry.pose
        flow.twist = odometry.twist
        self._flow_publisher.publish(flow)
        transform = TransformStamped()
        transform.header = odometry.header
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        if self.get_parameter("publish_odom_to_base_link_tf").value:
            self._tf_broadcaster.sendTransform(transform)

    def _publish_outputs(self) -> None:
        """Publish state, nominal simulator battery, and diagnostics."""
        odometry = self._latest_odom
        state = FcuState()
        state.acquisition_stamp = self.get_clock().now().to_msg()
        state.source_sequence = self._state.source_sequence
        state.source = FcuState.SOURCE_SIMULATOR
        state.mode = self._state.mode
        state.motors_armed = self._state.motors_armed
        state.communication_ok = odometry is not None
        state.frame_id = "odom"
        state.aux1_us = 1500
        state.aux1_valid = True
        state.task3_control_allowed = True
        state.emergency_lock_active = False
        if odometry is not None:
            state.optical_flow_position_m.x = odometry.pose.pose.position.x
            state.optical_flow_position_m.y = odometry.pose.pose.position.y
            state.optical_flow_position_m.z = odometry.pose.pose.position.z
            state.altitude_m = odometry.pose.pose.position.z
        state.battery_voltage_v = 12.0
        state.optical_flow_quality = 255 if odometry is not None else 0
        self._state.source_sequence += 1
        self._state_publisher.publish(state)
        battery = BatteryState()
        battery.header.stamp = state.acquisition_stamp
        battery.voltage = 12.0
        battery.percentage = 1.0
        battery.present = True
        self._battery_publisher.publish(battery)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = state.acquisition_stamp
        diagnostics.status = [DiagnosticStatus(level=DiagnosticStatus.OK, name="simulator_fcu", message="Gazebo FCU adapter", values=[KeyValue(key="source", value="Gazebo Fortress")])]
        self._diagnostics_publisher.publish(diagnostics)

    def _publish_feedback(self, goal_handle: ServerGoalHandle, progress: float) -> None:
        """Publish the common executing feedback state."""
        feedback = FlightCommand.Feedback()
        feedback.execution_state = FlightCommand.Feedback.STATE_EXECUTING
        feedback.progress = progress
        feedback.correlation_id = goal_handle.request.correlation_id
        goal_handle.publish_feedback(feedback)

    def _finish(self, goal_handle: ServerGoalHandle, code: int, reason: str) -> FlightCommand.Result:
        """Set the action terminal status and construct its result."""
        if code == FlightCommand.Result.RESULT_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return self._result(code, reason)

    def _result(self, code: int, reason: str) -> FlightCommand.Result:
        """Construct a timestamped action result."""
        result = FlightCommand.Result()
        result.result_code = code
        result.completed_stamp = self.get_clock().now().to_msg()
        result.reason = reason
        return result

    def _publish_enable(self, enabled: bool) -> None:
        """Publish the native multicopter enable command."""
        message = Bool()
        message.data = enabled
        self._enable_publisher.publish(message)

    def _publish_zero_command(self) -> None:
        """Stop velocity control without changing FCU state ownership."""
        self._command_publisher.publish(Twist())

    @staticmethod
    def _position(odometry: Odometry) -> Position3D:
        """Extract the ENU position from ground-truth odometry."""
        position = odometry.pose.pose.position
        return Position3D(position.x, position.y, position.z)


def main(args: list[str] | None = None) -> None:
    """Run the simulator FCU adapter."""
    rclpy.init(args=args)
    node = SimulatorFcuNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
