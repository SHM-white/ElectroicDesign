"""ROS action execution adapters for legacy ACK and realtime V7 backends."""

from __future__ import annotations

import math
from typing import Protocol

from builtin_interfaces.msg import Time
from ed_uav_interfaces.action import FlightCommand

from .actions import CommandKind, CommandRejectedError, CommandRequest, ResultCode
from .command_validation import goal_rejection_reason
from .realtime_control import (
    PositionTarget,
    RealtimeHoverRequest,
    RealtimeMoveRequest,
    RealtimeResult,
    RealtimeResultCode,
    use_realtime_backend,
)
from .session import NativeV7Bridge


class FlightGoalHandle(Protocol):
    """The ROS action-handle capabilities used by both command backends."""

    request: FlightCommand.Goal

    @property
    def is_cancel_requested(self) -> bool: ...

    def abort(self) -> None: ...
    def canceled(self) -> None: ...
    def succeed(self) -> None: ...
    def publish_feedback(self, feedback: FlightCommand.Feedback) -> None: ...


class CommandEvent(Protocol):
    def clear(self) -> None: ...
    def wait(self, timeout: float | None = None) -> bool: ...


class ParameterValue(Protocol):
    @property
    def value(self) -> bool | int | float | str: ...


class RosNow(Protocol):
    def to_msg(self) -> Time: ...


class RosClock(Protocol):
    def now(self) -> RosNow: ...


class ActionNodeContext(Protocol):
    _bridge: NativeV7Bridge
    _command_result: CommandEvent

    def get_parameter(self, name: str) -> ParameterValue: ...
    def get_clock(self) -> RosClock: ...
    def _steady_now(self) -> float: ...


def execute_goal(
    context: ActionNodeContext,
    goal_handle: FlightGoalHandle,
) -> FlightCommand.Result:
    """Validate and execute one ROS goal through its source-selected V7 backend."""
    goal = goal_handle.request
    rejection_reason = goal_rejection_reason(goal)
    if rejection_reason is not None:
        _log_goal(context, goal, f"REJECTED reason={rejection_reason}")
        return _reject_goal(goal_handle, rejection_reason)
    realtime_capable = goal.command in (
        FlightCommand.Goal.COMMAND_MOVE,
        FlightCommand.Goal.COMMAND_HOVER,
    )
    _log_goal(context, goal, f"ACCEPT backend={'realtime' if use_realtime_backend(realtime_capable) else 'legacy'}")
    if use_realtime_backend(realtime_capable):
        return _execute_realtime(context, goal_handle)
    return _execute_legacy(context, goal_handle)


def _log_goal(context: ActionNodeContext, goal: FlightCommand.Goal, tag: str) -> None:
    """Emit one flight-command goal log line with target context."""
    log = getattr(context, "get_logger", None)
    if log is None:
        return
    try:
        logger = log()
    except (AttributeError, RuntimeError):
        # Unit boundaries and interrupted construction can legitimately expose
        # a Node method before rclpy has installed its logger.
        return
    pos = goal.target_pose.pose.position
    vel = goal.target_velocity.linear
    logger.info(
        f"flight_command.goal cmd={goal.command} corr={goal.correlation_id}"
        f" target=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f})"
        f" vel=({vel.x:.2f},{vel.y:.2f})"
        f" {tag}"
    )


def _execute_realtime(
    context: ActionNodeContext,
    goal_handle: FlightGoalHandle,
) -> FlightCommand.Result:
    goal = goal_handle.request
    duration_s = _goal_timeout(context, goal)
    match goal.command:
        case FlightCommand.Goal.COMMAND_MOVE:
            requested_speed = math.hypot(
                goal.target_velocity.linear.x,
                goal.target_velocity.linear.y,
            )
            max_speed_cmps = (
                round(requested_speed * 100.0)
                if requested_speed > 0.0
                else int(context.get_parameter("move_speed_cmps").value)
            )
            request = RealtimeMoveRequest(
                PositionTarget(
                    forward_m=goal.target_pose.pose.position.y,
                    right_m=goal.target_pose.pose.position.x,
                ),
                max_speed_cmps,
                duration_s,
            )
        case FlightCommand.Goal.COMMAND_HOVER:
            request = RealtimeHoverRequest(duration_s)
        case _:
            return _reject_goal(goal_handle, "goal is not realtime-capable")
    feedback = FlightCommand.Feedback()
    feedback.execution_state = FlightCommand.Feedback.STATE_EXECUTING
    feedback.correlation_id = goal.correlation_id
    goal_handle.publish_feedback(feedback)
    completed = context._bridge.realtime.execute(
        request,
        lambda: goal_handle.is_cancel_requested,
    )
    return _finish_realtime(context, goal_handle, completed)


def _log_result(context: ActionNodeContext, goal: FlightCommand.Goal, result: FlightCommand.Result) -> None:
    log = getattr(context, "get_logger", None)
    if log is None:
        return
    try:
        logger = log()
    except (AttributeError, RuntimeError):
        return
    code_names = {
        FlightCommand.Result.RESULT_SUCCEEDED: "SUCCEEDED",
        FlightCommand.Result.RESULT_REJECTED: "REJECTED",
        FlightCommand.Result.RESULT_TIMEOUT: "TIMEOUT",
        FlightCommand.Result.RESULT_FCU_ERROR: "FCU_ERROR",
    }
    logger.info(
        f"flight_command.result cmd={goal.command} corr={goal.correlation_id}"
        f" code={code_names.get(result.result_code, result.result_code)}"
        f" reason={result.reason}"
    )


def _finish_realtime(
    context: ActionNodeContext,
    goal_handle: FlightGoalHandle,
    completed: RealtimeResult,
) -> FlightCommand.Result:
    result = FlightCommand.Result()
    result.completed_stamp = context.get_clock().now().to_msg()
    result.reason = completed.reason
    match completed.code:
        case RealtimeResultCode.SUCCEEDED:
            result.result_code = FlightCommand.Result.RESULT_SUCCEEDED
            goal_handle.succeed()
        case RealtimeResultCode.CANCELLED:
            result.result_code = FlightCommand.Result.RESULT_REJECTED
            goal_handle.canceled()
        case RealtimeResultCode.TIMEOUT:
            result.result_code = FlightCommand.Result.RESULT_TIMEOUT
            goal_handle.abort()
        case RealtimeResultCode.CONTROL_GATED | RealtimeResultCode.REJECTED:
            result.result_code = FlightCommand.Result.RESULT_REJECTED
            goal_handle.abort()
        case RealtimeResultCode.FCU_ERROR:
            result.result_code = FlightCommand.Result.RESULT_FCU_ERROR
            goal_handle.abort()
    _log_result(context, goal_handle.request, result)
    return result


def _execute_legacy(
    context: ActionNodeContext,
    goal_handle: FlightGoalHandle,
) -> FlightCommand.Result:
    goal = goal_handle.request
    try:
        request = _legacy_request(context, goal)
    except ValueError as error:
        return _reject_goal(goal_handle, str(error))
    if request is None:
        return _reject_goal(
            goal_handle,
            "goal does not map to a supported high-level V7 command",
        )
    result = FlightCommand.Result()
    context._command_result.clear()
    try:
        pending = context._bridge.start(
            request,
            context._steady_now(),
            _goal_timeout(context, goal),
        )
    except (CommandRejectedError, ValueError) as error:
        result.result_code = FlightCommand.Result.RESULT_REJECTED
        result.reason = str(error)
        goal_handle.abort()
        _log_result(context, goal, result)
        return result
    feedback = FlightCommand.Feedback()
    feedback.execution_state = FlightCommand.Feedback.STATE_SENT
    feedback.correlation_id = goal.correlation_id
    goal_handle.publish_feedback(feedback)
    context._command_result.wait(_goal_timeout(context, goal) + 0.10)
    completed = context._bridge.actions.last_result
    if completed is None or completed.command is not pending.command:
        completed = context._bridge.tick(context._steady_now())
    if completed is None:
        result.result_code = FlightCommand.Result.RESULT_TIMEOUT
        result.reason = "V7 acknowledgement deadline elapsed"
        goal_handle.abort()
        return result
    result.result_code, result.reason = _legacy_result(completed.code, completed.reason)
    result.completed_stamp = context.get_clock().now().to_msg()
    if completed.code is ResultCode.SUCCEEDED:
        goal_handle.succeed()
    else:
        goal_handle.abort()
    _log_result(context, goal, result)
    return result


def _legacy_request(
    context: ActionNodeContext,
    goal: FlightCommand.Goal,
) -> CommandRequest | None:
    match goal.command:
        case FlightCommand.Goal.COMMAND_ARM:
            return CommandRequest.unlock()
        case FlightCommand.Goal.COMMAND_DISARM:
            return CommandRequest(CommandKind.LOCK)
        case FlightCommand.Goal.COMMAND_SET_MODE:
            return CommandRequest(CommandKind.SET_MODE, mode=goal.requested_mode)
        case FlightCommand.Goal.COMMAND_TAKEOFF:
            return CommandRequest(
                CommandKind.TAKEOFF,
                height_cm=round(goal.target_pose.pose.position.z * 100.0),
            )
        case FlightCommand.Goal.COMMAND_MOVE:
            return _legacy_move_request(context, goal)
        case FlightCommand.Goal.COMMAND_HOVER:
            return CommandRequest.hover()
        case FlightCommand.Goal.COMMAND_LAND:
            return CommandRequest.land()
        case _:
            return None


def _legacy_move_request(
    context: ActionNodeContext,
    goal: FlightCommand.Goal,
) -> CommandRequest | None:
    position = context._bridge.snapshot(context._steady_now()).position
    if position is None or not position.valid:
        return None
    forward_m = goal.target_pose.pose.position.y - position.forward_m
    right_m = goal.target_pose.pose.position.x - position.right_m
    distance_cm = round(math.hypot(forward_m, right_m) * 100.0)
    direction_deg = round(math.degrees(math.atan2(right_m, forward_m)) % 360.0)
    requested_speed = math.hypot(
        goal.target_velocity.linear.x,
        goal.target_velocity.linear.y,
    )
    speed_cmps = (
        round(requested_speed * 100.0)
        if requested_speed > 0.0
        else int(context.get_parameter("move_speed_cmps").value)
    )
    return CommandRequest.move(distance_cm, speed_cmps, direction_deg)


def _goal_timeout(context: ActionNodeContext, goal: FlightCommand.Goal) -> float:
    if goal.timeout_sec > 0.0:
        return goal.timeout_sec
    return float(context.get_parameter("command_ack_timeout_s").value)


def _legacy_result(code: ResultCode, reason: str) -> tuple[int, str]:
    match code:
        case ResultCode.SUCCEEDED:
            return FlightCommand.Result.RESULT_SUCCEEDED, reason
        case ResultCode.TIMEOUT:
            return FlightCommand.Result.RESULT_TIMEOUT, reason
        case ResultCode.REJECTED:
            return FlightCommand.Result.RESULT_REJECTED, reason
        case ResultCode.FCU_ERROR:
            return FlightCommand.Result.RESULT_FCU_ERROR, reason


def _reject_goal(
    goal_handle: FlightGoalHandle,
    reason: str,
) -> FlightCommand.Result:
    result = FlightCommand.Result()
    result.result_code = FlightCommand.Result.RESULT_REJECTED
    result.reason = reason
    goal_handle.abort()
    return result
