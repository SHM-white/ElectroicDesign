"""Action-boundary regressions for invalid FCU FlightCommand goals."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import pytest
from ed_uav_interfaces.action import FlightCommand
from ed_uav_fcu_bridge import node as bridge_node


@dataclass(slots=True)
class RecordingGoalHandle:  # noqa: MUTABLE_OK
    """Mutable action-handle fake used to verify boundary rejection effects."""

    request: FlightCommand.Goal
    aborted: bool = False
    succeeded: bool = False
    feedback: list[FlightCommand.Feedback] = field(default_factory=list)

    def abort(self) -> None:
        self.aborted = True

    def succeed(self) -> None:
        self.succeeded = True

    def publish_feedback(self, feedback: FlightCommand.Feedback) -> None:
        self.feedback.append(feedback)


class ForbiddenCommandEvent:
    def clear(self) -> None:
        pytest.fail("invalid command cleared the wait event")

    def wait(self, timeout: float | None = None) -> bool:
        pytest.fail(f"invalid command waited for completion: {timeout}")


class ForbiddenBridge:
    def start(self, request: object, steady_now: float, timeout_s: float) -> object:
        pytest.fail(f"invalid command reached bridge transport: {request}")

    def tick(self, steady_now: float) -> object:
        pytest.fail(f"invalid command ticked bridge transport: {steady_now}")


def _new_goal(command: int) -> FlightCommand.Goal:
    goal = FlightCommand.Goal()
    goal.command = command
    goal.timeout_sec = 0.5
    return goal


def _execute_invalid_goal(goal: FlightCommand.Goal) -> FlightCommand.Result:
    node = bridge_node.FcuBridgeNode.__new__(bridge_node.FcuBridgeNode)
    node._command_result = ForbiddenCommandEvent()
    node._bridge = ForbiddenBridge()
    goal_handle = RecordingGoalHandle(goal)

    result = bridge_node.FcuBridgeNode._execute(node, goal_handle)

    assert goal_handle.aborted is True
    assert goal_handle.succeeded is False
    assert goal_handle.feedback == []
    return result


@pytest.mark.parametrize(
    ("timeout_sec", "reason_fragment"),
    (
        (math.inf, "timeout"),
        (math.nan, "timeout"),
        (-0.1, "timeout"),
        (60.1, "timeout"),
    ),
    ids=("infinite-timeout", "nan-timeout", "negative-timeout", "oversized-timeout"),
)
def test_invalid_command_timeout_is_rejected_before_wait(
    timeout_sec: float,
    reason_fragment: str,
) -> None:
    # Given: a syntactically supported command with an invalid boundary timeout.
    goal = _new_goal(FlightCommand.Goal.COMMAND_HOVER)
    goal.timeout_sec = timeout_sec

    # When: the node action boundary executes the goal.
    result = _execute_invalid_goal(goal)

    # Then: invalid timeouts are rejected before Event.wait() can raise or block.
    assert result.result_code == FlightCommand.Result.RESULT_REJECTED
    assert reason_fragment in result.reason


@pytest.mark.parametrize(
    ("height_m", "reason_fragment"),
    (
        (math.inf, "takeoff height"),
        (math.nan, "takeoff height"),
        (-0.01, "takeoff height"),
        (655.36, "takeoff height"),
    ),
    ids=("infinite-height", "nan-height", "negative-height", "height-over-u16"),
)
def test_invalid_takeoff_height_is_rejected_before_encoding(
    height_m: float,
    reason_fragment: str,
) -> None:
    # Given: a takeoff command whose height cannot produce a safe V7 u16 payload.
    goal = _new_goal(FlightCommand.Goal.COMMAND_TAKEOFF)
    goal.target_pose.pose.position.z = height_m

    # When: the node action boundary executes the goal.
    result = _execute_invalid_goal(goal)

    # Then: invalid heights return RESULT_REJECTED rather than OverflowError.
    assert result.result_code == FlightCommand.Result.RESULT_REJECTED
    assert reason_fragment in result.reason


@pytest.mark.parametrize(
    ("field_name", "field_value"),
    (
        ("x", math.inf),
        ("y", math.nan),
        ("vx", math.inf),
        ("vy", math.nan),
    ),
    ids=("pose-x-inf", "pose-y-nan", "velocity-x-inf", "velocity-y-nan"),
)
def test_invalid_move_numeric_fields_are_rejected_before_conversion(
    field_name: str,
    field_value: float,
) -> None:
    # Given: a move command with one non-finite pose or requested-speed field.
    goal = _new_goal(FlightCommand.Goal.COMMAND_MOVE)
    match field_name:
        case "x":
            goal.target_pose.pose.position.x = field_value
        case "y":
            goal.target_pose.pose.position.y = field_value
        case "vx":
            goal.target_velocity.linear.x = field_value
        case "vy":
            goal.target_velocity.linear.y = field_value
        case unreachable:
            raise AssertionError(unreachable)

    # When: the node action boundary executes the goal.
    result = _execute_invalid_goal(goal)

    # Then: bad numeric input is rejected before math/round/bridge transport.
    assert result.result_code == FlightCommand.Result.RESULT_REJECTED
    assert "move" in result.reason
