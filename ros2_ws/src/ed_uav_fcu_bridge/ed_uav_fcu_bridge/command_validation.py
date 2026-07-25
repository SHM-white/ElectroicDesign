"""Boundary validation for ROS FlightCommand goals before V7 conversion."""

from __future__ import annotations

import math
from typing import Final

from ed_uav_interfaces.action import FlightCommand

MAX_GOAL_TIMEOUT_S: Final = 60.0
MAX_TAKEOFF_HEIGHT_CM: Final = 0xFFFF
MIN_MOVE_SPEED_CMPS: Final = 10
MAX_MOVE_SPEED_CMPS: Final = 300


def goal_rejection_reason(goal: FlightCommand.Goal) -> str | None:
    timeout_reason = _timeout_rejection_reason(goal.timeout_sec)
    if timeout_reason is not None:
        return timeout_reason
    match goal.command:
        case FlightCommand.Goal.COMMAND_ARM:
            return None
        case FlightCommand.Goal.COMMAND_DISARM:
            return None
        case FlightCommand.Goal.COMMAND_SET_MODE:
            if goal.requested_mode not in (0, 1, 2, 3):
                return "requested mode must be in the inclusive range 0..3"
            return None
        case FlightCommand.Goal.COMMAND_TAKEOFF:
            return _takeoff_rejection_reason(goal)
        case FlightCommand.Goal.COMMAND_MOVE:
            return _move_rejection_reason(goal)
        case FlightCommand.Goal.COMMAND_HOVER:
            return None
        case FlightCommand.Goal.COMMAND_LAND:
            return None
        case _:
            return None


def _timeout_rejection_reason(timeout_sec: float) -> str | None:
    if timeout_sec == 0.0:
        return None
    if not math.isfinite(timeout_sec) or timeout_sec < 0.0 or timeout_sec > MAX_GOAL_TIMEOUT_S:
        return f"timeout_sec must be 0 for the configured default or finite in range (0, {MAX_GOAL_TIMEOUT_S}]"
    return None


def _takeoff_rejection_reason(goal: FlightCommand.Goal) -> str | None:
    height_m = goal.target_pose.pose.position.z
    if not math.isfinite(height_m) or height_m < 0.0:
        return "takeoff height must be finite and non-negative"
    height_cm = round(height_m * 100.0)
    if height_cm > MAX_TAKEOFF_HEIGHT_CM:
        return "takeoff height must fit the V7 uint16 centimeter field"
    return None


def _move_rejection_reason(goal: FlightCommand.Goal) -> str | None:
    position = goal.target_pose.pose.position
    velocity = goal.target_velocity.linear
    if not all(
        math.isfinite(value)
        for value in (position.x, position.y, velocity.x, velocity.y)
    ):
        return "move pose and requested speed fields must be finite"
    requested_speed = math.hypot(velocity.x, velocity.y)
    speed_cmps = round(requested_speed * 100.0)
    if requested_speed > 0.0 and not MIN_MOVE_SPEED_CMPS <= speed_cmps <= MAX_MOVE_SPEED_CMPS:
        return "move requested speed must be zero for the configured default or fit the V7 speed range"
    return None
