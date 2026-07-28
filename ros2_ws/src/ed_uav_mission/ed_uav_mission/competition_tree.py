from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto


class CompetitionStep(Enum):
    TAKEOFF = auto()
    HOVER = auto()
    NAVIGATE_FORWARD = auto()
    NAVIGATE_RETURN = auto()
    LAND = auto()
    DISARM = auto()


@dataclass(frozen=True, slots=True)
class MapPose:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("map pose values must be finite")


@dataclass(frozen=True, slots=True)
class MoveGoal:
    x_m: float
    y_m: float
    altitude_m: float
    yaw_rad: float
    label: str


def competition_sequence() -> tuple[CompetitionStep, ...]:
    return (
        CompetitionStep.TAKEOFF,
        CompetitionStep.HOVER,
        CompetitionStep.NAVIGATE_FORWARD,
        CompetitionStep.NAVIGATE_RETURN,
        CompetitionStep.LAND,
        CompetitionStep.DISARM,
    )


def forward_goal(start: MapPose, distance_m: float) -> MapPose:
    if not math.isfinite(distance_m) or distance_m <= 0.0:
        raise ValueError("forward distance must be finite and positive")
    return MapPose(
        x_m=start.x_m + distance_m * math.cos(start.yaw_rad),
        y_m=start.y_m + distance_m * math.sin(start.yaw_rad),
        yaw_rad=start.yaw_rad,
    )


def return_goal(start: MapPose) -> MapPose:
    return start


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    if not all(math.isfinite(value) for value in (qx, qy, qz, qw)):
        raise ValueError("quaternion values must be finite")
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def moves_from_planner_path(
    path: tuple[MapPose, ...], *, altitude_m: float, label: str
) -> tuple[MoveGoal, ...]:
    if not path:
        raise ValueError("planner path is empty")
    if not math.isfinite(altitude_m) or altitude_m <= 0.0:
        raise ValueError("move altitude must be finite and positive")
    return tuple(
        MoveGoal(
            x_m=pose.x_m,
            y_m=pose.y_m,
            altitude_m=altitude_m,
            yaw_rad=pose.yaw_rad,
            label=f"{label}_{index}",
        )
        for index, pose in enumerate(path)
    )
