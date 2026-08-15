"""Typed observations and events accepted by the D-task reducer."""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import assert_never

from ed_uav_mission.d_task_model import (
    DTaskEffect,
    DTaskFault,
    PayloadState,
    RouteStage,
)
from ed_uav_mission.touchdown import TouchdownUpdate


@dataclass(frozen=True, slots=True)
class DTaskRuntimeConfig:
    stable_s: float = 3.0
    start_deadline_s: float = 15.0
    b_deadline_s: float = 45.0
    d_deadline_s: float = 75.0
    mission_deadline_s: float = 90.0
    vehicle_freshness_s: float = 0.5
    target_freshness_s: float = 0.2
    maximum_relative_error_m: float = 2.0
    right_offset_m: float = 0.75
    search_distance_m: float = 2.0


@dataclass(frozen=True, slots=True)
class VehicleSnapshot:
    observed_at_s: float
    sequence: int
    started: bool
    heartbeat_alive: bool
    speed_m_s: float
    displacement_m: float
    heading_rad: float
    yaw_rate_rad_s: float
    route_stage: RouteStage


@dataclass(frozen=True, slots=True)
class TargetSnapshot:
    observed_at_s: float
    sequence: int
    valid: bool
    relative_x_m: float
    relative_y_m: float
    relative_z_m: float
    relative_error_m: float
    rejection_reason: str = ""


@dataclass(frozen=True, slots=True)
class Tick:
    now_s: float


@dataclass(frozen=True, slots=True)
class VehicleObserved:
    now_s: float
    vehicle: VehicleSnapshot
    payload_state: PayloadState = PayloadState.SECURED


@dataclass(frozen=True, slots=True)
class TargetObserved:
    now_s: float
    target: TargetSnapshot


@dataclass(frozen=True, slots=True)
class CommandCompleted:
    now_s: float
    effect: DTaskEffect


@dataclass(frozen=True, slots=True)
class CommandFailed:
    now_s: float
    effect: DTaskEffect
    reason: str


@dataclass(frozen=True, slots=True)
class ContactObserved:
    update: TouchdownUpdate


@dataclass(frozen=True, slots=True)
class SafetyInterrupted:
    now_s: float
    fault: DTaskFault
    reason: str


DTaskEvent = (
    Tick
    | VehicleObserved
    | TargetObserved
    | CommandCompleted
    | CommandFailed
    | ContactObserved
    | SafetyInterrupted
)


def event_time(event: DTaskEvent) -> float:
    match event:
        case ContactObserved(update=update):
            return update.now_monotonic_s
        case Tick(now_s=now_s) | VehicleObserved(now_s=now_s) | TargetObserved(now_s=now_s):
            return now_s
        case CommandCompleted(now_s=now_s) | CommandFailed(now_s=now_s):
            return now_s
        case SafetyInterrupted(now_s=now_s):
            return now_s
        case unreachable:
            assert_never(unreachable)
