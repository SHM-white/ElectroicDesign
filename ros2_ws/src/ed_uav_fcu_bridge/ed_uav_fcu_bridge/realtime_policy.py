"""Configuration, safety gates, and velocity policy for V7 realtime control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Final, TypeAlias

from .telemetry import PositionSample, TelemetrySnapshot
from .v7_codec import RealtimeControlFields

# 需无桨确认机型/固件轴向；不得依据 0x08 位置字段盲推 SPD_Y 符号。
REALTIME_SPD_Y_SIGN: Final = 1

# 本地手册死区，均为可调常量；修改前需无桨确认遥控中位与机型/固件行为。
RC_CENTER_US: Final = 1500
ROLL_PITCH_DEADBAND_US: Final = 40
THR_YAW_DEADBAND_US: Final = 80
AUX1_MODE_2_MIN_US: Final = 1400
AUX1_MODE_2_MAX_US: Final = 1600
ZERO_CONTROL: Final = RealtimeControlFields(0, 0, 0, 0, 0, 0, 0)


@dataclass(frozen=True, slots=True)
class RealtimeControlConfig:
    """Safety gates and tunables for the V7 realtime stream."""

    enable_realtime_control: bool = False
    # Matches the existing 20 ms poll as a starting point; tune without propellers.
    # This is not claimed to be a protocol-mandated transmission rate.
    stream_period_s: float = 0.02
    stop_frame_count: int = 3
    position_tolerance_m: float = 0.05
    proportional_gain_cmps_per_m: float = 100.0
    # Require distinct fresh arrivals to avoid completing on one transient sample.
    arrival_confirmation_samples: int = 3

    def __post_init__(self) -> None:
        if self.stream_period_s <= 0.0:
            raise ValueError("realtime stream period must be positive")
        if self.stop_frame_count < 1:
            raise ValueError("realtime stop frame count must be at least one")
        if self.position_tolerance_m < 0.0:
            raise ValueError("realtime position tolerance cannot be negative")
        if self.proportional_gain_cmps_per_m <= 0.0:
            raise ValueError("realtime proportional gain must be positive")
        if self.arrival_confirmation_samples < 1:
            raise ValueError("realtime arrival confirmation must be at least one")


@dataclass(frozen=True, slots=True)
class PositionTarget:
    forward_m: float
    right_m: float


@dataclass(frozen=True, slots=True)
class RealtimeMoveRequest:
    target: PositionTarget
    max_speed_cmps: int
    timeout_s: float


@dataclass(frozen=True, slots=True)
class RealtimeHoverRequest:
    duration_s: float


RealtimeRequest: TypeAlias = RealtimeMoveRequest | RealtimeHoverRequest


class RealtimeResultCode(Enum):
    SUCCEEDED = auto()
    CANCELLED = auto()
    TIMEOUT = auto()
    CONTROL_GATED = auto()
    REJECTED = auto()
    FCU_ERROR = auto()


@dataclass(frozen=True, slots=True)
class RealtimeResult:
    code: RealtimeResultCode
    reason: str
    completed_steady_s: float


def _strictly_centered(channel_us: int, deadband_us: int) -> bool:
    return RC_CENTER_US - deadband_us < channel_us < RC_CENTER_US + deadband_us


def nonzero_control_allowed(
    config: RealtimeControlConfig,
    snapshot: TelemetrySnapshot,
) -> bool:
    """Require fresh position, mode 2, and strict local-manual RC deadbands."""
    position = snapshot.position
    status = snapshot.status
    aux = snapshot.aux
    if (
        not config.enable_realtime_control
        or position is None
        or not position.valid
        or status is None
        or not status.valid
        or status.mode != 2
        or aux is None
        or not aux.valid
        or len(aux.channels_us) != 10
    ):
        return False
    roll_us, pitch_us, throttle_us, yaw_us = aux.channels_us[:4]
    return (
        AUX1_MODE_2_MIN_US < aux.aux1_us < AUX1_MODE_2_MAX_US
        and _strictly_centered(roll_us, ROLL_PITCH_DEADBAND_US)
        and _strictly_centered(pitch_us, ROLL_PITCH_DEADBAND_US)
        and _strictly_centered(throttle_us, THR_YAW_DEADBAND_US)
        and _strictly_centered(yaw_us, THR_YAW_DEADBAND_US)
    )


def move_control(
    position: PositionSample,
    request: RealtimeMoveRequest,
    config: RealtimeControlConfig,
) -> RealtimeControlFields:
    """Map forward/right position error to bounded V7 horizontal velocity."""
    forward_error_m = request.target.forward_m - position.forward_m
    right_error_m = request.target.right_m - position.right_m
    distance_m = math.hypot(forward_error_m, right_error_m)
    if distance_m <= config.position_tolerance_m:
        return ZERO_CONTROL
    forward_cmps = forward_error_m * config.proportional_gain_cmps_per_m
    right_cmps = right_error_m * config.proportional_gain_cmps_per_m
    requested_speed = math.hypot(forward_cmps, right_cmps)
    scale = min(1.0, request.max_speed_cmps / requested_speed)
    return RealtimeControlFields(
        roll=0,
        pitch=0,
        thr=0,
        yaw_dps=0,
        spd_x=round(forward_cmps * scale),
        spd_y=round(right_cmps * scale) * REALTIME_SPD_Y_SIGN,
        spd_z=0,
    )
