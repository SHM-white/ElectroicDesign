"""Configuration and bounded velocity policy for V7 realtime control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Final, TypeAlias

from .telemetry import PositionSample, TelemetrySnapshot
from .v7_codec import RealtimeControlFields

# 需无桨确认机型/固件轴向；不得依据 0x08 位置字段盲推 SPD_Y 符号。
REALTIME_SPD_Y_SIGN: Final = 1

ZERO_CONTROL: Final = RealtimeControlFields(0, 0, 0, 0, 0, 0, 0)


@dataclass(frozen=True, slots=True)
class RealtimeControlConfig:
    """Safety gates and tunables for the V7 realtime stream."""

    # 硬件默认关闭；仅在显式验证后启用 0x41 实时控制
    enable_realtime_control: bool = False
    # 以现有 20ms 轮询为起点，后续需无桨实测调参；不是协议规定频率
    stream_period_s: float = 0.02
    # 停止帧数量，用于结束/中断后稳定输出归零
    stop_frame_count: int = 3
    # 到达判定容差，避免过早判定目标到达
    position_tolerance_m: float = 0.05
    # 速度比例增益，需无桨实测校准
    proportional_gain_cmps_per_m: float = 100.0
    # 连续新鲜到达帧阈值，防止单帧抖动误判到达
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


def nonzero_control_allowed(
    config: RealtimeControlConfig,
    snapshot: TelemetrySnapshot,
) -> bool:
    """Require only the configured backend and a usable position sample.

    AUX mode windows and stick-centering duplicated the external pilot boundary
    and made autonomous commands intermittently impossible.  The independent
    emergency-lock latch remains enforced in ``RealtimeController``.
    """
    position = snapshot.position
    return bool(
        config.enable_realtime_control
        and position is not None
        and position.valid
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
