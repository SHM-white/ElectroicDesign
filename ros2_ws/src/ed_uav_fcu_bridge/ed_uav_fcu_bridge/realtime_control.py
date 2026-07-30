"""Fail-closed Lingxiao V7 0x41 realtime MOVE and HOVER control."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Final, Protocol, TypeAlias

from typing_extensions import assert_never

from .telemetry import PositionSample, TelemetrySnapshot
from .v7_codec import RealtimeControlFields, cmd_realtime_control

ED_UAV_LINGXIAO_REALTIME_CONTROL: Final = True

# 需无桨确认机型/固件轴向；不得依据 0x08 位置字段盲推 SPD_Y 符号。
REALTIME_SPD_Y_SIGN: Final = 1

MODE_2_CHANNEL_MIN_US: Final = 1400
MODE_2_CHANNEL_MAX_US: Final = 1600
ZERO_CONTROL: Final = RealtimeControlFields(0, 0, 0, 0, 0, 0, 0)


class WireWriter(Protocol):
    def __call__(self, data: bytes) -> int | None: ...


class SnapshotProvider(Protocol):
    def __call__(self, steady_now: float) -> TelemetrySnapshot: ...


class SteadyClock(Protocol):
    def __call__(self) -> float: ...


class Sleeper(Protocol):
    def __call__(self, duration_s: float, /) -> None: ...


class CancellationProbe(Protocol):
    def __call__(self) -> bool: ...


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

    def __post_init__(self) -> None:
        if self.stream_period_s <= 0.0:
            raise ValueError("realtime stream period must be positive")
        if self.stop_frame_count < 1:
            raise ValueError("realtime stop frame count must be at least one")
        if self.position_tolerance_m < 0.0:
            raise ValueError("realtime position tolerance cannot be negative")
        if self.proportional_gain_cmps_per_m <= 0.0:
            raise ValueError("realtime proportional gain must be positive")


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


@dataclass(frozen=True, slots=True)
class RealtimeDependencies:
    writer: WireWriter
    snapshot: SnapshotProvider
    clock: SteadyClock
    sleeper: Sleeper


class SerializedWireWriter:
    """Serialize complete frame writes shared by the 0xE0 and 0x41 paths."""

    def __init__(self, writer: WireWriter) -> None:
        self._writer = writer
        self._lock = threading.Lock()

    def __call__(self, data: bytes) -> int | None:
        with self._lock:
            return self._writer(data)


class WireWriteError(RuntimeError):
    """Raised when the transport does not accept one complete V7 frame."""


def use_realtime_backend(realtime_capable_command: bool) -> bool:
    """Apply the source-level rollback macro to MOVE/HOVER backend selection."""
    return ED_UAV_LINGXIAO_REALTIME_CONTROL and realtime_capable_command


def nonzero_control_allowed(
    config: RealtimeControlConfig,
    snapshot: TelemetrySnapshot,
) -> bool:
    """Require the complete V7 mode-2 safety gate before nonzero velocity."""
    position = snapshot.position
    status = snapshot.status
    aux = snapshot.aux
    return (
        config.enable_realtime_control
        and position is not None
        and position.valid
        and status is not None
        and status.valid
        and status.mode == 2
        and aux is not None
        and aux.valid
        and len(aux.channels_us) == 10
        and MODE_2_CHANNEL_MIN_US <= aux.aux1_us <= MODE_2_CHANNEL_MAX_US
        and all(
            MODE_2_CHANNEL_MIN_US <= channel <= MODE_2_CHANNEL_MAX_US
            for channel in aux.channels_us[:4]
        )
    )


def _move_control(
    position: PositionSample,
    request: RealtimeMoveRequest,
    config: RealtimeControlConfig,
) -> RealtimeControlFields:
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


class RealtimeController:
    """Single-flight monotonic MOVE/HOVER stream with terminal stop frames."""

    def __init__(
        self,
        dependencies: RealtimeDependencies,
        config: RealtimeControlConfig,
    ) -> None:
        self._dependencies = dependencies
        self.config = config
        self._execution_lock = threading.Lock()

    def execute(
        self,
        request: RealtimeRequest,
        cancelled: CancellationProbe,
    ) -> RealtimeResult:
        """Run one realtime request and stop output on every started terminal path."""
        now = self._dependencies.clock()
        if not self.config.enable_realtime_control:
            return RealtimeResult(
                RealtimeResultCode.REJECTED,
                "realtime control is not explicitly enabled",
                now,
            )
        if not self._execution_lock.acquire(blocking=False):
            return RealtimeResult(
                RealtimeResultCode.REJECTED,
                "another realtime command is already executing",
                now,
            )
        try:
            try:
                result = self._dispatch(request, cancelled)
            except WireWriteError as error:
                result = RealtimeResult(
                    RealtimeResultCode.FCU_ERROR,
                    str(error),
                    self._dependencies.clock(),
                )
            try:
                self._send_stop_frames()
            except WireWriteError as error:
                return RealtimeResult(
                    RealtimeResultCode.FCU_ERROR,
                    str(error),
                    self._dependencies.clock(),
                )
            return result
        finally:
            self._execution_lock.release()

    def _dispatch(
        self,
        request: RealtimeRequest,
        cancelled: CancellationProbe,
    ) -> RealtimeResult:
        match request:
            case RealtimeMoveRequest():
                return self._run_move(request, cancelled)
            case RealtimeHoverRequest():
                return self._run_hover(request, cancelled)
            case unreachable:
                assert_never(unreachable)

    def _run_move(
        self,
        request: RealtimeMoveRequest,
        cancelled: CancellationProbe,
    ) -> RealtimeResult:
        started = self._dependencies.clock()
        deadline = started + request.timeout_s
        while True:
            now = self._dependencies.clock()
            if cancelled():
                return RealtimeResult(RealtimeResultCode.CANCELLED, "goal canceled", now)
            if now >= deadline:
                return RealtimeResult(RealtimeResultCode.TIMEOUT, "MOVE deadline elapsed", now)
            snapshot = self._dependencies.snapshot(now)
            if not nonzero_control_allowed(self.config, snapshot):
                return RealtimeResult(
                    RealtimeResultCode.CONTROL_GATED,
                    "MOVE realtime mode gate is not satisfied",
                    now,
                )
            position = snapshot.position
            if position is None:
                return RealtimeResult(
                    RealtimeResultCode.CONTROL_GATED,
                    "MOVE position is unavailable",
                    now,
                )
            fields = _move_control(position, request, self.config)
            if fields == ZERO_CONTROL:
                return RealtimeResult(RealtimeResultCode.SUCCEEDED, "MOVE target reached", now)
            self._write(fields)
            self._dependencies.sleeper(self.config.stream_period_s)

    def _run_hover(
        self,
        request: RealtimeHoverRequest,
        cancelled: CancellationProbe,
    ) -> RealtimeResult:
        deadline = self._dependencies.clock() + request.duration_s
        while True:
            now = self._dependencies.clock()
            if cancelled():
                return RealtimeResult(RealtimeResultCode.CANCELLED, "goal canceled", now)
            if now >= deadline:
                return RealtimeResult(RealtimeResultCode.SUCCEEDED, "HOVER duration completed", now)
            self._write(ZERO_CONTROL)
            self._dependencies.sleeper(self.config.stream_period_s)

    def _send_stop_frames(self) -> None:
        for _ in range(self.config.stop_frame_count):
            self._write(ZERO_CONTROL)

    def _write(self, fields: RealtimeControlFields) -> None:
        raw = cmd_realtime_control(fields)
        written = self._dependencies.writer(raw)
        if written is not None and written != len(raw):
            raise WireWriteError("serial transport did not accept the complete V7 realtime frame")
