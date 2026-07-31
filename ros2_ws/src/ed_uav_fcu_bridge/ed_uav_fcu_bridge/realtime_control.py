"""Fail-closed Lingxiao V7 0x41 realtime MOVE and HOVER controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Protocol

from typing_extensions import assert_never

from .command_arbiter import CommandArbiter, SerializedWireWriter, WireWriter
from .realtime_policy import (
    REALTIME_SPD_Y_SIGN,
    ZERO_CONTROL,
    PositionTarget,
    RealtimeControlConfig,
    RealtimeHoverRequest,
    RealtimeMoveRequest,
    RealtimeRequest,
    RealtimeResult,
    RealtimeResultCode,
    move_control,
    nonzero_control_allowed,
)
from .telemetry import TelemetrySnapshot
from .v7_codec import RealtimeControlFields, cmd_realtime_control

ED_UAV_LINGXIAO_REALTIME_CONTROL: Final = True


class SnapshotProvider(Protocol):
    def __call__(self, steady_now: float) -> TelemetrySnapshot: ...


class SteadyClock(Protocol):
    def __call__(self) -> float: ...


class Sleeper(Protocol):
    def __call__(self, duration_s: float, /) -> None: ...


class CancellationProbe(Protocol):
    def __call__(self) -> bool: ...


class EmergencyLockProbe(Protocol):
    def __call__(self) -> bool: ...


@dataclass(frozen=True, slots=True)
class RealtimeDependencies:
    writer: WireWriter
    snapshot: SnapshotProvider
    clock: SteadyClock
    sleeper: Sleeper


class WireWriteError(RuntimeError):
    """Raised when the transport does not accept one complete V7 frame."""


def use_realtime_backend(realtime_capable_command: bool) -> bool:
    """Apply the source-level rollback macro to MOVE/HOVER backend selection."""
    return ED_UAV_LINGXIAO_REALTIME_CONTROL and realtime_capable_command


class RealtimeController:
    """Monotonic MOVE/HOVER stream sharing one semantic FCU command lease."""

    def __init__(
        self,
        dependencies: RealtimeDependencies,
        config: RealtimeControlConfig,
        arbiter: CommandArbiter | None = None,
    ) -> None:
        # 调参：stream_period_s / stop_frame_count / position_tolerance_m 均来自 RealtimeControlConfig
        self._dependencies = dependencies
        self.config = config
        self._arbiter = arbiter if arbiter is not None else CommandArbiter()
        self._emergency_lock_active: EmergencyLockProbe = lambda: False

    def set_emergency_lock_probe(self, probe: EmergencyLockProbe) -> None:
        self._emergency_lock_active = probe

    def execute(
        self,
        request: RealtimeRequest,
        cancelled: CancellationProbe,
    ) -> RealtimeResult:
        """Run one request and stop output on every started terminal path."""
        now = self._dependencies.clock()
        if self._emergency_lock_active():
            return self._emergency_result(now)
        if not self.config.enable_realtime_control:
            return RealtimeResult(
                RealtimeResultCode.REJECTED,
                "realtime control is not explicitly enabled",
                now,
            )
        if not self._arbiter.try_acquire():
            return RealtimeResult(
                RealtimeResultCode.REJECTED,
                "another FCU command is already active",
                now,
            )
        try:
            if self._emergency_lock_active():
                return self._emergency_result(self._dependencies.clock())
            try:
                result = self._dispatch(request, cancelled)
            except WireWriteError as error:
                result = RealtimeResult(
                    RealtimeResultCode.FCU_ERROR,
                    str(error),
                    self._dependencies.clock(),
                )
            if not self._emergency_lock_active():
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
            self._arbiter.release()

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
        deadline = self._dependencies.clock() + request.timeout_s
        last_sequence: int | None = None
        arrival_count = 0
        while True:
            now = self._dependencies.clock()
            terminal = self._loop_terminal(cancelled, now, deadline, "MOVE")
            if terminal is not None:
                return terminal
            snapshot = self._dependencies.snapshot(now)
            if not nonzero_control_allowed(self.config, snapshot):
                return self._gated("MOVE", now)
            position = snapshot.position
            if position is None:
                return self._gated("MOVE", now)
            fields = move_control(position, request, self.config)
            if position.source_sequence != last_sequence:
                last_sequence = position.source_sequence
                arrival_count = arrival_count + 1 if fields == ZERO_CONTROL else 0
            if arrival_count >= self.config.arrival_confirmation_samples:
                return RealtimeResult(
                    RealtimeResultCode.SUCCEEDED,
                    "MOVE target reached",
                    now,
                )
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
            terminal = self._loop_terminal(cancelled, now, deadline, "HOVER")
            if terminal is not None:
                if terminal.code is RealtimeResultCode.TIMEOUT:
                    return RealtimeResult(
                        RealtimeResultCode.SUCCEEDED,
                        "HOVER duration completed",
                        now,
                    )
                return terminal
            if not nonzero_control_allowed(
                self.config,
                self._dependencies.snapshot(now),
            ):
                return self._gated("HOVER", now)
            # 保持当前航向并持续输出零速度，直到悬停时间结束
            self._write(ZERO_CONTROL)
            self._dependencies.sleeper(self.config.stream_period_s)

    def _loop_terminal(
        self,
        cancelled: CancellationProbe,
        now: float,
        deadline: float,
        command_name: str,
    ) -> RealtimeResult | None:
        if self._emergency_lock_active():
            return self._emergency_result(now)
        if cancelled():
            return RealtimeResult(RealtimeResultCode.CANCELLED, "goal canceled", now)
        if now >= deadline:
            return RealtimeResult(
                RealtimeResultCode.TIMEOUT,
                f"{command_name} deadline elapsed",
                now,
            )
        return None

    @staticmethod
    def _gated(command_name: str, now: float) -> RealtimeResult:
        return RealtimeResult(
            RealtimeResultCode.CONTROL_GATED,
            f"{command_name} realtime mode gate is not satisfied",
            now,
        )

    @staticmethod
    def _emergency_result(now: float) -> RealtimeResult:
        return RealtimeResult(RealtimeResultCode.REJECTED, "emergency lock is latched", now)

    def _send_stop_frames(self) -> None:
        for _ in range(self.config.stop_frame_count):
            self._write(ZERO_CONTROL)

    def _write(self, fields: RealtimeControlFields) -> None:
        if self._emergency_lock_active():
            return
        raw = cmd_realtime_control(fields)
        written = self._dependencies.writer(raw)
        if written is not None and written != len(raw):
            raise WireWriteError(
                "serial transport did not accept the complete V7 realtime frame"
            )
