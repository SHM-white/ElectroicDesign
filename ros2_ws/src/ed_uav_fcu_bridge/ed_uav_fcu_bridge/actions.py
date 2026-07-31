"""ACK-correlated high-level V7 command state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from .command_arbiter import CommandArbiter
from .v7_codec import (
    V7Frame,
    cmd_ascend,
    cmd_descend,
    cmd_hover,
    cmd_land,
    cmd_lock,
    cmd_mode,
    cmd_move,
    cmd_takeoff,
    cmd_target_height,
    cmd_target_position,
    cmd_unlock,
)


class CommandKind(Enum):
    UNLOCK = auto()
    SET_MODE = auto()
    TAKEOFF = auto()
    MOVE = auto()
    HOVER = auto()
    LAND = auto()
    LOCK = auto()
    TARGET_POSITION = auto()
    TARGET_HEIGHT = auto()
    ASCEND = auto()
    DESCEND = auto()


class ResultCode(Enum):
    SUCCEEDED = auto()
    REJECTED = auto()
    TIMEOUT = auto()
    FCU_ERROR = auto()


@dataclass(frozen=True, slots=True)
class MoveSpec:
    distance_cm: int
    speed_cmps: int
    direction_deg: int


@dataclass(frozen=True, slots=True)
class TargetPositionSpec:
    """Manual-defined position fields whose axes are not named in the cited pages."""

    first_cm: int
    second_cm: int


@dataclass(frozen=True, slots=True)
class VerticalSpec:
    distance_cm: int
    speed_cmps: int


@dataclass(frozen=True, slots=True)
class CommandRequest:
    command: CommandKind
    mode: int | None = None
    height_cm: int | None = None
    move_spec: MoveSpec | None = None
    target_position_spec: TargetPositionSpec | None = None
    vertical_spec: VerticalSpec | None = None

    @classmethod
    def unlock(cls) -> CommandRequest:
        return cls(CommandKind.UNLOCK)

    @classmethod
    def land(cls) -> CommandRequest:
        return cls(CommandKind.LAND)

    @classmethod
    def hover(cls) -> CommandRequest:
        return cls(CommandKind.HOVER)

    @classmethod
    def move(cls, distance_cm: int, speed_cmps: int, direction_deg: int) -> CommandRequest:
        return cls(CommandKind.MOVE, move_spec=MoveSpec(distance_cm, speed_cmps, direction_deg))

    @classmethod
    def target_position(cls, first_cm: int, second_cm: int) -> CommandRequest:
        return cls(
            CommandKind.TARGET_POSITION,
            target_position_spec=TargetPositionSpec(first_cm, second_cm),
        )

    @classmethod
    def target_height(cls, height_cm: int) -> CommandRequest:
        return cls(CommandKind.TARGET_HEIGHT, height_cm=height_cm)

    @classmethod
    def ascend(cls, distance_cm: int, speed_cmps: int) -> CommandRequest:
        return cls(CommandKind.ASCEND, vertical_spec=VerticalSpec(distance_cm, speed_cmps))

    @classmethod
    def descend(cls, distance_cm: int, speed_cmps: int) -> CommandRequest:
        return cls(CommandKind.DESCEND, vertical_spec=VerticalSpec(distance_cm, speed_cmps))

    def to_frame(self) -> bytes:
        """Build the native V7 command associated with this typed request."""
        match self.command:
            case CommandKind.UNLOCK:
                return cmd_unlock()
            case CommandKind.SET_MODE if self.mode is not None:
                return cmd_mode(self.mode)
            case CommandKind.TAKEOFF if self.height_cm is not None:
                return cmd_takeoff(self.height_cm)
            case CommandKind.MOVE if self.move_spec is not None:
                return cmd_move(
                    self.move_spec.distance_cm,
                    self.move_spec.speed_cmps,
                    self.move_spec.direction_deg,
                )
            case CommandKind.HOVER:
                return cmd_hover()
            case CommandKind.LAND:
                return cmd_land()
            case CommandKind.LOCK:
                return cmd_lock()
            case CommandKind.TARGET_POSITION if self.target_position_spec is not None:
                return cmd_target_position(
                    self.target_position_spec.first_cm,
                    self.target_position_spec.second_cm,
                )
            case CommandKind.TARGET_HEIGHT if self.height_cm is not None:
                return cmd_target_height(self.height_cm)
            case CommandKind.ASCEND if self.vertical_spec is not None:
                return cmd_ascend(self.vertical_spec.distance_cm, self.vertical_spec.speed_cmps)
            case CommandKind.DESCEND if self.vertical_spec is not None:
                return cmd_descend(self.vertical_spec.distance_cm, self.vertical_spec.speed_cmps)
            case _:
                raise ValueError(f"incomplete request for {self.command.name}")


@dataclass(frozen=True, slots=True)
class CommandResult:
    command: CommandKind
    code: ResultCode
    reason: str
    acknowledged: bool
    completed_steady_s: float


@dataclass(slots=True)
class PendingCommand:  # noqa: MUTABLE_OK
    """Mutable command lifecycle state, changed only by the action controller."""

    command: CommandKind
    raw: bytes
    deadline_steady_s: float


class WireWriter(Protocol):
    def __call__(self, data: bytes) -> int | None: ...


class CommandAllowed(Protocol):
    def __call__(self) -> bool: ...


class CommandRejectedError(RuntimeError):
    """Raised when a new high-level command would overlap a pending command."""


class FlightActionController:
    """Single-flight ACK controller; late and duplicate acknowledgements are ignored."""

    def __init__(
        self,
        writer: WireWriter,
        arbiter: CommandArbiter | None = None,
        command_allowed: CommandAllowed | None = None,
    ) -> None:
        self._writer = writer
        self._arbiter = arbiter if arbiter is not None else CommandArbiter()
        self._command_allowed = command_allowed if command_allowed is not None else lambda: True
        self.pending: PendingCommand | None = None
        self.last_result: CommandResult | None = None
        self._used_ack_signatures: set[tuple[int, int, int]] = set()

    def start(self, request: CommandRequest, steady_now: float, timeout_s: float) -> PendingCommand:
        """Transmit a command and begin awaiting its checksum-bound V7 acknowledgement."""
        if not self._command_allowed():
            raise CommandRejectedError("emergency lock is latched")
        if self.pending is not None:
            raise CommandRejectedError("another FCU command is already awaiting acknowledgement")
        raw = request.to_frame()
        signature = (raw[2], raw[-2], raw[-1])
        if signature in self._used_ack_signatures:
            raise CommandRejectedError(
                "V7 protocol cannot correlate repeated identical command acknowledgements"
            )
        if not self._arbiter.try_acquire():
            raise CommandRejectedError("another FCU command is already active")
        started = False
        try:
            self.last_result = None
            written = self._writer(raw)
            if not self._command_allowed():
                self.last_result = CommandResult(
                    request.command,
                    ResultCode.REJECTED,
                    "emergency lock is latched",
                    False,
                    steady_now,
                )
                raise CommandRejectedError("emergency lock is latched")
            if written is not None and written != len(raw):
                self.last_result = CommandResult(
                    request.command,
                    ResultCode.FCU_ERROR,
                    "short serial write",
                    False,
                    steady_now,
                )
                raise CommandRejectedError(
                    "serial transport did not accept the complete V7 command"
                )
            self.pending = PendingCommand(request.command, raw, steady_now + timeout_s)
            self._used_ack_signatures.add(signature)
            started = True
            return self.pending
        finally:
            if not started:
                self._arbiter.release()

    def handle_frame(self, frame: V7Frame, steady_now: float) -> CommandResult | None:
        """Resolve the pending command only for a matching ID=0 checksum acknowledgement."""
        pending = self.pending
        if pending is None or frame.frame_id != 0x00 or len(frame.data) < 3:
            return None
        sent = pending.raw
        if tuple(frame.data[:3]) != (sent[2], sent[-2], sent[-1]):
            return None
        result = CommandResult(pending.command, ResultCode.SUCCEEDED, "matching V7 acknowledgement", True, steady_now)
        self.pending = None
        self.last_result = result
        self._arbiter.release()
        return result

    def preempt_for_emergency_lock(self, steady_now: float) -> CommandResult | None:
        pending = self.pending
        if pending is None:
            return None
        result = CommandResult(
            pending.command,
            ResultCode.REJECTED,
            "emergency lock is latched",
            False,
            steady_now,
        )
        self.pending = None
        self.last_result = result
        self._arbiter.release()
        return result

    def tick(self, steady_now: float) -> CommandResult | None:
        """Expire the one pending command without accepting future late acknowledgements."""
        pending = self.pending
        if pending is None or steady_now <= pending.deadline_steady_s:
            return None
        result = CommandResult(pending.command, ResultCode.TIMEOUT, "V7 acknowledgement deadline exceeded", False, steady_now)
        self.pending = None
        self.last_result = result
        self._arbiter.release()
        return result
