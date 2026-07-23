"""ACK-correlated high-level V7 command state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from .v7_codec import V7Frame, cmd_hover, cmd_land, cmd_lock, cmd_mode, cmd_move, cmd_takeoff, cmd_unlock


class CommandKind(Enum):
    UNLOCK = auto()
    SET_MODE = auto()
    TAKEOFF = auto()
    MOVE = auto()
    HOVER = auto()
    LAND = auto()
    LOCK = auto()


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
class CommandRequest:
    command: CommandKind
    mode: int | None = None
    height_cm: int | None = None
    move_spec: MoveSpec | None = None

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


class CommandRejectedError(RuntimeError):
    """Raised when a new high-level command would overlap a pending command."""


class FlightActionController:
    """Single-flight ACK controller; late and duplicate acknowledgements are ignored."""

    def __init__(self, writer: WireWriter) -> None:
        self._writer = writer
        self.pending: PendingCommand | None = None
        self.last_result: CommandResult | None = None

    def start(self, request: CommandRequest, steady_now: float, timeout_s: float) -> PendingCommand:
        """Transmit a command and begin awaiting its checksum-bound V7 acknowledgement."""
        if self.pending is not None:
            raise CommandRejectedError("another FCU command is already awaiting acknowledgement")
        raw = request.to_frame()
        written = self._writer(raw)
        if written is not None and written != len(raw):
            self.last_result = CommandResult(request.command, ResultCode.FCU_ERROR, "short serial write", False, steady_now)
            raise CommandRejectedError("serial transport did not accept the complete V7 command")
        self.pending = PendingCommand(request.command, raw, steady_now + timeout_s)
        return self.pending

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
        return result

    def tick(self, steady_now: float) -> CommandResult | None:
        """Expire the one pending command without accepting future late acknowledgements."""
        pending = self.pending
        if pending is None or steady_now <= pending.deadline_steady_s:
            return None
        result = CommandResult(pending.command, ResultCode.TIMEOUT, "V7 acknowledgement deadline exceeded", False, steady_now)
        self.pending = None
        self.last_result = result
        return result
