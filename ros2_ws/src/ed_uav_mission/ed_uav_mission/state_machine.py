"""Typed mission state machine with exhaustive transitions."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import ClassVar


class MissionState(Enum):
    IDLE = auto()
    ARMED = auto()
    TAKEOFF = auto()
    EXECUTING = auto()
    RETURNING = auto()
    LANDING = auto()
    COMPLETE = auto()
    ABORTED = auto()


_VALID_TRANSITIONS: dict[MissionState, frozenset[MissionState]] = {
    MissionState.IDLE: frozenset({MissionState.ARMED}),
    MissionState.ARMED: frozenset({MissionState.TAKEOFF, MissionState.ABORTED}),
    MissionState.TAKEOFF: frozenset({MissionState.EXECUTING, MissionState.ABORTED}),
    MissionState.EXECUTING: frozenset({MissionState.RETURNING, MissionState.ABORTED, MissionState.COMPLETE}),
    MissionState.RETURNING: frozenset({MissionState.LANDING, MissionState.ABORTED}),
    MissionState.LANDING: frozenset({MissionState.COMPLETE, MissionState.ABORTED}),
    MissionState.COMPLETE: frozenset({MissionState.IDLE}),
    MissionState.ABORTED: frozenset({MissionState.IDLE}),
}


_TERMINAL_STATES: frozenset[MissionState] = frozenset({MissionState.COMPLETE, MissionState.ABORTED})


@dataclass
class MissionFSM:
    """Mutable finite-state machine for mission lifecycle tracking."""

    state: MissionState = MissionState.IDLE
    reason: str = ""

    def transition(self, target: MissionState, reason: str = "") -> None:
        allowed = _VALID_TRANSITIONS.get(self.state, frozenset())
        if target not in allowed:
            raise ValueError(
                f"invalid transition: {self.state.name} -> {target.name}"
                f" (allowed: {[s.name for s in allowed]})"
            )
        self.state = target
        self.reason = reason

    @property
    def is_terminal(self) -> bool:
        return self.state in _TERMINAL_STATES

    @property
    def is_active(self) -> bool:
        return self.state not in {MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORTED}
