"""Deterministic fault effects for virtual source timestamps and health."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from typing_extensions import assert_never

from .model import FaultKind, FaultWindow, Stream


FAULT_LATENCY_NS: Final = 1_000_000_000


@dataclass(frozen=True, slots=True)
class FaultEffect:
    """The source-level result of applying at most one active fault window."""

    acquisition_time_ns: int
    drop: bool = False
    alive: bool = True
    valid: bool = True
    reason: str = ""
    kind: FaultKind | None = None


@dataclass(frozen=True, slots=True)
class FaultEngine:
    """Selects bounded, deterministic effects without wall-clock timers."""

    windows: tuple[FaultWindow, ...]
    tick_duration_ns: int

    def active_windows(self, tick: int) -> tuple[FaultWindow, ...]:
        """Return every window active at one virtual tick."""
        return tuple(window for window in self.windows if window.start_tick <= tick < window.end_tick)

    def activations(self, tick: int) -> tuple[FaultWindow, ...]:
        """Return windows activated at one virtual tick."""
        return tuple(window for window in self.windows if window.start_tick == tick)

    def recoveries(self, tick: int) -> tuple[FaultWindow, ...]:
        """Return windows recovered at one virtual tick."""
        return tuple(window for window in self.windows if window.end_tick == tick)

    def apply(self, stream: Stream, tick: int, acquisition_time_ns: int) -> FaultEffect:
        """Apply a matching active fault to one source message."""
        active = tuple(window for window in self.active_windows(tick) if window.stream.value == stream.value)
        if not active:
            return FaultEffect(acquisition_time_ns=acquisition_time_ns)
        window = active[0]
        match window.kind:
            case FaultKind.DROP:
                return FaultEffect(acquisition_time_ns=acquisition_time_ns, drop=True, reason="drop", kind=window.kind)
            case FaultKind.FREEZE:
                frozen_stamp = acquisition_time_ns - (tick - window.start_tick) * self.tick_duration_ns
                return FaultEffect(acquisition_time_ns=frozen_stamp, reason="freeze", kind=window.kind)
            case FaultKind.CORRUPTION:
                return FaultEffect(acquisition_time_ns=acquisition_time_ns, valid=False, reason="corrupt", kind=window.kind)
            case FaultKind.LATENCY:
                return FaultEffect(acquisition_time_ns=acquisition_time_ns - FAULT_LATENCY_NS, reason="latency", kind=window.kind)
            case FaultKind.TIME_REGRESSION:
                return FaultEffect(acquisition_time_ns=acquisition_time_ns - FAULT_LATENCY_NS, reason="time_regression", kind=window.kind)
            case FaultKind.PROCESS_DEATH:
                return FaultEffect(acquisition_time_ns=acquisition_time_ns, alive=False, reason="process_death", kind=window.kind)
            case unreachable:
                assert_never(unreachable)


__all__ = ["FaultEffect", "FaultEngine", "FaultKind", "FaultWindow"]
