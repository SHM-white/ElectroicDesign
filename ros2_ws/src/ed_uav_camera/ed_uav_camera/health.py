"""Deterministic per-camera acquisition provenance and recovery accounting."""

from __future__ import annotations

from dataclasses import dataclass
from enum import unique

from .string_enum import StrEnum
from typing_extensions import assert_never

from .model import CameraRole


@unique
class HealthCode(StrEnum):
    """Operator-visible stream status categories."""

    HEALTHY = "healthy"
    RECOVERED = "recovered"
    UNAVAILABLE = "unavailable"
    STALE = "stale"
    NONMONOTONIC_STAMP = "nonmonotonic_stamp"
    WAITING_FOR_FRAME = "waiting_for_frame"


@dataclass(frozen=True, slots=True)
class InvalidHealthPeriodError(Exception):
    """Raised when health timing configuration cannot represent a frame cadence."""

    expected_period_ns: int

    def __str__(self) -> str:
        return f"expected_period_ns must be positive, received {self.expected_period_ns}"


@dataclass(frozen=True, slots=True)
class HealthReport:
    """A truthful immutable snapshot for diagnostics aggregation."""

    role: CameraRole
    code: HealthCode
    accepted_frames: int
    rejected_nonmonotonic_frames: int
    inferred_drops: int
    max_jitter_ns: int
    restart_count: int
    last_acquisition_stamp_ns: int | None
    last_observed_steady_ns: int | None


class CameraHealth:  # noqa: MUTABLE_OK
    """Mutable stream accumulator; mutation is required to retain per-camera history."""

    def __init__(self, role: CameraRole, expected_period_ns: int) -> None:
        if expected_period_ns <= 0:
            raise InvalidHealthPeriodError(expected_period_ns)
        self.role = role
        self.expected_period_ns = expected_period_ns
        self.accepted_frames = 0
        self.rejected_nonmonotonic_frames = 0
        self.inferred_drops = 0
        self.max_jitter_ns = 0
        self.restart_count = 0
        self.last_acquisition_stamp_ns: int | None = None
        self.last_observed_steady_ns: int | None = None
        self.available = True
        self.recovered = False
        self.last_event_was_nonmonotonic = False

    def record_frame(self, acquisition_stamp_ns: int, observed_steady_ns: int) -> bool:
        """Accept only increasing acquisition stamps and infer source-time gaps."""
        previous_stamp = self.last_acquisition_stamp_ns
        if previous_stamp is not None and acquisition_stamp_ns <= previous_stamp:
            self.rejected_nonmonotonic_frames += 1
            self.last_event_was_nonmonotonic = True
            return False
        if previous_stamp is not None:
            interval_ns = acquisition_stamp_ns - previous_stamp
            self.inferred_drops += max(interval_ns // self.expected_period_ns - 1, 0)
            self.max_jitter_ns = max(self.max_jitter_ns, abs(interval_ns - self.expected_period_ns))
        self.accepted_frames += 1
        self.last_acquisition_stamp_ns = acquisition_stamp_ns
        self.last_observed_steady_ns = observed_steady_ns
        self.available = True
        self.last_event_was_nonmonotonic = False
        return True

    def mark_unplugged(self, observed_steady_ns: int) -> None:
        """Record an observed disconnect without affecting another camera accumulator."""
        self.available = False
        self.last_observed_steady_ns = observed_steady_ns

    def mark_restarted(self, observed_steady_ns: int) -> None:
        """Record independent driver restart before the next accepted frame."""
        self.available = True
        self.recovered = True
        self.restart_count += 1
        self.last_observed_steady_ns = observed_steady_ns

    def snapshot(self, now_steady_ns: int, stale_after_ns: int) -> HealthReport:
        """Produce a local-steady-clock health report without using ROS-clock age."""
        code = self._status(now_steady_ns, stale_after_ns)
        return HealthReport(
            self.role,
            code,
            self.accepted_frames,
            self.rejected_nonmonotonic_frames,
            self.inferred_drops,
            self.max_jitter_ns,
            self.restart_count,
            self.last_acquisition_stamp_ns,
            self.last_observed_steady_ns,
        )

    def _status(self, now_steady_ns: int, stale_after_ns: int) -> HealthCode:
        if not self.available:
            return HealthCode.UNAVAILABLE
        if self.last_observed_steady_ns is None:
            return HealthCode.WAITING_FOR_FRAME
        if now_steady_ns - self.last_observed_steady_ns > stale_after_ns:
            return HealthCode.STALE
        if self.last_event_was_nonmonotonic:
            return HealthCode.NONMONOTONIC_STAMP
        if self.recovered:
            return HealthCode.RECOVERED
        return HealthCode.HEALTHY


def health_code_label(code: HealthCode) -> str:
    """Render a stable label through exhaustive status matching for CLI consumers."""
    match code:
        case HealthCode.HEALTHY:
            return "HEALTHY"
        case HealthCode.RECOVERED:
            return "RECOVERED"
        case HealthCode.UNAVAILABLE:
            return "UNAVAILABLE"
        case HealthCode.STALE:
            return "STALE"
        case HealthCode.NONMONOTONIC_STAMP:
            return "NONMONOTONIC_STAMP"
        case HealthCode.WAITING_FOR_FRAME:
            return "WAITING_FOR_FRAME"
        case unreachable:
            assert_never(unreachable)
