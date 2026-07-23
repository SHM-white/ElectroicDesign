"""Launch-independent assertions shared by static and ROS launch verification."""

from __future__ import annotations

from dataclasses import dataclass

from .model import EventType, FaultWindow, ScenarioReport


@dataclass(frozen=True, slots=True)
class LaunchAssertionError(Exception):
    """Raised when a replay cannot prove its declared fault contract."""

    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class LaunchAssertions:
    """Assertions over event evidence rather than process log text."""

    report: ScenarioReport

    def assert_fault_matrix(self, faults: tuple[FaultWindow, ...]) -> None:
        """Require activation, degraded health, recovery, and source recovery for each fault."""
        for fault in faults:
            if not self.report.has_fault_activation(fault.kind):
                raise LaunchAssertionError(f"missing fault activation: {fault.kind.value}")
            if not self.report.has_degradation(fault.kind):
                raise LaunchAssertionError(f"missing fault degradation: {fault.kind.value}")
            if not self.report.has_fault_recovery(fault.kind):
                raise LaunchAssertionError(f"missing fault recovery: {fault.kind.value}")
            if not self.report.has_stream_recovery(fault.stream):
                raise LaunchAssertionError(f"missing source recovery: {fault.stream.value}")

    def assert_no_stale_reuse(self) -> None:
        """Require every rejected stale sequence to remain absent from accepted samples."""
        for stream in self.report.configured_streams:
            rejected = set(self.report.rejected_sequences(stream))
            accepted = set(self.report.accepted_sequences(stream))
            if rejected & accepted:
                raise LaunchAssertionError(f"reused rejected sequence: {stream.value}")

    def assert_completed(self) -> None:
        """Require a completed report before accepting a launch artifact."""
        if not self.report.completed:
            raise LaunchAssertionError("scenario did not complete")
