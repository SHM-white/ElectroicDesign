"""Typed deterministic scenario values shared by the verification harness."""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Final


NANOSECONDS_PER_SECOND: Final = 1_000_000_000


class Stream(str, Enum):
    """Synthetic sources exposed by the offline harness."""

    FCU = "fcu"
    GPIO = "gpio"
    LASER = "laser"
    LIDAR_POINTS = "lidar_points"
    LIDAR_IMU = "lidar_imu"
    NARROW_IMAGE = "narrow_image"
    WIDE_IMAGE = "wide_image"
    ODOM = "odom"


class FaultKind(str, Enum):
    """Bounded source failure modes exercised by offline scenarios."""

    DROP = "drop"
    FREEZE = "freeze"
    CORRUPTION = "corruption"
    LATENCY = "latency"
    TIME_REGRESSION = "time_regression"
    PROCESS_DEATH = "process_death"


class EventType(str, Enum):
    """Machine-readable replay event categories."""

    SAMPLE = "sample"
    FAULT_ACTIVATED = "fault_activated"
    FAULT_RECOVERED = "fault_recovered"
    MESSAGE_REJECTED = "message_rejected"
    HEALTH_DEGRADED = "health_degraded"
    HEALTH_ACTIVE = "health_active"


@dataclass(frozen=True, slots=True)
class ScenarioConfigurationError(Exception):
    """Raised before a malformed scenario can start replay."""

    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class ScenarioBoundError(Exception):
    """Raised when a scenario would exceed its declared deterministic budget."""

    requested_ticks: int
    max_ticks: int

    def __str__(self) -> str:
        return f"requested {self.requested_ticks} ticks exceeds maximum {self.max_ticks}"


@dataclass(frozen=True, slots=True)
class FaultWindow:
    """One non-overlapping fault window for one synthetic source."""

    kind: FaultKind
    stream: Stream
    start_tick: int
    duration_ticks: int

    def __post_init__(self) -> None:
        if self.start_tick < 0:
            raise ScenarioConfigurationError("fault start_tick must be non-negative")
        if self.duration_ticks < 1:
            raise ScenarioConfigurationError("fault duration_ticks must be positive")

    @property
    def end_tick(self) -> int:
        """Return the first tick after this fault window."""
        return self.start_tick + self.duration_ticks


@dataclass(frozen=True, slots=True)
class ScenarioConfig:
    """Immutable parameters for one virtual-time replay."""

    seed: int
    duration_seconds: int
    rate_hz: int
    faults: tuple[FaultWindow, ...] = ()
    max_ticks: int = 120_000
    start_time_ns: int = 1_700_000_000_000_000_000

    def __post_init__(self) -> None:
        if self.duration_seconds < 1:
            raise ScenarioConfigurationError("duration_seconds must be positive")
        if self.rate_hz < 1:
            raise ScenarioConfigurationError("rate_hz must be positive")
        if self.max_ticks < 1:
            raise ScenarioConfigurationError("max_ticks must be positive")
        for fault in self.faults:
            if fault.end_tick >= self.tick_count:
                raise ScenarioConfigurationError("fault recovery must occur before scenario completion")
        self._validate_non_overlapping_faults()

    @property
    def tick_count(self) -> int:
        """Return the exact number of virtual 20Hz-style clock steps."""
        return self.duration_seconds * self.rate_hz

    @property
    def tick_duration_ns(self) -> int:
        """Return the virtual duration of one source tick."""
        return NANOSECONDS_PER_SECOND // self.rate_hz

    def _validate_non_overlapping_faults(self) -> None:
        for index, left in enumerate(self.faults):
            for right in self.faults[index + 1 :]:
                same_stream = left.stream.value == right.stream.value
                overlaps = left.start_tick < right.end_tick and right.start_tick < left.end_tick
                if same_stream and overlaps:
                    raise ScenarioConfigurationError("fault windows cannot overlap on one stream")


@dataclass(frozen=True, slots=True)
class Event:
    """One stable, source-timestamped event in the JSON replay stream."""

    event_type: EventType
    stream: Stream
    sequence: int
    simulated_time_ns: int
    acquisition_time_ns: int
    accepted: bool
    reason: str = ""
    fault: FaultKind | None = None
    payload_sha256: str = ""

    def as_json_value(self) -> dict[str, int | str | bool | None]:
        """Convert this immutable event into a canonical JSON record."""
        return {
            "accepted": self.accepted,
            "acquisition_time_ns": self.acquisition_time_ns,
            "event_type": self.event_type.value,
            "fault": self.fault.value if self.fault is not None else None,
            "payload_sha256": self.payload_sha256,
            "reason": self.reason,
            "sequence": self.sequence,
            "simulated_time_ns": self.simulated_time_ns,
            "stream": self.stream.value,
        }


@dataclass(frozen=True, slots=True)
class ScenarioReport:
    """Completed or interrupted deterministic replay output."""

    config: ScenarioConfig
    events: tuple[Event, ...]
    completed: bool
    tick_count: int

    @property
    def simulated_duration_ns(self) -> int:
        """Return the exact virtual duration represented by completed ticks."""
        return self.tick_count * self.config.tick_duration_ns

    @property
    def event_json(self) -> bytes:
        """Return canonical byte-stable JSON suitable for hash comparison."""
        document = {
            "completed": self.completed,
            "duration_seconds": self.config.duration_seconds,
            "events": [event.as_json_value() for event in self.events],
            "rate_hz": self.config.rate_hz,
            "schema_version": 1,
            "seed": self.config.seed,
            "tick_count": self.tick_count,
        }
        return (json.dumps(document, separators=(",", ":"), sort_keys=True) + "\n").encode("utf-8")

    def has_fault_activation(self, kind: FaultKind) -> bool:
        """Return whether a fault activation was recorded."""
        return any(event.event_type is EventType.FAULT_ACTIVATED and event.fault is kind for event in self.events)

    def has_fault_recovery(self, kind: FaultKind) -> bool:
        """Return whether a fault recovery was recorded."""
        return any(event.event_type is EventType.FAULT_RECOVERED and event.fault is kind for event in self.events)

    def has_degradation(self, kind: FaultKind) -> bool:
        """Return whether a fault caused explicit degraded health."""
        return any(event.event_type is EventType.HEALTH_DEGRADED and event.fault is kind for event in self.events)

    def has_stream_recovery(self, stream: Stream) -> bool:
        """Return whether a degraded stream later became active."""
        return any(event.event_type is EventType.HEALTH_ACTIVE and event.stream is stream for event in self.events)

    def rejection_reasons(self, stream: Stream) -> tuple[str, ...]:
        """Return unique rejection reasons in their deterministic event order."""
        reasons: dict[str, None] = {}
        for event in self.events:
            if event.event_type is EventType.MESSAGE_REJECTED and event.stream is stream:
                reasons[event.reason] = None
        return tuple(reasons)

    def accepted_sequences(self, stream: Stream) -> tuple[int, ...]:
        """Return source sequences accepted for one stream."""
        return tuple(
            event.sequence
            for event in self.events
            if event.event_type is EventType.SAMPLE and event.stream is stream and event.accepted
        )

    def rejected_sequences(self, stream: Stream) -> tuple[int, ...]:
        """Return source sequences rejected for one stream."""
        return tuple(
            event.sequence
            for event in self.events
            if event.event_type is EventType.MESSAGE_REJECTED and event.stream is stream
        )

    @property
    def configured_streams(self) -> tuple[Stream, ...]:
        """Return streams in stable first-event order for generic launch assertions."""
        streams: dict[Stream, None] = {}
        for event in self.events:
            streams[event.stream] = None
        return tuple(streams)
