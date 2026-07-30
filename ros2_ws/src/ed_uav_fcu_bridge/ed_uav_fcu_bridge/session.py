"""Composition root for native V7 serial decoding, state, and action ACKs."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .actions import (
    CommandRequest,
    CommandResult,
    FlightActionController,
    PendingCommand,
    WireWriter,
)
from .command_arbiter import CommandArbiter, SerializedWireWriter
from .realtime_control import (
    RealtimeControlConfig,
    RealtimeController,
    RealtimeDependencies,
)
from .telemetry import FreshnessPolicy, TelemetryCache, TelemetrySnapshot
from .v7_codec import V7StreamDecoder


@dataclass(frozen=True, slots=True)
class BridgeConfig:
    """Native bridge defaults; experimental sensor injection remains disabled."""

    freshness: FreshnessPolicy = field(default_factory=FreshnessPolicy)
    enable_experimental_position_velocity_injection: bool = False
    realtime_control: RealtimeControlConfig = field(
        default_factory=RealtimeControlConfig
    )


class NativeV7Bridge:
    """Mutable owner of the native V7 protocol state for one FCU serial endpoint."""

    def __init__(self, writer: WireWriter, config: BridgeConfig | None = None) -> None:
        resolved_config = config if config is not None else BridgeConfig()
        self.config = resolved_config
        self.decoder = V7StreamDecoder()
        self.telemetry = TelemetryCache(resolved_config.freshness)
        serialized_writer = SerializedWireWriter(writer)
        command_arbiter = CommandArbiter()
        self.actions = FlightActionController(serialized_writer, command_arbiter)
        self.realtime = RealtimeController(
            RealtimeDependencies(
                serialized_writer,
                self.snapshot,
                time.monotonic,
                time.sleep,
            ),
            resolved_config.realtime_control,
            command_arbiter,
        )

    def feed(self, chunk: bytes, steady_now: float, source_stamp_ns: int | None = None) -> tuple[CommandResult, ...]:
        """Decode serial input, update source-separated telemetry, and resolve matching ACKs."""
        results: list[CommandResult] = []
        for frame in self.decoder.feed(chunk):
            self.telemetry.ingest_frame(frame, steady_now, source_stamp_ns)
            result = self.actions.handle_frame(frame, steady_now)
            if result is not None:
                results.append(result)
        return tuple(results)

    def start(self, request: CommandRequest, steady_now: float, timeout_s: float) -> PendingCommand:
        """Send one high-level V7 command and await its checksum-bound acknowledgement."""
        return self.actions.start(request, steady_now, timeout_s)

    def tick(self, steady_now: float) -> CommandResult | None:
        """Advance acknowledgement timeouts using the local monotonic clock."""
        return self.actions.tick(steady_now)

    def snapshot(self, steady_now: float) -> TelemetrySnapshot:
        """Read source-separated state with steady-clock validity flags."""
        return self.telemetry.snapshot(steady_now)

    def mission_ready(self, steady_now: float) -> bool:
        """Require fresh position, status, link, and AUX6 start switch before mission start."""
        snapshot = self.snapshot(steady_now)
        return (
            snapshot.position is not None
            and snapshot.position.valid
            and snapshot.status is not None
            and snapshot.status.valid
            and snapshot.link.valid
            and self.telemetry.has_fresh_start_switch(steady_now)
        )
