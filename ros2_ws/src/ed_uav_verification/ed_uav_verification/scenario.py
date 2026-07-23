"""Deterministic virtual-time replay, freshness rejection, and fault recovery."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import Final

from .clock import VirtualMonotonicClock
from .faults import FaultEffect, FaultEngine
from .fcu import position_v7_frame
from .io_fakes import FakeGpioLaser
from .model import Event, EventType, FaultKind, ScenarioBoundError, ScenarioConfig, ScenarioReport, Stream
from .sensors import DeterministicSensors, SyntheticSample


SOURCE_STREAMS: Final = (
    Stream.FCU,
    Stream.GPIO,
    Stream.LASER,
    Stream.LIDAR_POINTS,
    Stream.LIDAR_IMU,
    Stream.NARROW_IMAGE,
    Stream.WIDE_IMAGE,
    Stream.ODOM,
)

FRESHNESS_NS: Final = {
    Stream.FCU: 500_000_000,
    Stream.GPIO: 500_000_000,
    Stream.LASER: 500_000_000,
    Stream.LIDAR_POINTS: 150_000_000,
    Stream.LIDAR_IMU: 150_000_000,
    Stream.NARROW_IMAGE: 200_000_000,
    Stream.WIDE_IMAGE: 200_000_000,
    Stream.ODOM: 150_000_000,
}


@dataclass(slots=True)
class _StreamState:  # noqa: MUTABLE_OK
    """Mutable acceptance state is required to model freshness across source ticks."""

    latest_acquisition_ns: int | None = None
    degraded: bool = False


class DeterministicScenario:
    """Runs a fixed seed and virtual clock without threads, wall time, or hardware."""

    def __init__(self, config: ScenarioConfig) -> None:
        self._config = config
        self._clock = VirtualMonotonicClock(config.start_time_ns, config.tick_duration_ns)
        self._sensors = DeterministicSensors(config.seed, config.tick_duration_ns)
        self._faults = FaultEngine(config.faults, config.tick_duration_ns)
        self._outputs = FakeGpioLaser()

    def run(self, stop_after_ticks: int | None = None) -> ScenarioReport:
        """Run a complete or intentionally bounded virtual-time replay."""
        requested_ticks = self._config.tick_count
        if requested_ticks > self._config.max_ticks:
            raise ScenarioBoundError(requested_ticks=requested_ticks, max_ticks=self._config.max_ticks)
        target_ticks = self._bounded_tick_count(stop_after_ticks)
        states = {stream: _StreamState() for stream in SOURCE_STREAMS}
        events: list[Event] = []
        for tick in range(target_ticks):
            current_time_ns = self._clock.timestamp_for_tick(tick)
            self._record_fault_boundaries(events, tick, current_time_ns)
            for stream in SOURCE_STREAMS:
                sample = self._sample(stream, tick, current_time_ns)
                effect = self._faults.apply(stream, tick, sample.acquisition_time_ns)
                self._evaluate_sample(events, states[stream], sample, effect, current_time_ns)
        return ScenarioReport(
            config=self._config,
            events=tuple(events),
            completed=target_ticks == requested_ticks,
            tick_count=target_ticks,
        )

    def _bounded_tick_count(self, stop_after_ticks: int | None) -> int:
        if stop_after_ticks is None:
            return self._config.tick_count
        if stop_after_ticks < 0:
            raise ScenarioBoundError(requested_ticks=stop_after_ticks, max_ticks=self._config.tick_count)
        return min(stop_after_ticks, self._config.tick_count)

    def _record_fault_boundaries(self, events: list[Event], tick: int, current_time_ns: int) -> None:
        for fault in self._faults.activations(tick):
            events.append(
                Event(
                    event_type=EventType.FAULT_ACTIVATED,
                    stream=fault.stream,
                    sequence=tick,
                    simulated_time_ns=current_time_ns,
                    acquisition_time_ns=current_time_ns,
                    accepted=False,
                    fault=fault.kind,
                    reason=fault.kind.value,
                )
            )
        for fault in self._faults.recoveries(tick):
            events.append(
                Event(
                    event_type=EventType.FAULT_RECOVERED,
                    stream=fault.stream,
                    sequence=tick,
                    simulated_time_ns=current_time_ns,
                    acquisition_time_ns=current_time_ns,
                    accepted=False,
                    fault=fault.kind,
                    reason=fault.kind.value,
                )
            )

    def _sample(self, stream: Stream, tick: int, current_time_ns: int) -> SyntheticSample:
        match stream:
            case Stream.FCU:
                odom = self._sensors.odom(tick, current_time_ns)
                frame = position_v7_frame(round(odom.x_m * 100), round(odom.y_m * 100))
                return SyntheticSample(stream, tick, current_time_ns, "fcu_link", hashlib.sha256(frame).hexdigest())
            case Stream.GPIO:
                snapshot = self._outputs.snapshot()
                payload = f"gpio:{snapshot.sequence}:{tick}".encode("ascii")
                return SyntheticSample(stream, tick, current_time_ns, "base_link", hashlib.sha256(payload).hexdigest())
            case Stream.LASER:
                snapshot = self._outputs.snapshot()
                payload = f"laser:{int(snapshot.laser_enabled)}:{snapshot.sequence}:{tick}".encode("ascii")
                return SyntheticSample(stream, tick, current_time_ns, "base_link", hashlib.sha256(payload).hexdigest())
            case Stream.LIDAR_POINTS | Stream.LIDAR_IMU | Stream.NARROW_IMAGE | Stream.WIDE_IMAGE | Stream.ODOM:
                return self._sensors.sample(stream, tick, current_time_ns)
            case unreachable:
                raise AssertionError(f"unhandled stream: {unreachable}")

    def _evaluate_sample(
        self,
        events: list[Event],
        state: _StreamState,
        sample: SyntheticSample,
        effect: FaultEffect,
        current_time_ns: int,
    ) -> None:
        if effect.drop:
            self._degrade_if_stale(events, state, sample, effect, current_time_ns)
            return
        if not effect.alive:
            self._reject(events, state, sample, effect, current_time_ns, "process_death")
            return
        if not effect.valid:
            self._reject(events, state, sample, effect, current_time_ns, "corrupt")
            return
        if effect.kind is FaultKind.TIME_REGRESSION:
            self._reject(events, state, sample, effect, current_time_ns, "time_regression")
            return
        if state.latest_acquisition_ns is not None and effect.acquisition_time_ns < state.latest_acquisition_ns:
            self._reject(events, state, sample, effect, current_time_ns, "time_regression")
            return
        if current_time_ns - effect.acquisition_time_ns >= FRESHNESS_NS[sample.stream]:
            self._reject(events, state, sample, effect, current_time_ns, "stale")
            return
        state.latest_acquisition_ns = effect.acquisition_time_ns
        events.append(
            Event(
                event_type=EventType.SAMPLE,
                stream=sample.stream,
                sequence=sample.sequence,
                simulated_time_ns=current_time_ns,
                acquisition_time_ns=effect.acquisition_time_ns,
                accepted=True,
                fault=effect.kind,
                payload_sha256=sample.payload_sha256,
            )
        )
        if state.degraded:
            state.degraded = False
            events.append(
                Event(
                    event_type=EventType.HEALTH_ACTIVE,
                    stream=sample.stream,
                    sequence=sample.sequence,
                    simulated_time_ns=current_time_ns,
                    acquisition_time_ns=effect.acquisition_time_ns,
                    accepted=True,
                    reason="recovered",
                )
            )

    def _degrade_if_stale(
        self,
        events: list[Event],
        state: _StreamState,
        sample: SyntheticSample,
        effect: FaultEffect,
        current_time_ns: int,
    ) -> None:
        if state.latest_acquisition_ns is None:
            self._reject(events, state, sample, effect, current_time_ns, "drop")
            return
        is_stale = current_time_ns - state.latest_acquisition_ns >= FRESHNESS_NS[sample.stream]
        if is_stale:
            self._reject(events, state, sample, effect, current_time_ns, "drop")

    def _reject(
        self,
        events: list[Event],
        state: _StreamState,
        sample: SyntheticSample,
        effect: FaultEffect,
        current_time_ns: int,
        reason: str,
    ) -> None:
        events.append(
            Event(
                event_type=EventType.MESSAGE_REJECTED,
                stream=sample.stream,
                sequence=sample.sequence,
                simulated_time_ns=current_time_ns,
                acquisition_time_ns=effect.acquisition_time_ns,
                accepted=False,
                fault=effect.kind,
                reason=reason,
                payload_sha256=sample.payload_sha256,
            )
        )
        if state.degraded:
            return
        state.degraded = True
        events.append(
            Event(
                event_type=EventType.HEALTH_DEGRADED,
                stream=sample.stream,
                sequence=sample.sequence,
                simulated_time_ns=current_time_ns,
                acquisition_time_ns=effect.acquisition_time_ns,
                accepted=False,
                fault=effect.kind,
                reason=reason,
            )
        )
