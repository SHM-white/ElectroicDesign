"""Freshness-aware telemetry cache for native V7 input frames."""

from __future__ import annotations

import struct
from dataclasses import dataclass, replace
from math import isclose

from .v7_codec import FrameDecodeError, V7Frame, decode_frame


@dataclass(frozen=True, slots=True)
class FreshnessPolicy:
    """Steady-clock freshness limits for the bridge's telemetry sources."""

    position_max_age_s: float = 0.20
    aux_status_max_age_s: float = 0.50
    link_max_age_s: float = 0.50


@dataclass(frozen=True, slots=True)
class PositionSample:
    source_sequence: int
    received_steady_s: float
    forward_m: float
    right_m: float
    valid: bool
    steady_age_s: float
    source_stamp_ns: int | None


@dataclass(frozen=True, slots=True)
class StatusSample:
    source_sequence: int
    received_steady_s: float
    mode: int
    motors_armed: bool
    valid: bool
    steady_age_s: float
    source_stamp_ns: int | None


@dataclass(frozen=True, slots=True)
class AuxSample:
    source_sequence: int
    received_steady_s: float
    channels_us: tuple[int, ...]
    valid: bool
    steady_age_s: float
    source_stamp_ns: int | None

    @property
    def aux1_us(self) -> int:
        """Return AUX1, the fifth channel in the complete 0x40 payload."""
        return self.channels_us[4]

    @property
    def aux6_us(self) -> int:
        """Return AUX6, preserving the existing mission-start switch mapping."""
        return self.channels_us[9]


@dataclass(frozen=True, slots=True)
class FlowDiagnostic51:
    source_sequence: int
    received_steady_s: float
    mode: int
    state: int
    quality: int | None
    integrated_x_cm: int | None
    integrated_y_cm: int | None
    source_stamp_ns: int | None


@dataclass(frozen=True, slots=True)
class LinkSample:
    source_sequence: int
    received_steady_s: float | None
    valid: bool
    steady_age_s: float


@dataclass(frozen=True, slots=True)
class TelemetrySnapshot:
    position: PositionSample | None
    status: StatusSample | None
    aux: AuxSample | None
    flow_diagnostic: FlowDiagnostic51 | None
    altitude_m: float | None
    battery_voltage_v: float | None
    link: LinkSample


class TelemetryCache:
    """Mutable V7 cache; only ID 0x08 is allowed to mutate continuous position."""

    def __init__(self, policy: FreshnessPolicy) -> None:
        self.policy = policy
        self._position: PositionSample | None = None
        self._status: StatusSample | None = None
        self._aux: AuxSample | None = None
        self._flow_diagnostic: FlowDiagnostic51 | None = None
        self._altitude_m: float | None = None
        self._battery_voltage_v: float | None = None
        self._position_sequence = 0
        self._status_sequence = 0
        self._aux_sequence = 0
        self._diagnostic_sequence = 0
        self._link_sequence = 0
        self._last_link_steady_s: float | None = None

    def ingest_raw(self, raw: bytes, steady_now: float, source_stamp_ns: int | None = None) -> bool:
        """Decode and cache one raw V7 frame, returning False for malformed input."""
        try:
            frame = decode_frame(raw)
        except FrameDecodeError:
            return False
        self.ingest_frame(frame, steady_now, source_stamp_ns)
        return True

    def ingest_frame(self, frame: V7Frame, steady_now: float, source_stamp_ns: int | None = None) -> None:
        """Cache one checksum-verified V7 frame with receive-time provenance."""
        self._link_sequence += 1
        self._last_link_steady_s = steady_now
        match frame.frame_id:
            case 0x08 if len(frame.data) >= 8:
                self._position_sequence += 1
                forward_cm, right_cm = struct.unpack_from("<ii", frame.data)
                self._position = PositionSample(
                    self._position_sequence,
                    steady_now,
                    forward_cm / 100.0,
                    right_cm / 100.0,
                    True,
                    0.0,
                    source_stamp_ns,
                )
            case 0x51 if len(frame.data) >= 2:
                self._cache_flow_diagnostic(frame.data, steady_now, source_stamp_ns)
            case 0x06 if len(frame.data) >= 2:
                self._status_sequence += 1
                self._status = StatusSample(
                    self._status_sequence,
                    steady_now,
                    frame.data[0],
                    frame.data[1] == 1,
                    True,
                    0.0,
                    source_stamp_ns,
                )
            case 0x40 if len(frame.data) >= 20:
                self._aux_sequence += 1
                channels_us = struct.unpack_from("<10h", frame.data)
                self._aux = AuxSample(
                    self._aux_sequence,
                    steady_now,
                    channels_us,
                    True,
                    0.0,
                    source_stamp_ns,
                )
            case 0x05 if len(frame.data) >= 4:
                self._altitude_m = struct.unpack_from("<i", frame.data)[0] / 100.0
            case 0x0D if len(frame.data) >= 2:
                self._battery_voltage_v = struct.unpack_from("<H", frame.data)[0] / 100.0
            case _:
                pass

    def _cache_flow_diagnostic(self, data: bytes, steady_now: float, source_stamp_ns: int | None) -> None:
        self._diagnostic_sequence += 1
        mode, state = data[:2]
        match mode:
            case 2 if len(data) >= 15:
                integrated_x_cm, integrated_y_cm = struct.unpack_from("<hh", data, 10)
                quality: int | None = data[14]
            case _:
                integrated_x_cm = None
                integrated_y_cm = None
                quality = data[-1] if len(data) >= 5 else None
        self._flow_diagnostic = FlowDiagnostic51(
            self._diagnostic_sequence,
            steady_now,
            mode,
            state,
            quality,
            integrated_x_cm,
            integrated_y_cm,
            source_stamp_ns,
        )

    def has_fresh_start_switch(self, steady_now: float) -> bool:
        """Return true only for a fresh AUX6 mission-start switch assertion."""
        aux = self._with_aux_age(steady_now)
        return aux is not None and aux.valid and aux.aux6_us > 1700

    def snapshot(self, steady_now: float) -> TelemetrySnapshot:
        """Return source-separated state with validity derived from steady-clock age."""
        return TelemetrySnapshot(
            position=self._with_position_age(steady_now),
            status=self._with_status_age(steady_now),
            aux=self._with_aux_age(steady_now),
            flow_diagnostic=self._flow_diagnostic,
            altitude_m=self._altitude_m,
            battery_voltage_v=self._battery_voltage_v,
            link=self._link_snapshot(steady_now),
        )

    def _with_position_age(self, steady_now: float) -> PositionSample | None:
        if self._position is None:
            return None
        age = steady_now - self._position.received_steady_s
        return replace(self._position, valid=self._within(age, self.policy.position_max_age_s), steady_age_s=age)

    def _with_status_age(self, steady_now: float) -> StatusSample | None:
        if self._status is None:
            return None
        age = steady_now - self._status.received_steady_s
        return replace(self._status, valid=self._within(age, self.policy.aux_status_max_age_s), steady_age_s=age)

    def _with_aux_age(self, steady_now: float) -> AuxSample | None:
        if self._aux is None:
            return None
        age = steady_now - self._aux.received_steady_s
        return replace(self._aux, valid=self._within(age, self.policy.aux_status_max_age_s), steady_age_s=age)

    def _link_snapshot(self, steady_now: float) -> LinkSample:
        if self._last_link_steady_s is None:
            return LinkSample(self._link_sequence, None, False, float("inf"))
        age = steady_now - self._last_link_steady_s
        return LinkSample(self._link_sequence, self._last_link_steady_s, self._within(age, self.policy.link_max_age_s), age)

    @staticmethod
    def _within(age: float, limit: float) -> bool:
        return age < limit or isclose(age, limit, rel_tol=0.0, abs_tol=1e-12)
