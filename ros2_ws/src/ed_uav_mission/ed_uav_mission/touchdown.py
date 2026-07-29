"""Pure moving-platform contact and dwell boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Literal, Protocol

from typing_extensions import assert_never

from ed_uav_mission.payload_config import PayloadBoundaryConfig
from ed_uav_mission.plugins.payload import SAFE_RECOVERY_PLAN, RecoveryPlan


class ContactState(IntEnum):
    AIRBORNE = 0
    HOME = 1
    VEHICLE = 2


class DwellInterruptionReason(str, Enum):
    CONTACT_LOST = "contact_lost"
    CONTACT_STALE = "contact_stale"
    TARGET_STALE = "target_stale"
    VEHICLE_STALE = "vehicle_stale"
    VEHICLE_STOPPED = "vehicle_stopped"
    LOCALIZATION_LOST = "localization_lost"
    CANCELLED = "cancelled"
    CLOCK_INVALID = "clock_invalid"


class PayloadContactStateLike(Protocol):
    contract_version: int
    source_sequence: int
    contact_state: int
    contact_stable: bool
    owner: str
    frame_id: str


@dataclass(frozen=True, slots=True)
class PayloadContactContractError(Exception):
    detail: str

    def __str__(self) -> str:
        return self.detail


@dataclass(frozen=True, slots=True)
class ContactObservation:
    sequence: int
    state: ContactState
    stable: bool
    owner: str
    frame_id: str
    observed_at_monotonic_s: float


def adapt_payload_contact_state(
    message: PayloadContactStateLike, observed_at_monotonic_s: float
) -> ContactObservation:
    """Adapt Todo 1 ``PayloadContactState`` fields to a monotonic observation."""
    if message.contract_version != 1:
        raise PayloadContactContractError(detail="unsupported payload contact contract")
    if not math.isfinite(observed_at_monotonic_s):
        raise PayloadContactContractError(detail="contact observation time is invalid")
    try:
        state = ContactState(message.contact_state)
    except ValueError as error:
        raise PayloadContactContractError(detail="unknown payload contact state") from error
    return ContactObservation(
        sequence=message.source_sequence,
        state=state,
        stable=message.contact_stable,
        owner=message.owner,
        frame_id=message.frame_id,
        observed_at_monotonic_s=observed_at_monotonic_s,
    )


@dataclass(frozen=True, slots=True)
class TouchdownUpdate:
    now_monotonic_s: float
    target_observed_at_s: float
    vehicle_observed_at_s: float
    vehicle_speed_m_s: float
    contact: ContactObservation
    cancelled: bool
    localization_valid: bool = True


@dataclass(frozen=True, slots=True)
class DwellProgress:
    elapsed_s: float
    completed: Literal[False] = False


@dataclass(frozen=True, slots=True)
class DwellComplete:
    elapsed_s: float
    completed: Literal[True] = True


@dataclass(frozen=True, slots=True)
class DwellInterrupted:
    reason: DwellInterruptionReason
    recovery: RecoveryPlan = SAFE_RECOVERY_PLAN
    elapsed_s: float = 0.0
    completed: Literal[False] = False


DwellResult = DwellProgress | DwellComplete | DwellInterrupted


class TouchdownDwellTracker:
    """Mutable accumulator for continuous contact against injected monotonic time."""

    def __init__(self, config: PayloadBoundaryConfig) -> None:
        self._config = config
        self._qualifying_since_s: float | None = None
        self._last_now_s: float | None = None
        self._completed = False

    def _interrupt(self, reason: DwellInterruptionReason) -> DwellInterrupted:
        self._qualifying_since_s = None
        return DwellInterrupted(reason=reason)

    def _freshness_reason(
        self, update: TouchdownUpdate
    ) -> DwellInterruptionReason | None:
        timestamps = (
            update.now_monotonic_s,
            update.target_observed_at_s,
            update.vehicle_observed_at_s,
            update.contact.observed_at_monotonic_s,
        )
        if any(not math.isfinite(value) for value in timestamps):
            return DwellInterruptionReason.CLOCK_INVALID
        if any(value > update.now_monotonic_s for value in timestamps[1:]):
            return DwellInterruptionReason.CLOCK_INVALID
        if (
            update.now_monotonic_s - update.target_observed_at_s
            > self._config.freshness_timeout_s
        ):
            return DwellInterruptionReason.TARGET_STALE
        if (
            update.now_monotonic_s - update.vehicle_observed_at_s
            > self._config.freshness_timeout_s
        ):
            return DwellInterruptionReason.VEHICLE_STALE
        if (
            update.now_monotonic_s - update.contact.observed_at_monotonic_s
            > self._config.freshness_timeout_s
        ):
            return DwellInterruptionReason.CONTACT_STALE
        return None

    def update(self, update: TouchdownUpdate) -> DwellResult:
        """Advance or reset the dwell using one coherent monotonic snapshot."""
        if self._completed:
            return DwellComplete(elapsed_s=self._config.contact_dwell_s)
        freshness_reason = self._freshness_reason(update)
        if freshness_reason is not None:
            return self._interrupt(freshness_reason)
        if self._last_now_s is not None and update.now_monotonic_s < self._last_now_s:
            return self._interrupt(DwellInterruptionReason.CLOCK_INVALID)
        self._last_now_s = update.now_monotonic_s
        if update.cancelled:
            return self._interrupt(DwellInterruptionReason.CANCELLED)
        if not update.localization_valid:
            return self._interrupt(DwellInterruptionReason.LOCALIZATION_LOST)
        match update.contact.state:
            case ContactState.VEHICLE:
                if not update.contact.stable:
                    return self._interrupt(DwellInterruptionReason.CONTACT_LOST)
            case ContactState.AIRBORNE | ContactState.HOME:
                return self._interrupt(DwellInterruptionReason.CONTACT_LOST)
            case unreachable:
                assert_never(unreachable)
        if not math.isfinite(update.vehicle_speed_m_s):
            return self._interrupt(DwellInterruptionReason.CLOCK_INVALID)
        if update.vehicle_speed_m_s < self._config.minimum_vehicle_speed_m_s:
            return self._interrupt(DwellInterruptionReason.VEHICLE_STOPPED)
        if self._qualifying_since_s is None:
            self._qualifying_since_s = update.now_monotonic_s
        elapsed_s = update.now_monotonic_s - self._qualifying_since_s
        if elapsed_s < self._config.contact_dwell_s:
            return DwellProgress(elapsed_s=elapsed_s)
        self._completed = True
        return DwellComplete(elapsed_s=elapsed_s)
