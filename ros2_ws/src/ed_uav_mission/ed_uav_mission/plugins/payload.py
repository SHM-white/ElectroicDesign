"""Typed, hardware-neutral payload release boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from threading import Lock
from typing import Final, Protocol

from typing_extensions import assert_never

from ed_uav_mission.mission_model import PayloadParams
from ed_uav_mission.payload_config import PayloadBoundaryConfig


class ReleasePhase(str, Enum):
    TASK1_ESCORT = "task1_escort"
    TASK1_RELEASE = "task1_release"
    TASK2_DESCENT = "task2_descent"
    RECOVERY = "recovery"


class RecoveryAction(str, Enum):
    HOVER = "hover"
    RETURN_HOME = "return_home"
    LAND = "land"


class ReleaseRejectionReason(str, Enum):
    BAD_PHASE = "bad_phase"
    TARGET_STALE = "target_stale"
    VEHICLE_STALE = "vehicle_stale"
    LOCALIZATION_STALE = "localization_stale"
    CALIBRATION_INVALID = "calibration_invalid"
    STANDOFF_UNSAFE = "standoff_unsafe"
    CANCELLED = "cancelled"
    CLOCK_INVALID = "clock_invalid"
    LATCH_UNAVAILABLE = "latch_unavailable"
    ALREADY_ATTEMPTED = "already_attempted"
    ACTUATOR_REJECTED = "actuator_rejected"
    ACTUATOR_TIMEOUT = "actuator_timeout"
    ACTUATOR_UNKNOWN = "actuator_unknown"


@dataclass(frozen=True, slots=True)
class RecoveryPlan:
    """Ordered recommendation for the runtime; this boundary issues no commands."""

    actions: tuple[RecoveryAction, ...]


SAFE_RECOVERY_PLAN: Final = RecoveryPlan(
    actions=(
        RecoveryAction.HOVER,
        RecoveryAction.RETURN_HOME,
        RecoveryAction.LAND,
    )
)


@dataclass(frozen=True, slots=True)
class ReleaseContext:
    request_id: str
    now_monotonic_s: float
    phase: ReleasePhase
    target_observed_at_s: float
    vehicle_observed_at_s: float
    localization_observed_at_s: float
    calibration_valid: bool
    standoff_m: float
    cancelled: bool


@dataclass(frozen=True, slots=True)
class ReleaseAuthorized:
    request_id: str


@dataclass(frozen=True, slots=True)
class ReleaseRejected:
    request_id: str
    reason: ReleaseRejectionReason
    recovery: RecoveryPlan = SAFE_RECOVERY_PLAN


ReleaseInterlockDecision = ReleaseAuthorized | ReleaseRejected


@dataclass(frozen=True, slots=True)
class ActuatorCommand:
    request_id: str
    timeout_s: float


@dataclass(frozen=True, slots=True)
class ActuatorAcknowledged:
    acknowledgement_id: str


@dataclass(frozen=True, slots=True)
class ActuatorRejected:
    detail: str


@dataclass(frozen=True, slots=True)
class ActuatorTimedOut:
    elapsed_s: float


@dataclass(frozen=True, slots=True)
class ActuatorUnknown:
    detail: str


ActuatorResult = (
    ActuatorAcknowledged | ActuatorRejected | ActuatorTimedOut | ActuatorUnknown
)


class PayloadActuator(Protocol):
    """Hardware adapter contract; GPIO and serial implementations live elsewhere."""

    def release(self, command: ActuatorCommand) -> ActuatorResult: ...


@dataclass(slots=True)
class FakePayloadActuator:
    """Deterministic scripted actuator whose mutation records boundary calls."""

    outcomes: tuple[ActuatorResult, ...]
    commands: list[ActuatorCommand] = field(default_factory=list, init=False)
    _next_outcome: int = field(default=0, init=False)

    def release(self, command: ActuatorCommand) -> ActuatorResult:
        self.commands.append(command)
        if self._next_outcome >= len(self.outcomes):
            return ActuatorUnknown(detail="no scripted actuator result")
        result = self.outcomes[self._next_outcome]
        self._next_outcome += 1
        return result


@dataclass(frozen=True, slots=True)
class ReleaseSucceeded:
    request_id: str
    acknowledgement_id: str


ReleaseResult = ReleaseSucceeded | ReleaseRejected


class ReleaseLatch:
    """Atomically retain one release attempt for a mission boundary."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._attempted_request_id: str | None = None

    @property
    def attempted_request_id(self) -> str | None:
        return self._attempted_request_id

    def claim(self, request_id: str) -> bool:
        with self._lock:
            if self._attempted_request_id is not None:
                return False
            self._attempted_request_id = request_id
            return True


def _rejected(context: ReleaseContext, reason: ReleaseRejectionReason) -> ReleaseRejected:
    return ReleaseRejected(request_id=context.request_id, reason=reason)


def evaluate_release_interlock(
    context: ReleaseContext, config: PayloadBoundaryConfig
) -> ReleaseInterlockDecision:
    """Purely decide whether one Task 1 release may reach the actuator."""
    if context.cancelled:
        return _rejected(context, ReleaseRejectionReason.CANCELLED)
    match context.phase:
        case ReleasePhase.TASK1_RELEASE:
            pass
        case (
            ReleasePhase.TASK1_ESCORT
            | ReleasePhase.TASK2_DESCENT
            | ReleasePhase.RECOVERY
        ):
            return _rejected(context, ReleaseRejectionReason.BAD_PHASE)
        case unreachable:
            assert_never(unreachable)

    timestamps = (
        context.now_monotonic_s,
        context.target_observed_at_s,
        context.vehicle_observed_at_s,
        context.localization_observed_at_s,
    )
    if any(not math.isfinite(value) for value in timestamps):
        return _rejected(context, ReleaseRejectionReason.CLOCK_INVALID)
    if any(value > context.now_monotonic_s for value in timestamps[1:]):
        return _rejected(context, ReleaseRejectionReason.CLOCK_INVALID)
    if context.now_monotonic_s - context.target_observed_at_s > config.freshness_timeout_s:
        return _rejected(context, ReleaseRejectionReason.TARGET_STALE)
    if context.now_monotonic_s - context.vehicle_observed_at_s > config.freshness_timeout_s:
        return _rejected(context, ReleaseRejectionReason.VEHICLE_STALE)
    if (
        context.now_monotonic_s - context.localization_observed_at_s
        > config.freshness_timeout_s
    ):
        return _rejected(context, ReleaseRejectionReason.LOCALIZATION_STALE)
    if not context.calibration_valid:
        return _rejected(context, ReleaseRejectionReason.CALIBRATION_INVALID)
    if not math.isfinite(context.standoff_m):
        return _rejected(context, ReleaseRejectionReason.STANDOFF_UNSAFE)
    if context.standoff_m < config.minimum_standoff_m:
        return _rejected(context, ReleaseRejectionReason.STANDOFF_UNSAFE)
    return ReleaseAuthorized(request_id=context.request_id)


class PayloadPlugin:
    """Preserve legacy generation and own the exactly-once release latch."""

    def __init__(self, release_latch: ReleaseLatch | None = None) -> None:
        self._release_latch = release_latch

    def generate(self, params: PayloadParams) -> PayloadParams:
        return params

    def release(
        self,
        context: ReleaseContext,
        actuator: PayloadActuator,
        config: PayloadBoundaryConfig,
    ) -> ReleaseResult:
        decision = evaluate_release_interlock(context, config)
        match decision:
            case ReleaseRejected():
                return decision
            case ReleaseAuthorized():
                pass
            case unreachable:
                assert_never(unreachable)

        if self._release_latch is None:
            return _rejected(context, ReleaseRejectionReason.LATCH_UNAVAILABLE)
        if not self._release_latch.claim(context.request_id):
            return _rejected(context, ReleaseRejectionReason.ALREADY_ATTEMPTED)
        outcome = actuator.release(
            ActuatorCommand(
                request_id=context.request_id,
                timeout_s=config.actuator_timeout_s,
            )
        )
        match outcome:
            case ActuatorAcknowledged(acknowledgement_id=acknowledgement_id):
                return ReleaseSucceeded(
                    request_id=context.request_id,
                    acknowledgement_id=acknowledgement_id,
                )
            case ActuatorRejected():
                return _rejected(context, ReleaseRejectionReason.ACTUATOR_REJECTED)
            case ActuatorTimedOut():
                return _rejected(context, ReleaseRejectionReason.ACTUATOR_TIMEOUT)
            case ActuatorUnknown():
                return _rejected(context, ReleaseRejectionReason.ACTUATOR_UNKNOWN)
            case unreachable:
                assert_never(unreachable)
