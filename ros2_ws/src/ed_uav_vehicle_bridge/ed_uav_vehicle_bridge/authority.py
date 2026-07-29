"""Authoritative prestart, arm, and exactly-once start state machine."""

from __future__ import annotations

from threading import RLock
from typing_extensions import assert_never

from .models import (
    AuthorityDecision,
    AuthorityState,
    BootEpoch,
    ExecuteMissionCommand,
    MissionSelectionValue,
    RejectCode,
    SelectMissionCommand,
    SelectionAckValue,
    SelectionId,
)


class BridgeAuthority:
    """Mutable synchronized authority state for one vehicle run."""

    def __init__(self, mission_timeout_seconds: float = 90.0) -> None:
        if mission_timeout_seconds <= 0.0:
            raise ValueError("mission timeout must be positive")
        self._lock = RLock()
        self._state = AuthorityState.BOOT_LOCKED
        self._car_epoch: BootEpoch | None = None
        self._pending: MissionSelectionValue | None = None
        self._committed: MissionSelectionValue | None = None
        self._mission_timeout_seconds = mission_timeout_seconds

    def observe_car_epoch(self, epoch: BootEpoch, fcu_armed: bool) -> AuthorityDecision:
        with self._lock:
            if self._car_epoch == epoch:
                return self._result(True, "CAR_SESSION_CURRENT")
            self._car_epoch = epoch
            self._pending = None
            self._committed = None
            self._state = AuthorityState.FAULT if fcu_armed else AuthorityState.PRESTART
            reason = RejectCode.FCU_ALREADY_ARMED if fcu_armed else "CAR_SESSION_READY"
            return self._result(not fcu_armed, reason)

    def request_selection(
        self, selection: MissionSelectionValue, fcu_armed: bool
    ) -> AuthorityDecision:
        with self._lock:
            if self._car_epoch is None:
                return self._result(False, RejectCode.NO_CAR_SESSION)
            if self._state is AuthorityState.CAR_RUNNING:
                return self._result(False, RejectCode.READ_ONLY_AFTER_START)
            if fcu_armed:
                return self._result(False, RejectCode.FCU_ALREADY_ARMED)
            if selection.car_boot_epoch != self._car_epoch:
                return self._result(False, RejectCode.CAR_EPOCH_MISMATCH)
            if self._state is AuthorityState.SELECTED and selection == self._committed:
                acknowledgement = SelectionAckValue(
                    contract_version=1,
                    selection_id=selection.selection_id,
                    car_boot_epoch=selection.car_boot_epoch,
                    accepted=True,
                    state=self._state,
                    reason="RECONFIRMED",
                )
                return self._result(
                    True,
                    acknowledgement.reason,
                    acknowledgement=acknowledgement,
                )
            if self._state is not AuthorityState.PRESTART:
                return self._result(False, RejectCode.SELECTION_ALREADY_COMMITTED)
            self._pending = selection
            self._state = AuthorityState.SELECT_PENDING
            return self._result(
                True,
                "SELECTION_PENDING",
                select_command=SelectMissionCommand(selection),
            )

    def commit_selection(
        self,
        selection_id: SelectionId,
        accepted: bool,
        reason: str,
        fcu_armed: bool,
    ) -> AuthorityDecision:
        with self._lock:
            if self._pending is None:
                return self._result(False, RejectCode.SELECTION_NOT_PENDING)
            if self._pending.selection_id != selection_id:
                return self._result(False, RejectCode.SELECTION_ID_MISMATCH)
            pending = self._pending
            if fcu_armed:
                self._pending = None
                self._state = AuthorityState.FAULT
                return self._result(False, RejectCode.ARMED_DURING_SELECTION)
            if not accepted:
                self._pending = None
                self._state = AuthorityState.PRESTART
                return self._result(False, reason or RejectCode.SELECTION_REJECTED)

            self._pending = None
            self._committed = pending
            self._state = AuthorityState.SELECTED
            acknowledgement = SelectionAckValue(
                contract_version=1,
                selection_id=pending.selection_id,
                car_boot_epoch=pending.car_boot_epoch,
                accepted=True,
                state=self._state,
                reason=reason or "ACK",
            )
            return self._result(True, acknowledgement.reason, acknowledgement=acknowledgement)

    def observe_arm(self, armed: bool) -> AuthorityDecision:
        with self._lock:
            if not armed:
                if self._state is AuthorityState.ARMED_READY:
                    self._state = AuthorityState.SELECTED
                return self._result(True, "FCU_UNARMED")
            match self._state:
                case AuthorityState.SELECTED:
                    self._state = AuthorityState.ARMED_READY
                    return self._result(True, "ARMED_READY")
                case AuthorityState.ARMED_READY:
                    return self._result(True, "ARMED_READY")
                case AuthorityState.CAR_RUNNING:
                    return self._result(True, "CAR_RUNNING")
                case AuthorityState.BOOT_LOCKED | AuthorityState.PRESTART | AuthorityState.SELECT_PENDING:
                    return self._result(False, RejectCode.NO_COMMITTED_SELECTION)
                case AuthorityState.FAULT:
                    return self._result(False, RejectCode.FAULT_LATCHED)
                case unreachable:
                    assert_never(unreachable)

    def observe_car_start(self, epoch: BootEpoch) -> AuthorityDecision:
        with self._lock:
            if self._car_epoch != epoch:
                return self._result(False, RejectCode.CAR_EPOCH_MISMATCH)
            match self._state:
                case AuthorityState.BOOT_LOCKED | AuthorityState.PRESTART | AuthorityState.SELECT_PENDING:
                    return self._result(False, RejectCode.NO_COMMITTED_SELECTION)
                case AuthorityState.SELECTED:
                    return self._result(False, RejectCode.FCU_NOT_ARMED)
                case AuthorityState.ARMED_READY:
                    if self._committed is None:
                        return self._result(False, RejectCode.NO_COMMITTED_SELECTION)
                    selection = self._committed
                    self._state = AuthorityState.CAR_RUNNING
                    command = ExecuteMissionCommand(
                        mission_id=selection.mission_id,
                        field_profile_id=selection.deployment_preset_id,
                        timeout_seconds=self._mission_timeout_seconds,
                    )
                    return self._result(True, "MISSION_DISPATCH", execute_command=command)
                case AuthorityState.CAR_RUNNING:
                    return self._result(False, RejectCode.START_ALREADY_CONSUMED)
                case AuthorityState.FAULT:
                    return self._result(False, RejectCode.FAULT_LATCHED)
                case unreachable:
                    assert_never(unreachable)

    def telemetry_fault(self) -> AuthorityDecision:
        with self._lock:
            self._state = AuthorityState.FAULT
            return self._result(False, RejectCode.TELEMETRY_STALE)

    def _result(
        self,
        accepted: bool,
        reason: str,
        *,
        select_command: SelectMissionCommand | None = None,
        acknowledgement: SelectionAckValue | None = None,
        execute_command: ExecuteMissionCommand | None = None,
    ) -> AuthorityDecision:
        return AuthorityDecision(
            accepted=accepted,
            state=self._state,
            reason=str(reason),
            select_command=select_command,
            acknowledgement=acknowledgement,
            execute_command=execute_command,
        )
