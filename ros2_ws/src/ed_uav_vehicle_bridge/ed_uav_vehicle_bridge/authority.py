"""Authoritative prestart, arm, and exactly-once start state machine."""

from __future__ import annotations

from threading import RLock
from typing_extensions import assert_never

from .models import (
    AuthorityDecision,
    AuthorityState,
    BootId,
    DTask,
    ExecuteMissionCommand,
    MissionSelectionValue,
    MissionPhase,
    MissionStatusFlag,
    MissionStatusValue,
    RejectCode,
    SelectMissionCommand,
    SelectionId,
    Task3FcuAuxGate,
    Task3FlightTestIdentity,
)


class BridgeAuthority:
    """Mutable synchronized authority state for one vehicle run."""

    def __init__(
        self,
        mission_timeout_seconds: float = 90.0,
        *,
        hmi_boot_id: BootId = BootId(0),
        no_car_mode: bool = False,
    ) -> None:
        if mission_timeout_seconds <= 0.0:
            raise ValueError("mission timeout must be positive")
        self._lock = RLock()
        self._state = AuthorityState.BOOT_LOCKED
        self._car_boot_id: BootId | None = None
        self._hmi_boot_id = hmi_boot_id
        self._pending: MissionSelectionValue | None = None
        self._committed: MissionSelectionValue | None = None
        self._task3_gate_consumed = False
        self._mission_timeout_seconds = mission_timeout_seconds
        self._no_car_mode = no_car_mode

    @property
    def committed_selection(self) -> MissionSelectionValue | None:
        with self._lock:
            return self._committed

    def observe_car_epoch(self, boot_id: BootId, fcu_armed: bool) -> AuthorityDecision:
        with self._lock:
            if self._car_boot_id == boot_id:
                return self._result(True, "CAR_SESSION_CURRENT")
            self._car_boot_id = boot_id
            self._pending = None
            self._committed = None
            self._task3_gate_consumed = False
            self._state = AuthorityState.FAULT if fcu_armed else AuthorityState.PRESTART
            reason = RejectCode.FCU_ALREADY_ARMED if fcu_armed else "CAR_SESSION_READY"
            return self._result(not fcu_armed, reason)

    def request_selection(
        self, selection: MissionSelectionValue, fcu_armed: bool
    ) -> AuthorityDecision:
        with self._lock:
            if self._no_car_mode:
                # 无小车模式: 无 CAR 会话/无真实飞控, 地面站选择本身即会话。
                if self._state is AuthorityState.CAR_RUNNING:
                    return self._result(False, RejectCode.READ_ONLY_AFTER_START)
                if self._car_boot_id is None:
                    self._car_boot_id = selection.car_boot_id
                if self._state is AuthorityState.BOOT_LOCKED:
                    self._state = AuthorityState.PRESTART
            elif self._car_boot_id is None:
                return self._result(False, RejectCode.NO_CAR_SESSION)
            if self._state is AuthorityState.CAR_RUNNING:
                return self._result(False, RejectCode.READ_ONLY_AFTER_START)
            if fcu_armed and not self._no_car_mode:
                return self._result(False, RejectCode.FCU_ALREADY_ARMED)
            if selection.car_boot_id != self._car_boot_id and not self._no_car_mode:
                return self._result(False, RejectCode.CAR_EPOCH_MISMATCH)
            if self._state is AuthorityState.SELECTED and selection == self._committed:
                acknowledgement = self._mission_status(
                    selection_id=selection.selection_id,
                    car_boot_id=selection.car_boot_id,
                    task=int(selection.task),
                )
                return self._result(
                    True,
                    "RECONFIRMED",
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
            if fcu_armed and not self._no_car_mode:
                self._pending = None
                self._state = AuthorityState.FAULT
                return self._result(False, RejectCode.ARMED_DURING_SELECTION)
            if not accepted:
                self._pending = None
                self._state = AuthorityState.PRESTART
                return self._result(False, reason or RejectCode.SELECTION_REJECTED)

            self._pending = None
            self._committed = pending
            self._task3_gate_consumed = False
            self._state = AuthorityState.SELECTED
            acknowledgement = self._mission_status(
                selection_id=pending.selection_id,
                car_boot_id=pending.car_boot_id,
                task=int(pending.task),
            )
            return self._result(True, reason or "ACK", acknowledgement=acknowledgement)

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

    def observe_task3_flight_gate(
        self,
        identity: Task3FlightTestIdentity,
        gate: Task3FcuAuxGate,
    ) -> AuthorityDecision:
        with self._lock:
            if self._committed is None:
                return self._result(False, RejectCode.NO_COMMITTED_SELECTION)
            if self._committed.task is not DTask.STABILITY_TEST:
                return self._result(False, "TASK3_SELECTION_REQUIRED")
            if self._state is AuthorityState.CAR_RUNNING or self._task3_gate_consumed:
                return self._result(False, RejectCode.START_ALREADY_CONSUMED)
            if self._state is not AuthorityState.SELECTED:
                return self._result(False, RejectCode.FCU_NOT_ARMED)
            if not (
                gate.communication_fresh
                and gate.motors_armed
                and gate.channel_5_task_permission
            ):
                return self._result(False, "TASK3_FCU_AUX_GATE_INCOMPLETE")
            self._task3_gate_consumed = True
            self._state = AuthorityState.CAR_RUNNING
            command = ExecuteMissionCommand(
                mission_id=identity.mission_id,
                field_profile_id=identity.field_profile_id,
                timeout_seconds=identity.timeout_seconds,
            )
            return self._result(True, "MISSION_DISPATCH", execute_command=command)

    def observe_no_car_start(
        self,
        identity: Task3FlightTestIdentity | None = None,
    ) -> AuthorityDecision:
        """Immediate start: the HMI selection commit itself is the start.

        Used when no car telemetry / AUX gate should gate the dispatch
        (simulated flight, or debug with immediate_start). The caller decides
        when this path applies; this authority no longer requires the
        construction-time no_car_mode flag.
        """
        with self._lock:
            if self._committed is None:
                return self._result(False, RejectCode.NO_COMMITTED_SELECTION)
            if self._state is not AuthorityState.SELECTED:
                return self._result(False, RejectCode.FCU_NOT_ARMED)
            selection = self._committed
            self._state = AuthorityState.CAR_RUNNING
            task_name = selection.task.name.lower()
            command = ExecuteMissionCommand(
                mission_id=(
                    identity.mission_id
                    if identity is not None
                    else f"d-task-{task_name}"
                ),
                field_profile_id=(
                    identity.field_profile_id
                    if identity is not None
                    else f"d-task-{task_name}"
                ),
                timeout_seconds=(
                    identity.timeout_seconds
                    if identity is not None
                    else self._mission_timeout_seconds
                ),
            )
            return self._result(True, "MISSION_DISPATCH", execute_command=command)

    def observe_car_start(self, boot_id: BootId) -> AuthorityDecision:
        with self._lock:
            if self._car_boot_id != boot_id:
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
                    task_name = selection.task.name.lower()
                    command = ExecuteMissionCommand(
                        mission_id=f"d-task-{task_name}",
                        field_profile_id=f"d-task-{task_name}",
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
        acknowledgement: MissionStatusValue | None = None,
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

    def _mission_status(
        self, *, selection_id: SelectionId, car_boot_id: BootId, task: int
    ) -> MissionStatusValue:
        return MissionStatusValue(
            selection_id=selection_id,
            car_boot_id=car_boot_id,
            hmi_boot_id=self._hmi_boot_id,
            phase=MissionPhase.SELECTION_ACKED,
            selected_task=task,
            reason_flags=0,
            status_flags=MissionStatusFlag.ROS_READY,
        )
