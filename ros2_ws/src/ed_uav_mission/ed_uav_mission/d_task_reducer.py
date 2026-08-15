"""Deterministic event boundary for the 2026 D-task state reducer."""

from __future__ import annotations

from dataclasses import replace

from typing_extensions import assert_never


def _log(msg: str) -> None:
    print(f"[REDUCER] {msg}", flush=True)

from ed_uav_mission.d_task_events import (
    CommandCompleted,
    CommandFailed,
    ContactObserved,
    DTaskEvent,
    DTaskRuntimeConfig,
    SafetyInterrupted,
    TargetObserved,
    TargetSnapshot,
    Tick,
    VehicleObserved,
    VehicleSnapshot,
    event_time,
)
from ed_uav_mission.d_task_model import (
    DTaskEffect,
    DTaskFault,
    DTaskKind,
    DTaskPhase,
    DTaskSelection,
    DTaskState,
    DTaskTransition,
    PayloadState,
    RouteStage,
)
from ed_uav_mission.payload_config import PayloadBoundaryConfig
from ed_uav_mission.touchdown import (
    DwellComplete,
    DwellInterrupted,
    DwellProgress,
    TouchdownDwellTracker,
    TouchdownUpdate,
)


class DTaskRuntime:
    """Own immutable reducer states and the Todo 7 contact accumulator."""

    def __init__(
        self,
        selection: DTaskSelection,
        config: DTaskRuntimeConfig,
        payload_config: PayloadBoundaryConfig,
    ) -> None:
        self.selection, self.config = selection, config
        self.state = DTaskState(
            phase=DTaskPhase.WAITING_START,
            task=selection.task,
            phase_started_at_s=selection.committed_at_s,
        )
        self._dwell = TouchdownDwellTracker(payload_config)
        self._last_vehicle_sequence: int | None = None
        self._last_target_sequence: int | None = None

    def advance(self, event: DTaskEvent) -> DTaskTransition:
        now_s = event_time(event)
        deadline = self._deadline_fault(now_s)
        if deadline is not None:
            _log(f"deadline fault={deadline.value} phase={self.state.phase.value}")
            transition = self._interrupt(now_s, deadline, deadline.value)
            self.state = transition.state
            return transition
        old_phase = self.state.phase
        match event:
            case Tick():
                transition = self._on_tick(now_s)
            case VehicleObserved(vehicle=vehicle, payload_state=payload_state):
                transition = self._on_vehicle(now_s, vehicle, payload_state)
            case TargetObserved(target=target):
                transition = self._on_target(now_s, target)
            case CommandCompleted(effect=effect):
                transition = self._on_command_completed(now_s, effect)
            case CommandFailed(reason=reason):
                transition = self._on_command_failed(now_s, reason)
            case ContactObserved(update=update):
                transition = self._on_contact(update)
            case SafetyInterrupted(fault=fault, reason=reason):
                if fault is DTaskFault.HARD_LOCKED:
                    transition = self._abort(now_s, fault, reason)
                else:
                    transition = self._interrupt(now_s, fault, reason)
            case unreachable:
                assert_never(unreachable)
        new_phase = transition.state.phase
        if old_phase is not new_phase:
            _log(f"{old_phase.value} → {new_phase.value} effect={transition.effect}")
        self.state = transition.state
        return transition

    def _deadline_fault(self, now_s: float) -> DTaskFault | None:
        started_at = self.state.mission_started_at_s
        if self.state.phase is DTaskPhase.WAITING_START:
            if now_s - self.selection.committed_at_s >= self.config.start_deadline_s:
                return DTaskFault.NEVER_STARTED
            return None
        if started_at is None or self.state.phase in (
            DTaskPhase.SUCCEEDED,
            DTaskPhase.ABORTED,
            DTaskPhase.SAFE_HOVER,
            DTaskPhase.SAFE_RETURN,
            DTaskPhase.SAFE_LAND,
        ):
            return None
        elapsed_s = now_s - started_at
        if elapsed_s >= self.config.mission_deadline_s:
            return DTaskFault.MISSION_DEADLINE_MISSED
        if (
            self.state.phase in (DTaskPhase.ESCORTING, DTaskPhase.TRACKING, DTaskPhase.SEARCHING)
            and elapsed_s >= self.config.b_deadline_s
        ):
            return DTaskFault.B_DEADLINE_MISSED
        if self.state.phase in (
            DTaskPhase.SEARCHING,
            DTaskPhase.RELEASING,
            DTaskPhase.DESCENDING,
            DTaskPhase.VEHICLE_DWELL,
        ) and elapsed_s >= self.config.d_deadline_s:
            return DTaskFault.D_DEADLINE_MISSED
        return None

    def _on_tick(self, now_s: float) -> DTaskTransition:
        if (
            self.state.phase is DTaskPhase.STABILIZING
            and now_s - self.state.phase_started_at_s + 1e-9 >= self.config.stable_s
        ):
            return self._transition(DTaskPhase.MOVE_RIGHT, now_s, DTaskEffect.MOVE_RIGHT)
        if self.state.phase is DTaskPhase.SEARCHING:
            # Start forward search when entering SEARCHING phase
            return self._transition(DTaskPhase.SEARCHING, now_s, DTaskEffect.SEARCH_FORWARD)
        return DTaskTransition(state=self.state)

    def _on_vehicle(self, now_s: float, vehicle: VehicleSnapshot, payload_state: PayloadState) -> DTaskTransition:
        if self.state.phase is DTaskPhase.WAITING_START:
            if not vehicle.started:
                _log("vehicle not started yet, waiting...")
                return DTaskTransition(state=self.state)
            if self.state.task is DTaskKind.PAYLOAD_DROP and payload_state.value == 0:
                return self._abort(now_s, DTaskFault.PAYLOAD_UNKNOWN, "payload state unknown")
            if not vehicle.heartbeat_alive or now_s - vehicle.observed_at_s > self.config.vehicle_freshness_s:
                return self._abort(now_s, DTaskFault.VEHICLE_LOST, "vehicle start is stale")
            self._last_vehicle_sequence = vehicle.sequence
            state = replace(
                self.state,
                phase=DTaskPhase.TAKEOFF,
                phase_started_at_s=now_s,
                mission_started_at_s=now_s,
            )
            _log("vehicle started! → TAKEOFF")
            return DTaskTransition(state=state, effect=DTaskEffect.TAKEOFF)
        if not vehicle.heartbeat_alive or now_s - vehicle.observed_at_s > self.config.vehicle_freshness_s:
            return self._interrupt(now_s, DTaskFault.VEHICLE_LOST, "vehicle telemetry lost")
        if self._last_vehicle_sequence is not None and vehicle.sequence <= self._last_vehicle_sequence:
            return self._interrupt(now_s, DTaskFault.VEHICLE_LOST, "vehicle sequence did not advance")
        self._last_vehicle_sequence = vehicle.sequence
        if vehicle.route_stage >= RouteStage.D and self.state.phase in (
            DTaskPhase.ESCORTING,
            DTaskPhase.TRACKING,
            DTaskPhase.RELEASING,
            DTaskPhase.DESCENDING,
        ):
            return self._interrupt(now_s, DTaskFault.D_DEADLINE_MISSED, "D reached before task action")
        if vehicle.route_stage is RouteStage.B:
            if self.state.phase is DTaskPhase.ESCORTING:
                return self._transition(DTaskPhase.RELEASING, now_s, DTaskEffect.RELEASE_PAYLOAD)
            if self.state.phase is DTaskPhase.TRACKING:
                return self._transition(DTaskPhase.DESCENDING, now_s, DTaskEffect.DESCEND_TO_VEHICLE)
        return DTaskTransition(state=self.state)

    def _on_target(self, now_s: float, target: TargetSnapshot) -> DTaskTransition:
        if now_s - target.observed_at_s > self.config.target_freshness_s:
            _log(f"target stale: age={now_s - target.observed_at_s:.1f}s > {self.config.target_freshness_s:.1f}s")
            return self._interrupt(now_s, DTaskFault.TARGET_STALE, "target observation stale")
        if (
            not target.valid
            or target.relative_error_m > self.config.maximum_relative_error_m
            or (self._last_target_sequence is not None and target.sequence <= self._last_target_sequence)
        ):
            _log(f"target outlier: valid={target.valid} err={target.relative_error_m:.2f}m reason={target.rejection_reason}")
            return self._interrupt(now_s, DTaskFault.TARGET_OUTLIER, target.rejection_reason or "target outlier")
        self._last_target_sequence = target.sequence
        _log(f"target observed: phase={self.state.phase.value} err={target.relative_error_m:.2f}m")
        if self.state.phase is DTaskPhase.ACQUIRING:
            phase = (
                DTaskPhase.ESCORTING
                if self.state.task is DTaskKind.PAYLOAD_DROP
                else DTaskPhase.TRACKING
            )
            return self._transition(phase, now_s, DTaskEffect.TRACK_TARGET)
        if self.state.phase is DTaskPhase.SEARCHING:
            phase = (
                DTaskPhase.ESCORTING
                if self.state.task is DTaskKind.PAYLOAD_DROP
                else DTaskPhase.TRACKING
            )
            return self._transition(phase, now_s, DTaskEffect.TRACK_TARGET)
        return DTaskTransition(state=self.state, effect=DTaskEffect.TRACK_TARGET)

    def _on_command_completed(self, now_s: float, effect: DTaskEffect) -> DTaskTransition:
        phase = self.state.phase
        if phase is DTaskPhase.TAKEOFF and effect is DTaskEffect.TAKEOFF:
            return self._transition(DTaskPhase.STABILIZING, now_s, DTaskEffect.HOVER)
        if phase is DTaskPhase.MOVE_RIGHT and effect is DTaskEffect.MOVE_RIGHT:
            return self._transition(DTaskPhase.SEARCHING, now_s)
        if phase is DTaskPhase.SEARCHING and effect is DTaskEffect.SEARCH_FORWARD:
            # Search distance exceeded without finding target - abort
            return self._interrupt(now_s, DTaskFault.SEARCH_DISTANCE_EXCEEDED, "search distance exceeded without finding target")
        if phase is DTaskPhase.RELEASING and effect is DTaskEffect.RELEASE_PAYLOAD:
            state = replace(self.state, release_attempted=True)
            self.state = state
            return self._transition(DTaskPhase.RETURNING_HOME, now_s, DTaskEffect.RETURN_HOME)
        if phase is DTaskPhase.DESCENDING and effect is DTaskEffect.DESCEND_TO_VEHICLE:
            return self._transition(DTaskPhase.VEHICLE_DWELL, now_s)
        if phase is DTaskPhase.RETAKEOFF and effect is DTaskEffect.TAKEOFF:
            return self._transition(DTaskPhase.RETURNING_HOME, now_s, DTaskEffect.RETURN_HOME)
        if phase is DTaskPhase.RETURNING_HOME and effect is DTaskEffect.RETURN_HOME:
            return self._transition(DTaskPhase.LANDING_HOME, now_s, DTaskEffect.LAND_HOME)
        if phase is DTaskPhase.LANDING_HOME and effect is DTaskEffect.LAND_HOME:
            return self._transition(DTaskPhase.SUCCEEDED, now_s, complete=True)
        if phase is DTaskPhase.SAFE_HOVER and effect is DTaskEffect.HOVER:
            return self._transition(DTaskPhase.SAFE_RETURN, now_s, DTaskEffect.RETURN_HOME)
        if phase is DTaskPhase.SAFE_RETURN and effect is DTaskEffect.RETURN_HOME:
            return self._transition(DTaskPhase.SAFE_LAND, now_s, DTaskEffect.LAND_HOME)
        if phase is DTaskPhase.SAFE_LAND and effect is DTaskEffect.LAND_HOME:
            return self._transition(DTaskPhase.ABORTED, now_s, complete=True)
        return DTaskTransition(state=self.state)

    def _on_command_failed(self, now_s: float, reason: str) -> DTaskTransition:
        if self.state.phase in (DTaskPhase.SAFE_HOVER, DTaskPhase.SAFE_RETURN):
            return self._transition(DTaskPhase.SAFE_LAND, now_s, DTaskEffect.LAND_HOME)
        if self.state.phase is DTaskPhase.SAFE_LAND:
            return self._transition(DTaskPhase.ABORTED, now_s, complete=True)
        return self._interrupt(now_s, DTaskFault.FLIGHT_COMMAND_FAILED, reason)

    def _on_contact(self, update: TouchdownUpdate) -> DTaskTransition:
        if self.state.phase is not DTaskPhase.VEHICLE_DWELL:
            return DTaskTransition(state=self.state)
        result = self._dwell.update(update)
        match result:
            case DwellProgress():
                return DTaskTransition(state=self.state)
            case DwellComplete():
                return self._transition(DTaskPhase.RETAKEOFF, update.now_monotonic_s, DTaskEffect.TAKEOFF)
            case DwellInterrupted(reason=reason):
                return self._interrupt(
                    update.now_monotonic_s,
                    DTaskFault.CONTACT_INTERRUPTED,
                    reason.value,
                )
            case unreachable:
                assert_never(unreachable)

    def _interrupt(self, now_s: float, fault: DTaskFault, reason: str) -> DTaskTransition:
        if self.state.phase is DTaskPhase.WAITING_START:
            return self._abort(now_s, fault, reason)
        state = replace(
            self.state,
            phase=DTaskPhase.SAFE_HOVER,
            phase_started_at_s=now_s,
            fault=fault,
            reason=reason,
        )
        return DTaskTransition(state=state, effect=DTaskEffect.HOVER)

    def _abort(self, now_s: float, fault: DTaskFault, reason: str) -> DTaskTransition:
        state = replace(
            self.state,
            phase=DTaskPhase.ABORTED,
            phase_started_at_s=now_s,
            fault=fault,
            reason=reason,
        )
        return DTaskTransition(state=state, complete=True)

    def _transition(
        self,
        phase: DTaskPhase,
        now_s: float,
        effect: DTaskEffect | None = None,
        *,
        complete: bool = False,
    ) -> DTaskTransition:
        return DTaskTransition(
            state=replace(self.state, phase=phase, phase_started_at_s=now_s),
            effect=effect,
            complete=complete,
        )
