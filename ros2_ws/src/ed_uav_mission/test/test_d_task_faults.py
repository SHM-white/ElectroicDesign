from __future__ import annotations

from dataclasses import replace

import pytest
from d_task_fakes import (
    contact_update,
    payload_config,
    selection,
    stale_target,
    target,
    vehicle,
)
from ed_uav_mission.d_task_model import (
    DTaskEffect,
    DTaskFault,
    DTaskKind,
    DTaskPhase,
    PayloadState,
    RouteStage,
)
from ed_uav_mission.d_task_reducer import (
    CommandCompleted,
    CommandFailed,
    ContactObserved,
    DTaskRuntime,
    DTaskRuntimeConfig,
    SafetyInterrupted,
    TargetObserved,
    Tick,
    VehicleObserved,
)


def _airborne_runtime(task: DTaskKind = DTaskKind.PAYLOAD_DROP) -> DTaskRuntime:
    runtime = DTaskRuntime(selection(task), DTaskRuntimeConfig(), payload_config())
    runtime.advance(VehicleObserved(1.0, vehicle(1.0)))
    runtime.advance(CommandCompleted(2.0, DTaskEffect.TAKEOFF))
    runtime.advance(Tick(5.0))
    runtime.advance(CommandCompleted(5.5, DTaskEffect.MOVE_RIGHT))
    runtime.advance(TargetObserved(6.0, target(6.0)))
    return runtime


def test_never_start_aborts_without_issuing_flight_effect() -> None:
    runtime = DTaskRuntime(selection(DTaskKind.PAYLOAD_DROP), DTaskRuntimeConfig(), payload_config())

    result = runtime.advance(Tick(15.0))

    assert result.state.phase is DTaskPhase.ABORTED
    assert result.state.fault is DTaskFault.NEVER_STARTED
    assert result.effect is None


@pytest.mark.parametrize(
    ("task", "event", "fault"),
    [
        (DTaskKind.PAYLOAD_DROP, Tick(46.0), DTaskFault.B_DEADLINE_MISSED),
        (
            DTaskKind.PAYLOAD_DROP,
            VehicleObserved(20.0, vehicle(20.0, RouteStage.D)),
            DTaskFault.D_DEADLINE_MISSED,
        ),
        (
            DTaskKind.DYNAMIC_LANDING,
            VehicleObserved(20.0, vehicle(20.0, RouteStage.D)),
            DTaskFault.D_DEADLINE_MISSED,
        ),
        (DTaskKind.PAYLOAD_DROP, Tick(91.0), DTaskFault.MISSION_DEADLINE_MISSED),
    ],
)
def test_deadline_faults_enter_explicit_safe_hover(task, event, fault) -> None:
    runtime = _airborne_runtime(task)

    result = runtime.advance(event)

    assert result.state.phase is DTaskPhase.SAFE_HOVER
    assert result.state.fault is fault
    assert result.effect is DTaskEffect.HOVER


@pytest.mark.parametrize(
    ("event", "fault"),
    [
        (TargetObserved(6.0, stale_target(6.0)), DTaskFault.TARGET_STALE),
        (
            TargetObserved(6.0, replace(target(6.0), relative_error_m=2.01)),
            DTaskFault.TARGET_OUTLIER,
        ),
        (
            VehicleObserved(6.0, replace(vehicle(5.0), heartbeat_alive=False)),
            DTaskFault.VEHICLE_LOST,
        ),
        (
            SafetyInterrupted(6.0, DTaskFault.CANCELLED, "cancelled by user"),
            DTaskFault.CANCELLED,
        ),
        (
            SafetyInterrupted(6.0, DTaskFault.LOCALIZATION_LOST, "localization lost"),
            DTaskFault.LOCALIZATION_LOST,
        ),
        (
            CommandFailed(6.0, DTaskEffect.TRACK_TARGET, "V7 acknowledgement timeout"),
            DTaskFault.FLIGHT_COMMAND_FAILED,
        ),
    ],
)
def test_active_faults_recover_via_hover_return_land(event, fault) -> None:
    runtime = _airborne_runtime()

    hover = runtime.advance(event)
    returning = runtime.advance(CommandCompleted(6.1, DTaskEffect.HOVER))
    landing = runtime.advance(CommandCompleted(7.0, DTaskEffect.RETURN_HOME))
    aborted = runtime.advance(CommandCompleted(8.0, DTaskEffect.LAND_HOME))

    assert hover.state.fault is fault
    assert returning.state.phase is DTaskPhase.SAFE_RETURN
    assert returning.effect is DTaskEffect.RETURN_HOME
    assert landing.state.phase is DTaskPhase.SAFE_LAND
    assert landing.effect is DTaskEffect.LAND_HOME
    assert aborted.state.phase is DTaskPhase.ABORTED


@pytest.mark.parametrize("task", [DTaskKind.PAYLOAD_DROP, DTaskKind.DYNAMIC_LANDING])
def test_hard_lock_aborts_immediately_without_safe_recovery(task) -> None:
    """AUX6 hard lock means propellers are physically locked — skip safe recovery."""
    runtime = _airborne_runtime(task)

    result = runtime.advance(SafetyInterrupted(6.0, DTaskFault.HARD_LOCKED, "physical hard lock"))

    assert result.state.phase is DTaskPhase.ABORTED
    assert result.state.fault is DTaskFault.HARD_LOCKED
    assert result.effect is None
    assert result.complete is True


def test_payload_unknown_blocks_task1_at_start() -> None:
    runtime = DTaskRuntime(selection(DTaskKind.PAYLOAD_DROP), DTaskRuntimeConfig(), payload_config())

    result = runtime.advance(
        VehicleObserved(1.0, vehicle(1.0), payload_state=PayloadState.UNKNOWN)
    )

    assert result.state.phase is DTaskPhase.ABORTED
    assert result.state.fault is DTaskFault.PAYLOAD_UNKNOWN


@pytest.mark.parametrize("fault_kind", ["gap", "stopped"])
def test_contact_gap_or_stopped_car_interrupts_task2_dwell(fault_kind: str) -> None:
    runtime = _airborne_runtime(DTaskKind.DYNAMIC_LANDING)
    runtime.advance(VehicleObserved(20.0, vehicle(20.0, RouteStage.B)))
    runtime.advance(CommandCompleted(20.5, DTaskEffect.DESCEND_TO_VEHICLE))
    runtime.advance(ContactObserved(contact_update(21.0, 1)))
    update = contact_update(21.3 if fault_kind == "gap" else 21.1, 2)
    if fault_kind == "stopped":
        update = replace(update, vehicle_speed_m_s=0.0)

    result = runtime.advance(ContactObserved(update))

    assert result.state.phase is DTaskPhase.SAFE_HOVER
    assert result.state.fault is DTaskFault.CONTACT_INTERRUPTED


def test_repeated_recovery_interruption_falls_forward_to_safe_land() -> None:
    runtime = _airborne_runtime()
    runtime.advance(SafetyInterrupted(6.0, DTaskFault.CANCELLED, "cancelled"))

    result = runtime.advance(CommandFailed(6.1, DTaskEffect.HOVER, "hover timeout"))

    assert result.state.phase is DTaskPhase.SAFE_LAND
    assert result.effect is DTaskEffect.LAND_HOME
