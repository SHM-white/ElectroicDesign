from __future__ import annotations

from ed_uav_mission.d_task_model import DTaskEffect, DTaskKind, DTaskPhase, RouteStage
from ed_uav_mission.d_task_reducer import (
    CommandCompleted,
    ContactObserved,
    DTaskRuntime,
    DTaskRuntimeConfig,
    TargetObserved,
    Tick,
    VehicleObserved,
)

from d_task_fakes import contact_update, payload_config, selection, target, vehicle


def _advance(runtime: DTaskRuntime, event, trace: list[DTaskPhase]):
    transition = runtime.advance(event)
    trace.append(transition.state.phase)
    return transition


def test_task1_releases_once_before_d_and_lands_home_within_90_seconds() -> None:
    # Given: a committed Task 1 selection and deterministic fresh replay inputs.
    runtime = DTaskRuntime(selection(DTaskKind.PAYLOAD_DROP), DTaskRuntimeConfig(), payload_config())
    trace = [DTaskPhase.WAITING_START]

    # When: start, takeoff, three stable seconds, target acquisition, B, release, and H execute.
    takeoff = _advance(runtime, VehicleObserved(1.0, vehicle(1.0)), trace)
    stabilizing = _advance(runtime, CommandCompleted(2.0, DTaskEffect.TAKEOFF), trace)
    assert stabilizing.effect is DTaskEffect.HOVER
    _advance(runtime, Tick(4.99), trace)
    acquiring = _advance(runtime, Tick(5.0), trace)
    assert acquiring.state.phase is DTaskPhase.ACQUIRING
    escort = _advance(runtime, TargetObserved(5.1, target(5.1)), trace)
    assert escort.effect is DTaskEffect.TRACK_TARGET
    release = _advance(runtime, VehicleObserved(20.0, vehicle(20.0, RouteStage.B)), trace)
    assert release.effect is DTaskEffect.RELEASE_PAYLOAD
    returning = _advance(runtime, CommandCompleted(20.2, DTaskEffect.RELEASE_PAYLOAD), trace)
    assert returning.effect is DTaskEffect.RETURN_HOME
    landing = _advance(runtime, CommandCompleted(40.0, DTaskEffect.RETURN_HOME), trace)
    complete = _advance(runtime, CommandCompleted(45.0, DTaskEffect.LAND_HOME), trace)

    # Then: release occurs once, before D, with the exact required phase order and terminal success.
    assert takeoff.effect is DTaskEffect.TAKEOFF
    assert landing.effect is DTaskEffect.LAND_HOME
    assert complete.state.phase is DTaskPhase.SUCCEEDED
    assert complete.state.release_attempted is True
    assert trace == [
        DTaskPhase.WAITING_START,
        DTaskPhase.TAKEOFF,
        DTaskPhase.STABILIZING,
        DTaskPhase.STABILIZING,
        DTaskPhase.ACQUIRING,
        DTaskPhase.ESCORTING,
        DTaskPhase.RELEASING,
        DTaskPhase.RETURNING_HOME,
        DTaskPhase.LANDING_HOME,
        DTaskPhase.SUCCEEDED,
    ]


def test_task2_tracks_descends_dwells_five_seconds_then_lands_home() -> None:
    # Given: a committed Task 2 selection and dense moving contact samples.
    runtime = DTaskRuntime(selection(DTaskKind.DYNAMIC_LANDING), DTaskRuntimeConfig(), payload_config())
    trace = [DTaskPhase.WAITING_START]

    # When: the branch tracks, descends before D, proves dwell, retakes off, and returns H.
    _advance(runtime, VehicleObserved(1.0, vehicle(1.0)), trace)
    _advance(runtime, CommandCompleted(2.0, DTaskEffect.TAKEOFF), trace)
    _advance(runtime, Tick(5.0), trace)
    _advance(runtime, TargetObserved(5.1, target(5.1)), trace)
    descent = _advance(runtime, VehicleObserved(20.0, vehicle(20.0, RouteStage.B)), trace)
    dwell = _advance(runtime, CommandCompleted(20.5, DTaskEffect.DESCEND_TO_VEHICLE), trace)
    assert descent.effect is DTaskEffect.DESCEND_TO_VEHICLE
    assert dwell.state.phase is DTaskPhase.VEHICLE_DWELL
    for index in range(26):
        transition = _advance(
            runtime,
            ContactObserved(contact_update(21.0 + index * 0.2, 100 + index)),
            trace,
        )
    assert transition.effect is DTaskEffect.TAKEOFF
    _advance(runtime, CommandCompleted(26.2, DTaskEffect.TAKEOFF), trace)
    _advance(runtime, CommandCompleted(40.0, DTaskEffect.RETURN_HOME), trace)
    complete = _advance(runtime, CommandCompleted(45.0, DTaskEffect.LAND_HOME), trace)

    # Then: completion follows one continuous >=5 s dwell and the home landing.
    assert DTaskPhase.TRACKING in trace
    assert DTaskPhase.DESCENDING in trace
    assert DTaskPhase.RETAKEOFF in trace
    assert complete.state.phase is DTaskPhase.SUCCEEDED
