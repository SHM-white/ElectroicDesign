from __future__ import annotations

from collections.abc import Coroutine
from dataclasses import dataclass, field

import pytest
from d_task_fakes import contact_update, payload_config, selection, target, vehicle
from ed_uav_interfaces.action import ExecuteMission
from ed_uav_mission.competition_runtime import (
    CompetitionCallbacks,
    CompetitionRuntime,
    DTaskMissionAborted,
)
from ed_uav_mission.d_task_events import (
    ContactObserved,
    DTaskEvent,
    TargetObserved,
    Tick,
    VehicleObserved,
)
from ed_uav_mission.d_task_model import (
    DTaskKind,
    DTaskPhase,
    DTaskTransition,
    RouteStage,
)
from ed_uav_mission.mission_model import CompetitionParams


def _run_immediate(coroutine: Coroutine[None, None, None]) -> None:
    """Complete a replay coroutine whose deterministic fakes never suspend."""
    try:
        coroutine.send(None)
    except StopIteration:
        return
    coroutine.close()
    raise AssertionError("deterministic replay unexpectedly suspended")


@dataclass(slots=True)  # Mutation records the fake action surface's observable trace.
class FakeActionSurface:
    events: list[DTaskEvent]
    now: float = 0.0
    effects: list[str] = field(default_factory=list)
    phases: list[DTaskPhase] = field(default_factory=list)
    releases: int = 0

    async def next_event(self) -> DTaskEvent:
        if not self.events:
            raise RuntimeError(
                "replay exhausted before terminal phase: "
                f"phases={[phase.value for phase in self.phases]}, effects={self.effects}"
            )
        event = self.events.pop(0)
        match event:
            case ContactObserved(update=update):
                self.now = update.now_monotonic_s
            case Tick(now_s=now_s) | VehicleObserved(now_s=now_s) | TargetObserved(now_s=now_s):
                self.now = now_s
            case _:
                raise RuntimeError("unsupported replay event")
        return event

    async def takeoff(self, feedback: ExecuteMission.Feedback) -> None:
        self.effects.append("takeoff")
        self.now += 0.1

    async def hover(self, duration_s: float) -> None:
        self.effects.append(f"hover:{duration_s:.1f}")
        self.now += duration_s

    async def track(self, target_value, vehicle_value, altitude_m: float) -> None:
        self.effects.append(f"track:{altitude_m:.1f}")
        self.now += 0.1

    async def release(self, target_value, vehicle_value) -> None:
        self.effects.append("release")
        self.releases += 1
        self.now += 0.1

    async def descend(self, target_value, vehicle_value) -> None:
        self.effects.append("descend")
        self.now += 0.2

    async def return_home(self) -> None:
        self.effects.append("return_home")
        self.now += 0.5

    async def land_home(self, feedback: ExecuteMission.Feedback) -> None:
        self.effects.append("land_home")
        self.now += 0.2

    def capture_home(self) -> None:
        self.effects.append("capture_home")

    def publish(self, transition: DTaskTransition, feedback: ExecuteMission.Feedback) -> None:
        self.phases.append(transition.state.phase)

    def callbacks(self) -> CompetitionCallbacks:
        return CompetitionCallbacks(
            execute_takeoff=self.takeoff,
            send_hover=self.hover,
            track_target=self.track,
            release_payload=self.release,
            descend_to_vehicle=self.descend,
            return_home=self.return_home,
            land_home=self.land_home,
            capture_home=self.capture_home,
            next_event=self.next_event,
            publish_transition=self.publish,
            now_s=lambda: self.now,
        )


def _params() -> CompetitionParams:
    return CompetitionParams(
        mission_profile_id="d2026-profile",
        deployment_preset_id="simulation",
        target_revision="d2026-circle-cross-v1",
    )


def test_task1_fake_action_surface_records_release_and_home_teardown() -> None:
    surface = FakeActionSurface(
        events=[
            VehicleObserved(1.0, vehicle(1.0)),
            TargetObserved(4.2, target(4.2)),
            VehicleObserved(10.0, vehicle(10.0, RouteStage.B)),
        ]
    )

    async def scenario() -> None:
        runtime = CompetitionRuntime(surface.callbacks(), payload_config())
        await runtime.run(
            _params(),
            selection(DTaskKind.PAYLOAD_DROP),
            ExecuteMission.Feedback(),
        )

    _run_immediate(scenario())

    assert surface.releases == 1
    assert surface.effects == [
        "takeoff",
        "hover:3.0",
        "capture_home",
        "track:1.5",
        "release",
        "return_home",
        "land_home",
    ]
    assert surface.phases[-1] is DTaskPhase.SUCCEEDED


def test_task2_fake_action_surface_records_dense_dwell_and_home_teardown() -> None:
    contact_events = [
        ContactObserved(contact_update(11.0 + index * 0.2, 100 + index))
        for index in range(26)
    ]
    surface = FakeActionSurface(
        events=[
            VehicleObserved(1.0, vehicle(1.0)),
            TargetObserved(4.2, target(4.2)),
            VehicleObserved(10.0, vehicle(10.0, RouteStage.B)),
            *contact_events,
        ]
    )

    async def scenario() -> None:
        runtime = CompetitionRuntime(surface.callbacks(), payload_config())
        await runtime.run(
            _params(),
            selection(DTaskKind.DYNAMIC_LANDING),
            ExecuteMission.Feedback(),
        )

    _run_immediate(scenario())

    assert "descend" in surface.effects
    assert surface.effects.count("takeoff") == 2
    assert surface.effects[-2:] == ["return_home", "land_home"]
    assert surface.phases[-1] is DTaskPhase.SUCCEEDED


def test_never_start_fake_action_surface_aborts_without_motion() -> None:
    surface = FakeActionSurface(events=[Tick(15.0)])

    async def scenario() -> None:
        runtime = CompetitionRuntime(surface.callbacks(), payload_config())
        with pytest.raises(DTaskMissionAborted, match="never_started"):
            await runtime.run(
                _params(),
                selection(DTaskKind.PAYLOAD_DROP),
                ExecuteMission.Feedback(),
            )

    _run_immediate(scenario())

    assert surface.effects == []
    assert surface.phases[-1] is DTaskPhase.ABORTED
