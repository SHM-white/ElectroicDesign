from __future__ import annotations

import math
from pathlib import Path

from ed_uav_mission.action_lifecycle import ActiveGoals, MissionDeadline

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class FakeGoal:
    def __init__(self) -> None:
        self.cancel_requests = 0

    def cancel_goal_async(self) -> None:
        self.cancel_requests += 1


def test_active_goals_cancel_flight_and_planner_once() -> None:
    # Given: accepted FlightCommand and planner goals are both active.
    flight = FakeGoal()
    planner = FakeGoal()
    active = ActiveGoals(flight=flight, planner=planner)

    # When: the parent mission cancellation propagates.
    active.cancel_active()

    # Then: both action servers receive exactly one cancellation request.
    assert flight.cancel_requests == 1
    assert planner.cancel_requests == 1


def test_mission_deadline_uses_the_minimum_positive_limit() -> None:
    # Given: a goal timeout, a shorter config timeout, and a steady start time.
    deadline = MissionDeadline.from_limits(now_sec=100.0, limits=(0.0, 40.0, 12.0))

    # When: remaining mission time is inspected.
    # Then: zero means unbounded and the shortest positive limit wins.
    assert deadline.remaining_sec(105.0) == 7.0
    assert deadline.remaining_sec(112.0) == 0.0
    assert MissionDeadline.from_limits(now_sec=0.0, limits=(math.inf, 3.0)).deadline_sec == 3.0


def test_lifecycle_sources_use_steady_timer_deadlines_and_recovery() -> None:
    # Given: the production lifecycle, executor, and competition runtime sources.
    lifecycle_source = (PACKAGE_ROOT / "ed_uav_mission" / "action_lifecycle.py").read_text(
        encoding="utf-8"
    )
    executor_source = (PACKAGE_ROOT / "ed_uav_mission" / "executor.py").read_text(
        encoding="utf-8"
    )
    runtime_source = (PACKAGE_ROOT / "ed_uav_mission" / "competition_runtime.py").read_text(
        encoding="utf-8"
    )
    planner_source = (PACKAGE_ROOT / "ed_uav_mission" / "competition_planner.py").read_text(
        encoding="utf-8"
    )

    # When: cancellation, deadline, and recovery paths are inspected.
    # Then: active goals are canceled, waits are steady-clock bounded, and recovery is FlightCommand only.
    assert "ClockType.STEADY_TIME" in lifecycle_source
    assert "node.create_timer" in lifecycle_source
    assert "future.cancel()" in lifecycle_source
    assert "future.cancelled()" in lifecycle_source
    assert "class MissionCancelled" in lifecycle_source
    assert "class MissionTimeout" in lifecycle_source
    assert "self._active_flight_goal" in executor_source
    assert "self._competition_planner.cancel_active()" in executor_source
    assert executor_source.count("await wait_with_deadline") >= 2
    assert "goal.timeout_sec + 0.5" in executor_source
    assert "goal_handle.request.timeout_sec" in executor_source
    assert "config.timeout_sec" in executor_source
    assert "await self._recover_after_airborne_failure()" in executor_source
    assert "Recovery command failed" in executor_source
    assert "goal_handle.canceled()" in executor_source
    assert "RESULT_TIMEOUT" in executor_source
    recovery_source = executor_source[
        executor_source.index("    async def _recover_after_airborne_failure"):
    ]
    assert recovery_source.index("self._send_hover(1.0") < recovery_source.index(
        "self._send_land("
    ) < recovery_source.index("self._send_disarm(")
    assert "_active_planner_goal" in planner_source
    assert "def cancel_active" in planner_source
    assert "await wait_with_deadline" in planner_source
    assert "GoalStatus.STATUS_SUCCEEDED" in planner_source
    assert "asyncio" not in lifecycle_source + executor_source + runtime_source + planner_source
