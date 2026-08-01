"""Executable runner for the third stability-test mission tree."""

from __future__ import annotations

import math
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace

from typing_extensions import assert_never

from ed_uav_interfaces.action import ExecuteMission

from ed_uav_mission.d_task_events import (
    CommandCompleted,
    CommandFailed,
    ContactObserved,
    DTaskEvent,
    SafetyInterrupted,
    TargetObserved,
    Tick,
    VehicleObserved,
)
from ed_uav_mission.d_task_model import (
    DTaskEffect,
    DTaskFault,
    DTaskPhase,
    DTaskSelection,
    DTaskState,
    DTaskTransition,
)
from ed_uav_mission.mission_model import StabilityParams


@dataclass(frozen=True, slots=True)
class StabilityCallbacks:
    """Executor-owned callback surface used by the stability mission."""

    execute_takeoff: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    send_hover: Callable[[float], Awaitable[None]]
    send_move: Callable[[float, float, float], Awaitable[None]]
    land_home: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    next_event: Callable[[], Awaitable[DTaskEvent]]
    publish_transition: Callable[[DTaskTransition, ExecuteMission.Feedback], None]
    now_s: Callable[[], float]
    capture_home: Callable[[], None]
    capture_pose: Callable[[], tuple[float, float, float]]


@dataclass(frozen=True, slots=True)
class StabilityWaypoint:
    x_m: float
    y_m: float
    label: str


class StabilityRunner:
    """Run takeoff, square, circle, hover and land sequence deterministically."""

    def __init__(self, callbacks: StabilityCallbacks, params: StabilityParams) -> None:
        self._callbacks = callbacks
        # 调参：正方形/圆形轨迹采样密度、悬停时间、航向保持均由 StabilityParams 控制
        self._params = params

    async def run(self, selection: DTaskSelection, feedback: ExecuteMission.Feedback) -> None:
        state = DTaskState(
            phase=DTaskPhase.STABILITY_PRE_HOVER,
            task=selection.task,
            phase_started_at_s=selection.committed_at_s,
            mission_started_at_s=selection.committed_at_s,
        )
        self._callbacks.publish_transition(DTaskTransition(state=state), feedback)

        # 起飞
        await self._callbacks.execute_takeoff(feedback)
        state = replace(state, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(
            DTaskTransition(state=state, effect=DTaskEffect.STABILITY_HOVER), feedback
        )

        # 起飞后悬停
        await self._callbacks.send_hover(self._params.pre_hover_sec)
        self._callbacks.capture_home()
        x, y, yaw = self._callbacks.capture_pose()

        # 顺时针正方形轨迹
        state = replace(state, phase=DTaskPhase.STABILITY_SQUARE, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(
            DTaskTransition(state=state, effect=DTaskEffect.STABILITY_WAYPOINT), feedback
        )
        for waypoint in self._square_waypoints(x, y, yaw):
            await self._handle_next_event(state, feedback)
            await self._callbacks.send_move(waypoint.x_m, waypoint.y_m, self._params.altitude_m)
            self._callbacks.publish_transition(
                DTaskTransition(
                    state=replace(state, phase_started_at_s=self._callbacks.now_s()),
                    effect=DTaskEffect.STABILITY_WAYPOINT,
                ),
                feedback,
            )

        # 顺时针圆形轨迹
        state = replace(state, phase=DTaskPhase.STABILITY_CIRCLE, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(
            DTaskTransition(state=state, effect=DTaskEffect.STABILITY_WAYPOINT), feedback
        )
        for waypoint in self._circle_waypoints(x, y, yaw):
            await self._handle_next_event(state, feedback)
            await self._callbacks.send_move(waypoint.x_m, waypoint.y_m, self._params.altitude_m)
            self._callbacks.publish_transition(
                DTaskTransition(
                    state=replace(state, phase_started_at_s=self._callbacks.now_s()),
                    effect=DTaskEffect.STABILITY_WAYPOINT,
                ),
                feedback,
            )

        # 降落前悬停
        state = replace(state, phase=DTaskPhase.STABILITY_POST_HOVER, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(
            DTaskTransition(state=state, effect=DTaskEffect.STABILITY_HOVER), feedback
        )
        await self._callbacks.send_hover(self._params.post_hover_sec)

        # 降落
        self._callbacks.publish_transition(
            DTaskTransition(
                state=replace(state, phase=DTaskPhase.RETURNING_HOME, phase_started_at_s=self._callbacks.now_s()),
                effect=DTaskEffect.STABILITY_HOVER,
            ),
            feedback,
        )
        self._callbacks.publish_transition(
            DTaskTransition(
                state=replace(state, phase=DTaskPhase.LANDING_HOME, phase_started_at_s=self._callbacks.now_s()),
                effect=DTaskEffect.LAND_HOME,
            ),
            feedback,
        )
        await self._callbacks.land_home(feedback)
        state = replace(state, phase=DTaskPhase.SUCCEEDED, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(DTaskTransition(state=state, complete=True), feedback)

    async def _handle_next_event(self, state: DTaskState, feedback: ExecuteMission.Feedback) -> None:
        event = await self._callbacks.next_event()
        match event:
            case SafetyInterrupted(fault=DTaskFault.HARD_LOCKED, now_s=now_s, reason=reason):
                aborted = replace(
                    state,
                    phase=DTaskPhase.ABORTED,
                    phase_started_at_s=now_s,
                    fault=DTaskFault.HARD_LOCKED,
                    reason=reason,
                )
                self._callbacks.publish_transition(
                    DTaskTransition(state=aborted, complete=True),
                    feedback,
                )
                raise RuntimeError(reason or "physical hard lock")
            case SafetyInterrupted(now_s=now_s, fault=fault, reason=reason):
                safe_state = replace(
                    state,
                    phase=DTaskPhase.SAFE_HOVER,
                    phase_started_at_s=now_s,
                    fault=fault,
                    reason=reason,
                )
                self._callbacks.publish_transition(DTaskTransition(state=safe_state), feedback)
                await self._callbacks.send_hover(0.5)
                safe_state = replace(
                    safe_state,
                    phase=DTaskPhase.SAFE_LAND,
                    phase_started_at_s=self._callbacks.now_s(),
                )
                self._callbacks.publish_transition(DTaskTransition(state=safe_state), feedback)
                await self._callbacks.land_home(feedback)
                raise RuntimeError(reason or "stability mission interrupted")
            case Tick() | VehicleObserved() | TargetObserved() | ContactObserved() | CommandCompleted() | CommandFailed():
                return
            case unreachable:
                assert_never(unreachable)

    def _square_waypoints(self, start_x: float, start_y: float, yaw_rad: float) -> list[StabilityWaypoint]:
        # 顺时针正方形：先沿航向前进，然后向机体右侧转弯，保持航向角不变
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)
        right_x = math.cos(yaw_rad - math.pi / 2.0)
        right_y = math.sin(yaw_rad - math.pi / 2.0)
        side_m = self._params.square_side_m

        p0 = (start_x, start_y)
        p1 = (p0[0] + forward_x * side_m, p0[1] + forward_y * side_m)
        p2 = (p1[0] + right_x * side_m, p1[1] + right_y * side_m)
        p3 = (p2[0] - forward_x * side_m, p2[1] - forward_y * side_m)
        # p4 should return to start within planner tolerance

        return [
            StabilityWaypoint(x_m=p1[0], y_m=p1[1], label="stability_square_1"),
            StabilityWaypoint(x_m=p2[0], y_m=p2[1], label="stability_square_2"),
            StabilityWaypoint(x_m=p3[0], y_m=p3[1], label="stability_square_3"),
            StabilityWaypoint(x_m=p0[0], y_m=p0[1], label="stability_square_4"),
        ]

    def _circle_waypoints(self, start_x: float, start_y: float, yaw_rad: float) -> list[StabilityWaypoint]:
        # 顺时针圆形轨迹：以起点为圆弧起点，保持恒定航向
        radius = self._params.circle_diameter_m / 2.0
        center_x = start_x + radius * math.cos(yaw_rad - math.pi / 2.0)
        center_y = start_y + radius * math.sin(yaw_rad - math.pi / 2.0)
        point_count = self._circle_point_count()

        waypoints: list[StabilityWaypoint] = []
        for index in range(1, point_count + 1):
            angle = yaw_rad + math.pi / 2.0 - (2.0 * math.pi * index / point_count)
            waypoints.append(
                StabilityWaypoint(
                    x_m=center_x + radius * math.cos(angle),
                    y_m=center_y + radius * math.sin(angle),
                    label=f"stability_circle_{index}",
                )
            )
        return waypoints

    def _circle_point_count(self) -> int:
        # 调参：圆形轨迹离散段数，越大越接近圆弧，但会增加任务时长
        circumference = math.pi * self._params.circle_diameter_m
        segment_m = max(self._params.circle_segment_m, 1e-6)
        return max(8, round(circumference / segment_m))
