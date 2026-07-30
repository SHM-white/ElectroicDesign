"""Runtime for the third stability-test mission branch."""

from __future__ import annotations

import math
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace

from ed_uav_interfaces.action import ExecuteMission
from typing_extensions import assert_never

from ed_uav_mission.d_task_events import (
    CommandCompleted,
    CommandFailed,
    DTaskEvent,
    SafetyInterrupted,
    TargetObserved,
    Tick,
    VehicleObserved,
    ContactObserved,
)
from ed_uav_mission.d_task_model import (
    DTaskEffect,
    DTaskPhase,
    DTaskSelection,
    DTaskState,
    DTaskTransition,
)
from ed_uav_mission.mission_model import StabilityParams


class DTaskMissionAborted(RuntimeError):
    """The reducer completed its explicit safe recovery chain."""


@dataclass(frozen=True, slots=True)
class StabilityCallbacks:
    """Executor-owned callback surface used by the stability mission."""

    execute_takeoff: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    send_hover: Callable[[float], Awaitable[None]]
    capture_home: Callable[[], None]
    send_move: Callable[[float, float, float], Awaitable[None]]
    land_home: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    next_event: Callable[[], Awaitable[DTaskEvent]]
    publish_transition: Callable[[DTaskTransition, ExecuteMission.Feedback], None]
    now_s: Callable[[], float]
    capture_pose: Callable[[], tuple[float, float, float]]


class StabilityRuntime:
    """Run the third stability-test branch with deterministic waypoints."""

    def __init__(
        self,
        callbacks: StabilityCallbacks,
        params: StabilityParams,
    ) -> None:
        # 调参：航向保持、轨迹采样密度均来自 StabilityParams
        self._callbacks = callbacks
        self._params = params

    def cancel_active(self) -> None:
        """Cancellation is owned by the FlightCommand action boundary."""

    async def run(
        self,
        selection: DTaskSelection,
        feedback: ExecuteMission.Feedback,
    ) -> None:
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
        self._callbacks.publish_transition(DTaskTransition(state=state, effect=DTaskEffect.STABILITY_HOVER), feedback)

        # 起飞后悬停
        await self._callbacks.send_hover(self._params.pre_hover_sec)
        self._callbacks.capture_home()
        x, y, yaw = self._callbacks.capture_pose()
        waypoints = self._build_path(x, y, yaw)
        square_index = 1
        square_end_index = 5
        circle_index = square_end_index
        circle_end_index = len(waypoints)

        state = replace(state, phase=DTaskPhase.STABILITY_SQUARE, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(DTaskTransition(state=state, effect=DTaskEffect.STABILITY_WAYPOINT), feedback)

        # 顺时针正方形轨迹
        for index in range(square_index, square_end_index):
            event = await self._next_with_interrupt(state)
            if isinstance(event, SafetyInterrupted):
                await self._safe_land(state, event, feedback)
            target = waypoints[index]
            await self._callbacks.send_move(target[0], target[1], self._params.altitude_m)
            self._callbacks.publish_transition(
                DTaskTransition(state=replace(state, phase_started_at_s=self._callbacks.now_s()), effect=DTaskEffect.STABILITY_WAYPOINT),
                feedback,
            )

        # 顺时针圆形轨迹
        state = replace(state, phase=DTaskPhase.STABILITY_CIRCLE, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(DTaskTransition(state=state, effect=DTaskEffect.STABILITY_WAYPOINT), feedback)
        for index in range(circle_index, circle_end_index):
            event = await self._next_with_interrupt(state)
            if isinstance(event, SafetyInterrupted):
                await self._safe_land(state, event, feedback)
            target = waypoints[index]
            await self._callbacks.send_move(target[0], target[1], self._params.altitude_m)
            self._callbacks.publish_transition(
                DTaskTransition(state=replace(state, phase_started_at_s=self._callbacks.now_s()), effect=DTaskEffect.STABILITY_WAYPOINT),
                feedback,
            )

        # 降落前悬停
        state = replace(state, phase=DTaskPhase.STABILITY_POST_HOVER, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(DTaskTransition(state=state, effect=DTaskEffect.STABILITY_HOVER), feedback)
        await self._callbacks.send_hover(self._params.post_hover_sec)

        # 降落
        self._callbacks.publish_transition(
            DTaskTransition(state=replace(state, phase=DTaskPhase.LANDING_HOME, phase_started_at_s=self._callbacks.now_s()), effect=DTaskEffect.LAND_HOME),
            feedback,
        )
        await self._callbacks.land_home(feedback)
        state = replace(state, phase=DTaskPhase.SUCCEEDED, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(DTaskTransition(state=state, complete=True), feedback)

    async def _safe_land(
        self,
        state: DTaskState,
        event: SafetyInterrupted,
        feedback: ExecuteMission.Feedback,
    ) -> None:
        state = replace(state, phase=DTaskPhase.SAFE_HOVER, phase_started_at_s=event.now_s, fault=event.fault, reason=event.reason)
        self._callbacks.publish_transition(DTaskTransition(state=state, effect=DTaskEffect.HOVER), feedback)
        await self._callbacks.send_hover(0.5)
        state = replace(state, phase=DTaskPhase.SAFE_LAND, phase_started_at_s=self._callbacks.now_s())
        self._callbacks.publish_transition(DTaskTransition(state=state, effect=DTaskEffect.LAND_HOME), feedback)
        await self._callbacks.land_home(feedback)
        raise DTaskMissionAborted(state.reason or "stability aborted")

    async def _next_with_interrupt(self, state: DTaskState) -> DTaskEvent:
        event = await self._callbacks.next_event()
        match event:
            case SafetyInterrupted():
                return event
            case Tick() | VehicleObserved() | TargetObserved() | CommandCompleted() | CommandFailed() | ContactObserved():
                return event
            case unreachable:
                assert_never(unreachable)

    def _square_segment_count(self) -> int:
        side_m = max(self._params.square_side_m, 1e-9)
        segment_m = max(self._params.square_segment_m, 1e-9)
        return int(max(1, round(side_m / segment_m)))

    def _circle_point_count(self) -> int:
        circumference = math.pi * self._params.circle_diameter_m
        segment_m = max(self._params.circle_segment_m, 1e-9)
        return int(max(8, round(circumference / segment_m)))

    def _build_path(self, start_x: float, start_y: float, yaw_rad: float) -> list[tuple[float, float]]:
        # 顺时针正方形：沿航向前进，然后向机体右侧转弯，保持航向角不变
        forward_rad = yaw_rad
        right_rad = yaw_rad - math.pi / 2.0
        waypoints: list[tuple[float, float]] = []
        x, y = start_x, start_y
        for index in range(4):
            dx = math.cos(forward_rad) if index == 0 else math.cos(right_rad) if index == 1 else -math.cos(forward_rad) if index == 2 else -math.cos(right_rad)
            dy = math.sin(forward_rad) if index == 0 else math.sin(right_rad) if index == 1 else -math.sin(forward_rad) if index == 2 else -math.sin(right_rad)
            x += dx * self._params.square_side_m
            y += dy * self._params.square_side_m
            waypoints.append((x, y))
        radius = self._params.circle_diameter_m / 2.0
        center_x = start_x + radius * math.cos(right_rad)
        center_y = start_y + radius * math.sin(right_rad)
        angle = yaw_rad + math.pi / 2.0
        point_count = self._circle_point_count()
        for _ in range(point_count):
            angle -= (2.0 * math.pi) / point_count
            waypoints.append((center_x + radius * math.cos(angle), center_y + radius * math.sin(angle)))
        return waypoints
