"""Action adapter that executes the immutable D-task reducer effects."""

from __future__ import annotations

import math
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace

from ed_uav_interfaces.action import ExecuteMission
from typing_extensions import assert_never

from ed_uav_mission.action_lifecycle import MissionCancelled, MissionTimeout
from ed_uav_mission.d_task_events import (
    CommandCompleted,
    CommandFailed,
    DTaskEvent,
    DTaskRuntimeConfig,
    SafetyInterrupted,
    TargetObserved,
    TargetSnapshot,
    Tick,
    VehicleObserved,
    VehicleSnapshot,
)
from ed_uav_mission.d_task_model import (
    DTaskEffect,
    DTaskFault,
    DTaskKind,
    DTaskPhase,
    DTaskSelection,
    DTaskState,
    DTaskTransition,
)
from ed_uav_mission.d_task_reducer import DTaskRuntime
from ed_uav_mission.mission_model import CompetitionParams, StabilityParams
from ed_uav_mission.payload_config import PayloadBoundaryConfig


class DTaskMissionAborted(RuntimeError):
    """The reducer completed its explicit safe recovery chain."""


class DTaskEffectError(RuntimeError):
    """An executor-owned D-task effect was rejected at its typed boundary."""


@dataclass(frozen=True, slots=True)
class CompetitionCallbacks:
    execute_takeoff: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    send_hover: Callable[[float], Awaitable[None]]
    move_right: Callable[[ExecuteMission.Feedback, float], Awaitable[None]]
    track_target: Callable[[TargetSnapshot, VehicleSnapshot, float], Awaitable[None]]
    release_payload: Callable[[TargetSnapshot, VehicleSnapshot], Awaitable[None]]
    descend_to_vehicle: Callable[[TargetSnapshot, VehicleSnapshot], Awaitable[None]]
    return_home: Callable[[], Awaitable[None]]
    land_home: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    capture_home: Callable[[], None]
    next_event: Callable[[], Awaitable[DTaskEvent]]
    publish_transition: Callable[[DTaskTransition, ExecuteMission.Feedback], None]
    now_s: Callable[[], float]


from ed_uav_mission.stability_runner import StabilityCallbacks, StabilityRunner


class CompetitionRuntime:
    """Drive both D-task branches through one reducer and callback surface."""

    def __init__(
        self,
        callbacks: CompetitionCallbacks,
        payload_config: PayloadBoundaryConfig,
        stability_callbacks: StabilityCallbacks | None = None,
    ) -> None:
        self._callbacks = callbacks
        self._payload_config = payload_config
        self._stability_callbacks = stability_callbacks

    def cancel_active(self) -> None:
        """Planner and FlightCommand cancellation remain executor-owned."""

    async def run(
        self,
        params: CompetitionParams | None,
        selection: DTaskSelection,
        feedback: ExecuteMission.Feedback,
        stability_params: StabilityParams | None = None,
    ) -> None:
        if stability_params is not None:
            if selection.task is not DTaskKind.STABILITY_TEST:
                raise RuntimeError("stability execution requires a committed Task3 selection")
            if params is not None and params.mission_variant != "stability":
                raise RuntimeError("stability_params require mission_variant=stability")
            if self._stability_callbacks is None:
                raise RuntimeError("stability callbacks are not configured")
            runner = StabilityRunner(self._stability_callbacks, stability_params)
            await runner.run(selection, feedback)
            return
        if params is None:
            raise RuntimeError("competition params not loaded")
        config = DTaskRuntimeConfig(
            stable_s=params.stable_sec,
            start_deadline_s=params.start_deadline_s,
            b_deadline_s=params.b_deadline_s,
            d_deadline_s=params.d_deadline_s,
            mission_deadline_s=90.0,
            vehicle_freshness_s=params.vehicle_freshness_s,
            target_freshness_s=params.target_freshness_s,
            maximum_relative_error_m=params.maximum_relative_error_m,
            right_offset_m=params.right_offset_m,
        )
        runtime = DTaskRuntime(selection, config, self._payload_config)
        latest_target: TargetSnapshot | None = None
        latest_vehicle: VehicleSnapshot | None = None
        self._callbacks.publish_transition(
            DTaskTransition(state=runtime.state),
            feedback,
        )
        while runtime.state.phase not in (DTaskPhase.SUCCEEDED, DTaskPhase.ABORTED):
            event = await self._callbacks.next_event()
            match event:
                case TargetObserved(target=target):
                    latest_target = target
                case VehicleObserved(vehicle=vehicle):
                    latest_vehicle = vehicle
                case _:
                    pass
            transition = runtime.advance(event)
            self._callbacks.publish_transition(transition, feedback)
            transition = await self._execute_effect(
                runtime,
                transition,
                latest_target,
                latest_vehicle,
                feedback,
            )
        if runtime.state.phase is DTaskPhase.ABORTED:
            raise DTaskMissionAborted(runtime.state.reason or "D-task aborted")

    async def _execute_effect(
        self,
        runtime: DTaskRuntime,
        transition: DTaskTransition,
        target: TargetSnapshot | None,
        vehicle: VehicleSnapshot | None,
        feedback: ExecuteMission.Feedback,
    ) -> DTaskTransition:
        effect = transition.effect
        while effect is not None:
            try:
                await self._call_effect(effect, target, vehicle, feedback, runtime.config)
                if transition.state.phase is DTaskPhase.STABILIZING:
                    event = Tick(now_s=self._callbacks.now_s())
                else:
                    event = CommandCompleted(now_s=self._callbacks.now_s(), effect=effect)
            except MissionCancelled as error:
                event = SafetyInterrupted(
                    now_s=self._callbacks.now_s(),
                    fault=DTaskFault.CANCELLED,
                    reason=str(error),
                )
            except (MissionTimeout, RuntimeError) as error:
                event = CommandFailed(
                    now_s=self._callbacks.now_s(),
                    effect=effect,
                    reason=str(error),
                )
            transition = runtime.advance(event)
            self._callbacks.publish_transition(transition, feedback)
            effect = transition.effect
        return transition

    async def _call_effect(
        self,
        effect: DTaskEffect,
        target: TargetSnapshot | None,
        vehicle: VehicleSnapshot | None,
        feedback: ExecuteMission.Feedback,
        config: DTaskRuntimeConfig,
    ) -> None:
        match effect:
            case DTaskEffect.TAKEOFF:
                await self._callbacks.execute_takeoff(feedback)
            case DTaskEffect.HOVER:
                await self._callbacks.send_hover(config.stable_s)
            case DTaskEffect.MOVE_RIGHT:
                await self._callbacks.move_right(feedback, config.right_offset_m)
                self._callbacks.capture_home()
            case DTaskEffect.TRACK_TARGET:
                target_value, vehicle_value = self._required_tracking(target, vehicle)
                await self._callbacks.track_target(target_value, vehicle_value, 1.5)
            case DTaskEffect.RELEASE_PAYLOAD:
                target_value, vehicle_value = self._required_tracking(target, vehicle)
                await self._callbacks.release_payload(target_value, vehicle_value)
            case DTaskEffect.DESCEND_TO_VEHICLE:
                target_value, vehicle_value = self._required_tracking(target, vehicle)
                await self._callbacks.descend_to_vehicle(target_value, vehicle_value)
            case DTaskEffect.RETURN_HOME:
                await self._callbacks.return_home()
            case DTaskEffect.LAND_HOME:
                await self._callbacks.land_home(feedback)
            case DTaskEffect.STABILITY_HOVER:
                # 稳定性任务中，HOVER 由 StabilityRunner 直接调用真实回调完成
                pass
            case DTaskEffect.STABILITY_WAYPOINT:
                # 稳定性任务中，航点移动由 StabilityRunner 直接调度
                pass
            case unreachable:
                assert_never(unreachable)

    @staticmethod
    def _required_tracking(
        target: TargetSnapshot | None,
        vehicle: VehicleSnapshot | None,
    ) -> tuple[TargetSnapshot, VehicleSnapshot]:
        if target is None or vehicle is None:
            raise RuntimeError("fresh target and vehicle state are required")
        return target, vehicle
