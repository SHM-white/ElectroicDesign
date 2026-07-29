"""Typed ROS inputs, selection, and MissionStatus output for D-task execution."""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Callable

from ed_uav_interfaces.msg import (
    MissionStatus,
    PayloadContactState,
    TargetObservation,
    VehicleTelemetry,
)
from ed_uav_interfaces.srv import SelectDTaskMission
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from ed_uav_mission.action_lifecycle import steady_now_sec
from ed_uav_mission.d_task_events import (
    ContactObserved,
    DTaskEvent,
    SafetyInterrupted,
    TargetObserved,
    TargetSnapshot,
    Tick,
    VehicleObserved,
    VehicleSnapshot,
)
from ed_uav_mission.d_task_inputs import (
    adapt_contact_message,
    adapt_target_observation,
    adapt_vehicle_telemetry,
)
from ed_uav_mission.d_task_model import (
    DTaskFault,
    DTaskKind,
    DTaskPhase,
    DTaskSelection,
    PayloadState,
    RouteStage,
    SelectionAccepted,
    SelectionStore,
)
from ed_uav_mission.d_task_status import mission_status_state
from ed_uav_mission.mission_model import CompetitionParams
from ed_uav_mission.touchdown import (
    ContactObservation,
    TouchdownUpdate,
)


class DTaskRosBoundary:
    """Own external D-task inputs while the executor retains motion authority."""

    def __init__(
        self,
        node: Node,
        mission_id: str,
        params: CompetitionParams,
        is_pre_arm: Callable[[], bool],
        localization_valid: Callable[[], bool],
    ) -> None:
        self._node = node
        self._mission_id = mission_id
        self._params = params
        self._is_pre_arm = is_pre_arm
        self._localization_valid = localization_valid
        self._selection_store = SelectionStore()
        self._events: deque[DTaskEvent] = deque(maxlen=256)
        self._waiter: Future | None = None
        self._latest_vehicle: VehicleSnapshot | None = None
        self._latest_target: TargetSnapshot | None = None
        self._latest_contact: ContactObservation | None = None
        self._payload_state = PayloadState.UNKNOWN
        self._status_sequence = 0
        group = ReentrantCallbackGroup()
        self._vehicle_sub = node.create_subscription(
            VehicleTelemetry,
            "/vehicle/telemetry",
            self._on_vehicle,
            20,
            callback_group=group,
        )
        self._target_sub = node.create_subscription(
            TargetObservation,
            "/target/observation",
            self._on_target,
            20,
            callback_group=group,
        )
        self._contact_sub = node.create_subscription(
            PayloadContactState,
            "/payload/contact_state",
            self._on_contact,
            20,
            callback_group=group,
        )
        self._selection_service = node.create_service(
            SelectDTaskMission,
            "/mission/select_d_task",
            self._on_selection,
            callback_group=group,
        )
        self._status_publisher = node.create_publisher(
            MissionStatus,
            "/mission/status",
            20,
        )
        self._tick_timer = node.create_timer(0.1, self._on_tick, callback_group=group)

    @property
    def selection(self) -> DTaskSelection | None:
        return self._selection_store.selection

    @property
    def latest_vehicle(self) -> VehicleSnapshot | None:
        return self._latest_vehicle

    @property
    def latest_target(self) -> TargetSnapshot | None:
        return self._latest_target

    def clear_after_terminal(self) -> None:
        self._selection_store.clear_after_terminal()
        self._events.clear()

    def interrupt(self, fault: DTaskFault, reason: str) -> None:
        self._push(
            SafetyInterrupted(
                now_s=steady_now_sec(),
                fault=fault,
                reason=reason,
            )
        )

    async def next_event(self) -> DTaskEvent:
        if self._events:
            return self._events.popleft()
        waiter = Future()
        self._waiter = waiter
        try:
            event = await waiter
        finally:
            self._waiter = None
        match event:
            case (
                Tick()
                | VehicleObserved()
                | TargetObserved()
                | ContactObserved()
                | SafetyInterrupted()
            ):
                return event
            case _:
                raise RuntimeError("D-task event future returned an invalid value")

    def publish_status(self, phase: DTaskPhase, route_stage: RouteStage, reason: str) -> None:
        message = MissionStatus()
        message.contract_version = MissionStatus.CONTRACT_VERSION
        message.acquisition_stamp = self._node.get_clock().now().to_msg()
        message.source_sequence = self._status_sequence
        self._status_sequence = (self._status_sequence + 1) % (1 << 32)
        message.mission_id = self._mission_id
        message.state = mission_status_state(phase)
        message.route_stage = int(route_stage)
        message.complete = phase in (DTaskPhase.SUCCEEDED, DTaskPhase.ABORTED)
        message.reason = reason[:96]
        self._status_publisher.publish(message)

    def _push(self, event: DTaskEvent) -> None:
        waiter = self._waiter
        if waiter is not None and not waiter.done():
            waiter.set_result(event)
            return
        if isinstance(event, Tick) and self._events:
            return
        self._events.append(event)

    def _on_tick(self) -> None:
        self._push(Tick(now_s=steady_now_sec()))

    def _on_vehicle(self, message: VehicleTelemetry) -> None:
        now_s = steady_now_sec()
        snapshot = adapt_vehicle_telemetry(message, now_s)
        self._latest_vehicle = snapshot
        self._push(
            VehicleObserved(
                now_s=now_s,
                vehicle=snapshot,
                payload_state=self._payload_state,
            )
        )

    def _on_target(self, message: TargetObservation) -> None:
        now_s = steady_now_sec()
        snapshot = adapt_target_observation(
            message,
            now_s,
            self._params.target_revision,
        )
        self._latest_target = snapshot
        self._push(TargetObserved(now_s=now_s, target=snapshot))

    def _on_contact(self, message: PayloadContactState) -> None:
        now_s = steady_now_sec()
        self._payload_state = PayloadState(message.payload_state)
        contact = adapt_contact_message(message, now_s)
        self._latest_contact = contact
        target_at = self._latest_target.observed_at_s if self._latest_target else -math.inf
        vehicle_at = self._latest_vehicle.observed_at_s if self._latest_vehicle else -math.inf
        speed = self._latest_vehicle.speed_m_s if self._latest_vehicle else 0.0
        self._push(
            ContactObserved(
                TouchdownUpdate(
                    now_monotonic_s=now_s,
                    target_observed_at_s=target_at,
                    vehicle_observed_at_s=vehicle_at,
                    vehicle_speed_m_s=speed,
                    contact=contact,
                    cancelled=False,
                    localization_valid=self._localization_valid(),
                )
            )
        )

    def _on_selection(
        self,
        request: SelectDTaskMission.Request,
        response: SelectDTaskMission.Response,
    ) -> SelectDTaskMission.Response:
        response.contract_version = SelectDTaskMission.Request.CONTRACT_VERSION
        reason = self._selection_rejection_reason(request)
        if reason:
            response.accepted = False
            response.reason = reason[:96]
            return response
        selection = DTaskSelection(
            mission_id=str(request.mission_id),
            mission_profile_id=str(request.mission_profile_id),
            deployment_preset_id=str(request.deployment_preset_id),
            target_revision=str(request.target_revision),
            task=DTaskKind(int(request.task)),
            committed_at_s=steady_now_sec(),
        )
        result = self._selection_store.commit(selection, pre_arm=self._is_pre_arm())
        response.accepted = result.accepted
        response.reason = "selection committed" if isinstance(result, SelectionAccepted) else result.reason
        return response

    def _selection_rejection_reason(self, request: SelectDTaskMission.Request) -> str:
        if request.contract_version != SelectDTaskMission.Request.CONTRACT_VERSION:
            return "unsupported selection contract"
        if request.mission_id != self._mission_id:
            return "selection mission_id does not match loaded mission"
        if request.mission_profile_id != self._params.mission_profile_id:
            return "selection mission profile does not match loaded profile"
        if request.deployment_preset_id != self._params.deployment_preset_id:
            return "selection deployment preset does not match loaded preset"
        if request.target_revision != self._params.target_revision:
            return "selection target revision does not match loaded revision"
        if request.task not in (int(DTaskKind.PAYLOAD_DROP), int(DTaskKind.DYNAMIC_LANDING)):
            return "selection task is unsupported"
        return ""
