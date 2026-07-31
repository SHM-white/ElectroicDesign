"""ExecuteMission action server — preflight validation and plugin orchestration.

All motion goes through the ``FlightCommand`` action (``ed_uav_interfaces``).
This module never imports serial, GPIO, or camera APIs.
"""

from __future__ import annotations

import asyncio
import math
import os
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ed_uav_interfaces.action import ExecuteMission, FlightCommand
from ed_uav_interfaces.msg import FcuState, LocalizationStatus
from ed_uav_localization.field_profile.loader import load_profile_text
from ed_uav_localization.field_profile.model import KnownFieldProfile
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ed_uav_mission.action_lifecycle import (
    MissionCancelled,
    MissionDeadline,
    MissionTimeout,
    steady_now_sec,
    wait_with_deadline,
)
from ed_uav_mission.competition_runtime import (
    CompetitionCallbacks,
    CompetitionRuntime,
    DTaskEffectError,
)
from ed_uav_mission.stability_runtime import StabilityCallbacks
from ed_uav_mission.d_task_capability import evaluate_d_task_capability
from ed_uav_mission.d_task_events import TargetSnapshot, VehicleSnapshot
from ed_uav_mission.d_task_model import (
    DTaskFault,
    DTaskKind,
    DTaskPhase,
    DTaskTransition,
    RouteStage,
)
from ed_uav_mission.d_task_ros import DTaskRosBoundary
from ed_uav_mission.d_task_selection import (
    DTaskSelectionContract,
    is_committed_task3_selection,
)
from ed_uav_mission.mission_config import (
    calibration_file_is_valid,
    load_mission_bundle,
    parse_mission_config_text,
)
from ed_uav_mission.mission_model import (
    MissionConfig,
    MissionType,
    Waypoint,
)
from ed_uav_mission.payload_config import load_payload_boundary_config
from ed_uav_mission.plugins.coverage import GridCoveragePlugin
from ed_uav_mission.plugins.patrol import WaypointPatrolPlugin
from ed_uav_mission.plugins.payload import (
    ActuatorAcknowledged,
    FakePayloadActuator,
    PayloadPlugin,
    ReleaseContext,
    ReleaseLatch,
    ReleasePhase,
    ReleaseRejected,
)
from ed_uav_mission.plugins.target_visit import TargetVisitPlugin
from ed_uav_mission.plugins.terminal_landing import LandingStep, TerminalLandingPlugin
from ed_uav_mission.state_machine import MissionFSM, MissionState


class PreflightCode(Enum):
    OK = auto()
    STALE_AUX = auto()
    NO_FCU_LINK = auto()
    LOCALIZATION_LOST = auto()
    PROFILE_INVALID = auto()
    CALIBRATION_MISSING = auto()
    FCU_SOURCE_MISMATCH = auto()
    CAPABILITY_BLOCKED = auto()


@dataclass(frozen=True, slots=True)
class PreflightResult:
    code: PreflightCode
    reason: str = ""


def bounded_failure_reason(error: Exception) -> str:
    """Format an exception for the fixed-width ExecuteMission result field."""
    message = str(error)
    first_line = message.splitlines()[0] if message else type(error).__name__
    return first_line[:96]


def validate_preflight(
    *,
    fcu_communication_ok: bool,
    fcu_source: int,
    fcu_motors_armed: bool,
    simulation_only: bool,
    aux_start_active: bool,
    localization_active: bool,
    map_to_odom_valid: bool,
    profile_loaded: bool,
    calibration_valid: bool,
    capability_ready: bool = True,
) -> PreflightResult:
    """Pure-function preflight gate — testable without ROS infrastructure."""
    if not aux_start_active:
        return PreflightResult(PreflightCode.STALE_AUX, "AUX start switch is stale or off")
    if not fcu_communication_ok:
        return PreflightResult(PreflightCode.NO_FCU_LINK, "no FCU communication")
    expected_source = FcuState.SOURCE_SIMULATOR if simulation_only else FcuState.SOURCE_V7
    if fcu_source != expected_source:
        return PreflightResult(
            PreflightCode.FCU_SOURCE_MISMATCH,
            "FCU source does not match mission execution mode",
        )
    if not localization_active:
        return PreflightResult(PreflightCode.LOCALIZATION_LOST, "localization not active")
    if not map_to_odom_valid:
        return PreflightResult(PreflightCode.LOCALIZATION_LOST, "map-to-odom transform invalid")
    if not profile_loaded:
        return PreflightResult(PreflightCode.PROFILE_INVALID, "field profile not loaded or invalid")
    if not calibration_valid:
        return PreflightResult(PreflightCode.CALIBRATION_MISSING, "sensor calibration missing")
    if not capability_ready:
        return PreflightResult(
            PreflightCode.CAPABILITY_BLOCKED,
            "verified programmable capability is unavailable",
        )
    if not fcu_motors_armed:
        return PreflightResult(PreflightCode.CALIBRATION_MISSING, "motors not armed")
    return PreflightResult(PreflightCode.OK)


class MissionExecutorNode(Node):
    """ROS 2 node that serves the ``ExecuteMission`` action."""

    def __init__(self) -> None:
        super().__init__("mission_executor")
        self.declare_parameter("profile_path", "")
        self.declare_parameter("mission_config_path", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("simulation_only", False)
        self.declare_parameter("payload_config_path", "")
        self.declare_parameter("programmable_capability_report", "")
        self.declare_parameter("fcu_device_identity", "")
        self.declare_parameter("task3_mission_profile_id", "")
        self.declare_parameter("task3_deployment_preset_id", "")
        self.declare_parameter("task3_target_revision", "")
        self._fsm = MissionFSM()
        self._goal_handle: ServerGoalHandle | None = None
        self._cancel_requested = False
        self._active_flight_goal = None
        self._mission_deadline: MissionDeadline | None = None
        self._airborne = False
        self._aux_start_active = False
        self._hard_lock_active = False
        self._latest_fcu: FcuState | None = None
        self._latest_localization: LocalizationStatus | None = None
        self._latest_localization_at_s = 0.0
        self._profile: KnownFieldProfile | None = None
        self._mission_config: MissionConfig | None = None

        profile_path = str(self.get_parameter("profile_path").value)
        mission_path = str(self.get_parameter("mission_config_path").value)
        calibration_path = Path(str(self.get_parameter("calibration_file").value))
        simulation_only = bool(self.get_parameter("simulation_only").value)
        self._simulation_only = simulation_only
        if not profile_path or not mission_path:
            raise ValueError("profile_path and mission_config_path parameters are required")
        bundle = load_mission_bundle(
            Path(profile_path),
            Path(mission_path),
            allow_blocked_profile=simulation_only,
        )
        self._profile = bundle.profile
        self._mission_config = bundle.mission
        self._calibration_valid = calibration_file_is_valid(
            calibration_path,
            simulation_only=simulation_only,
        )
        capability = evaluate_d_task_capability(
            simulation_only=simulation_only,
            report_path=Path(
                str(self.get_parameter("programmable_capability_report").value)
            ),
            device_identity=str(self.get_parameter("fcu_device_identity").value),
            environment=os.environ,
        )
        self._capability_ready = capability.ready
        self._capability_reason = capability.reason
        payload_path_value = str(self.get_parameter("payload_config_path").value)
        payload_path = (
            Path(payload_path_value)
            if payload_path_value
            else Path(mission_path).parent.parent / "payload_adapter.yaml"
        )
        self._payload_config = load_payload_boundary_config(payload_path)
        self._payload_plugin = PayloadPlugin(ReleaseLatch())
        self._payload_actuator = FakePayloadActuator(
            outcomes=(ActuatorAcknowledged(acknowledgement_id="simulation-payload-ack"),)
        )

        cb_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self, ExecuteMission, "/mission/execute",
            execute_callback=self._execution_loop,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb_group,
        )
        self._flight_client = ActionClient(
            self, FlightCommand, "/fcu/flight_command"
        )
        self._competition_planner = None
        self._competition_runtime = None
        self._d_task_boundary = None
        self._visual_servo_controller = None
        if self._mission_config.mission_type in (MissionType.COMPETITION, MissionType.STABILITY_TEST):
            from ed_uav_mission.competition_planner import CompetitionPlanner

            self._competition_planner = CompetitionPlanner(
                self,
                self._send_move,
                self._deadline_for_timeout,
                self._raise_if_cancelled,
            )
            # Initialize visual servo controller for precision landing
            try:
                from ed_uav_perception.visual_servo import VisualServoConfig, VisualServoController
                self._visual_servo_controller = VisualServoController()
                self.get_logger().info("Visual servo controller initialized for precision landing")
            except ImportError:
                self.get_logger().warn("Visual servo module not available, precision landing disabled")
            
            if self._mission_config.mission_type == MissionType.COMPETITION:
                assert self._mission_config.competition is not None
                self._d_task_boundary = DTaskRosBoundary(
                    self,
                    self._mission_config.mission_id,
                    self._mission_config.competition,
                    lambda: self._fsm.state == MissionState.IDLE,
                    self._localization_is_valid,
                    field_profile_id=self._profile.profile_id,
                )
            else:
                self._d_task_boundary = DTaskRosBoundary(
                    self,
                    self._mission_config.mission_id,
                    None,
                    lambda: self._fsm.state == MissionState.IDLE,
                    self._localization_is_valid,
                    selection_contract=DTaskSelectionContract(
                        mission_id=self._mission_config.mission_id,
                        field_profile_id=self._profile.profile_id,
                        mission_profile_id=str(
                            self.get_parameter("task3_mission_profile_id").value
                        ),
                        deployment_preset_id=str(
                            self.get_parameter("task3_deployment_preset_id").value
                        ),
                        target_revision=str(
                            self.get_parameter("task3_target_revision").value
                        ),
                        allowed_tasks=frozenset((DTaskKind.STABILITY_TEST,)),
                    ),
                )
        if self._competition_planner is not None:
            self._competition_runtime = CompetitionRuntime(
                CompetitionCallbacks(
                    execute_takeoff=self._execute_takeoff,
                    send_hover=self._send_hover,
                    track_target=self._track_d_task_target,
                    release_payload=self._release_d_task_payload,
                    descend_to_vehicle=self._descend_to_vehicle,
                    return_home=self._return_d_task_home,
                    land_home=self._land_d_task_home,
                    capture_home=self._competition_planner.capture_home,
                    next_event=self._next_d_task_event,
                    publish_transition=self._publish_d_task_transition,
                    now_s=steady_now_sec,
                ),
                self._payload_config,
                stability_callbacks=StabilityCallbacks(
                    execute_takeoff=self._execute_takeoff,
                    send_hover=self._send_hover,
                    capture_home=self._competition_planner.capture_home,
                    send_move=self._send_stability_move,
                    land_home=self._land_d_task_home,
                    next_event=self._next_d_task_event,
                    publish_transition=self._publish_d_task_transition,
                    now_s=steady_now_sec,
                    capture_pose=self._capture_stability_pose,
                ),
            )

        self._fcu_sub = self.create_subscription(FcuState, "/fcu/state", self._on_fcu_state, 10)
        self._loc_sub = self.create_subscription(
            LocalizationStatus, "/localization/status", self._on_localization_status, 10
        )
        self.get_logger().info("Mission executor ready")

    def load_profile(self, yaml_text: str) -> None:
        profile = load_profile_text(yaml_text, "<inline>")
        if not isinstance(profile, KnownFieldProfile):
            raise TypeError("mission requires a known field profile")
        self._profile = profile

    def load_mission_config(self, yaml_text: str) -> None:
        self._mission_config = parse_mission_config_text(yaml_text)

    def _on_fcu_state(self, msg: FcuState) -> None:
        self._latest_fcu = msg
        hard_lock_started = msg.emergency_lock_active and not self._hard_lock_active
        self._aux_start_active = msg.aux1_valid and msg.task3_control_allowed
        self._hard_lock_active = msg.emergency_lock_active
        if hard_lock_started:
            self._cancel_active_flight()
            if self._fsm.is_active and self._d_task_boundary is not None:
                self._d_task_boundary.publish_status(
                    DTaskPhase.ABORTED,
                    RouteStage.START,
                    "physical hard lock",
                )
                self._d_task_boundary.interrupt(
                    DTaskFault.HARD_LOCKED,
                    "physical hard lock",
                )

    def _on_localization_status(self, msg: LocalizationStatus) -> None:
        self._latest_localization = msg
        self._latest_localization_at_s = steady_now_sec()
        if (
            self._fsm.is_active
            and not self._localization_is_valid()
            and self._d_task_boundary is not None
        ):
            self._d_task_boundary.interrupt(
                DTaskFault.LOCALIZATION_LOST,
                "localization lost during D-task mission",
            )

    def _on_goal(self, goal_request: ExecuteMission.Goal) -> GoalResponse:
        if self._fsm.is_active:
            return GoalResponse.REJECT
        if self._mission_config is None or self._profile is None:
            return GoalResponse.REJECT
        if (
            self._mission_config.mission_type == MissionType.COMPETITION
            and (
                self._d_task_boundary is None
                or self._d_task_boundary.selection is None
            )
        ):
            return GoalResponse.REJECT
        if (
            self._mission_config.mission_type == MissionType.STABILITY_TEST
            and not is_committed_task3_selection(
                self._d_task_boundary.selection
                if self._d_task_boundary is not None
                else None
            )
        ):
            return GoalResponse.REJECT
        if (
            goal_request.mission_id != self._mission_config.mission_id
            or goal_request.field_profile_id != self._profile.profile_id
        ):
            return GoalResponse.REJECT
        if self._fsm.is_terminal:
            self._fsm.transition(MissionState.IDLE, "ready for next goal")
        self.get_logger().info(f"Received mission goal: {goal_request.mission_id}")
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Cancel requested")
        self._cancel_requested = True
        self._cancel_active_flight()
        if self._competition_planner is not None:
            self._competition_planner.cancel_active()
        if self._d_task_boundary is not None:
            self._d_task_boundary.interrupt(DTaskFault.CANCELLED, "cancelled by user")
        return CancelResponse.ACCEPT

    async def _execution_loop(self, goal_handle: ServerGoalHandle) -> ExecuteMission.Result:
        self._goal_handle = goal_handle
        self._cancel_requested = False
        config = self._mission_config
        self._mission_deadline = MissionDeadline.from_limits(
            now_sec=steady_now_sec(),
            limits=(
                goal_handle.request.timeout_sec,
                config.timeout_sec if config is not None else 0.0,
            ),
        )
        feedback = ExecuteMission.Feedback()
        result = ExecuteMission.Result()

        try:
            self._fsm.transition(MissionState.ARMED, "goal accepted")
            preflight = self._run_preflight()
            if preflight.code != PreflightCode.OK:
                self._fsm.transition(MissionState.ABORTED, preflight.reason)
                result.result_code = ExecuteMission.Result.RESULT_REJECTED
                result.reason = preflight.reason
                goal_handle.abort()
                return result

            self._raise_if_cancelled()
            if config is None:
                raise RuntimeError("mission config not loaded")
            if config.mission_type in (MissionType.COMPETITION, MissionType.STABILITY_TEST):
                if self._d_task_boundary is None or self._d_task_boundary.selection is None:
                    raise RuntimeError("committed D-task selection unavailable")
                if self._competition_runtime is None:
                    raise RuntimeError("D-task action runtime unavailable")
                await self._competition_runtime.run(
                    config.competition,
                    self._d_task_boundary.selection,
                    feedback,
                    stability_params=config.stability_params,
                )
            else:
                self._fsm.transition(MissionState.TAKEOFF, "preflight passed")
                await self._execute_takeoff(feedback)

                self._fsm.transition(MissionState.EXECUTING, "takeoff complete")
                waypoints = self._dispatch_plugin()
                for index, wp in enumerate(waypoints):
                    self._raise_if_cancelled()
                    await self._send_move(wp)
                    if wp.hover_sec > 0:
                        await self._send_hover(wp.hover_sec)
                    feedback.state_id = f"waypoint_{index}"
                    feedback.progress = float(index + 1) / float(max(len(waypoints), 1))
                    goal_handle.publish_feedback(feedback)

                self._fsm.transition(MissionState.RETURNING, "mission done")
                self._fsm.transition(MissionState.LANDING, "landing sequence")
                await self._execute_landing(feedback)
            self._raise_if_cancelled()
            self._fsm.transition(MissionState.COMPLETE, "landed")
            result.result_code = ExecuteMission.Result.RESULT_SUCCEEDED
            result.reason = "mission complete"
            goal_handle.succeed()
            return result
        except MissionCancelled as exc:
            await self._recover_after_airborne_failure()
            reason = bounded_failure_reason(exc)
            self._fsm.transition(MissionState.ABORTED, reason)
            result.result_code = ExecuteMission.Result.RESULT_ABORTED
            result.reason = reason
            goal_handle.canceled()
            return result
        except MissionTimeout as exc:
            await self._recover_after_airborne_failure()
            reason = bounded_failure_reason(exc)
            self._fsm.transition(MissionState.ABORTED, reason)
            result.result_code = ExecuteMission.Result.RESULT_TIMEOUT
            result.reason = reason
            goal_handle.abort()
            return result
        except Exception as exc:  # noqa: BLE001 - Action boundary converts faults to results.
            self.get_logger().error(f"Mission error: {exc}")
            await self._recover_after_airborne_failure()
            reason = bounded_failure_reason(exc)
            self._fsm.transition(MissionState.ABORTED, reason)
            result.result_code = ExecuteMission.Result.RESULT_ABORTED
            result.reason = reason
            goal_handle.abort()
            return result
        finally:
            if self._d_task_boundary is not None:
                self._d_task_boundary.clear_after_terminal()

    def _run_preflight(self) -> PreflightResult:
        fcu, loc = self._latest_fcu, self._latest_localization
        return validate_preflight(
            fcu_communication_ok=fcu is not None and fcu.communication_ok,
            fcu_source=fcu.source if fcu is not None else 0,
            fcu_motors_armed=fcu is not None and fcu.motors_armed,
            simulation_only=self._simulation_only,
            aux_start_active=self._aux_start_active,
            localization_active=loc is not None and loc.state == LocalizationStatus.STATE_ACTIVE,
            map_to_odom_valid=loc is not None and loc.map_to_odom_valid,
            profile_loaded=self._profile is not None,
            calibration_valid=self._calibration_valid,
            capability_ready=(
                self._capability_ready
                if self._mission_config is not None
                and self._mission_config.mission_type == MissionType.COMPETITION
                else True
            ),
        )

    def _localization_is_valid(self) -> bool:
        loc = self._latest_localization
        return (
            loc is not None
            and loc.state == LocalizationStatus.STATE_ACTIVE
            and loc.map_to_odom_valid
        )

    async def _next_d_task_event(self):
        if self._d_task_boundary is None:
            raise RuntimeError("D-task ROS boundary unavailable")
        return await self._d_task_boundary.next_event()

    def _publish_d_task_transition(
        self,
        transition: DTaskTransition,
        feedback: ExecuteMission.Feedback,
    ) -> None:
        phase = transition.state.phase
        self._transition_generic_state(phase)
        feedback.state_id = phase.value
        feedback.progress = 1.0 if transition.complete else 0.0
        if self._goal_handle is not None:
            self._goal_handle.publish_feedback(feedback)
        if self._d_task_boundary is not None:
            vehicle = self._d_task_boundary.latest_vehicle
            route = vehicle.route_stage if vehicle is not None else RouteStage.START
            self._d_task_boundary.publish_status(phase, route, transition.state.reason)

    def _transition_generic_state(self, phase: DTaskPhase) -> None:
        if phase is DTaskPhase.TAKEOFF and self._fsm.state == MissionState.ARMED:
            self._fsm.transition(MissionState.TAKEOFF, "D-task start observed")
        elif phase is DTaskPhase.STABILIZING and self._fsm.state == MissionState.TAKEOFF:
            self._fsm.transition(MissionState.EXECUTING, "takeoff complete")
        elif phase in (DTaskPhase.RETURNING_HOME, DTaskPhase.SAFE_RETURN):
            if self._fsm.state == MissionState.EXECUTING:
                self._fsm.transition(MissionState.RETURNING, phase.value)
        elif (
            phase in (DTaskPhase.LANDING_HOME, DTaskPhase.SAFE_LAND)
            and self._fsm.state == MissionState.RETURNING
        ):
            self._fsm.transition(MissionState.LANDING, phase.value)

    async def _track_d_task_target(
        self,
        target: TargetSnapshot,
        vehicle: VehicleSnapshot,
        altitude_m: float,
    ) -> None:
        if self._competition_planner is None:
            raise RuntimeError("competition planner unavailable")
        await self._competition_planner.track_target(target, vehicle, altitude_m)

    async def _release_d_task_payload(
        self,
        target: TargetSnapshot,
        vehicle: VehicleSnapshot,
    ) -> None:
        if self._mission_config is None:
            raise RuntimeError("mission config unavailable")
        now_s = steady_now_sec()
        context = ReleaseContext(
            request_id=f"{self._mission_config.mission_id}-release",
            now_monotonic_s=now_s,
            phase=ReleasePhase.TASK1_RELEASE,
            target_observed_at_s=target.observed_at_s,
            vehicle_observed_at_s=vehicle.observed_at_s,
            localization_observed_at_s=self._latest_localization_at_s,
            calibration_valid=self._calibration_valid,
            standoff_m=math.sqrt(
                target.relative_x_m**2
                + target.relative_y_m**2
                + target.relative_z_m**2
            ),
            cancelled=self._cancel_requested,
        )
        release = self._payload_plugin.release(
            context,
            self._payload_actuator,
            self._payload_config,
        )
        if isinstance(release, ReleaseRejected):
            raise DTaskEffectError(
                f"payload release rejected: {release.reason.value}"
            )

    async def _descend_to_vehicle(
        self,
        target: TargetSnapshot,
        vehicle: VehicleSnapshot,
    ) -> None:
        if self._competition_planner is None:
            raise RuntimeError("competition planner unavailable")
        
        # Use visual servo for precision landing if available
        if hasattr(self, '_visual_servo_controller') and self._visual_servo_controller is not None:
            self.get_logger().info("Using visual servo for precision landing")
            max_attempts = 50  # 5 seconds at 10Hz
            for attempt in range(max_attempts):
                landed = await self._competition_planner.precision_land_on_target(
                    target, vehicle, self._visual_servo_controller
                )
                if landed:
                    self.get_logger().info(f"Visual servo converged after {attempt + 1} iterations")
                    break
                # Small delay between iterations
                await asyncio.sleep(0.1)
            else:
                self.get_logger().warn("Visual servo did not converge, proceeding with land")
        else:
            # Fallback to simple track and land
            self.get_logger().info("Using simple track and land (no visual servo)")
            await self._competition_planner.descend_to_vehicle(target, vehicle)
        
        await self._send_land()

    async def _return_d_task_home(self) -> None:
        config = self._mission_config
        if config is None or config.competition is None:
            raise RuntimeError("competition params unavailable")
        if self._competition_planner is None:
            raise RuntimeError("competition planner unavailable")
        await self._competition_planner.return_home(config.competition)

    async def _land_d_task_home(self, feedback: ExecuteMission.Feedback) -> None:
        if self._competition_planner is None:
            raise RuntimeError("competition planner unavailable")
        home = self._competition_planner.home
        await self._execute_landing(feedback, home.x_m, home.y_m, True)

    def _dispatch_plugin(self) -> list[Waypoint]:
        if self._mission_config is None or self._profile is None:
            raise RuntimeError("mission config or field profile not loaded")
        config = self._mission_config
        match config.mission_type:
            case MissionType.COVERAGE:
                assert config.coverage is not None
                return GridCoveragePlugin().generate(self._profile, config.coverage)
            case MissionType.PATROL:
                assert config.patrol is not None
                return WaypointPatrolPlugin().generate(self._profile, config.patrol)
            case MissionType.TARGET_VISIT:
                assert config.target_visit is not None
                return TargetVisitPlugin().generate(config.target_visit)
            case MissionType.PAYLOAD:
                return []
            case MissionType.COMPETITION:
                return []
            case MissionType.STABILITY_TEST:
                return []
            case _:
                raise ValueError(f"unknown mission type: {config.mission_type}")

    async def _execute_takeoff(self, feedback: ExecuteMission.Feedback) -> None:
        config = self._mission_config
        if config is None:
            raise RuntimeError("mission config not loaded")
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_TAKEOFF
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.z = config.takeoff_altitude_m
        goal.timeout_sec = 30.0
        goal.correlation_id = "mission_takeoff"
        feedback.state_id = "takeoff"
        self._goal_handle.publish_feedback(feedback)
        await self._send_and_wait(goal)
        self._airborne = True

    async def _send_move(self, wp: Waypoint) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_MOVE
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = wp.x_m
        goal.target_pose.pose.position.y = wp.y_m
        goal.target_pose.pose.position.z = wp.altitude_m
        goal.target_pose.pose.orientation.z = math.sin(wp.heading_rad / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(wp.heading_rad / 2.0)
        goal.timeout_sec = 30.0
        goal.correlation_id = wp.label or "mission_move"
        await self._send_and_wait(goal)

    async def _send_hover(self, duration_sec: float, *, recovery: bool = False) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_HOVER
        goal.timeout_sec = duration_sec
        goal.correlation_id = "mission_hover"
        await self._send_and_wait(
            goal,
            respect_mission_deadline=not recovery,
            respect_cancellation=not recovery,
        )

    async def _send_stability_move(self, x_m: float, y_m: float, altitude_m: float) -> None:
        """Send one stability-track waypoint with constant orientation."""

        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_MOVE
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x_m
        goal.target_pose.pose.position.y = y_m
        goal.target_pose.pose.position.z = altitude_m
        goal.target_pose.pose.orientation.w = 1.0
        goal.timeout_sec = 30.0
        goal.correlation_id = "stability_waypoint"
        await self._send_and_wait(goal)

    def _capture_stability_pose(self) -> tuple[float, float, float]:
        if self._competition_planner is None:
            raise RuntimeError("competition planner unavailable")
        pose = self._competition_planner._capture_map_pose()
        return pose.x_m, pose.y_m, pose.yaw_rad

    async def _send_land(self, *, recovery: bool = False) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_LAND
        goal.timeout_sec = 15.0
        goal.correlation_id = "recovery_land"
        await self._send_and_wait(
            goal,
            respect_mission_deadline=not recovery,
            respect_cancellation=not recovery,
        )

    async def _execute_landing(
        self,
        feedback: ExecuteMission.Feedback,
        current_x_m: float = 0.0,
        current_y_m: float = 0.0,
        include_disarm: bool = True,
    ) -> None:
        config = self._mission_config
        land_params = config.terminal_landing if config else None
        steps = TerminalLandingPlugin().generate(current_x_m, current_y_m, land_params)
        _LANDING_COMMANDS = {
            LandingStep.DESCEND: (FlightCommand.Goal.COMMAND_MOVE, "landing_descend"),
            LandingStep.LAND: (FlightCommand.Goal.COMMAND_LAND, "landing_land"),
            LandingStep.DISARM: (FlightCommand.Goal.COMMAND_DISARM, "landing_disarm"),
        }
        for step, wp in steps:
            if step == LandingStep.DISARM and not include_disarm:
                continue
            cmd, corr_id = _LANDING_COMMANDS[step]
            goal = FlightCommand.Goal()
            goal.command = cmd
            goal.timeout_sec = 5.0 if step == LandingStep.DISARM else 15.0
            goal.correlation_id = corr_id
            if step == LandingStep.DESCEND and wp is not None:
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.pose.position.x = wp.x_m
                goal.target_pose.pose.position.y = wp.y_m
                goal.target_pose.pose.position.z = wp.altitude_m
            await self._send_and_wait(goal)
        feedback.state_id = "landed"
        self._goal_handle.publish_feedback(feedback)

    async def _send_disarm(self, *, recovery: bool = False) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_DISARM
        goal.timeout_sec = 5.0
        goal.correlation_id = "landing_disarm"
        await self._send_and_wait(
            goal,
            respect_mission_deadline=not recovery,
            respect_cancellation=not recovery,
        )

    def _raise_if_cancelled(self) -> None:
        if self._mission_deadline is not None:
            remaining_sec = self._mission_deadline.remaining_sec(steady_now_sec())
            if remaining_sec is not None and remaining_sec <= 0.0:
                raise MissionTimeout("mission deadline expired")
        if self._cancel_requested or (
            self._goal_handle is not None and self._goal_handle.is_cancel_requested
        ):
            raise MissionCancelled("cancelled by user")

    def _cancel_active_flight(self) -> None:
        if self._active_flight_goal is not None:
            self._active_flight_goal.cancel_goal_async()

    def _deadline_for_timeout(
        self,
        timeout_sec: float,
        *,
        respect_mission_deadline: bool = True,
    ) -> MissionDeadline:
        now_sec = steady_now_sec()
        limits = [timeout_sec]
        if respect_mission_deadline and self._mission_deadline is not None:
            remaining_sec = self._mission_deadline.remaining_sec(now_sec)
            if remaining_sec is not None:
                if remaining_sec <= 0.0:
                    raise MissionTimeout("mission deadline expired")
                limits.append(remaining_sec)
        return MissionDeadline.from_limits(now_sec=now_sec, limits=limits)

    async def _send_and_wait(
        self,
        goal: FlightCommand.Goal,
        *,
        respect_mission_deadline: bool = True,
        respect_cancellation: bool = True,
    ) -> FlightCommand.Result:
        deadline = self._deadline_for_timeout(
            goal.timeout_sec + 0.5,
            respect_mission_deadline=respect_mission_deadline,
        )
        remaining_sec = deadline.remaining_sec(steady_now_sec())
        if remaining_sec is None or not self._flight_client.wait_for_server(timeout_sec=remaining_sec):
            raise MissionTimeout("FlightCommand action server not available before deadline")
        if respect_cancellation:
            self._raise_if_cancelled()
        goal_handle = await wait_with_deadline(
            self,
            self._flight_client.send_goal_async(goal),
            deadline,
            self._cancel_active_flight,
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"FlightCommand rejected: {goal.correlation_id}")
        self._active_flight_goal = goal_handle
        try:
            if respect_cancellation and self._cancel_requested:
                self._cancel_active_flight()
                raise MissionCancelled("cancelled by user")
            result = await wait_with_deadline(
                self,
                goal_handle.get_result_async(),
                deadline,
                self._cancel_active_flight,
            )
        finally:
            self._active_flight_goal = None
        if (respect_cancellation and self._cancel_requested) or result.status == GoalStatus.STATUS_CANCELED:
            raise MissionCancelled("cancelled by user")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"FlightCommand action status {result.status}")
        if result.result.result_code != FlightCommand.Result.RESULT_SUCCEEDED:
            raise RuntimeError(
                f"FlightCommand failed [{goal.correlation_id}]: {result.result.reason}"
            )
        if goal.command == FlightCommand.Goal.COMMAND_DISARM:
            self._airborne = False
        return result.result

    async def _recover_after_airborne_failure(self) -> None:
        if self._hard_lock_active or not self._airborne:
            return
        for recovery in (
            lambda: self._send_hover(1.0, recovery=True),
            lambda: self._send_land(recovery=True),
            lambda: self._send_disarm(recovery=True),
        ):
            try:
                await recovery()
            except (MissionCancelled, MissionTimeout, RuntimeError) as exc:
                self.get_logger().error(f"Recovery command failed: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
