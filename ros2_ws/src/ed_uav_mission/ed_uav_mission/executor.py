"""ExecuteMission action server — preflight validation and plugin orchestration.

All motion goes through the ``FlightCommand`` action (``ed_uav_interfaces``).
This module never imports serial, GPIO, or camera APIs.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from ed_uav_interfaces.action import ExecuteMission, FlightCommand
from ed_uav_interfaces.msg import FcuState, LocalizationStatus
from ed_uav_localization.field_profile.loader import load_profile_text
from ed_uav_localization.field_profile.model import KnownFieldProfile

from ed_uav_mission.mission_model import (
    MISSION_SCHEMA, MissionConfig, MissionType, Waypoint,
)
from ed_uav_mission.plugins.coverage import GridCoveragePlugin
from ed_uav_mission.plugins.patrol import WaypointPatrolPlugin
from ed_uav_mission.plugins.target_visit import TargetVisitPlugin
from ed_uav_mission.plugins.terminal_landing import LandingStep, TerminalLandingPlugin
from ed_uav_mission.plugins.payload import PayloadPlugin
from ed_uav_mission.state_machine import MissionFSM, MissionState


class PreflightCode(Enum):
    OK = auto()
    STALE_AUX = auto()
    NO_FCU_LINK = auto()
    LOCALIZATION_LOST = auto()
    PROFILE_INVALID = auto()
    CALIBRATION_MISSING = auto()


@dataclass(frozen=True, slots=True)
class PreflightResult:
    code: PreflightCode
    reason: str = ""


def validate_preflight(
    *,
    fcu_communication_ok: bool,
    fcu_motors_armed: bool,
    aux_start_active: bool,
    localization_active: bool,
    map_to_odom_valid: bool,
    profile_loaded: bool,
    calibration_valid: bool,
) -> PreflightResult:
    """Pure-function preflight gate — testable without ROS infrastructure."""
    if not aux_start_active:
        return PreflightResult(PreflightCode.STALE_AUX, "AUX start switch is stale or off")
    if not fcu_communication_ok:
        return PreflightResult(PreflightCode.NO_FCU_LINK, "no FCU communication")
    if not localization_active:
        return PreflightResult(PreflightCode.LOCALIZATION_LOST, "localization not active")
    if not map_to_odom_valid:
        return PreflightResult(PreflightCode.LOCALIZATION_LOST, "map-to-odom transform invalid")
    if not profile_loaded:
        return PreflightResult(PreflightCode.PROFILE_INVALID, "field profile not loaded or invalid")
    if not calibration_valid:
        return PreflightResult(PreflightCode.CALIBRATION_MISSING, "sensor calibration missing")
    if not fcu_motors_armed:
        return PreflightResult(PreflightCode.CALIBRATION_MISSING, "motors not armed")
    return PreflightResult(PreflightCode.OK)


class MissionExecutorNode(Node):
    """ROS 2 node that serves the ``ExecuteMission`` action."""

    def __init__(self) -> None:
        super().__init__("mission_executor")
        self._fsm = MissionFSM()
        self._goal_handle: ServerGoalHandle | None = None
        self._cancel_requested = False
        self._latest_fcu: FcuState | None = None
        self._latest_localization: LocalizationStatus | None = None
        self._profile: KnownFieldProfile | None = None
        self._mission_config: MissionConfig | None = None

        cb_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self, ExecuteMission, "/mission/execute",
            execute_callback=self._execution_loop,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb_group,
        )
        self._flight_client = rclpy.action.ActionClient(
            self, FlightCommand, "/fcu/flight_command"
        )
        self._fcu_sub = self.create_subscription(FcuState, "/fcu/state", self._on_fcu_state, 10)
        self._loc_sub = self.create_subscription(
            LocalizationStatus, "/localization/status", self._on_localization_status, 10
        )
        self.get_logger().info("Mission executor ready")

    def load_profile(self, yaml_text: str) -> None:
        profile = load_profile_text(yaml_text, "<inline>")
        if not isinstance(profile, KnownFieldProfile):
            raise ValueError("mission requires a known field profile")
        self._profile = profile

    def load_mission_config(self, yaml_text: str) -> None:
        self._mission_config = MISSION_SCHEMA.validate_json(yaml_text)

    def _on_fcu_state(self, msg: FcuState) -> None:
        self._latest_fcu = msg

    def _on_localization_status(self, msg: LocalizationStatus) -> None:
        self._latest_localization = msg

    def _on_goal(self, goal_request: ExecuteMission.Goal) -> GoalResponse:
        if self._fsm.is_active:
            return GoalResponse.REJECT
        self.get_logger().info(f"Received mission goal: {goal_request.mission_id}")
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Cancel requested")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execution_loop(self, goal_handle: ServerGoalHandle) -> ExecuteMission.Result:
        self._goal_handle = goal_handle
        self._cancel_requested = False
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

            self._fsm.transition(MissionState.TAKEOFF, "preflight passed")
            await self._execute_takeoff(feedback)

            self._fsm.transition(MissionState.EXECUTING, "takeoff complete")
            waypoints = self._dispatch_plugin()
            for index, wp in enumerate(waypoints):
                if self._cancel_requested or goal_handle.is_cancel_requested:
                    self._cancel_requested = True
                    self._fsm.transition(MissionState.ABORTED, "cancel requested")
                    result.result_code = ExecuteMission.Result.RESULT_ABORTED
                    result.reason = "cancelled by user"
                    goal_handle.abort()
                    return result
                await self._send_move(wp)
                if wp.hover_sec > 0:
                    await self._send_hover(wp.hover_sec)
                feedback.state_id = f"waypoint_{index}"
                feedback.progress = float(index + 1) / float(max(len(waypoints), 1))
                goal_handle.publish_feedback(feedback)

            self._fsm.transition(MissionState.RETURNING, "mission done")
            await self._execute_landing(feedback)
            self._fsm.transition(MissionState.COMPLETE, "landed")
            result.result_code = ExecuteMission.Result.RESULT_SUCCEEDED
            result.reason = "mission complete"
            goal_handle.succeed()
            return result
        except Exception as exc:
            self.get_logger().error(f"Mission error: {exc}")
            self._fsm.transition(MissionState.ABORTED, str(exc))
            result.result_code = ExecuteMission.Result.RESULT_ABORTED
            result.reason = str(exc)
            goal_handle.abort()
            return result

    def _run_preflight(self) -> PreflightResult:
        fcu, loc = self._latest_fcu, self._latest_localization
        return validate_preflight(
            fcu_communication_ok=fcu is not None and fcu.communication_ok,
            fcu_motors_armed=fcu is not None and fcu.motors_armed,
            aux_start_active=True,
            localization_active=loc is not None and loc.state == LocalizationStatus.STATE_ACTIVE,
            map_to_odom_valid=loc is not None and loc.map_to_odom_valid,
            profile_loaded=self._profile is not None,
            calibration_valid=True,
        )

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
            case _:
                raise ValueError(f"unknown mission type: {config.mission_type}")

    async def _execute_takeoff(self, feedback: ExecuteMission.Feedback) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_TAKEOFF
        goal.timeout_sec = 30.0
        goal.correlation_id = "mission_takeoff"
        feedback.state_id = "takeoff"
        self._goal_handle.publish_feedback(feedback)
        await self._send_and_wait(goal)

    async def _send_move(self, wp: Waypoint) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_MOVE
        goal.target_pose.pose.position.x = wp.x_m
        goal.target_pose.pose.position.y = wp.y_m
        goal.target_pose.pose.position.z = wp.altitude_m
        goal.timeout_sec = 30.0
        goal.correlation_id = wp.label or "mission_move"
        await self._send_and_wait(goal)

    async def _send_hover(self, duration_sec: float) -> None:
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_HOVER
        goal.timeout_sec = duration_sec
        goal.correlation_id = "mission_hover"
        await self._send_and_wait(goal)

    async def _execute_landing(self, feedback: ExecuteMission.Feedback) -> None:
        config = self._mission_config
        land_params = config.terminal_landing if config else None
        steps = TerminalLandingPlugin().generate(0.0, 0.0, land_params)
        _LANDING_COMMANDS = {
            LandingStep.DESCEND: (FlightCommand.Goal.COMMAND_MOVE, "landing_descend"),
            LandingStep.LAND: (FlightCommand.Goal.COMMAND_LAND, "landing_land"),
            LandingStep.DISARM: (FlightCommand.Goal.COMMAND_DISARM, "landing_disarm"),
        }
        for step, wp in steps:
            cmd, corr_id = _LANDING_COMMANDS[step]
            goal = FlightCommand.Goal()
            goal.command = cmd
            goal.timeout_sec = 5.0 if step == LandingStep.DISARM else 15.0
            goal.correlation_id = corr_id
            if step == LandingStep.DESCEND and wp is not None:
                goal.target_pose.pose.position.x = wp.x_m
                goal.target_pose.pose.position.y = wp.y_m
                goal.target_pose.pose.position.z = wp.altitude_m
            await self._send_and_wait(goal)
        feedback.state_id = "landed"
        self._goal_handle.publish_feedback(feedback)

    async def _send_and_wait(self, goal: FlightCommand.Goal) -> FlightCommand.Result:
        if not self._flight_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("FlightCommand action server not available")
        goal_handle = await self._flight_client.send_goal_async(goal)
        if not goal_handle.accepted:
            raise RuntimeError(f"FlightCommand rejected: {goal.correlation_id}")
        result = await goal_handle.get_result_async()
        if result.result.result_code != FlightCommand.Result.RESULT_SUCCEEDED:
            raise RuntimeError(
                f"FlightCommand failed [{goal.correlation_id}]: {result.result.reason}"
            )
        return result.result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
