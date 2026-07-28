"""Nav2 and TF runtime for the planner-only competition mission."""

from __future__ import annotations

import math
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing_extensions import assert_never

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ed_uav_interfaces.action import ExecuteMission
from ed_uav_mission.competition_tree import (
    CompetitionStep,
    MapPose,
    competition_sequence,
    forward_goal,
    moves_from_planner_path,
    return_goal,
    yaw_from_quaternion,
)
from ed_uav_mission.mission_model import CompetitionParams, Waypoint
from ed_uav_mission.state_machine import MissionState
from ed_uav_mission.action_lifecycle import (
    MissionCancelled,
    MissionDeadline,
    MissionTimeout,
    steady_now_sec,
    wait_with_deadline,
)


@dataclass(frozen=True, slots=True)
class CompetitionCallbacks:
    """Executor-owned operations exposed to the competition runtime."""

    execute_takeoff: Callable[[ExecuteMission.Feedback], Awaitable[None]]
    send_hover: Callable[[float], Awaitable[None]]
    send_move: Callable[[Waypoint], Awaitable[None]]
    execute_landing: Callable[[ExecuteMission.Feedback, float, float, bool], Awaitable[None]]
    send_disarm: Callable[[], Awaitable[None]]
    raise_if_cancelled: Callable[[], None]
    deadline_for_timeout: Callable[[float], MissionDeadline]
    transition: Callable[[MissionState, str], None]


class CompetitionRuntime:
    """Execute the competition tree using Nav2 paths and TF poses."""

    def __init__(self, node: Node, callbacks: CompetitionCallbacks) -> None:
        self._node = node
        self._callbacks = callbacks
        self._planner_client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        self._active_planner_goal = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    async def run(
        self,
        params: CompetitionParams | None,
        feedback: ExecuteMission.Feedback,
    ) -> None:
        if params is None:
            raise RuntimeError("competition params not loaded")
        start: MapPose | None = None
        for step in competition_sequence():
            self._raise_if_cancelled()
            match step:
                case CompetitionStep.TAKEOFF:
                    self._callbacks.transition(MissionState.TAKEOFF, "preflight passed")
                    await self._callbacks.execute_takeoff(feedback)
                    self._callbacks.transition(MissionState.EXECUTING, "takeoff complete")
                case CompetitionStep.HOVER:
                    await self._callbacks.send_hover(params.hover_sec)
                    start = self._capture_map_pose()
                case CompetitionStep.NAVIGATE_FORWARD:
                    if start is None:
                        raise RuntimeError("competition start pose unavailable")
                    forward = forward_goal(start, params.forward_distance_m)
                    forward_moves = await self._planned_moves(
                        start, forward, params, "competition_forward"
                    )
                    await self._send_moves(forward_moves)
                case CompetitionStep.NAVIGATE_RETURN:
                    if start is None:
                        raise RuntimeError("competition return pose unavailable")
                    return_start = self._capture_map_pose()
                    return_moves = await self._planned_moves(
                        return_start,
                        return_goal(start),
                        params,
                        "competition_return",
                    )
                    await self._send_moves(return_moves)
                case CompetitionStep.LAND:
                    if start is None:
                        raise RuntimeError("competition landing pose unavailable")
                    self._callbacks.transition(
                        MissionState.RETURNING, "competition return complete"
                    )
                    self._callbacks.transition(MissionState.LANDING, "landing sequence")
                    await self._callbacks.execute_landing(
                        feedback,
                        start.x_m,
                        start.y_m,
                        False,
                    )
                case CompetitionStep.DISARM:
                    await self._callbacks.send_disarm()
                case unreachable:
                    assert_never(unreachable)

    def _raise_if_cancelled(self) -> None:
        self._callbacks.raise_if_cancelled()

    def cancel_active(self) -> None:
        if self._active_planner_goal is not None:
            self._active_planner_goal.cancel_goal_async()

    def _capture_map_pose(self) -> MapPose:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as exc:
            raise RuntimeError(f"map to base_link transform unavailable: {exc}") from exc
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return MapPose(
            x_m=translation.x,
            y_m=translation.y,
            yaw_rad=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    async def _planned_moves(
        self,
        start: MapPose,
        destination: MapPose,
        params: CompetitionParams,
        label: str,
    ) -> tuple[Waypoint, ...]:
        path = await self._request_planner_path(start, destination, params.planner_timeout_sec)
        moves = moves_from_planner_path(path, altitude_m=params.altitude_m, label=label)
        return tuple(
            Waypoint(
                x_m=move.x_m,
                y_m=move.y_m,
                altitude_m=move.altitude_m,
                heading_rad=move.yaw_rad,
                label=move.label,
            )
            for move in moves
        )

    async def _send_moves(self, moves: tuple[Waypoint, ...]) -> None:
        for move in moves:
            self._raise_if_cancelled()
            await self._callbacks.send_move(move)

    async def _request_planner_path(
        self,
        start: MapPose,
        destination: MapPose,
        timeout_sec: float,
    ) -> tuple[MapPose, ...]:
        deadline = self._callbacks.deadline_for_timeout(timeout_sec)
        remaining_sec = deadline.remaining_sec(steady_now_sec())
        if remaining_sec is None or not self._planner_client.wait_for_server(timeout_sec=remaining_sec):
            raise MissionTimeout("ComputePathToPose action server not available before deadline")
        self._raise_if_cancelled()
        request = ComputePathToPose.Goal()
        request.start = self._map_pose_stamped(start)
        request.goal = self._map_pose_stamped(destination)
        request.use_start = True
        goal_handle = await wait_with_deadline(
            self._node,
            self._planner_client.send_goal_async(request),
            deadline,
            self.cancel_active,
        )
        if not goal_handle.accepted:
            raise RuntimeError("ComputePathToPose rejected")
        self._active_planner_goal = goal_handle
        try:
            result = await wait_with_deadline(
                self._node,
                goal_handle.get_result_async(),
                deadline,
                self.cancel_active,
            )
        finally:
            self._active_planner_goal = None
        if result.status == GoalStatus.STATUS_CANCELED:
            raise MissionCancelled("ComputePathToPose canceled")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ComputePathToPose failed with status {result.status}")
        return self._planner_path_to_map_poses(result.result.path)

    def _map_pose_stamped(self, pose: MapPose) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.header.stamp = self._node.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x_m
        stamped.pose.position.y = pose.y_m
        stamped.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        stamped.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        return stamped

    def _planner_path_to_map_poses(self, path: NavPath) -> tuple[MapPose, ...]:
        if path.header.frame_id != "map":
            raise RuntimeError("ComputePathToPose path is not in map frame")
        poses: list[MapPose] = []
        for stamped in path.poses:
            if stamped.header.frame_id and stamped.header.frame_id != "map":
                raise RuntimeError("ComputePathToPose pose is not in map frame")
            position = stamped.pose.position
            orientation = stamped.pose.orientation
            poses.append(
                MapPose(
                    x_m=position.x,
                    y_m=position.y,
                    yaw_rad=yaw_from_quaternion(
                        orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w,
                    ),
                )
            )
        return tuple(poses)
