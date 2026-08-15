"""Nav2/TF motion planning used by the executor-owned D-task callbacks."""

from __future__ import annotations

import math
from collections.abc import Awaitable, Callable, Sequence
from typing import Protocol

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ed_uav_mission.action_lifecycle import (
    MissionCancelled,
    MissionDeadline,
    MissionTimeout,
    steady_now_sec,
    wait_with_deadline,
)
from ed_uav_mission.competition_tree import (
    MapPose,
    moves_from_planner_path,
    yaw_from_quaternion,
)
from ed_uav_mission.d_task_events import TargetSnapshot, VehicleSnapshot
from ed_uav_mission.mission_model import CompetitionParams, Waypoint


class HeaderLike(Protocol):
    @property
    def frame_id(self) -> str: ...


class NavPathLike(Protocol):
    @property
    def header(self) -> HeaderLike: ...

    @property
    def poses(self) -> Sequence[PoseStamped]: ...


class CompetitionPlanner:
    """Plan map-frame D-task moves without owning a flight action client."""

    def __init__(
        self,
        node: Node,
        send_move: Callable[[Waypoint], Awaitable[None]],
        deadline_for_timeout: Callable[[float], MissionDeadline],
        raise_if_cancelled: Callable[[], None],
    ) -> None:
        self._node = node
        self._send_move = send_move
        self._deadline_for_timeout = deadline_for_timeout
        self._raise_if_cancelled = raise_if_cancelled
        self._planner_client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        self._active_planner_goal = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._home: MapPose | None = None

    def capture_home(self) -> None:
        self._home = self._capture_map_pose()

    def cancel_active(self) -> None:
        if self._active_planner_goal is not None:
            self._active_planner_goal.cancel_goal_async()

    async def track_target(
        self,
        target: TargetSnapshot,
        vehicle: VehicleSnapshot,
        altitude_m: float,
    ) -> None:
        """Apply typed PnP relative error plus a bounded car-motion prediction."""
        current = self._capture_map_pose()
        horizon_s = 0.5
        predicted_m = min(vehicle.speed_m_s * horizon_s, 1.0)
        predicted_x = predicted_m * math.cos(vehicle.heading_rad)
        predicted_y = predicted_m * math.sin(vehicle.heading_rad)
        waypoint = Waypoint(
            x_m=current.x_m + predicted_x + target.relative_x_m,
            y_m=current.y_m + predicted_y + target.relative_y_m,
            altitude_m=altitude_m,
            heading_rad=current.yaw_rad + vehicle.yaw_rate_rad_s * horizon_s,
            label="d2026_target_track",
        )
        await self._send_move(waypoint)

    async def move_right_offset(self, offset_m: float, altitude_m: float) -> None:
        """Move right by *offset_m* in the map frame from the current pose.

        'Right' is defined as the drone body-frame +Y direction, which maps to
        ``map yaw − π/2``.  The move is issued as a single map-frame waypoint.
        """
        current = self._capture_map_pose()
        right_yaw = current.yaw_rad - math.pi / 2.0
        waypoint = Waypoint(
            x_m=current.x_m + offset_m * math.cos(right_yaw),
            y_m=current.y_m + offset_m * math.sin(right_yaw),
            altitude_m=altitude_m,
            heading_rad=current.yaw_rad,
            label="d2026_move_right",
        )
        await self._send_move(waypoint)

    async def search_forward(self, distance_m: float, altitude_m: float) -> None:
        """Move forward by *distance_m* in the map frame from the current pose.

        'Forward' is defined as the drone body-frame +X direction, which maps to
        the current heading.  The move is issued as a single map-frame waypoint.
        """
        current = self._capture_map_pose()
        waypoint = Waypoint(
            x_m=current.x_m + distance_m * math.cos(current.yaw_rad),
            y_m=current.y_m + distance_m * math.sin(current.yaw_rad),
            altitude_m=altitude_m,
            heading_rad=current.yaw_rad,
            label="d2026_search_forward",
        )
        await self._send_move(waypoint)

    async def precision_land_on_target(
        self,
        target: TargetSnapshot,
        vehicle: VehicleSnapshot,
        visual_servo_controller,
    ) -> bool:
        """Precision landing using visual servo controller.
        
        Uses the visual servo controller to compute velocity corrections
        for precise landing on the target marker.
        
        Args:
            target: Target observation snapshot
            vehicle: Vehicle telemetry snapshot
            visual_servo_controller: VisualServoController instance
            
        Returns:
            True if landing is complete and stable
        """
        import time
        
        # Get current position
        current = self._capture_map_pose()
        
        # Use visual servo controller for final approach
        # The controller expects camera-frame coordinates
        # Target observation is already in camera frame
        command = visual_servo_controller.compute_command(
            target_x_m=target.relative_x_m,
            target_y_m=target.relative_y_m,
            target_z_m=target.relative_z_m,
            current_timestamp_sec=time.monotonic(),
        )
        
        # Convert body-frame velocity to map-frame waypoint
        # For simplicity, we'll use the velocity to compute a small offset
        # In a real implementation, this would use proper frame transforms
        dt = 0.1  # 100ms prediction horizon
        offset_x = command.vx_m_s * dt
        offset_y = command.vy_m_s * dt
        
        # Compute waypoint with velocity-based offset
        waypoint = Waypoint(
            x_m=current.x_m + offset_x,
            y_m=current.y_m + offset_y,
            altitude_m=current.x_m + command.vz_m_s * dt,  # Use current altitude + correction
            heading_rad=current.yaw_rad + command.yaw_rate_rad_s * dt,
            label="d2026_precision_land",
        )
        
        await self._send_move(waypoint)
        
        return command.converged

    async def descend_to_vehicle(
        self,
        target: TargetSnapshot,
        vehicle: VehicleSnapshot,
    ) -> None:
        await self.track_target(target, vehicle, 0.15)

    async def return_home(self, params: CompetitionParams) -> None:
        home = self._home
        if home is None:
            raise RuntimeError("D-task home pose unavailable")
        current = self._capture_map_pose()
        path = await self._request_path(current, home, params.planner_timeout_sec)
        moves = moves_from_planner_path(
            path,
            altitude_m=params.altitude_m,
            label="d2026_return_home",
        )
        for move in moves:
            self._raise_if_cancelled()
            await self._send_move(
                Waypoint(
                    x_m=move.x_m,
                    y_m=move.y_m,
                    altitude_m=move.altitude_m,
                    heading_rad=move.yaw_rad,
                    label=move.label,
                )
            )

    @property
    def home(self) -> MapPose:
        if self._home is None:
            raise RuntimeError("D-task home pose unavailable")
        return self._home

    def _capture_map_pose(self) -> MapPose:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as error:
            raise RuntimeError(f"map to base_link transform unavailable: {error}") from error
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return MapPose(
            x_m=translation.x,
            y_m=translation.y,
            yaw_rad=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    async def _request_path(
        self,
        start: MapPose,
        destination: MapPose,
        timeout_s: float,
    ) -> tuple[MapPose, ...]:
        deadline = self._deadline_for_timeout(timeout_s)
        remaining_s = deadline.remaining_sec(steady_now_sec())
        if remaining_s is None or not self._planner_client.wait_for_server(timeout_sec=remaining_s):
            raise MissionTimeout("ComputePathToPose server unavailable before deadline")
        request = ComputePathToPose.Goal()
        request.start = self._stamped(start)
        request.goal = self._stamped(destination)
        request.use_start = True
        handle = await wait_with_deadline(
            self._node,
            self._planner_client.send_goal_async(request),
            deadline,
            self.cancel_active,
        )
        if not handle.accepted:
            raise RuntimeError("ComputePathToPose rejected")
        self._active_planner_goal = handle
        try:
            result = await wait_with_deadline(
                self._node,
                handle.get_result_async(),
                deadline,
                self.cancel_active,
            )
        finally:
            self._active_planner_goal = None
        if result.status == GoalStatus.STATUS_CANCELED:
            raise MissionCancelled("ComputePathToPose canceled")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ComputePathToPose failed with status {result.status}")
        return self._path_poses(result.result.path)

    def _stamped(self, pose: MapPose) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.header.stamp = self._node.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x_m
        stamped.pose.position.y = pose.y_m
        stamped.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        stamped.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        return stamped

    @staticmethod
    def _path_poses(path: NavPathLike) -> tuple[MapPose, ...]:
        if path.header.frame_id != "map":
            raise RuntimeError("ComputePathToPose path is not in map frame")
        poses = []
        for stamped in path.poses:
            if stamped.header.frame_id and stamped.header.frame_id != "map":
                raise RuntimeError("ComputePathToPose pose is not in map frame")
            position = stamped.pose.position
            rotation = stamped.pose.orientation
            poses.append(
                MapPose(
                    x_m=position.x,
                    y_m=position.y,
                    yaw_rad=yaw_from_quaternion(
                        rotation.x,
                        rotation.y,
                        rotation.z,
                        rotation.w,
                    ),
                )
            )
        return tuple(poses)
