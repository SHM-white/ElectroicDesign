"""No-car mode simulation environment.

Publishes the FCU/localization contracts the mission executor's preflight
requires and acknowledges every FlightCommand goal as SUCCEEDED, so the whole
task flow can be exercised end-to-end with a simulated module pipeline and no
physical car, FCU, lidar, or camera.

The node never exits on errors: timer/action exceptions are logged and the
loop keeps serving.
"""

from __future__ import annotations

import asyncio

import rclpy
from ed_uav_interfaces.action import FlightCommand
from ed_uav_interfaces.msg import FcuState, LocalizationStatus
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

_COMMAND_NAMES = {
    FlightCommand.Goal.COMMAND_ARM: "ARM",
    FlightCommand.Goal.COMMAND_DISARM: "DISARM",
    FlightCommand.Goal.COMMAND_TAKEOFF: "TAKEOFF",
    FlightCommand.Goal.COMMAND_MOVE: "MOVE",
    FlightCommand.Goal.COMMAND_HOVER: "HOVER",
    FlightCommand.Goal.COMMAND_LAND: "LAND",
    FlightCommand.Goal.COMMAND_SET_MODE: "SET_MODE",
}
_SIMULATED_DELAY_SECONDS = 0.2


class NoCarSimNode(Node):
    """Standalone fake FCU/localization + FlightCommand ack server."""

    def __init__(self) -> None:
        super().__init__("no_car_sim")
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("simulated_delay_seconds", _SIMULATED_DELAY_SECONDS)
        self._delay_seconds = float(self.get_parameter("simulated_delay_seconds").value)
        self._group = ReentrantCallbackGroup()
        self._fcu_pub = self.create_publisher(FcuState, "/fcu/state", 10)
        self._loc_pub = self.create_publisher(LocalizationStatus, "/localization/status", 10)
        rate_hz = float(self.get_parameter("state_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._publish_states, callback_group=self._group)
        self._action_server = ActionServer(
            self,
            FlightCommand,
            "/fcu/flight_command",
            execute_callback=self._execute_command,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group,
        )
        self._publish_static_tf()
        self.get_logger().info("no_car_sim.ready  (无小车模式: 模拟飞控/定位/飞行指令)")

    def _publish_static_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        StaticTransformBroadcaster(self).sendTransform(transform)

    def _publish_states(self) -> None:
        try:
            fcu = FcuState()
            fcu.source = FcuState.SOURCE_SIMULATOR
            fcu.mode = FcuState.MODE_STABILIZE
            fcu.motors_armed = True
            fcu.communication_ok = True
            fcu.aux1_valid = True
            fcu.task3_control_allowed = True
            fcu.altitude_m = 0.0
            fcu.battery_voltage_v = 24.5
            fcu.frame_id = "base_link"
            self._fcu_pub.publish(fcu)

            loc = LocalizationStatus()
            loc.source = LocalizationStatus.SOURCE_FUSED
            loc.state = LocalizationStatus.STATE_ACTIVE
            loc.map_to_odom_valid = True
            self._loc_pub.publish(loc)
        except Exception:  # noqa: BLE001 - daemon contract: log and continue
            self.get_logger().error("no_car_sim 状态发布异常(已隔离)", exc_info=True)

    def _on_goal(self, goal: FlightCommand.Goal) -> GoalResponse:
        if goal.command not in _COMMAND_NAMES:
            self.get_logger().warn(f"no_car_sim 收到未知指令 {goal.command}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_command(self, goal_handle: ServerGoalHandle) -> FlightCommand.Result:
        request = goal_handle.request
        position = request.target_pose.pose.position
        self.get_logger().info(
            f"[no-car-sim] 模拟调用飞行模块 {_COMMAND_NAMES.get(request.command, request.command)}"
            f" corr={request.correlation_id} timeout={request.timeout_sec}s"
            f" target=({position.x:.2f}, {position.y:.2f}, {position.z:.2f})"
        )
        await asyncio.sleep(self._delay_seconds)
        result = FlightCommand.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.result_code = FlightCommand.Result.RESULT_REJECTED
            result.reason = "canceled by no-car sim"
            return result
        goal_handle.succeed()
        result.result_code = FlightCommand.Result.RESULT_SUCCEEDED
        result.reason = "no-car simulation acknowledged"
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    backoff = 1.0
    while True:
        node = None
        try:
            node = NoCarSimNode()
            rclpy.spin(node)
            return
        except (KeyboardInterrupt, ExternalShutdownException):
            return
        except Exception:  # noqa: BLE001 - daemon contract: never exit on errors
            if node is not None:
                node.get_logger().error(
                    f"no_car_sim 异常退出, {backoff:.1f}s 后重建节点"
                    "(守护契约: 进程不退出)",
                    exc_info=True,
                )
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 30.0)
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass
            rclpy.try_shutdown()
        try:
            rclpy.init(args=args)
        except Exception:  # noqa: BLE001
            print("rclpy 重新初始化失败, 重试", file=sys.stderr)
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 30.0)
            continue
        backoff = 1.0


if __name__ == "__main__":
    main()
