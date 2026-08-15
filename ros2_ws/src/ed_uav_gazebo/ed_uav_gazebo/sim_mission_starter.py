"""Autostart one hardware-free D-task mission after localization becomes healthy."""

from __future__ import annotations

import rclpy
from ed_uav_interfaces.action import ExecuteMission, FlightCommand
from ed_uav_interfaces.msg import FcuState, LocalizationStatus, PayloadContactState
from ed_uav_interfaces.srv import SelectDTaskMission
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


class SimMissionStarter(Node):
    """Select, arm, execute, and release the car without an HMI or serial device."""

    def __init__(self) -> None:
        super().__init__("sim_mission_starter")
        self._mission_id = str(self.declare_parameter("mission_id", "d-arena-competition-2026").value)
        self._field_profile_id = str(self.declare_parameter("field_profile_id", "d-arena-2026").value)
        self._mission_profile_id = str(self.declare_parameter("mission_profile_id", "d2026-competition").value)
        self._deployment_preset_id = str(self.declare_parameter("deployment_preset_id", "field-2026").value)
        self._target_revision = str(self.declare_parameter("target_revision", "d2026-apriltag-v1").value)
        self._task = int(self.declare_parameter("task", SelectDTaskMission.Request.TASK_PAYLOAD_DROP).value)
        self._group = ReentrantCallbackGroup()
        self._state = "WAIT_READY"
        self._fcu: FcuState | None = None
        self._localization: LocalizationStatus | None = None
        self._start_published_at_ns: int | None = None
        self._payload_sequence = 0
        self._selection = self.create_client(
            SelectDTaskMission,
            "/mission/select_d_task",
            callback_group=self._group,
        )
        self._flight = ActionClient(self, FlightCommand, "/fcu/flight_command", callback_group=self._group)
        self._mission = ActionClient(self, ExecuteMission, "/mission/execute", callback_group=self._group)
        self._start_pub = self.create_publisher(Bool, "/simulation/competition_start", 10)
        self._payload_pub = self.create_publisher(PayloadContactState, "/payload/contact_state", 10)
        self.create_subscription(FcuState, "/fcu/state", self._on_fcu, 10, callback_group=self._group)
        self.create_subscription(
            LocalizationStatus,
            "/localization/status",
            self._on_localization,
            10,
            callback_group=self._group,
        )
        self.create_timer(0.1, self._tick, callback_group=self._group)

    def _on_fcu(self, message: FcuState) -> None:
        self._fcu = message

    def _on_localization(self, message: LocalizationStatus) -> None:
        self._localization = message

    def _tick(self) -> None:
        self._publish_payload_secured()
        if self._state == "WAIT_READY" and self._ready_to_select():
            self._send_selection()
        elif self._state == "WAIT_ARM_STATE" and self._fcu is not None and self._fcu.motors_armed:
            self._send_mission()
        elif self._state == "RUNNING":
            # Repeat briefly so a late-created controller cannot miss the simulation-only trigger.
            now_ns = self.get_clock().now().nanoseconds
            if self._start_published_at_ns is None or now_ns - self._start_published_at_ns <= 2_000_000_000:
                self._start_pub.publish(Bool(data=True))

    def _ready_to_select(self) -> bool:
        fcu = self._fcu
        localization = self._localization
        return bool(
            fcu is not None
            and fcu.communication_ok
            and fcu.source == FcuState.SOURCE_SIMULATOR
            and localization is not None
            and localization.state == LocalizationStatus.STATE_ACTIVE
            and localization.map_to_odom_valid
            and self._selection.service_is_ready()
            and self._flight.wait_for_server(timeout_sec=0.0)
            and self._mission.wait_for_server(timeout_sec=0.0)
        )

    def _send_selection(self) -> None:
        self._state = "SELECTING"
        request = SelectDTaskMission.Request()
        request.contract_version = SelectDTaskMission.Request.CONTRACT_VERSION
        request.mission_id = self._mission_id
        request.field_profile_id = self._field_profile_id
        request.mission_profile_id = self._mission_profile_id
        request.deployment_preset_id = self._deployment_preset_id
        request.target_revision = self._target_revision
        request.task = self._task
        self._selection.call_async(request).add_done_callback(self._selection_done)

    def _selection_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self._fail(f"selection call failed: {error}")
            return
        if response is None or not response.accepted:
            self._fail(f"selection rejected: {getattr(response, 'reason', 'no response')}")
            return
        self._send_arm()

    def _send_arm(self) -> None:
        self._state = "ARMING"
        goal = FlightCommand.Goal()
        goal.command = FlightCommand.Goal.COMMAND_ARM
        goal.timeout_sec = 3.0
        goal.correlation_id = "sim-auto-arm"
        self._flight.send_goal_async(goal).add_done_callback(self._arm_goal_done)

    def _arm_goal_done(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._fail("simulator arm goal rejected")
            return
        handle.get_result_async().add_done_callback(self._arm_result_done)

    def _arm_result_done(self, future) -> None:
        wrapped = future.result()
        if wrapped.result.result_code != FlightCommand.Result.RESULT_SUCCEEDED:
            self._fail(f"simulator arm failed: {wrapped.result.reason}")
            return
        self._state = "WAIT_ARM_STATE"

    def _send_mission(self) -> None:
        self._state = "STARTING_MISSION"
        goal = ExecuteMission.Goal()
        goal.mission_id = self._mission_id
        goal.field_profile_id = self._field_profile_id
        goal.timeout_sec = 90.0
        self._mission.send_goal_async(goal).add_done_callback(self._mission_goal_done)

    def _mission_goal_done(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._fail("competition mission goal rejected")
            return
        self._state = "RUNNING"
        self._start_published_at_ns = self.get_clock().now().nanoseconds
        self._start_pub.publish(Bool(data=True))
        self.get_logger().info("simulation mission accepted; car start released")
        handle.get_result_async().add_done_callback(self._mission_result_done)

    def _mission_result_done(self, future) -> None:
        wrapped = future.result()
        self._state = "COMPLETE" if wrapped.result.result_code == ExecuteMission.Result.RESULT_SUCCEEDED else "FAILED"
        self.get_logger().info(
            f"simulation mission result={wrapped.result.result_code} reason={wrapped.result.reason}"
        )

    def _publish_payload_secured(self) -> None:
        message = PayloadContactState()
        message.contract_version = PayloadContactState.CONTRACT_VERSION
        message.acquisition_stamp = self.get_clock().now().to_msg()
        message.source_sequence = self._payload_sequence
        self._payload_sequence = (self._payload_sequence + 1) % (1 << 32)
        message.payload_state = PayloadContactState.PAYLOAD_SECURED
        message.contact_state = PayloadContactState.CONTACT_HOME
        message.contact_stable = True
        message.contact_duration_s = 0.0
        message.owner = "sim_mission_starter"
        message.frame_id = "base_link"
        self._payload_pub.publish(message)

    def _fail(self, reason: str) -> None:
        self._state = "FAILED"
        self.get_logger().error(reason)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimMissionStarter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
