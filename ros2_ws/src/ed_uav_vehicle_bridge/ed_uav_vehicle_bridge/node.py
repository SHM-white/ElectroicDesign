"""ROS 2 adapter for the authenticated vehicle/HMI UDP boundary."""  # noqa: SIZE_OK

from __future__ import annotations

import time
from collections.abc import Callable
from queue import Empty, SimpleQueue
from typing_extensions import assert_never

from builtin_interfaces.msg import Time
from ed_uav_interfaces.action import ExecuteMission
from ed_uav_interfaces.msg import FcuState, MissionStatus, VehicleTelemetry
from ed_uav_interfaces.srv import SelectDTaskMission
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

from .authority import BridgeAuthority
from .config import load_bridge_config
from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    AuthenticatedDatagram,
    AuthorityDecision,
    BootEpoch,
    DTask,
    MessageType,
    MissionSelectionValue,
    MissionPhase,
    MissionStatusFlag,
    MissionStatusValue,
    ReceiptSeconds,
    RouteEvent,
    SelectionId,
    Sequence,
    Task3FcuAuxGate,
    Task3FlightTestIdentity,
    TaskMode,
    VehicleTelemetryValue,
)
from .payloads import (
    decode_car_telemetry,
    decode_task_selection,
)
from .protocol import decode_datagram
from .ros_mapping import (
    encode_mission_status_for_hmi,
    to_execute_goal,
    to_selection_request,
    to_stale_vehicle_message,
    to_vehicle_message,
)
from .ros_config import declare_bridge_provisioning
from .hmi_sender import HmiSender, HmiSenderConfig
from .session import PeerPolicy, SessionTracker
from .udp_socket import BoundUdpSocket, ReceivedDatagram


class VehicleBridgeNode(Node):
    """Own UDP transport and request mission actions without FCU authority."""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("vehicle_bridge", parameter_overrides=parameter_overrides)
        config = load_bridge_config(declare_bridge_provisioning(self))
        provision = config.provisioning
        self._task3_flight_test_mode = bool(self.get_parameter("task3_flight_test_mode").value)
        self._task3_identity = (
            Task3FlightTestIdentity(
                mission_id=str(self.get_parameter("task3_mission_id").value),
                field_profile_id=str(self.get_parameter("task3_field_profile_id").value),
                mission_profile_id=str(self.get_parameter("task3_mission_profile_id").value),
                deployment_preset_id=str(
                    self.get_parameter("task3_deployment_preset_id").value
                ),
                target_revision=str(self.get_parameter("task3_target_revision").value),
                timeout_seconds=float(self.get_parameter("task3_timeout_seconds").value),
            )
            if self._task3_flight_test_mode
            else None
        )
        self._key = config.hmac_key
        self._socket = BoundUdpSocket(provision.bind)
        self._hmi_sender = HmiSender(
            HmiSenderConfig(
                socket=self._socket,
                destination=provision.hmi_peer,
                sender_id=provision.bridge_sender_id,
                key=self._key,
            )
        )
        self._no_car_mode = bool(self.get_parameter("no_car_mode").value)
        self._immediate_start = bool(self.get_parameter("task3_immediate_start").value)
        self._authority = BridgeAuthority(
            provision.mission_timeout_seconds,
            no_car_mode=self._no_car_mode,
        )
        self._car_session = SessionTracker(
            PeerPolicy(provision.car_sender_id, provision.car_peer, frozenset({MessageType.CAR_TELEMETRY}))
        )
        self._hmi_session = SessionTracker(
            PeerPolicy(provision.hmi_sender_id, provision.hmi_peer, frozenset({MessageType.TASK_SELECTION}))
        )
        self._stale_seconds = 0.75
        self._fcu_armed = False
        self._mission_idle = False
        self._car_epoch = BootEpoch(0)
        self._last_telemetry: VehicleTelemetryValue | None = None
        self._last_sequence = Sequence(0)
        self._start_stamp = Time()
        self._callbacks = MutuallyExclusiveCallbackGroup()
        self._selection_completions: SimpleQueue[tuple[MissionSelectionValue, Future]] = SimpleQueue()

        self._vehicle_pub = self.create_publisher(VehicleTelemetry, "/d_task/vehicle/telemetry", 10)
        self._fcu_sub = self.create_subscription(
            FcuState,
            "/fcu/state",
            self._on_fcu_state,
            10,
            callback_group=self._callbacks,
        )
        self._status_sub = self.create_subscription(
            MissionStatus,
            "/d_task/mission_status",
            self._on_mission_status,
            10,
            callback_group=self._callbacks,
        )
        self._selection_client = self.create_client(
            SelectDTaskMission,
            "/d_task/pre_arm/select_mission",
            callback_group=self._callbacks,
        )
        self._mission_client = ActionClient(
            self,
            ExecuteMission,
            "/mission/execute",
            callback_group=self._callbacks,
        )
        self._udp_timer = self.create_timer(0.01, self._drain_udp, callback_group=self._callbacks)
        self._freshness_timer = self.create_timer(0.05, self._check_freshness, callback_group=self._callbacks)
        self.get_logger().info("vehicle_bridge.ready")

    def _guard(self, label: str, action: Callable[[], None]) -> None:
        """Run one callback body; any exception is logged and isolated.

        The ground-station communication module must NEVER exit on errors:
        a single bad datagram or transient ROS failure must not kill the
        process that keeps the UDP boundary alive.
        """
        try:
            action()
        except Exception:  # noqa: BLE001 - daemon contract: log and continue
            self.get_logger().error(f"{label} 异常(已隔离, 通信模块继续运行)", exc_info=True)

    def _drain_udp(self) -> None:
        while True:
            try:
                selection, future = self._selection_completions.get_nowait()
            except Empty:
                break
            self._guard(
                "selection_completion",
                lambda selection=selection, future=future: self._finish_selection(selection, future),
            )
        try:
            packets = self._socket.receive(32)
        except Exception:  # noqa: BLE001 - socket faults must not kill the daemon
            self.get_logger().error("udp.receive 异常(已隔离)", exc_info=True)
            packets = ()
        for packet in packets:
            self._guard("udp_packet", lambda packet=packet: self._handle_packet_safely(packet))

    def _handle_packet_safely(self, packet: ReceivedDatagram) -> None:
        try:
            self._handle_packet(packet)
        except ProtocolError as error:
            self.get_logger().warning(f"udp.reject code={error.code} detail={error}")

    def _handle_packet(self, packet: ReceivedDatagram) -> None:
        datagram = decode_datagram(packet.data, self._key)
        frame = datagram.frame
        receipt = ReceiptSeconds(time.monotonic())
        match frame.message_type:
            case MessageType.CAR_TELEMETRY:
                telemetry = decode_car_telemetry(frame.payload)
                accepted = self._car_session.accept(datagram, packet.source, receipt)
                if accepted.session_changed:
                    self._car_epoch = frame.boot_id
                    self._authority.observe_car_epoch(frame.boot_id, self._fcu_armed)
                self._publish_telemetry(telemetry, datagram)
                if telemetry.event is RouteEvent.START and not self._task3_flight_test_mode:
                    self._apply_start(self._authority.observe_car_start(frame.boot_id))
            case MessageType.TASK_SELECTION:
                selection = decode_task_selection(frame.payload)
                self._hmi_session.accept(datagram, packet.source, receipt)
                self._apply_selection(selection)
            case MessageType.HEARTBEAT | MessageType.DIAGNOSTIC:
                return
            case MessageType.MISSION_STATUS:
                raise ProtocolError(
                    ProtocolErrorCode.MESSAGE_TYPE_FORBIDDEN,
                    "bridge does not accept ROS-to-HMI message types",
                )
            case unreachable:
                assert_never(unreachable)

    def _apply_selection(self, selection: MissionSelectionValue) -> None:
        if not self._mission_idle:
            self._send_rejection(selection.selection_id, selection.car_boot_id, "MISSION_NOT_IDLE")
            return
        if selection.mode is TaskMode.SIMULATED:
            # 模拟飞 (no-car): 无 CAR 会话, 地面站选择即启动; 任务类型由 executor
            # 的 selection contract 校验, 此处不做 task3 专有 gate。
            self._car_epoch = selection.car_boot_id
        elif self._no_car_mode:
            # 兼容旧配置: 启动参数 no_car_mode=true 等价于始终模拟飞
            self._car_epoch = selection.car_boot_id
        elif self._task3_flight_test_mode:
            if selection.task is not DTask.STABILITY_TEST:
                self._send_rejection(selection.selection_id, selection.car_boot_id, "TASK3_SELECTION_REQUIRED")
                return
            epoch = self._authority.observe_car_epoch(selection.car_boot_id, self._fcu_armed)
            if not epoch.accepted:
                self._send_rejection(selection.selection_id, selection.car_boot_id, epoch.reason)
                return
            self._car_epoch = selection.car_boot_id
        decision = self._authority.request_selection(selection, self._fcu_armed)
        if decision.acknowledgement is not None:
            self._send_mission_status(decision.acknowledgement)
            return
        if decision.select_command is None:
            self._send_rejection(selection.selection_id, selection.car_boot_id, decision.reason)
            return
        if not self._selection_client.service_is_ready():
            self._authority.commit_selection(
                selection.selection_id,
                False,
                "SELECTION_SERVICE_UNAVAILABLE",
                self._fcu_armed,
            )
            self._send_rejection(selection.selection_id, selection.car_boot_id, "SELECTION_SERVICE_UNAVAILABLE")
            return
        future = self._selection_client.call_async(
            to_selection_request(selection, self._task3_identity)
        )
        future.add_done_callback(
            lambda completed: self._selection_completions.put((selection, completed))
        )

    def _finish_selection(self, selection: MissionSelectionValue, future: Future) -> None:
        failure = future.exception()
        if failure is not None:
            decision = self._authority.commit_selection(
                selection.selection_id, False, "SELECTION_SERVICE_FAILED", self._fcu_armed
            )
        else:
            response = future.result()
            decision = self._authority.commit_selection(
                selection.selection_id,
                bool(response.accepted),
                str(response.reason),
                self._fcu_armed,
            )
        if decision.acknowledgement is not None:
            self._send_mission_status(decision.acknowledgement)
        else:
            self._send_rejection(selection.selection_id, selection.car_boot_id, decision.reason)
        if (self._no_car_mode or selection.mode is TaskMode.SIMULATED or self._immediate_start) and decision.acknowledgement is not None:
            # 立即启动: 地面站 TASK 指令提交即开始任务 — 模拟飞 (no_car_sim 应答),
            # 或调试阶段 task3_immediate_start 开关 (跳过小车 START / AUX gate 等待)。
            start = self._authority.observe_no_car_start(self._task3_identity)
            if start.execute_command is not None:
                self._apply_start(start)

    def _apply_start(self, decision: AuthorityDecision) -> None:
        if decision.execute_command is None:
            self._send_rejection(SelectionId(0), self._car_epoch, decision.reason)
            return
        if not self._mission_client.server_is_ready():
            self.get_logger().error("mission.dispatch unavailable")
            self._send_rejection(SelectionId(0), self._car_epoch, "MISSION_ACTION_UNAVAILABLE")
            return
        self._mission_client.send_goal_async(to_execute_goal(decision.execute_command))
        self.get_logger().info("mission.dispatch requested")

    def _publish_telemetry(
        self,
        value: VehicleTelemetryValue,
        datagram: AuthenticatedDatagram,
    ) -> None:
        now = self.get_clock().now().to_msg()
        if value.start_event:
            self._start_stamp = now
        self._last_telemetry = value
        self._last_sequence = datagram.frame.sequence
        self._vehicle_pub.publish(to_vehicle_message(value, datagram, now, self._start_stamp))

    def _check_freshness(self) -> None:
        self._guard("freshness", self._check_freshness_impl)

    def _check_freshness_impl(self) -> None:
        fault = self._car_session.telemetry_fault_if_stale(
            ReceiptSeconds(time.monotonic()), self._stale_seconds
        )
        if fault is None or self._last_telemetry is None:
            return
        decision = self._authority.telemetry_fault()
        sequence = Sequence((self._last_sequence + 1) & 0xFFFFFFFF)
        message = to_stale_vehicle_message(
            self._last_telemetry, sequence, self.get_clock().now().to_msg(), self._start_stamp
        )
        self._vehicle_pub.publish(message)
        self._send_rejection(SelectionId(0), fault.car_boot_epoch, decision.reason)

    def _on_fcu_state(self, message: FcuState) -> None:
        self._guard("fcu_state", lambda: self._on_fcu_state_impl(message))

    def _on_fcu_state_impl(self, message: FcuState) -> None:
        self._fcu_armed = bool(message.motors_armed and message.communication_ok)
        if self._task3_flight_test_mode:
            if self._task3_identity is not None:
                decision = self._authority.observe_task3_flight_gate(
                    self._task3_identity,
                    Task3FcuAuxGate(
                        communication_fresh=bool(message.communication_ok),
                        motors_armed=bool(message.motors_armed),
                        channel_5_task_permission=bool(
                            message.task3_control_allowed
                        ),
                    ),
                )
                if decision.execute_command is not None:
                    self._apply_start(decision)
            return
        decision = self._authority.observe_arm(self._fcu_armed)
        if message.motors_armed and not decision.accepted:
            self._send_rejection(SelectionId(0), self._car_epoch, decision.reason)

    def _on_mission_status(self, message: MissionStatus) -> None:
        self._guard("mission_status", lambda: self._on_mission_status_impl(message))

    def _on_mission_status_impl(self, message: MissionStatus) -> None:
        self._mission_idle = message.state == MissionStatus.STATE_PRE_ARM and not message.complete
        self._hmi_sender.send(
            MessageType.MISSION_STATUS,
            encode_mission_status_for_hmi(
                message,
                self._authority.committed_selection
                if (self._task3_flight_test_mode or self._no_car_mode)
                else None,
            ),
        )

    def _send_rejection(
        self,
        selection_id: SelectionId,
        car_epoch: BootEpoch,
        reason: str,
    ) -> None:
        self._send_mission_status(
            MissionStatusValue(
                selection_id=selection_id,
                car_boot_id=car_epoch,
                hmi_boot_id=0,
                phase=MissionPhase.FAULT,
                selected_task=0,
                reason_flags=0,
                status_flags=MissionStatusFlag(0),
            )
        )

    def _send_mission_status(self, status: MissionStatusValue) -> None:
        try:
            self._hmi_sender.send(
                MessageType.MISSION_STATUS,
                encode_mission_status_for_hmi(status),
            )
        except Exception:  # noqa: BLE001 - a UDP send fault must not kill the daemon
            self.get_logger().error("hmi.send 异常(已隔离)", exc_info=True)

    def destroy_node(self) -> None:
        self._socket.close()
        super().destroy_node()
