"""ROS 2 adapter for the authenticated vehicle/HMI UDP boundary."""

from __future__ import annotations

import time
from queue import Empty, SimpleQueue

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
    AuthorityState,
    BootEpoch,
    MessageType,
    MissionSelectionValue,
    ReceiptSeconds,
    SelectionAckValue,
    SelectionId,
    Sequence,
    VehicleTelemetryValue,
)
from .payloads import (
    decode_mission_selection,
    decode_vehicle_telemetry,
    encode_mission_status,
    encode_selection_ack,
)
from .protocol import decode_datagram
from .ros_mapping import (
    from_mission_status,
    to_execute_goal,
    to_selection_request,
    to_stale_vehicle_message,
    to_vehicle_message,
)
from .ros_config import declare_bridge_provisioning
from .hmi_sender import HmiSender, HmiSenderConfig
from .session import PeerPolicy, RouteTracker, SessionTracker
from .udp_socket import BoundUdpSocket, ReceivedDatagram


class VehicleBridgeNode(Node):
    """Own UDP transport and request mission actions without FCU authority."""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("vehicle_bridge", parameter_overrides=parameter_overrides)
        config = load_bridge_config(declare_bridge_provisioning(self))
        provision = config.provisioning
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
        self._authority = BridgeAuthority(provision.mission_timeout_seconds)
        self._route = RouteTracker()
        self._car_session = SessionTracker(
            PeerPolicy(provision.car_sender_id, provision.car_peer, frozenset({MessageType.CAR_TELEMETRY}))
        )
        self._hmi_session = SessionTracker(
            PeerPolicy(provision.hmi_sender_id, provision.hmi_peer, frozenset({MessageType.HMI_SELECTION}))
        )
        self._stale_seconds = provision.telemetry_stale_seconds
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

    def _drain_udp(self) -> None:
        while True:
            try:
                selection, future = self._selection_completions.get_nowait()
            except Empty:
                break
            self._finish_selection(selection, future)
        for packet in self._socket.receive(32):
            try:
                self._handle_packet(packet)
            except ProtocolError as error:
                self.get_logger().warning(f"udp.reject code={error.code}")

    def _handle_packet(self, packet: ReceivedDatagram) -> None:
        datagram = decode_datagram(packet.data, self._key)
        frame = datagram.frame
        receipt = ReceiptSeconds(time.monotonic())
        match frame.message_type:
            case MessageType.CAR_TELEMETRY:
                telemetry = decode_vehicle_telemetry(frame.payload)
                accepted = self._car_session.accept(datagram, packet.source, receipt)
                if accepted.session_changed:
                    self._route.reset()
                    self._car_epoch = frame.boot_epoch
                    self._authority.observe_car_epoch(frame.boot_epoch, self._fcu_armed)
                self._route.accept(telemetry)
                self._publish_telemetry(telemetry, datagram)
                if telemetry.start_event:
                    self._apply_start(self._authority.observe_car_start(frame.boot_epoch))
            case MessageType.HMI_SELECTION:
                selection = decode_mission_selection(frame.payload)
                self._hmi_session.accept(datagram, packet.source, receipt)
                self._apply_selection(selection)
            case MessageType.SELECTION_ACK | MessageType.MISSION_STATUS:
                raise ProtocolError(
                    ProtocolErrorCode.MESSAGE_TYPE_FORBIDDEN,
                    "bridge does not accept ROS-to-HMI message types",
                )

    def _apply_selection(self, selection: MissionSelectionValue) -> None:
        if not self._mission_idle:
            self._send_rejection(selection.selection_id, selection.car_boot_epoch, "MISSION_NOT_IDLE")
            return
        decision = self._authority.request_selection(selection, self._fcu_armed)
        if decision.acknowledgement is not None:
            self._send_ack(decision.acknowledgement)
            return
        if decision.select_command is None:
            self._send_rejection(selection.selection_id, selection.car_boot_epoch, decision.reason)
            return
        if not self._selection_client.service_is_ready():
            self._authority.commit_selection(
                selection.selection_id,
                False,
                "SELECTION_SERVICE_UNAVAILABLE",
                self._fcu_armed,
            )
            self._send_rejection(selection.selection_id, selection.car_boot_epoch, "SELECTION_SERVICE_UNAVAILABLE")
            return
        future = self._selection_client.call_async(to_selection_request(selection))
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
            self._send_ack(decision.acknowledgement)
        else:
            self._send_rejection(selection.selection_id, selection.car_boot_epoch, decision.reason)

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
        self._fcu_armed = bool(message.motors_armed and message.communication_ok)
        decision = self._authority.observe_arm(self._fcu_armed)
        if message.motors_armed and not decision.accepted:
            self._send_rejection(SelectionId(0), self._car_epoch, decision.reason)

    def _on_mission_status(self, message: MissionStatus) -> None:
        self._mission_idle = message.state == MissionStatus.STATE_PRE_ARM and not message.complete
        self._hmi_sender.send(
            MessageType.MISSION_STATUS,
            encode_mission_status(from_mission_status(message)),
        )

    def _send_rejection(
        self,
        selection_id: SelectionId,
        car_epoch: BootEpoch,
        reason: str,
    ) -> None:
        self._send_ack(SelectionAckValue(1, selection_id, car_epoch, False, AuthorityState.FAULT, reason))

    def _send_ack(self, acknowledgement: SelectionAckValue) -> None:
        self._hmi_sender.send(
            MessageType.SELECTION_ACK,
            encode_selection_ack(acknowledgement),
        )

    def destroy_node(self) -> None:
        self._socket.close()
        super().destroy_node()
