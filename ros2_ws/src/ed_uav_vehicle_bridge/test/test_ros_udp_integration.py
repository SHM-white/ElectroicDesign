from pathlib import Path
import socket
from threading import Event, Thread
import time

import pytest


rclpy = pytest.importorskip("rclpy")

from ed_uav_interfaces.action import ExecuteMission
from ed_uav_interfaces.msg import FcuState, MissionStatus, VehicleTelemetry
from ed_uav_interfaces.srv import SelectDTaskMission
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from ed_uav_vehicle_bridge.models import (
    AuthorityState,
    BootEpoch,
    DTask,
    MessageType,
    MissionSelectionValue,
    MotionKind,
    OutboundFrame,
    RouteStage,
    SelectionId,
    Sequence,
    SourceMillis,
    TurnClass,
    VehicleTelemetryValue,
)
from ed_uav_vehicle_bridge.node import VehicleBridgeNode
from ed_uav_vehicle_bridge.payloads import (
    decode_selection_ack,
    encode_mission_selection,
    encode_vehicle_telemetry,
)
from ed_uav_vehicle_bridge.protocol import decode_datagram, encode_datagram


KEY = bytes(range(32))
CAR_EPOCH = BootEpoch(0x12345678)


class FakeMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_mission_backend")
        self.armed = False
        self.goal_count = 0
        self.goal_event = Event()
        self.stale_event = Event()
        self.telemetry_count = 0
        self._selection = self.create_service(
            SelectDTaskMission,
            "/d_task/pre_arm/select_mission",
            self._select,
        )
        self._action = ActionServer(
            self,
            ExecuteMission,
            "/mission/execute",
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            execute_callback=self._execute,
        )
        self._fcu = self.create_publisher(FcuState, "/fcu/state", 10)
        self._status = self.create_publisher(MissionStatus, "/d_task/mission_status", 10)
        self._telemetry = self.create_subscription(
            VehicleTelemetry,
            "/d_task/vehicle/telemetry",
            self._on_telemetry,
            10,
        )
        self._timer = self.create_timer(0.02, self._publish_state)

    def _select(self, request, response):
        response.accepted = request.contract_version == 1
        response.contract_version = 1
        response.reason = "approved"
        return response

    async def _execute(self, goal_handle):
        self.goal_count += 1
        self.goal_event.set()
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.result_code = ExecuteMission.Result.RESULT_SUCCEEDED
        result.reason = "fake accepted"
        return result

    def _publish_state(self) -> None:
        fcu = FcuState()
        fcu.motors_armed = self.armed
        fcu.communication_ok = True
        self._fcu.publish(fcu)
        status = MissionStatus()
        status.contract_version = 1
        status.source_sequence = self.telemetry_count
        status.mission_id = "mission-44"
        status.state = MissionStatus.STATE_PRE_ARM
        status.route_stage = RouteStage.START
        status.complete = False
        status.reason = "idle"
        self._status.publish(status)

    def _on_telemetry(self, message: VehicleTelemetry) -> None:
        self.telemetry_count += 1
        if not message.heartbeat_alive:
            self.stale_event.set()


class ObservedBridge(VehicleBridgeNode):
    def __init__(self, overrides: list[Parameter]) -> None:
        self.idle_event = Event()
        self.arm_event = Event()
        super().__init__(parameter_overrides=overrides)

    def _on_mission_status(self, message: MissionStatus) -> None:
        super()._on_mission_status(message)
        if message.state == MissionStatus.STATE_PRE_ARM:
            self.idle_event.set()

    def _on_fcu_state(self, message: FcuState) -> None:
        super()._on_fcu_state(message)
        if message.motors_armed and message.communication_ok:
            self.arm_event.set()


def _frame(
    message_type: MessageType,
    sender_id: str,
    sequence: int,
    payload: bytes,
) -> bytes:
    return encode_datagram(
        OutboundFrame(
            message_type,
            sender_id,
            CAR_EPOCH,
            Sequence(sequence),
            SourceMillis(sequence * 50),
            payload,
        ),
        KEY,
    )


def _telemetry(start: bool) -> bytes:
    return encode_vehicle_telemetry(
        VehicleTelemetryValue(
            1,
            "car-1",
            start,
            True,
            MotionKind.DISPLACEMENT,
            0.1,
            0.2,
            0.0,
            0.0,
            TurnClass.STRAIGHT,
            RouteStage.START,
            False,
            "vehicle_start",
        )
    )


def _receive_ack(hmi: socket.socket):
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            raw, _ = hmi.recvfrom(512)
        except socket.timeout:
            continue
        datagram = decode_datagram(raw, KEY)
        if datagram.frame.message_type is MessageType.SELECTION_ACK:
            return decode_selection_ack(datagram.frame.payload)
    raise AssertionError("authenticated selection ACK not received before deadline")


def test_real_udp_ros_select_arm_start_replay_stale_and_cleanup(tmp_path: Path) -> None:
    key_file = tmp_path / "udp.key"
    key_file.write_text(KEY.hex(), encoding="ascii")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as car, socket.socket(
        socket.AF_INET, socket.SOCK_DGRAM
    ) as hmi:
        car.bind(("127.0.0.1", 0))
        hmi.bind(("127.0.0.1", 0))
        hmi.settimeout(0.1)
        car_port = int(car.getsockname()[1])
        hmi_port = int(hmi.getsockname()[1])
        overrides = [
            Parameter("bind_host", value="127.0.0.1"),
            Parameter("bind_port", value=0),
            Parameter("car_peer_host", value="127.0.0.1"),
            Parameter("car_peer_port", value=car_port),
            Parameter("hmi_peer_host", value="127.0.0.1"),
            Parameter("hmi_peer_port", value=hmi_port),
            Parameter("car_sender_id", value="CAR-01"),
            Parameter("hmi_sender_id", value="HMI-01"),
            Parameter("bridge_sender_id", value="ROS-01"),
            Parameter("hmac_key_file", value=str(key_file)),
            Parameter("telemetry_stale_seconds", value=0.1),
        ]

        rclpy.init()
        fake = FakeMissionNode()
        bridge = ObservedBridge(overrides)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(fake)
        executor.add_node(bridge)
        spin = Thread(target=executor.spin, daemon=True)
        spin.start()
        bridge_endpoint = (bridge._socket.endpoint.host, bridge._socket.endpoint.port)
        descriptor = bridge._socket.fileno()
        try:
            assert bridge.idle_event.wait(2.0)
            assert bridge._selection_client.wait_for_service(timeout_sec=2.0)
            assert bridge._mission_client.wait_for_server(timeout_sec=2.0)

            malformed = b"bad"
            car.sendto(malformed, bridge_endpoint)
            car.sendto(_frame(MessageType.CAR_TELEMETRY, "CAR-01", 0, _telemetry(False)), bridge_endpoint)

            selection = MissionSelectionValue(
                1,
                SelectionId(44),
                CAR_EPOCH,
                "mission-44",
                "profile-44",
                "field-44",
                "target-v1",
                DTask.PAYLOAD_DROP,
            )
            hmi.sendto(
                _frame(MessageType.HMI_SELECTION, "HMI-01", 0, encode_mission_selection(selection)),
                bridge_endpoint,
            )
            ack = _receive_ack(hmi)
            assert ack.accepted is True
            assert ack.state is AuthorityState.SELECTED

            fake.armed = True
            assert bridge.arm_event.wait(2.0)
            start_packet = _frame(MessageType.CAR_TELEMETRY, "CAR-01", 1, _telemetry(True))
            car.sendto(start_packet, bridge_endpoint)
            assert fake.goal_event.wait(2.0)
            assert fake.goal_count == 1

            car.sendto(start_packet, bridge_endpoint)
            assert fake.stale_event.wait(2.0)
            assert fake.goal_count == 1
            assert fake.telemetry_count >= 3
        finally:
            executor.shutdown(timeout_sec=2.0)
            bridge.destroy_node()
            fake.destroy_node()
            spin.join(timeout=2.0)
            rclpy.shutdown()

        assert spin.is_alive() is False
        assert bridge._socket.fileno() == -1
        assert descriptor >= 0
