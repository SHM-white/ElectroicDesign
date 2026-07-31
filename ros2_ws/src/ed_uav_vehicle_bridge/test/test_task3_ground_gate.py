import pytest


pytest.importorskip("rclpy")

from ed_uav_interfaces.msg import MissionStatus

from ed_uav_vehicle_bridge.authority import BridgeAuthority
from ed_uav_vehicle_bridge.models import (
    AuthorityState,
    BootId,
    DTask,
    MissionSelectionValue,
    SelectionId,
    Task3FcuAuxGate,
    Task3FlightTestIdentity,
)
from ed_uav_vehicle_bridge.payloads import decode_mission_status
from ed_uav_vehicle_bridge.ros_mapping import (
    encode_mission_status_for_hmi,
    to_execute_goal,
    to_selection_request,
)


@pytest.fixture
def task3_identity() -> Task3FlightTestIdentity:
    return Task3FlightTestIdentity(
        mission_id="flight-test-stability-2026",
        field_profile_id="flight-test-field-2026",
        mission_profile_id="flight-test-profile-2026",
        deployment_preset_id="flight-test-preset-2026",
        target_revision="d2026-circle-cross-v1",
        timeout_seconds=120.0,
    )


@pytest.fixture
def task3_selection() -> MissionSelectionValue:
    return MissionSelectionValue(
        selection_id=SelectionId(73),
        car_boot_id=BootId(0x3A3A3A3A),
        task=DTask.STABILITY_TEST,
    )


@pytest.fixture
def committed_task3_authority(
    task3_identity: Task3FlightTestIdentity,
    task3_selection: MissionSelectionValue,
) -> BridgeAuthority:
    authority = BridgeAuthority(mission_timeout_seconds=task3_identity.timeout_seconds)
    authority.observe_car_epoch(task3_selection.car_boot_id, fcu_armed=False)
    pending = authority.request_selection(task3_selection, fcu_armed=False)
    committed = authority.commit_selection(
        task3_selection.selection_id,
        accepted=True,
        reason="approved",
        fcu_armed=False,
    )

    assert pending.select_command is not None
    assert committed.state is AuthorityState.SELECTED
    return authority


def test_task3_select_request_maps_configured_hmi_identity(
    task3_identity: Task3FlightTestIdentity,
    task3_selection: MissionSelectionValue,
) -> None:
    # Given: a configured Task 3 flight-test identity and HMI selection.
    # When: the bridge maps the selection to the mission-owned service request.
    request = to_selection_request(task3_selection, task3_identity)

    # Then: every service identity field and Task 3 reach the service unchanged.
    assert (
        request.contract_version,
        request.mission_id,
        getattr(request, "field_profile_id"),
        request.mission_profile_id,
        request.deployment_preset_id,
        request.target_revision,
        request.task,
    ) == (
        request.CONTRACT_VERSION,
        task3_identity.mission_id,
        task3_identity.field_profile_id,
        task3_identity.mission_profile_id,
        task3_identity.deployment_preset_id,
        task3_identity.target_revision,
        int(DTask.STABILITY_TEST),
    )


def test_task3_fcu_aux_gate_dispatches_configured_goal_once_without_car_start(
    committed_task3_authority: BridgeAuthority,
    task3_identity: Task3FlightTestIdentity,
) -> None:
    # Given: Task 3 selection committed before an FCU/AUX gate becomes ready.
    gate = Task3FcuAuxGate(
        communication_fresh=True,
        motors_armed=True,
        channel_5_task_permission=True,
    )

    # When: the fresh FCU, armed-motor, and channel-5 gate is observed twice.
    observe_gate = getattr(committed_task3_authority, "observe_task3_flight_gate")
    first = observe_gate(task3_identity, gate)
    replayed = observe_gate(task3_identity, gate)
    goal = to_execute_goal(first.execute_command)

    # Then: it directly emits one configured ExecuteMission goal and consumes the gate.
    assert (goal.mission_id, goal.field_profile_id, goal.timeout_sec) == (
        task3_identity.mission_id,
        task3_identity.field_profile_id,
        task3_identity.timeout_seconds,
    )
    assert replayed.execute_command is None


@pytest.mark.parametrize(
    "gate",
    (
        Task3FcuAuxGate(False, True, True),
        Task3FcuAuxGate(True, False, True),
        Task3FcuAuxGate(True, True, False),
    ),
    ids=("stale_fcu_communication", "motors_not_armed", "channel_5_not_permitted"),
)
def test_task3_fcu_aux_gate_requires_every_binary_permission(
    committed_task3_authority: BridgeAuthority,
    task3_identity: Task3FlightTestIdentity,
    gate: Task3FcuAuxGate,
) -> None:
    # Given: Task 3 selection is committed while one FCU/AUX permission is absent.
    # When: the Task 3 flight-test gate is evaluated without a car start event.
    observe_gate = getattr(committed_task3_authority, "observe_task3_flight_gate")
    decision = observe_gate(task3_identity, gate)

    # Then: no ExecuteMission goal is emitted.
    assert decision.execute_command is None


@pytest.mark.parametrize("task", (DTask.PAYLOAD_DROP, DTask.DYNAMIC_LANDING))
def test_task3_fcu_aux_gate_never_dispatches_competition_tasks(
    task: DTask,
    task3_identity: Task3FlightTestIdentity,
) -> None:
    # Given: a generic competition selection committed under explicit Task 3 mode.
    selection = MissionSelectionValue(
        selection_id=SelectionId(80 + int(task)),
        car_boot_id=BootId(0x3A3A3A3A),
        task=task,
    )
    authority = BridgeAuthority(mission_timeout_seconds=task3_identity.timeout_seconds)
    authority.observe_car_epoch(selection.car_boot_id, fcu_armed=False)
    authority.request_selection(selection, fcu_armed=False)
    committed = authority.commit_selection(selection.selection_id, True, "approved", False)
    gate = Task3FcuAuxGate(True, True, True)
    assert committed.state is AuthorityState.SELECTED

    # When: only the Task 3 FCU/AUX gate is observed.
    observe_gate = getattr(authority, "observe_task3_flight_gate")
    decision = observe_gate(task3_identity, gate)

    # Then: Task 1 and Task 2 retain their ordinary car-start dispatch behavior.
    assert decision.execute_command is None


def test_task3_subsequent_mission_status_retains_committed_selection_context(
    committed_task3_authority: BridgeAuthority,
    task3_selection: MissionSelectionValue,
) -> None:
    # Given: Task 3 selection has committed and ROS reports a later mission state.
    message = MissionStatus()
    message.contract_version = MissionStatus.CONTRACT_VERSION
    message.mission_id = "flight-test-stability-2026"
    message.state = MissionStatus.STATE_TAKEOFF
    message.complete = False

    # When: the later status is mapped for the HMI using the committed selection.
    payload = encode_mission_status_for_hmi(message, task3_selection)
    status = decode_mission_status(payload)

    # Then: the HMI retains the selected ID, car epoch, and Task 3 across updates.
    assert (
        status.selection_id,
        status.car_boot_id,
        status.selected_task,
    ) == (
        task3_selection.selection_id,
        task3_selection.car_boot_id,
        int(DTask.STABILITY_TEST),
    )
