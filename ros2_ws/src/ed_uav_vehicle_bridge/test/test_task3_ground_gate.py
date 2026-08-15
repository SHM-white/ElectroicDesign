"""Regression tests for the common D-task start path.

The historical file name is retained so downstream test selectors keep
working.  Task 3 no longer owns a separate FCU/AUX admission gate.
"""

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
    Task3FlightTestIdentity,
)
from ed_uav_vehicle_bridge.payloads import decode_mission_status
from ed_uav_vehicle_bridge.ros_mapping import (
    encode_mission_status_for_hmi,
    to_execute_goal,
    to_selection_request,
)


@pytest.fixture
def competition_identity() -> Task3FlightTestIdentity:
    return Task3FlightTestIdentity(
        mission_id="d-arena-competition-2026",
        field_profile_id="d-arena-2026",
        mission_profile_id="d2026-competition",
        deployment_preset_id="field-2026",
        target_revision="d2026-apriltag-v1",
        timeout_seconds=90.0,
    )


@pytest.mark.parametrize("task", tuple(DTask))
def test_every_d_task_maps_the_same_launch_loaded_identity(
    competition_identity: Task3FlightTestIdentity,
    task: DTask,
) -> None:
    selection = MissionSelectionValue(
        selection_id=SelectionId(70 + int(task)),
        car_boot_id=BootId(0x3A3A3A3A),
        task=task,
    )

    request = to_selection_request(selection, competition_identity)

    assert request.contract_version == request.CONTRACT_VERSION
    assert request.mission_id == competition_identity.mission_id
    assert request.field_profile_id == competition_identity.field_profile_id
    assert request.mission_profile_id == competition_identity.mission_profile_id
    assert request.deployment_preset_id == competition_identity.deployment_preset_id
    assert request.target_revision == competition_identity.target_revision
    assert request.task == int(task)


@pytest.mark.parametrize("task", tuple(DTask))
def test_every_real_task_uses_arm_then_car_start_without_aux_gate(
    competition_identity: Task3FlightTestIdentity,
    task: DTask,
) -> None:
    epoch = BootId(0x3A3A3A3A)
    selection = MissionSelectionValue(SelectionId(80 + int(task)), epoch, task)
    authority = BridgeAuthority()
    authority.observe_car_epoch(epoch, fcu_armed=False)
    authority.request_selection(selection, fcu_armed=False)
    committed = authority.commit_selection(
        selection.selection_id,
        accepted=True,
        reason="approved",
        fcu_armed=False,
    )

    armed = authority.observe_arm(True)
    first = authority.observe_car_start(epoch, competition_identity)
    replayed = authority.observe_car_start(epoch, competition_identity)
    goal = to_execute_goal(first.execute_command)

    assert committed.state is AuthorityState.SELECTED
    assert armed.state is AuthorityState.ARMED_READY
    assert (goal.mission_id, goal.field_profile_id, goal.timeout_sec) == (
        competition_identity.mission_id,
        competition_identity.field_profile_id,
        competition_identity.timeout_seconds,
    )
    assert replayed.execute_command is None


def test_task3_status_retains_committed_selection_context() -> None:
    selection = MissionSelectionValue(
        selection_id=SelectionId(73),
        car_boot_id=BootId(0x3A3A3A3A),
        task=DTask.STABILITY_TEST,
    )
    message = MissionStatus()
    message.contract_version = MissionStatus.CONTRACT_VERSION
    message.mission_id = "d-arena-competition-2026"
    message.state = MissionStatus.STATE_TAKEOFF
    message.complete = False

    status = decode_mission_status(encode_mission_status_for_hmi(message, selection))

    assert status.selection_id == selection.selection_id
    assert status.car_boot_id == selection.car_boot_id
    assert status.selected_task == int(DTask.STABILITY_TEST)
