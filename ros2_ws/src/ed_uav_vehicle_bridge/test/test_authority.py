from concurrent.futures import ThreadPoolExecutor

from ed_uav_vehicle_bridge.authority import BridgeAuthority
from ed_uav_vehicle_bridge.models import (
    AuthorityState,
    BootId,
    DTask,
    MissionSelectionValue,
    MissionStatusValue,
    RejectCode,
    SelectionId,
    Task3FlightTestIdentity,
)


EPOCH = BootId(100)
SELECTION = MissionSelectionValue(SelectionId(44), EPOCH, DTask.PAYLOAD_DROP)


def test_happy_select_ack_arm_start_dispatches_exactly_once() -> None:
    # Given: an unarmed FCU and a current authenticated car epoch.
    authority = BridgeAuthority(mission_timeout_seconds=90.0)
    prestart = authority.observe_car_epoch(EPOCH, fcu_armed=False)

    # When: selection, mission approval, external arm, and car start arrive in order.
    pending = authority.request_selection(SELECTION, fcu_armed=False)
    acknowledged = authority.commit_selection(SelectionId(44), True, "approved", False)
    armed = authority.observe_arm(True)
    started = authority.observe_car_start(EPOCH)
    replayed = authority.observe_car_start(EPOCH)

    # Then: ACK binds selection and epoch, and one action request is emitted.
    assert prestart.state is AuthorityState.PRESTART
    assert pending.select_command is not None
    assert acknowledged.acknowledgement is not None
    assert isinstance(acknowledged.acknowledgement, MissionStatusValue)
    assert acknowledged.acknowledgement.selection_id == SelectionId(44)
    assert acknowledged.acknowledgement.car_boot_id == EPOCH
    assert armed.state is AuthorityState.ARMED_READY
    assert started.execute_command is not None
    assert started.execute_command.mission_id == "d-task-payload_drop"
    assert started.execute_command.field_profile_id == "d-task-payload_drop"
    assert replayed.reason == RejectCode.START_ALREADY_CONSUMED


def test_car_start_dispatches_the_launch_loaded_mission_identity() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    authority.request_selection(SELECTION, fcu_armed=False)
    authority.commit_selection(SelectionId(44), True, "approved", False)
    authority.observe_arm(True)
    identity = Task3FlightTestIdentity(
        mission_id="d-arena-competition-2026",
        field_profile_id="d-arena-2026",
        mission_profile_id="d2026-competition",
        deployment_preset_id="field-2026",
        target_revision="d2026-apriltag-v1",
        timeout_seconds=120.0,
    )

    started = authority.observe_car_start(EPOCH, identity)

    assert started.execute_command is not None
    assert started.execute_command.mission_id == identity.mission_id
    assert started.execute_command.field_profile_id == identity.field_profile_id


def test_acknowledgement_tracks_current_hmi_boot_epoch() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    authority.observe_hmi_epoch(BootId(0xAABBCCDD))
    authority.request_selection(SELECTION, fcu_armed=False)

    acknowledged = authority.commit_selection(SelectionId(44), True, "approved", False)

    assert acknowledged.acknowledgement is not None
    assert acknowledged.acknowledgement.hmi_boot_id == BootId(0xAABBCCDD)


def test_start_without_committed_selection_has_exact_reason() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    rejected = authority.observe_car_start(EPOCH)
    assert rejected.accepted is False
    assert rejected.reason == RejectCode.NO_COMMITTED_SELECTION


def test_every_out_of_order_transition_rejects() -> None:
    authority = BridgeAuthority()
    no_session = authority.request_selection(SELECTION, fcu_armed=False)
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    early_arm = authority.observe_arm(True)
    armed_selection = authority.request_selection(SELECTION, fcu_armed=True)
    authority.observe_arm(False)
    authority.request_selection(SELECTION, fcu_armed=False)
    early_start = authority.observe_car_start(EPOCH)
    wrong_ack = authority.commit_selection(SelectionId(45), True, "approved", False)

    assert no_session.reason == RejectCode.NO_CAR_SESSION
    assert early_arm.reason == RejectCode.NO_COMMITTED_SELECTION
    assert armed_selection.reason == RejectCode.FCU_ALREADY_ARMED
    assert early_start.reason == RejectCode.NO_COMMITTED_SELECTION
    assert wrong_ack.reason == RejectCode.SELECTION_ID_MISMATCH


def test_reboot_invalidates_selection_and_requires_fresh_confirmation() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    authority.request_selection(SELECTION, fcu_armed=False)
    authority.commit_selection(SelectionId(44), True, "approved", False)

    reboot = authority.observe_car_epoch(BootId(200), fcu_armed=False)
    stale_start = authority.observe_car_start(BootId(200))

    assert reboot.state is AuthorityState.PRESTART
    assert stale_start.reason == RejectCode.NO_COMMITTED_SELECTION


def test_hmi_reconnect_can_reconfirm_identical_unarmed_selection_only() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    authority.request_selection(SELECTION, fcu_armed=False)
    authority.commit_selection(SelectionId(44), True, "approved", False)

    reconfirmed = authority.request_selection(SELECTION, fcu_armed=False)
    mutated = authority.request_selection(
        MissionSelectionValue(SelectionId(45), EPOCH, DTask.DYNAMIC_LANDING),
        fcu_armed=False,
    )

    assert reconfirmed.acknowledgement is not None
    assert reconfirmed.acknowledgement.selection_id == SelectionId(44)
    assert mutated.reason == RejectCode.SELECTION_ALREADY_COMMITTED


def test_start_race_produces_one_execute_command() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    authority.request_selection(SELECTION, fcu_armed=False)
    authority.commit_selection(SelectionId(44), True, "approved", False)
    authority.observe_arm(True)

    with ThreadPoolExecutor(max_workers=8) as pool:
        decisions = list(pool.map(authority.observe_car_start, [EPOCH] * 32))

    assert sum(decision.execute_command is not None for decision in decisions) == 1
    assert sum(decision.reason == RejectCode.START_ALREADY_CONSUMED for decision in decisions) == 31


def test_stale_telemetry_latches_typed_fault_without_dispatch() -> None:
    authority = BridgeAuthority()
    authority.observe_car_epoch(EPOCH, fcu_armed=False)
    fault = authority.telemetry_fault()
    assert fault.state is AuthorityState.FAULT
    assert fault.reason == RejectCode.TELEMETRY_STALE
    assert fault.execute_command is None
