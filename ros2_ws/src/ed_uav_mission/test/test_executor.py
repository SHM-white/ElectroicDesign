"""Tests for the mission executor preflight validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from ed_uav_interfaces.msg import FcuState
from ed_uav_mission.executor import PreflightCode, bounded_failure_reason, validate_preflight

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_executor_preflight_has_no_aux_or_capability_admission_parameters() -> None:
    """Deployment owns caller admission; mission preflight owns flight state."""
    import inspect

    parameters = inspect.signature(validate_preflight).parameters

    assert "aux_start_active" not in parameters
    assert "capability_ready" not in parameters


def test_preflight_rejects_unarmed_fcu() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_source=FcuState.SOURCE_V7,
        fcu_motors_armed=False,
        simulation_only=False,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.MOTORS_NOT_ARMED


def test_preflight_rejects_no_fcu_link() -> None:
    result = validate_preflight(
        fcu_communication_ok=False,
        fcu_source=0,
        fcu_motors_armed=True,
        simulation_only=False,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.NO_FCU_LINK


def test_preflight_rejects_lost_localization() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_source=FcuState.SOURCE_V7,
        fcu_motors_armed=True,
        simulation_only=False,
        localization_active=False,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.LOCALIZATION_LOST


def test_preflight_rejects_invalid_map_to_odom() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_source=FcuState.SOURCE_V7,
        fcu_motors_armed=True,
        simulation_only=False,
        localization_active=True,
        map_to_odom_valid=False,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.LOCALIZATION_LOST


def test_preflight_rejects_missing_profile() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_source=FcuState.SOURCE_V7,
        fcu_motors_armed=True,
        simulation_only=False,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=False,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.PROFILE_INVALID


def test_preflight_all_clear() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_source=FcuState.SOURCE_V7,
        fcu_motors_armed=True,
        simulation_only=False,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.OK


@pytest.mark.parametrize(
    ("simulation_only", "fcu_source"),
    [
        (True, FcuState.SOURCE_V7),
        (False, FcuState.SOURCE_SIMULATOR),
    ],
)
def test_preflight_rejects_execution_mode_source_mismatch(
    simulation_only: bool,
    fcu_source: int,
) -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_source=fcu_source,
        fcu_motors_armed=True,
        simulation_only=simulation_only,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )

    assert result.code == PreflightCode.FCU_SOURCE_MISMATCH


def test_failure_reason_fits_action_boundary() -> None:
    error = ValueError("x" * 120 + "\nsecondary detail")

    reason = bounded_failure_reason(error)

    assert reason == "x" * 96


def test_empty_failure_reason_uses_exception_type() -> None:
    assert bounded_failure_reason(RuntimeError()) == "RuntimeError"


def test_d_task_ros_surface_uses_typed_inputs_status_and_selection_service() -> None:
    # Given: the production executor and its split D-task ROS boundary.
    executor_source = (PACKAGE_ROOT / "ed_uav_mission" / "executor.py").read_text(
        encoding="utf-8"
    )
    boundary_path = PACKAGE_ROOT / "ed_uav_mission" / "d_task_ros.py"

    # When/Then: all external state crosses typed ROS contracts owned by mission.
    assert boundary_path.is_file()
    boundary_source = boundary_path.read_text(encoding="utf-8")
    for contract in (
        "VehicleTelemetry",
        "TargetObservation",
        "PayloadContactState",
        "MissionStatus",
        "SelectDTaskMission",
    ):
        assert contract in boundary_source
    assert '"/vehicle/telemetry"' in boundary_source
    # 目标观测话题带 d_task 前缀 (感知节点发布 /d_task/target_observation)
    assert '"/d_task/target_observation"' in boundary_source
    assert '"/payload/contact_state"' in boundary_source
    assert '"/mission/status"' in boundary_source
    assert '"/mission/select_d_task"' in boundary_source
    assert "DTaskRosBoundary(" in executor_source


def test_field_d_task_preflight_delegates_network_admission_to_deployment() -> None:
    # Given: the mission runtime and retained compatibility adapter.
    executor_source = (PACKAGE_ROOT / "ed_uav_mission" / "executor.py").read_text(
        encoding="utf-8"
    )
    capability_path = PACKAGE_ROOT / "ed_uav_mission" / "d_task_capability.py"

    # When/Then: neither AUX nor capability/SROS state participates in mission
    # preflight; older diagnostics receive an explicit delegated decision.
    assert capability_path.is_file()
    capability_source = capability_path.read_text(encoding="utf-8")
    assert "capability admission delegated to deployment" in capability_source
    assert "CAPABILITY_BLOCKED" not in executor_source
    assert "capability_ready" not in executor_source
    assert "aux_start_active" not in executor_source
