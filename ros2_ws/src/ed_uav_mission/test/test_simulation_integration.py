from __future__ import annotations

from pathlib import Path
import sys

import pytest
import rclpy
from rclpy import _rclpy_pybind11
from rclpy.action import GoalResponse

from ed_uav_interfaces.action import ExecuteMission
from ed_uav_description.calibration import CalibrationError, load_calibration
from ed_uav_mission.state_machine import MissionState

from ed_uav_mission.mission_config import (
    calibration_file_is_valid,
    load_mission_bundle,
    parse_mission_config_text,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT.parent / "ed_uav_localization"))
PROFILE_PATH = (
    PACKAGE_ROOT.parent
    / "ed_uav_localization"
    / "config"
    / "fields"
    / "simulation_arena.yaml"
)
MISSION_PATH = PACKAGE_ROOT / "config" / "missions" / "simulation_patrol.yaml"
SYNTHETIC_CALIBRATION = (
    PACKAGE_ROOT.parent
    / "ed_uav_description"
    / "config"
    / "synthetic_calibrated.yaml"
)
UNCALIBRATED_CALIBRATION = (
    PACKAGE_ROOT.parent
    / "ed_uav_description"
    / "config"
    / "example_uncalibrated.yaml"
)


def test_simulation_bundle_loads_only_with_simulation_permission() -> None:
    with pytest.raises(ValueError, match="blocked"):
        load_mission_bundle(PROFILE_PATH, MISSION_PATH)

    bundle = load_mission_bundle(PROFILE_PATH, MISSION_PATH, allow_blocked_profile=True)

    assert bundle.profile.profile_id == "simulation-arena"
    assert bundle.profile.provenance.classification == "synthetic_simulation"
    assert bundle.mission.mission_id == "simulation-patrol"
    assert bundle.mission.patrol is not None
    assert bundle.mission.patrol.speed_m_s == 0.6
    assert [(waypoint.x_m, waypoint.y_m) for waypoint in bundle.mission.patrol.waypoints] == [
        (-1.0, -1.0),
        (1.0, -1.0),
        (1.0, 1.0),
        (-1.0, 1.0),
    ]


def test_mission_setup_exposes_executor_entrypoint() -> None:
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "mission_executor = ed_uav_mission.executor:main" in setup_text


def test_mission_declares_description_runtime_dependency() -> None:
    package_text = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>ed_uav_description</exec_depend>" in package_text


def test_mission_launch_preserves_public_action_topics() -> None:
    launch_text = (
        PACKAGE_ROOT / "launch" / "mission_executor.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="mission_executor"' in launch_text
    assert '"use_sim_time": use_sim_time' in launch_text
    assert "profile_path" in launch_text
    assert "mission_config_path" in launch_text
    assert 'calibration_file = LaunchConfiguration("calibration_file")' in launch_text
    assert '"calibration_file": calibration_file' in launch_text
    assert '"/mission/execute"' in (PACKAGE_ROOT / "ed_uav_mission" / "executor.py").read_text(
        encoding="utf-8"
    )
    assert '"/fcu/flight_command"' in (PACKAGE_ROOT / "ed_uav_mission" / "executor.py").read_text(
        encoding="utf-8"
    )


def test_simulator_forwards_bringup_calibration_to_mission_executor() -> None:
    # Given: the simulator launch owns one synthetic calibration path.
    launch_text = (
        PACKAGE_ROOT.parent / "ed_uav_gazebo" / "launch" / "sim.launch.py"
    ).read_text(encoding="utf-8")

    # When: its bringup and mission include arguments are inspected.
    calibration_forwarding = '"calibration_file": str(calibration)'

    # Then: both includes receive the same existing synthetic record.
    assert 'calibration = description_share / "config" / "synthetic_calibrated.yaml"' in launch_text
    assert launch_text.count(calibration_forwarding) == 2


def test_calibration_boundary_accepts_valid_synthetic_record() -> None:
    # Given: the hash-bound synthetic record used by simulator bringup.
    calibration_path = SYNTHETIC_CALIBRATION

    # When: the mission boundary validates the record.
    calibration_valid = calibration_file_is_valid(calibration_path, simulation_only=True)

    # Then: structurally valid calibration permits the preflight input.
    assert calibration_valid is True


def test_calibration_boundary_rejects_synthetic_record_outside_simulation() -> None:
    # Given: the structurally valid synthetic simulator calibration.
    calibration_path = SYNTHETIC_CALIBRATION

    # When: a non-simulation mission validates the record.
    calibration_valid = calibration_file_is_valid(calibration_path, simulation_only=False)

    # Then: synthetic calibration cannot authorize a competition-capable mission.
    assert calibration_valid is False


def test_calibration_boundary_rejects_uncalibrated_record_for_hardware() -> None:
    calibration_valid = calibration_file_is_valid(
        UNCALIBRATED_CALIBRATION,
        simulation_only=False,
    )

    assert calibration_valid is False


def test_calibration_boundary_rejects_missing_record(tmp_path: Path) -> None:
    # Given: a calibration path with no file.
    calibration_path = tmp_path / "missing.yaml"

    # When: the mission boundary validates the path.
    calibration_valid = calibration_file_is_valid(calibration_path)

    # Then: the calibration preflight input is false.
    assert calibration_valid is False


def test_calibration_boundary_rejects_malformed_record(tmp_path: Path) -> None:
    # Given: a calibration file containing malformed YAML.
    calibration_path = tmp_path / "malformed.yaml"
    calibration_path.write_text("calibration_status: [", encoding="utf-8")

    # When: the mission boundary validates the file.
    calibration_valid = calibration_file_is_valid(calibration_path)

    # Then: the calibration preflight input is false.
    assert calibration_valid is False


def test_calibration_boundary_rejects_hash_invalid_record(tmp_path: Path) -> None:
    # Given: the simulator record with a stale content hash.
    calibration_path = tmp_path / "stale-hash.yaml"
    calibration_path.write_text(
        SYNTHETIC_CALIBRATION.read_text(encoding="utf-8").replace(
            "e1ec326500451dc318cc55568cbc4f4f1247fe24fd9fb619577c36455310b37c",
            "stale-hash",
        ),
        encoding="utf-8",
    )

    # When: the mission boundary validates the file.
    calibration_valid = calibration_file_is_valid(calibration_path)

    # Then: the calibration preflight input is false.
    assert calibration_valid is False


def test_mission_yaml_rejects_duplicate_keys() -> None:
    source = MISSION_PATH.read_text(encoding="utf-8") + "\nmission_id: simulation-patrol\n"

    with pytest.raises(ValueError, match="duplicate YAML key: mission_id"):
        parse_mission_config_text(source)


def test_calibration_yaml_rejects_duplicate_keys(tmp_path: Path) -> None:
    path = tmp_path / "duplicate-calibration.yaml"
    path.write_text(
        SYNTHETIC_CALIBRATION.read_text(encoding="utf-8").replace(
            "calibration_status: SYNTHETIC",
            "calibration_status: SYNTHETIC\ncalibration_status: SYNTHETIC",
        ),
        encoding="utf-8",
    )

    with pytest.raises(CalibrationError, match="duplicate YAML key: calibration_status"):
        load_calibration(path)


def test_mission_executor_accepts_sim_time_override() -> None:
    # Given: the complete synthetic simulation bundle is passed as ROS parameters.
    from ed_uav_mission.executor import MissionExecutorNode

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"profile_path:={PROFILE_PATH}",
            "-p",
            f"mission_config_path:={MISSION_PATH}",
            "-p",
            f"calibration_file:={SYNTHETIC_CALIBRATION}",
            "-p",
            "simulation_only:=true",
        ]
    )
    node = None
    try:
        # When: the node is constructed under the same parameter overrides as Gazebo.
        node = MissionExecutorNode()

        # Then: TimeSource owns the parameter and the synthetic calibration is accepted.
        assert node.get_parameter("use_sim_time").value is True
        assert node._calibration_valid is True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_mission_main_cleans_up_invalid_context_shutdown(monkeypatch) -> None:
    # Given: Humble invalidates the context while the mission node is spinning.
    from ed_uav_mission import executor

    class FakeNode:
        destroyed = False

        def destroy_node(self) -> None:
            self.destroyed = True

    node = FakeNode()
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(executor, "MissionExecutorNode", lambda: node)
    monkeypatch.setattr(executor.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(executor.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        executor.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(
            _rclpy_pybind11.RCLError("context is not valid")
        ),
    )
    monkeypatch.setattr(executor.rclpy, "try_shutdown", lambda: shutdown_calls.append(True))

    # When: signal shutdown invalidates the context during spin.
    executor.main()

    # Then: the process exits normally and releases its node and context.
    assert node.destroyed
    assert shutdown_calls == [True]


@pytest.mark.parametrize("terminal_state", [MissionState.COMPLETE, MissionState.ABORTED])
def test_mission_executor_resets_terminal_state_before_next_goal(
    terminal_state: MissionState,
) -> None:
    from ed_uav_mission.executor import MissionExecutorNode

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"profile_path:={PROFILE_PATH}",
            "-p",
            f"mission_config_path:={MISSION_PATH}",
            "-p",
            f"calibration_file:={SYNTHETIC_CALIBRATION}",
            "-p",
            "simulation_only:=true",
        ]
    )
    node = MissionExecutorNode()
    try:
        node._fsm.state = terminal_state
        request = ExecuteMission.Goal()
        request.mission_id = "simulation-patrol"
        request.field_profile_id = "simulation-arena"

        response = node._on_goal(request)

        assert response == GoalResponse.ACCEPT
        assert node._fsm.state == MissionState.IDLE
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    ("mission_id", "field_profile_id"),
    [
        ("unexpected-mission", "simulation-arena"),
        ("simulation-patrol", "unexpected-field"),
    ],
)
def test_mission_executor_rejects_goal_for_different_loaded_bundle(
    mission_id: str,
    field_profile_id: str,
) -> None:
    from ed_uav_mission.executor import MissionExecutorNode

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"profile_path:={PROFILE_PATH}",
            "-p",
            f"mission_config_path:={MISSION_PATH}",
            "-p",
            f"calibration_file:={SYNTHETIC_CALIBRATION}",
            "-p",
            "simulation_only:=true",
        ]
    )
    node = MissionExecutorNode()
    try:
        request = ExecuteMission.Goal()
        request.mission_id = mission_id
        request.field_profile_id = field_profile_id

        assert node._on_goal(request) == GoalResponse.REJECT
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
