from __future__ import annotations

import math
from pathlib import Path

import pytest
from ed_uav_mission import competition_tree
from ed_uav_mission.competition_tree import (
    CompetitionStep,
    MapPose,
    forward_goal,
    moves_from_planner_path,
    return_goal,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = (
    PACKAGE_ROOT.parent
    / "ed_uav_localization"
    / "config"
    / "fields"
    / "simulation_arena.yaml"
)
COMPETITION_CONFIG = PACKAGE_ROOT / "config" / "missions" / "simulation_competition.yaml"


def test_competition_tree_declares_two_immutable_2026_task_branches() -> None:
    # Given: the competition mission's checked-in branch definitions.
    branches = getattr(competition_tree, "D_TASK_BRANCHES", None)

    # When/Then: synthetic navigation is replaced by exactly the two D-task branches.
    assert branches is not None
    assert tuple(branches) == (1, 2)
    assert not hasattr(competition_tree.CompetitionStep, "NAVIGATE_FORWARD")
    assert not hasattr(competition_tree.CompetitionStep, "NAVIGATE_RETURN")


def test_competition_tree_has_the_required_terminal_sequence() -> None:
    # Given: both immutable 2026 competition branches.
    branches = tuple(competition_tree.D_TASK_BRANCHES.values())

    # When/Then: both return to H, land there, and terminate successfully.
    assert all(
        branch.nominal_phases[-3:] == (
            CompetitionStep.RETURNING_HOME,
            CompetitionStep.LANDING_HOME,
            CompetitionStep.SUCCEEDED,
        )
        for branch in branches
    )


def test_forward_goal_uses_captured_start_yaw_and_finite_distance() -> None:
    # Given: a map-frame pose captured after takeoff and hover.
    start = MapPose(x_m=3.0, y_m=-2.0, yaw_rad=math.pi / 2.0)

    # When: the configured forward target is derived.
    goal = forward_goal(start, 2.0)

    # Then: XY follows the captured yaw and the orientation is retained.
    assert goal.x_m == pytest.approx(3.0)
    assert goal.y_m == pytest.approx(0.0)
    assert goal.yaw_rad == start.yaw_rad

    with pytest.raises(ValueError, match="finite"):
        forward_goal(start, math.inf)


def test_return_goal_is_the_captured_start_pose_exactly() -> None:
    # Given: a nontrivial captured map-frame start pose.
    start = MapPose(x_m=-1.25, y_m=4.5, yaw_rad=-0.75)

    # When: the return target is built.
    goal = return_goal(start)

    # Then: no recomputation or rounding changes the exact captured pose.
    assert goal == start


def test_competition_altitude_is_checked_against_field_bounds(tmp_path: Path) -> None:
    # Given: a competition profile with a fixed altitude above the field maximum.
    pytest.importorskip("pydantic")
    from ed_uav_mission.mission_config import load_mission_bundle

    invalid_config = tmp_path / "altitude-outside-field.yaml"
    invalid_config.write_text(
        COMPETITION_CONFIG.read_text(encoding="utf-8").replace(
            "altitude_m: 1.5", "altitude_m: 4.1"
        ),
        encoding="utf-8",
    )

    # When: the profile and mission are loaded at the configuration boundary.
    # Then: unsafe fixed-altitude motion is rejected before execution.
    with pytest.raises(ValueError, match="altitude"):
        load_mission_bundle(PROFILE_PATH, invalid_config, allow_blocked_profile=True)


def test_competition_params_reject_nonfinite_forward_distance() -> None:
    # Given: the Pydantic-backed mission model is available.
    pydantic = pytest.importorskip("pydantic")
    from ed_uav_mission.mission_model import CompetitionParams

    # When: a nonfinite forward distance crosses the configuration boundary.
    # Then: model parsing rejects it before execution.
    with pytest.raises(pydantic.ValidationError):
        CompetitionParams(forward_distance_m=math.inf)


def test_d_task_profile_fixes_2026_altitude_stability_and_deadline() -> None:
    # Given: a parsed competition profile for the immutable 2026 branches.
    from ed_uav_mission.mission_config import parse_mission_config_text

    config = parse_mission_config_text(COMPETITION_CONFIG.read_text(encoding="utf-8"))

    # When/Then: branch-critical values and selection identifiers are explicit.
    assert config.takeoff_altitude_m == 1.5
    assert config.timeout_sec == 90.0
    assert config.competition.stable_sec == 3.0
    assert config.competition.mission_profile_id == "d2026-simulation"
    assert config.competition.deployment_preset_id == "simulation"
    assert config.competition.target_revision == "d2026-circle-cross-v1"


def test_selection_store_commits_once_only_while_pre_arm() -> None:
    # Given: the shared D-task model and one valid pre-arm selection.
    from ed_uav_mission import d_task_model

    store_type = getattr(d_task_model, "SelectionStore", None)
    assert store_type is not None
    store = store_type()
    selection = d_task_model.DTaskSelection(
        mission_id="simulation-competition",
        mission_profile_id="d2026-simulation",
        deployment_preset_id="simulation",
        target_revision="d2026-circle-cross-v1",
        task=d_task_model.DTaskKind.PAYLOAD_DROP,
        committed_at_s=1.0,
    )

    # When: the same run attempts a second selection and an armed selection.
    first = store.commit(selection, pre_arm=True)
    duplicate = store.commit(selection, pre_arm=True)
    armed = store_type().commit(selection, pre_arm=False)

    # Then: only the original immutable selection is retained.
    assert first.accepted is True
    assert duplicate.accepted is False
    assert armed.accepted is False
    assert store.selection is selection


def test_empty_planner_path_is_rejected_before_any_move_goal() -> None:
    # Given: Nav2 accepted a planning request but returned no poses.
    # When: the path is converted to FlightCommand move targets.
    # Then: execution stops before an unsafe movement can be issued.
    with pytest.raises(ValueError, match="empty"):
        moves_from_planner_path((), altitude_m=1.5, label="forward")


def test_path_conversion_preserves_planar_pose_and_forces_altitude() -> None:
    # Given: planner poses with arbitrary source altitudes excluded from the pure model.
    path = (
        MapPose(x_m=0.0, y_m=1.0, yaw_rad=0.0),
        MapPose(x_m=2.0, y_m=3.0, yaw_rad=-1.2),
    )

    # When: they become fixed-altitude movement goals.
    moves = moves_from_planner_path(path, altitude_m=1.5, label="return")

    # Then: map XY/yaw are retained and every goal uses the configured altitude.
    assert [(move.x_m, move.y_m, move.yaw_rad) for move in moves] == [
        (0.0, 1.0, 0.0),
        (2.0, 3.0, -1.2),
    ]
    assert {move.altitude_m for move in moves} == {1.5}


def test_competition_ros_integration_remains_planner_only_and_flight_command_only() -> None:
    # Given: the checked-in production executor, package declaration, and simulator launch.
    executor_source = (PACKAGE_ROOT / "ed_uav_mission" / "executor.py").read_text(
        encoding="utf-8"
    )
    runtime_path = PACKAGE_ROOT / "ed_uav_mission" / "competition_runtime.py"
    assert runtime_path.is_file()
    runtime_source = runtime_path.read_text(encoding="utf-8")
    planner_path = PACKAGE_ROOT / "ed_uav_mission" / "competition_planner.py"
    assert planner_path.is_file()
    planner_source = planner_path.read_text(encoding="utf-8")
    package_source = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    sim_launch_source = (
        PACKAGE_ROOT.parent / "ed_uav_gazebo" / "launch" / "sim.launch.py"
    ).read_text(encoding="utf-8")

    # When: their planner and control-path boundaries are inspected.
    # Then: Nav2 only supplies paths and all movement remains FlightCommand based.
    assert "from ed_uav_mission.competition_runtime import (" in executor_source
    assert "self._competition_runtime = CompetitionRuntime(" in executor_source
    assert "CompetitionCallbacks(" in executor_source
    assert "execute_takeoff=self._execute_takeoff" in executor_source
    assert "send_hover=self._send_hover" in executor_source
    assert "track_target=self._track_d_task_target" in executor_source
    assert "release_payload=self._release_d_task_payload" in executor_source
    assert "descend_to_vehicle=self._descend_to_vehicle" in executor_source
    assert "return_home=self._return_d_task_home" in executor_source
    assert "land_home=self._land_d_task_home" in executor_source
    assert "class CompetitionCallbacks" in runtime_source
    assert "class CompetitionRuntime" in runtime_source
    assert "DTaskRuntime(" in runtime_source
    assert "from nav2_msgs.action import ComputePathToPose" in planner_source
    assert "from action_msgs.msg import GoalStatus" in planner_source
    assert "from geometry_msgs.msg import PoseStamped" in planner_source
    assert "def _path_poses(path: NavPathLike)" in planner_source
    assert "from rclpy.time import Time" in planner_source
    assert "from tf2_ros import Buffer, TransformException, TransformListener" in planner_source
    assert "from typing_extensions import assert_never" in runtime_source
    assert "from ed_uav_interfaces.action import FlightCommand" not in runtime_source
    assert "FlightCommand.Goal" not in runtime_source
    assert "from ed_uav_interfaces.action import FlightCommand" not in planner_source
    assert "FlightCommand.Goal" not in planner_source
    assert "from nav2_msgs.action import ComputePathToPose" not in executor_source
    assert "from action_msgs.msg import GoalStatus" in executor_source
    assert "from geometry_msgs.msg import PoseStamped" not in executor_source
    assert "from nav_msgs.msg import Path as NavPath" not in executor_source
    assert "from rclpy.time import Time" not in executor_source
    assert "from tf2_ros import Buffer, TransformException, TransformListener" not in executor_source
    assert "def _capture_map_pose" not in executor_source
    assert "_planner_client" not in executor_source
    assert "_tf_buffer" not in executor_source
    assert "_tf_listener" not in executor_source
    assert "COMMAND_MOVE" in executor_source
    assert 'target_pose.header.frame_id = "map"' in executor_source
    assert "goal.target_pose.pose.position.z = config.takeoff_altitude_m" in executor_source
    assert "import asyncio" not in executor_source
    assert "asyncio." not in executor_source
    assert "await wait_with_deadline" in planner_source
    assert "self._planner_client.send_goal_async(request)" in planner_source
    assert "handle.get_result_async()" in planner_source
    assert "GoalStatus.STATUS_SUCCEEDED" in planner_source
    assert "DTaskEffect.RELEASE_PAYLOAD" in runtime_source
    assert "DTaskEffect.DESCEND_TO_VEHICLE" in runtime_source
    assert not any(
        token in executor_source.lower()
        for token in (
            "cmd_vel",
            "controller_server",
            "bt_navigator",
            "import serial",
            "import gpio",
            "0x41",
        )
    )
    assert {"action_msgs", "nav2_msgs", "nav_msgs", "tf2_ros"} <= {
        line.split(">", 1)[1].split("<", 1)[0]
        for line in package_source.splitlines()
        if "<exec_depend>" in line
    }
    assert 'glob("config/missions/*.yaml")' in setup_source
    assert "simulation_competition.yaml" in sim_launch_source
    assert COMPETITION_CONFIG.is_file()
