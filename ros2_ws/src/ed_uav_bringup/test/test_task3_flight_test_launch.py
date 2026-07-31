from __future__ import annotations

import ast
import runpy
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
LAUNCH_PATH = REPOSITORY_ROOT / "ros2_ws/src/ed_uav_bringup/launch/task3_flight_test.launch.py"


def _launch_source() -> str:
    assert LAUNCH_PATH.is_file(), "missing Task3 flight-test launch: ed_uav_bringup/launch/task3_flight_test.launch.py"
    return LAUNCH_PATH.read_text(encoding="utf-8")


def _declared_arguments(tree: ast.AST) -> set[str]:
    return {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
    }


def _launch_configurations(tree: ast.AST) -> set[str]:
    return {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "LaunchConfiguration"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
    }


def _node_pairs(tree: ast.AST) -> set[tuple[str, str]]:
    pairs: set[tuple[str, str]] = set()
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        keywords = {
            keyword.arg: keyword.value.value
            for keyword in call.keywords
            if keyword.arg in {"package", "executable"}
            and isinstance(keyword.value, ast.Constant)
            and isinstance(keyword.value.value, str)
        }
        package = keywords.get("package")
        executable = keywords.get("executable")
        if package is not None and executable is not None:
            pairs.add((package, executable))
    return pairs


def _string_literals(tree: ast.AST) -> set[str]:
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def _parameter_literals(tree: ast.AST) -> dict[str, bool]:
    values: dict[str, bool] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values, strict=True):
            if (
                isinstance(key, ast.Constant)
                and isinstance(key.value, str)
                and isinstance(value, ast.Constant)
                and isinstance(value.value, bool)
            ):
                values[key.value] = value.value
    return values


def test_task3_launch_composes_the_real_flight_chain_without_rviz() -> None:
    # Given: the dedicated Task3 launch entry point.
    source = _launch_source()

    # When: its import surface and executable AST are loaded without a ROS launch service.
    pytest.importorskip("launch")
    namespace = runpy.run_path(str(LAUNCH_PATH))
    tree = ast.parse(source, filename=str(LAUNCH_PATH))

    # Then: every live-flight subsystem is composed, while visual and simulated modes stay absent.
    assert callable(namespace["generate_launch_description"])
    assert {
        ("ed_uav_fcu_bridge", "ed_uav_fcu_bridge"),
        ("ed_uav_mission", "mission_executor"),
        ("ed_uav_vehicle_bridge", "vehicle_bridge"),
        ("ed_uav_localization", "field_anchor"),
        ("ed_uav_localization", "source_supervisor"),
        ("ed_uav_localization", "lio_adapter"),
    } <= _node_pairs(tree)
    literals = _string_literals(tree)
    assert {
        "lidar.launch.py",
        "fast_lio.launch.py",
        "dual_uvc.launch.py",
        "target_observation.launch.py",
        "mid360",
        "d2026-apriltag-v1",
    } <= literals
    assert "rviz2" not in literals
    assert "simulation_only" not in _declared_arguments(tree)
    assert "use_fake_devices" not in _declared_arguments(tree)


def test_task3_launch_requires_security_inputs_and_fixed_control_policy() -> None:
    # Given: the dedicated Task3 launch entry point.
    tree = ast.parse(_launch_source(), filename=str(LAUNCH_PATH))

    # When: its declared and forwarded launch inputs are inspected.
    declared = _declared_arguments(tree)
    forwarded = _launch_configurations(tree)

    # Then: real runtime evidence, SROS settings, and Task3 identity are mandatory launch data.
    required = {
        "mission_config_path",
        "field_profile_path",
        "calibration_file",
        "camera_runtime_plan",
        "fcu_serial_port",
        "hmac_key_file",
        "mid360_driver_config_path",
        "fast_lio_launch_path",
        "ros_security_enable",
        "ros_security_strategy",
        "ros_security_keystore",
        "task3_identity",
    }
    assert required <= declared
    assert required <= forwarded
    assert "CALIBRATED" in _string_literals(tree)
    assert {
        "enable_flight_commands": True,
        "enable_realtime_control": True,
        "enable_programmable_commands": False,
    }.items() <= _parameter_literals(tree).items()
