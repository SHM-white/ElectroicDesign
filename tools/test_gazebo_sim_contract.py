from __future__ import annotations

import os
import re
import stat
import xml.etree.ElementTree as ET
from ast import Import, ImportFrom, parse, walk
from collections.abc import Iterable
from pathlib import Path
from typing import Final

REPOSITORY_MARKER: Final = "ros2_ws"
PACKAGE_NAME: Final = "ed_uav_gazebo"
REPOSITORY_ROOT: Final = next(
    parent for parent in Path(__file__).resolve().parents if (parent / REPOSITORY_MARKER / "src").is_dir()
)
TOOLS_DIR: Final = REPOSITORY_ROOT / "tools"
GAZEBO_PACKAGE: Final = REPOSITORY_ROOT / "ros2_ws" / "src" / PACKAGE_NAME
WORLD_PATH: Final = GAZEBO_PACKAGE / "worlds" / "ed_uav_arena.sdf"
MODEL_PATH: Final = GAZEBO_PACKAGE / "models" / "ed_quadrotor" / "model.sdf"
BRIDGE_PATH: Final = GAZEBO_PACKAGE / "config" / "bridge.yaml"
LAUNCH_PATH: Final = GAZEBO_PACKAGE / "launch" / "sim.launch.py"
COMPAT_LAUNCH_PATH: Final = GAZEBO_PACKAGE / "launch" / "gazebo_simulation.launch.py"
FIELD_PROFILE_PATH: Final = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_localization" / "config" / "fields" / "simulation_arena.yaml"
MISSION_PROFILE_PATH: Final = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_mission" / "config" / "missions" / "simulation_patrol.yaml"
SIM_DOC_PATH: Final = REPOSITORY_ROOT / "docs" / "testing" / "GAZEBO_SIM.md"
OFFLINE_RUNNERS: Final = (
    "run_offline_static.sh",
    "run_offline_sim.sh",
    "run_offline_rviz.sh",
    "run_offline_fcu_dry_run.sh",
    "run_offline_full_replay.sh",
)
GAZEBO_RUNNERS: Final = ("run_gazebo_sim.sh", "run_gazebo_smoke.sh")
FORBIDDEN_SERIAL_TOKENS: Final = frozenset(("/dev/tty", "ttyUSB", "ttyACM", "serial", "mavlink", "msp"))
EXPECTED_BRIDGES: Final = {
    "ros_topic_name: /clock",
    "gz_type_name: ignition.msgs.Clock",
    "ros_topic_name: /simulation/ground_truth/odom",
    "gz_type_name: ignition.msgs.Odometry",
    "ros_topic_name: /lidar/points",
    "gz_type_name: ignition.msgs.PointCloudPacked",
    "ros_topic_name: /rangefinder/range",
    "gz_type_name: ignition.msgs.LaserScan",
    "ros_topic_name: /camera/narrow/image_raw",
    "gz_type_name: ignition.msgs.Image",
    "ros_topic_name: /camera/narrow/camera_info",
    "gz_type_name: ignition.msgs.CameraInfo",
    "ros_topic_name: /camera/wide/image_raw",
    "ros_topic_name: /camera/wide/camera_info",
    "ros_topic_name: /simulation/cmd_vel",
    "gz_topic_name: /ed_quadrotor/simulation/cmd_vel",
    "ros_topic_name: /simulation/enable",
    "gz_topic_name: /ed_quadrotor/simulation/enable",
}
EXPECTED_SIM_TOPICS: Final = (
    "/clock",
    "/tf",
    "/simulation/ground_truth/odom",
    "/lidar/points",
    "/rangefinder/range",
    "/camera/narrow/image_raw",
    "/camera/narrow/camera_info",
    "/camera/wide/image_raw",
    "/camera/wide/camera_info",
    "/fcu/flight_command",
    "/mission/execute",
)
EXPECTED_DOCKER_PACKAGES: Final = (
    "ros-humble-ros-gz-sim",
    "ros-humble-ros-gz-bridge",
    "ignition-fortress",
    "ros-humble-laser-geometry",
)
ALLOWED_IMPORT_ROOTS: Final = frozenset(("__future__", "ast", "collections", "os", "pathlib", "re", "stat", "typing", "xml"))


def read_text(path: Path) -> str:
    assert path.is_file(), f"missing required source file: {path.relative_to(REPOSITORY_ROOT).as_posix()}"
    return path.read_text(encoding="utf-8")


def assert_contains_all(source: str, required: Iterable[str], owner: str) -> None:
    missing = sorted(token for token in required if token not in source)
    assert not missing, f"{owner} is missing required contract tokens: {missing}"


def assert_excludes_all(source: str, forbidden: Iterable[str], owner: str) -> None:
    present = sorted(token for token in forbidden if token.lower() in source.lower())
    assert not present, f"{owner} must not use production-only tokens: {present}"


def test_gazebo_package_declares_installable_assets() -> None:
    # Given: the planned simulator package is a normal ament_python package.
    required_files = (
        GAZEBO_PACKAGE / "package.xml",
        GAZEBO_PACKAGE / "setup.py",
        GAZEBO_PACKAGE / "resource" / PACKAGE_NAME,
        WORLD_PATH,
        MODEL_PATH,
        BRIDGE_PATH,
        LAUNCH_PATH,
        COMPAT_LAUNCH_PATH,
    )

    # When: the static package surface is inspected.
    missing = [path.relative_to(REPOSITORY_ROOT).as_posix() for path in required_files if not path.is_file()]

    # Then: every planned asset exists in source, not in generated build output.
    assert not missing, f"missing Gazebo simulator package assets: {missing}"
    setup_source = read_text(GAZEBO_PACKAGE / "setup.py")
    assert_contains_all(setup_source, ("worlds", "models", "config", "launch", "rviz"), "ed_uav_gazebo setup.py")
    package_xml = read_text(GAZEBO_PACKAGE / "package.xml")
    assert_contains_all(
        package_xml,
        (
            "ed_uav_localization",
            "ed_uav_mission",
            "ros_gz_sim",
            "ros_gz_bridge",
            "sensor_msgs",
            "nav_msgs",
            "tf2_ros",
        ),
        "ed_uav_gazebo package.xml",
    )


def test_world_and_sdf_are_self_contained_simulation_assets() -> None:
    # Given: a checked-in Gazebo world and SDF model, with no Fuel/network fetches.
    world = read_text(WORLD_PATH)
    sdf = read_text(MODEL_PATH)
    root = ET.parse(MODEL_PATH).getroot()

    # When: static world/model contents are inspected.
    plugins = list(root.iter("plugin"))
    sensors = [sensor.attrib.get("type", "") for sensor in root.iter("sensor")]

    # Then: physics, scene, and sensor systems are local and deterministic.
    assert_contains_all(world, ("Physics", "SceneBroadcaster", "Sensors"), WORLD_PATH.name)
    assert_excludes_all(world + sdf, ("fuel.gazebosim.org", "models.gazebosim.org", "http://", "https://"), "Gazebo world/SDF")

    # And: the airframe owns four rotors, one velocity controller, sensors, collision, and inertial data.
    motor_plugins = [plugin for plugin in plugins if "MulticopterMotorModel" in plugin.attrib.get("name", "")]
    velocity_plugins = [plugin for plugin in plugins if "MulticopterVelocityControl" in plugin.attrib.get("name", "")]
    assert len(motor_plugins) == 4, f"expected four MulticopterMotorModel plugins, found {len(motor_plugins)}"
    assert len(velocity_plugins) == 1, f"expected one MulticopterVelocityControl plugin, found {len(velocity_plugins)}"
    assert {"camera", "gpu_lidar", "imu"}.issubset(set(sensors)), f"missing required camera/lidar/IMU sensors: {sensors}"
    assert "ray" in sensors or "gpu_ray" in sensors, f"missing range sensor in SDF sensors: {sensors}"
    assert next(root.iter("collision"), None) is not None, "SDF model must define collision geometry"
    assert next(root.iter("inertial"), None) is not None, "SDF model must define inertial data"


def test_bridge_config_pins_exact_ros_gz_mappings() -> None:
    # Given: the simulator bridge is a static config file.
    bridge_source = read_text(BRIDGE_PATH)

    # When: the bridge mappings are inspected as text to preserve exact CLI syntax.
    # Then: every ROS-facing endpoint and Humble ignition message mapping is explicit.
    assert_contains_all(bridge_source, EXPECTED_BRIDGES, BRIDGE_PATH.name)
    assert "gz.msgs." not in bridge_source, "Humble bridge mappings must use ignition.msgs.* names"
    assert_excludes_all(bridge_source, FORBIDDEN_SERIAL_TOKENS, BRIDGE_PATH.name)


def test_simulator_launch_owns_sim_topics_and_excludes_offline_or_serial_launches() -> None:
    # Given: the planned Gazebo launch file is a source file, not a running launch.
    launch_source = read_text(LAUNCH_PATH)

    # When: ownership and launch inclusions are inspected.
    # Then: simulation time, optional RViz, and simulator-owned endpoints are explicit.
    assert_contains_all(launch_source, ("use_sim_time", "true", "use_rviz", "rviz2"), LAUNCH_PATH.name)
    assert_contains_all(
        launch_source,
        ("localization_simulation.launch.py", "mission_executor.launch.py", "synthetic_calibrated.yaml", "simulation_only"),
        LAUNCH_PATH.name,
    )
    assert_contains_all(launch_source, ("sim_fcu", "sim_localization", "ros_gz_bridge"), LAUNCH_PATH.name)

    # And: real hardware, serial bridges, and offline verification launches are not included.
    assert_excludes_all(
        launch_source,
        frozenset(("/dev/tty", "ed_uav_fcu_bridge", "offline_integration.launch.py", "offline_replay.launch.py", "fcu_dry_run.launch.py")),
        LAUNCH_PATH.name,
    )


def test_humble_dockerfile_declares_gazebo_and_bridge_dependencies() -> None:
    # Given: Humble runtime dependencies are declared in the checked-in Dockerfile.
    dockerfile = read_text(REPOSITORY_ROOT / "docker" / "Dockerfile.humble")

    # When: packages are inspected without building the image.
    # Then: Gazebo Fortress, ros_gz, and point cloud/laser conversion packages are explicit.
    assert_contains_all(dockerfile, EXPECTED_DOCKER_PACKAGES, "docker/Dockerfile.humble")


def test_gazebo_runners_use_humble_modes_evidence_and_preserve_offline_runners() -> None:
    # Given: Gazebo has two new runner scripts and five existing offline runners.
    offline_sources = {script: read_text(TOOLS_DIR / script) for script in OFFLINE_RUNNERS}

    for script in GAZEBO_RUNNERS:
        script_path = TOOLS_DIR / script

        # When: each Gazebo runner is inspected directly.
        assert script_path.is_file(), f"missing Gazebo runner: tools/{script}"
        source = read_text(script_path)
        mode = stat.S_IMODE(script_path.stat().st_mode)

        # Then: the runner is executable, wrapped by run_humble.sh, and writes bounded evidence.
        assert mode & stat.S_IXUSR, f"tools/{script} must be user-executable"
        assert source.startswith("#!/usr/bin/env bash"), f"tools/{script} must have a bash shebang"
        assert "set -euo pipefail" in source, f"tools/{script} must fail closed"
        assert 'bash "$repo_root/tools/run_humble.sh"' in source, f"tools/{script} must invoke run_humble.sh"
        assert ".omo/evidence/gazebo" in source, f"tools/{script} must create Gazebo evidence"
        assert_contains_all(
            source,
            ("mode=interactive" if script.endswith("sim.sh") else "mode=bounded-headless"),
            script,
        )
        assert_excludes_all(source, FORBIDDEN_SERIAL_TOKENS, script)
        assert not any(offline_runner in source for offline_runner in OFFLINE_RUNNERS), (
            f"tools/{script} must not dispatch through existing offline runner paths"
        )

    # And: existing offline runner paths remain separate from the Gazebo simulator surface.
    for script, source in offline_sources.items():
        assert "gazebo" not in source.lower(), f"tools/{script} must not dispatch Gazebo work"
        assert ".omo/evidence/gazebo" not in source, f"tools/{script} must not write Gazebo evidence"


def test_mission_localization_entry_points_and_synthetic_profiles_are_bounded_to_simulation() -> None:
    # Given: simulator smoke flows need CLI entry points and synthetic data only.
    mission_setup = read_text(REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_mission" / "setup.py")
    localization_setup = read_text(REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_localization" / "setup.py")

    # When: setup metadata and simulator profile files are inspected.
    # Then: mission/localization expose console entry points for simulator runners.
    assert_contains_all(mission_setup, ("console_scripts", "mission_executor", "ed_uav_mission.executor"), "ed_uav_mission setup.py")
    assert_contains_all(
        localization_setup,
        ("console_scripts", "source_supervisor", "field_anchor"),
        "ed_uav_localization setup.py",
    )

    # And: field, mission, and documentation profiles mark the surface as simulation-only.
    assert FIELD_PROFILE_PATH.is_file(), "missing simulation-only field profile config/fields/gazebo_sim_synthetic.yaml"
    assert MISSION_PROFILE_PATH.is_file(), "missing simulation-only Gazebo smoke mission YAML"
    assert SIM_DOC_PATH.is_file(), "missing simulation-only Gazebo documentation docs/testing/GAZEBO_SIM.md"
    field_source = read_text(FIELD_PROFILE_PATH)
    mission_source = read_text(MISSION_PROFILE_PATH)
    doc_source = read_text(SIM_DOC_PATH)
    assert_contains_all(field_source, ("synthetic", "synthetic_simulation", "activation: blocked"), FIELD_PROFILE_PATH.name)
    assert_contains_all(mission_source, ("simulation-patrol", "simulation-arena", "mission_id"), MISSION_PROFILE_PATH.name)
    assert_contains_all(doc_source, ("simulation-only", "synthetic", "run_gazebo_sim.sh", "run_gazebo_smoke.sh", "serial hardware"), SIM_DOC_PATH.name)
    assert_contains_all(field_source + mission_source, ("blocked", "simulation"), "simulation YAML profiles")


def test_contract_has_no_hidden_runtime_side_effects() -> None:
    # Given: these tests are static contracts for implementation planning.
    test_source = read_text(Path(__file__))

    # When: the test module source is inspected.
    imported_roots: set[str] = set()
    for node in walk(parse(test_source)):
        match node:
            case Import(names=names):
                imported_roots.update(alias.name.split(".")[0] for alias in names)
            case ImportFrom(module=module) if module is not None:
                imported_roots.add(module.split(".")[0])
            case ImportFrom(module=None):
                imported_roots.add("")

    # Then: it cannot launch ROS/Gazebo, use network, require hardware, or depend on generated directories.
    assert imported_roots <= ALLOWED_IMPORT_ROOTS, f"unexpected runtime-capable imports: {sorted(imported_roots - ALLOWED_IMPORT_ROOTS)}"
    assert not re.search(r"\b(build|install|log)\b", os.fspath(BRIDGE_PATH.relative_to(REPOSITORY_ROOT)))
