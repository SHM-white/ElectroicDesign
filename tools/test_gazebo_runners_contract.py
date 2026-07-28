from __future__ import annotations

import stat
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = REPOSITORY_ROOT / "tools"
RUNNER_NAMES = ("run_gazebo_sim.sh", "run_gazebo_smoke.sh")
SLAM_NAV_RUNNER = TOOLS_DIR / "run_gazebo_slam_nav.sh"
FAST_LIO_SIMULATION_PATCH = TOOLS_DIR / "patches" / "fast_lio_simulation.patch"
PROTECTED_PATHS = (
    "offline_integration.launch.py",
    "verification_harness.launch.py",
    "drone/start.sh",
    "drone/debug_start.sh",
    "drone/field_test.sh",
)


def test_gazebo_runners_are_executable_and_fail_closed() -> None:
    # Given: the two user-facing Gazebo operator entry points.
    for runner_name in RUNNER_NAMES:
        runner_path = TOOLS_DIR / runner_name

        # When: the entry point metadata is inspected.
        source = runner_path.read_text(encoding="utf-8")

        # Then: it is directly runnable and uses the shared fail-closed runner.
        assert runner_path.stat().st_mode & stat.S_IXUSR, f"{runner_name} is not executable"
        assert source.startswith("#!/usr/bin/env bash")
        assert "set -euo pipefail" in source
        assert 'bash "$repo_root/tools/run_humble.sh"' in source
        assert ".omo/evidence/gazebo" in source
        assert "ed_uav_gazebo gazebo_simulation.launch.py" in source


def test_gazebo_modes_have_explicit_gui_and_bounded_contracts() -> None:
    # Given: separate interactive GUI and bounded headless runner sources.
    gui_source = (TOOLS_DIR / "run_gazebo_sim.sh").read_text(encoding="utf-8")
    smoke_source = (TOOLS_DIR / "run_gazebo_smoke.sh").read_text(encoding="utf-8")

    # Then: GUI mode is explicit and smoke mode has readiness, movement, and cleanup checks.
    assert "HUMBLE_GUI=1" in gui_source
    assert "HUMBLE_INTERACTIVE=1" in gui_source
    assert "kill -INT" in gui_source
    assert "GZ_SIM_RESOURCE_PATH" in gui_source
    assert "IGN_GAZEBO_RESOURCE_PATH" in gui_source
    assert "HUMBLE_GUI=0" not in smoke_source
    assert "HUMBLE_GUI=unset" in smoke_source
    assert "HUMBLE_INTERACTIVE=0" in smoke_source
    assert "export ROS_DOMAIN_ID=42" in gui_source
    assert "export ROS_LOCALHOST_ONLY=1" in gui_source
    assert "export ROS_DOMAIN_ID=42" in smoke_source
    assert "export ROS_LOCALHOST_ONLY=1" in smoke_source
    assert "ros2 topic list" in smoke_source
    assert "ros2 topic echo" in smoke_source
    assert "ros2 topic pub" in smoke_source
    assert "/simulation/enable" in smoke_source
    assert "/simulation/cmd_vel" in smoke_source
    assert "x: 0.1" not in smoke_source
    assert "/simulation/ground_truth/odom" in smoke_source
    assert "mission-final-odom.log" in smoke_source
    assert "kill -INT" in smoke_source
    assert "SUCCESS" in smoke_source
    assert "FAILED" in smoke_source
    assert "physical" not in smoke_source.lower()
    assert '[[ ! -e "$evidence_dir/FAILED" ]]' in smoke_source


def test_gazebo_runners_source_humble_before_enabling_nounset() -> None:
    # Given: both runners execute a strict inner Bash script in Humble.
    expected_preamble = (
        "CONTAINER_SCRIPT'\n"
        "source /opt/ros/humble/setup.bash\n"
        "export ROS_DOMAIN_ID=42\n"
        "export ROS_LOCALHOST_ONLY=1\n"
        "set -euo pipefail"
    )

    # Then: Humble setup may initialize its optional variables before `set -u`.
    for runner_name in RUNNER_NAMES:
        source = (TOOLS_DIR / runner_name).read_text(encoding="utf-8")
        assert expected_preamble in source


def test_gazebo_runners_suspend_nounset_only_while_sourcing_overlay() -> None:
    # Given: colcon overlay setup reads optional trace variables.
    overlay_source = (
        "set +u\n"
        'source "$evidence_dir/install/setup.bash"\n'
        "set -u"
    )

    # Then: both runners restore nounset immediately after sourcing the overlay.
    for runner_name in RUNNER_NAMES:
        source = (TOOLS_DIR / runner_name).read_text(encoding="utf-8")
        assert overlay_source in source


def test_slam_nav_runner_isolates_fast_lio_and_executes_competition_mission() -> None:
    # Given: the interactive FAST-LIO/Nav2 planner-only operator entry point.
    runner_path = SLAM_NAV_RUNNER

    # When: its static runtime contract is inspected before a ROS environment exists.
    source = runner_path.read_text(encoding="utf-8")

    # Then: it is an interactive, evidence-local Humble execution path.
    assert runner_path.stat().st_mode & stat.S_IXUSR
    assert source.startswith("#!/usr/bin/env bash")
    assert "set -euo pipefail" in source
    assert '"$repo_root/tools/run_humble.sh"' in source
    assert "HUMBLE_GUI=1" in source
    assert "HUMBLE_INTERACTIVE=1" in source
    assert "export ROS_DOMAIN_ID=42" in source
    assert "export ROS_LOCALHOST_ONLY=1" in source
    assert 'host_workspace_root="$1"' in source
    assert 'host_evidence_dir="$2"' in source
    assert 'container_workspace_root="$3"' in source
    assert 'container_evidence_dir="$4"' in source
    assert 'workspace_root="$host_workspace_root"' in source
    assert 'workspace_root="$container_workspace_root"' in source
    assert 'evidence_dir="$host_evidence_dir"' in source
    assert 'evidence_dir="$container_evidence_dir"' in source
    assert 'third_party_dir="$evidence_dir/third_party"' in source
    assert 'build_base="$evidence_dir/build"' in source
    assert 'install_base="$evidence_dir/install"' in source
    assert 'log_base="$evidence_dir/log"' in source
    assert 'set +u\nsource "$install_base/setup.bash"\nset -u' in source
    assert '"/workspace"' in source
    assert '"/workspace/$evidence_relative"' in source
    assert '"$workspace_root/ros2_ws/src"' in source
    assert "ros2_ws/src/third_party" not in source

    # And: the authoritative JSON manifest is narrowed to the three FAST-LIO dependencies.
    assert "dependencies.repos" in source
    assert "json.load" in source
    assert 'selected_names = ("livox_sdk2", "livox_ros_driver2", "fast_lio_ros2")' in source
    assert "livox_sdk2" in source
    assert "livox_ros_driver2" in source
    assert "fast_lio_ros2" in source
    assert "ultralytics" not in source
    assert "vcs import" in source
    assert "submodule update --init --recursive" in source
    assert "package_ROS2.xml" in source
    assert "launch_ROS2" in source
    assert "-DROS_EDITION=ROS2" in source
    assert "-DDISTRO_ROS=humble" in source
    assert 'livox_sdk2_install="$evidence_dir/livox-sdk2-install"' in source
    assert 'cmake -S "$livox_sdk2_dir" -B "$livox_sdk2_build"' in source
    assert 'cmake --build "$livox_sdk2_build"' in source
    assert 'cmake --install "$livox_sdk2_build" --prefix "$livox_sdk2_install"' in source
    assert 'test -f "$livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so"' in source
    assert 'test -d "$livox_sdk2_install/include"' in source
    assert "livox-sdk2-configure.log" in source
    assert "livox-sdk2-build.log" in source
    assert "livox-sdk2-install.log" in source
    assert "-DLIVOX_LIDAR_SDK_LIBRARY=$livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so" in source
    assert "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$livox_sdk2_install/include" in source
    assert 'export LD_LIBRARY_PATH="$livox_sdk2_install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"' in source

    # And: the launch, bounded readiness, command sequence, and evidence use simulator-only contracts.
    assert "localization_mode:=fast_lio" in source
    assert "gui:=true" in source
    assert "use_rviz:=true" in source
    for endpoint in (
        "/clock",
        "/lidar/points",
        "/lidar/imu",
        "/localization/lio/odom",
        "/localization/lio/cloud_registered",
        "/localization/lio/map",
        "/localization/lio/path",
        "/localization/odom",
        "/map",
        "/compute_path_to_pose",
        "/fcu/flight_command",
        "/mission/execute",
    ):
        assert endpoint in source
    assert "LocalizationStatus.STATE_ACTIVE" in source
    assert "map_to_odom_valid: true" in source
    assert "for _ in" in source
    assert "timeout " in source
    assert "ed_uav_interfaces/action/FlightCommand" in source
    assert "command: 1" in source
    assert "ed_uav_interfaces/action/ExecuteMission" in source
    assert "mission_id: simulation-competition" in source
    assert "field_profile_id: simulation-arena" in source
    assert "status: SUCCEEDED" in source
    assert "motors_armed: false" in source
    assert "GAZEBO_SLAM_NAV_SUCCESS" in source
    assert 'if ((runner_status == 130)) && [[ -e "$evidence_dir/SUCCESS" ]]; then' in source
    assert "setsid bash -c" in source
    assert 'kill -INT -- "-$launch_pid"' in source
    assert "wait \"$launch_pid\"" in source
    assert "physical" not in source.lower()
    assert "cmd_vel" not in source
    for protected_path in PROTECTED_PATHS:
        assert protected_path not in source


def test_slam_nav_runner_patches_fast_lio_simulation_initialization() -> None:
    # Given: the pinned FAST-LIO source needs a longer simulation IMU initialization.
    runner = SLAM_NAV_RUNNER.read_text(encoding="utf-8")

    # When: the evidence-local source preparation contract is inspected.
    assert FAST_LIO_SIMULATION_PATCH.is_file()
    patch = FAST_LIO_SIMULATION_PATCH.read_text(encoding="utf-8")

    # Then: the exact pinned source is patched fail-closed before it is built.
    assert "-#define MAX_INI_COUNT (10)" in patch
    assert "+#define MAX_INI_COUNT (200)" in patch
    assert "+      last_lidar_end_time_ = meas.lidar_end_time;" in patch
    assert "patch --batch --forward --fuzz=0" in runner
    assert "fast-lio-simulation-patch.log" in runner


def test_runner_sources_do_not_touch_protected_legacy_surfaces() -> None:
    # Given: new runner sources only.
    combined_source = "\n".join(
        (TOOLS_DIR / runner_name).read_text(encoding="utf-8") for runner_name in RUNNER_NAMES
    )

    # Then: no protected launcher or offline harness is invoked by the new surface.
    for protected_path in PROTECTED_PATHS:
        assert protected_path not in combined_source


def test_humble_runner_declares_mode_and_timeout_boundaries() -> None:
    # Given: the shared Humble runner source.
    source = (TOOLS_DIR / "run_humble.sh").read_text(encoding="utf-8")
    support_source = (TOOLS_DIR / "run_humble_support.sh").read_text(encoding="utf-8")

    # Then: mode validation and the two distinct execution paths are visible in the contract.
    assert "HUMBLE_INTERACTIVE" in source
    assert "HUMBLE_TIMEOUT_SECONDS must be a positive number" in support_source
    assert "--interactive" in source
    assert "timeout --foreground" in source
    assert "--volume /mnt/wslg:/mnt/wslg:ro" in source


def test_humble_image_declares_released_fortress_and_sensor_packages() -> None:
    # Given: the explicit Jammy/Humble image package list.
    source = (REPOSITORY_ROOT / "docker" / "Dockerfile.humble").read_text(encoding="utf-8")
    package_lines = "\n".join(
        line for line in source.splitlines() if not line.lstrip().startswith("#")
    )

    # Then: released ROS/Gazebo and geometry/image dependencies are fail-fast apt inputs.
    for package_name in (
        "ignition-fortress",
        "libignition-gazebo6-plugins",
        "ros-humble-ros-gz-sim",
        "ros-humble-ros-gz-bridge",
        "ros-humble-ros-gz-interfaces",
        "ros-humble-ros-gz-image",
        "ros-humble-laser-geometry",
        "ros-humble-sensor-msgs",
        "ros-humble-geometry-msgs",
        "ros-humble-rviz2",
        "ros-humble-robot-state-publisher",
        "ros-humble-pcl-ros",
        "ros-humble-pcl-conversions",
        "libeigen3-dev",
        "python3-dev",
        "ros-humble-nav2-costmap-2d",
        "patch",
    ):
        assert package_name in package_lines
    assert "ros-humble-ros-gz-point-cloud" not in package_lines
    assert "apt-get install --no-install-recommends --yes" in source
