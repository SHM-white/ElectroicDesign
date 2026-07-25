from __future__ import annotations

import stat
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = REPOSITORY_ROOT / "tools"
RUNNER_NAMES = ("run_gazebo_sim.sh", "run_gazebo_smoke.sh")
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
    ):
        assert package_name in package_lines
    assert "ros-humble-ros-gz-point-cloud" not in package_lines
    assert "apt-get install --no-install-recommends --yes" in source
