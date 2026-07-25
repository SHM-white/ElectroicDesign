"""Wall-time contract tests for the live deterministic offline integration."""

from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import time
from pathlib import Path
from types import ModuleType

import pytest

pytest.importorskip("launch")
from launch import LaunchContext


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_LAUNCH = (
    REPOSITORY_ROOT / "ros2_ws/src/ed_uav_bringup/launch/offline_integration.launch.py"
)
HARNESS_LAUNCH = (
    REPOSITORY_ROOT
    / "ros2_ws/src/ed_uav_verification/launch/verification_harness.launch.py"
)
WALL_TIME_ERROR = (
    "live deterministic publisher has no /clock and requires use_sim_time=false"
)
EXPECTED_TOPICS = frozenset(
    {
        "/camera/narrow/image_raw",
        "/camera/wide/image_raw",
        "/fcu/optical_flow/odom",
        "/lidar/imu",
        "/lidar/points",
        "/localization/lio/odom",
    }
)


def _load_launch(path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_context(use_sim_time: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations["use_sim_time"] = use_sim_time
    context.launch_configurations["use_rviz"] = "false"
    context.launch_configurations["rviz_config"] = "unused.rviz"
    return context


def _ros_environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    return environment


def _run_ros_launch(
    surface: tuple[str, str], use_sim_time: str, duration: str = "1"
) -> subprocess.CompletedProcess[str]:
    package, launch_file = surface
    command = [
        "ros2",
        "launch",
        package,
        launch_file,
        f"use_sim_time:={use_sim_time}",
        f"duration_seconds:={duration}",
        "rate_hz:=20",
    ]
    if package == "ed_uav_bringup":
        command.append("use_rviz:=false")
    return subprocess.run(
        command,
        capture_output=True,
        check=False,
        env=_ros_environment(),
        text=True,
        timeout=20,
    )


@pytest.mark.parametrize("launch_path", [BRINGUP_LAUNCH, HARNESS_LAUNCH])
def test_true_is_rejected_before_launch_actions_are_created(launch_path: Path) -> None:
    # Given: either live deterministic launch boundary with simulated time enabled.
    module = _load_launch(launch_path)

    # When: its OpaqueFunction evaluates launch arguments.
    with pytest.raises(RuntimeError, match=WALL_TIME_ERROR):
        module._build_actions(_launch_context("true"))

    # Then: rejection occurs before the function can return any process or node action.


@pytest.mark.skipif(
    shutil.which("ros2") is None, reason="requires a sourced ROS 2 Humble environment"
)
@pytest.mark.parametrize(
    ("package", "launch_file"),
    [
        ("ed_uav_bringup", "offline_integration.launch.py"),
        ("ed_uav_verification", "verification_harness.launch.py"),
    ],
)
def test_real_true_launch_exits_nonzero_without_starting_a_process(
    package: str, launch_file: str
) -> None:
    # Given: an installed live deterministic launch surface.
    # When: the real ros2 launch command requests simulated time.
    result = _run_ros_launch((package, launch_file), "true")
    output = result.stdout + result.stderr

    # Then: launch fails at argument evaluation and no child process is started.
    assert result.returncode != 0
    assert WALL_TIME_ERROR in output
    assert "process started with pid" not in output


@pytest.mark.skipif(
    shutil.which("ros2") is None, reason="requires a sourced ROS 2 Humble environment"
)
def test_real_false_top_level_exits_cleanly() -> None:
    # Given: the installed top-level offline integration with wall time selected.
    # When: the finite integration is launched without RViz.
    result = _run_ros_launch(
        ("ed_uav_bringup", "offline_integration.launch.py"), "false"
    )
    output = result.stdout + result.stderr

    # Then: the publisher completes and its OnProcessExit shutdown returns zero.
    assert result.returncode == 0, output
    assert "ROS SCENARIO: GREEN virtual replay completed" in output
    assert "process has died" not in output


@pytest.mark.skipif(
    shutil.which("ros2") is None, reason="requires a sourced ROS 2 Humble environment"
)
def test_real_false_harness_exposes_six_topics_and_exits_cleanly() -> None:
    # Given: a long-enough finite wall-time launch for DDS discovery.
    environment = _ros_environment()
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ed_uav_verification",
            "verification_harness.launch.py",
            "use_sim_time:=false",
            "duration_seconds:=30",
            "rate_hz:=20",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=environment,
        text=True,
    )

    # When: the graph is observed before the finite publisher completes.
    observed_topics: set[str] = set()
    deadline = time.monotonic() + 8
    try:
        while process.poll() is None and time.monotonic() < deadline:
            topic_result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                check=False,
                env=environment,
                text=True,
                timeout=5,
            )
            observed_topics.update(topic_result.stdout.splitlines())
            if EXPECTED_TOPICS <= observed_topics:
                break
            time.sleep(0.1)
        output, _ = process.communicate(timeout=15)
    finally:
        if process.poll() is None:
            process.terminate()
            _ = process.wait(timeout=5)

    # Then: all six topics were visible and finite OnProcessExit shutdown was clean.
    assert EXPECTED_TOPICS <= observed_topics
    assert process.returncode == 0, output
    assert "ROS SCENARIO: GREEN virtual replay completed" in output
    assert "process has died" not in output
