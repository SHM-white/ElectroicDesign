from __future__ import annotations

import os
import shutil
import stat
import subprocess
from dataclasses import dataclass
from pathlib import Path

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPOSITORY_ROOT / "tools" / "run_lidar_odometry_offset_demo.sh"


@dataclass(frozen=True, slots=True)
class FakeWorkspace:
    root: Path
    runner: Path
    log: Path


def write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def create_fake_workspace(tmp_path: Path) -> FakeWorkspace:
    root = tmp_path / "workspace"
    tools_dir = root / "tools"
    install_dir = root / "ros2_ws" / "install"
    fake_bin = root / "fake-bin"
    tools_dir.mkdir(parents=True)
    install_dir.mkdir(parents=True)
    fake_bin.mkdir()
    runner = tools_dir / RUNNER_PATH.name
    shutil.copy2(RUNNER_PATH, runner)
    (install_dir / "setup.bash").write_text(":\n", encoding="utf-8")
    log = root / "fake-runtime.log"
    write_executable(
        fake_bin / "colcon",
        "#!/usr/bin/env bash\n"
        "printf 'colcon:%s\\n' \"$*\" >>\"${FAKE_RUNTIME_LOG:?}\"\n",
    )
    write_executable(
        fake_bin / "ros2",
        "#!/usr/bin/env bash\n"
        "printf 'ros2:%s\\n' \"$*\" >>\"${FAKE_RUNTIME_LOG:?}\"\n"
        "case \"$1 $2\" in\n"
        "  'topic type') printf '%s\\n' \"${FAKE_TOPIC_TYPE:-nav_msgs/msg/Odometry}\" ;;\n"
        "  'topic info') printf 'Type: %s\\nPublisher count: %s\\n' \"${FAKE_TOPIC_TYPE:-nav_msgs/msg/Odometry}\" \"${FAKE_PUBLISHER_COUNT:-1}\" ;;\n"
        "  'topic echo') exit \"${FAKE_MESSAGE_STATUS:-0}\" ;;\n"
        "  'run ed_uav_localization') exit \"${FAKE_DEMO_STATUS:-0}\" ;;\n"
        "  *) exit 41 ;;\n"
        "esac\n",
    )
    return FakeWorkspace(root, runner, log)


def run_runner(
    workspace: FakeWorkspace,
    arguments: tuple[str, ...] = (),
    environment: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    runtime_environment = {
        **os.environ,
        "FAKE_RUNTIME_LOG": str(workspace.log),
        "PATH": f"{workspace.root / 'fake-bin'}:{os.environ['PATH']}",
    }
    if environment is not None:
        runtime_environment.update(environment)
    return subprocess.run(
        [str(workspace.runner), *arguments],
        cwd=workspace.root,
        env=runtime_environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_runner_is_executable_and_never_launches_upstream_processes() -> None:
    # Given: the one-click startup-relative lidar odometry runner.
    source = RUNNER_PATH.read_text(encoding="utf-8")

    # When: its shell contract is inspected.
    executable = RUNNER_PATH.stat().st_mode & stat.S_IXUSR

    # Then: it sources Humble before nounset and owns no upstream launch surface.
    assert executable
    assert source.index("source /opt/ros/humble/setup.bash") < source.index("set -euo pipefail")
    assert "ros2 launch" not in source
    assert "livox_ros_driver2" not in source
    assert "fast_lio" not in source


def test_runner_builds_preflights_defaults_and_execs_foreground_demo(tmp_path: Path) -> None:
    # Given: an available fake localization chain and sourceable install overlay.
    workspace = create_fake_workspace(tmp_path)

    # When: the runner starts without overrides.
    result = run_runner(workspace)

    # Then: all preflights and the exec'd live demo use the default topic consistently.
    assert result.returncode == 0, result.stderr
    log = workspace.log.read_text(encoding="utf-8")
    assert "colcon:build --symlink-install --packages-up-to ed_uav_localization" in log
    assert "ros2:topic type /localization/odom" in log
    assert "ros2:topic info /localization/odom" in log
    assert "ros2:topic echo --once /localization/odom" in log
    assert "ros2:run ed_uav_localization lidar_odometry_offset_demo --odom-topic /localization/odom" in log


def test_runner_cli_override_wins_over_environment_for_preflight_and_exec(tmp_path: Path) -> None:
    # Given: conflicting environment and CLI topic values.
    workspace = create_fake_workspace(tmp_path)

    # When: the caller explicitly supplies the CLI topic and a demo flag.
    result = run_runner(
        workspace,
        ("--odom-topic", "/cli/odom", "--output-rate-hz", "4"),
        {"ODOM_TOPIC": "/environment/odom", "ED_ODOMETRY_OFFSET_SKIP_BUILD": "1"},
    )

    # Then: every ROS interaction uses only the CLI topic and forwards remaining flags.
    assert result.returncode == 0, result.stderr
    log = workspace.log.read_text(encoding="utf-8")
    assert "/environment/odom" not in log
    assert "ros2:topic type /cli/odom" in log
    assert "ros2:topic info /cli/odom" in log
    assert "ros2:topic echo --once /cli/odom" in log
    assert "ros2:run ed_uav_localization lidar_odometry_offset_demo --odom-topic /cli/odom --output-rate-hz 4" in log


@pytest.mark.parametrize(
    ("environment", "expected_reason"),
    (
        ({"FAKE_TOPIC_TYPE": "geometry_msgs/msg/Pose"}, "wrong type"),
        ({"FAKE_PUBLISHER_COUNT": "0"}, "publisher"),
        ({"FAKE_MESSAGE_STATUS": "1"}, "message"),
    ),
)
def test_runner_rejects_failed_preflights_before_foreground_exec(
    tmp_path: Path, environment: dict[str, str], expected_reason: str
) -> None:
    # Given: a fake chain with one invalid preflight condition.
    workspace = create_fake_workspace(tmp_path)

    # When: the runner validates it.
    result = run_runner(workspace, environment=environment)

    # Then: it fails explicitly before starting the demo process.
    assert result.returncode != 0
    assert expected_reason in result.stderr.lower()
    assert "ros2:run ed_uav_localization" not in workspace.log.read_text(encoding="utf-8")


def test_runner_explains_missing_overlay_when_build_is_skipped(tmp_path: Path) -> None:
    # Given: a caller who skips build without an existing overlay.
    workspace = create_fake_workspace(tmp_path)
    (workspace.root / "ros2_ws" / "install" / "setup.bash").unlink()

    # When: the runner starts.
    result = run_runner(workspace, environment={"ED_ODOMETRY_OFFSET_SKIP_BUILD": "1"})

    # Then: it fails before ROS interactions with a direct remediation.
    assert result.returncode != 0
    assert "install overlay is missing" in result.stderr.lower()
    assert not workspace.log.exists()
