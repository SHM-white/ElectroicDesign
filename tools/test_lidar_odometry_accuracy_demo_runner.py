from __future__ import annotations

import json
import os
import shutil
import signal
import stat
import subprocess
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import pytest

REPOSITORY_ROOT: Final = Path(__file__).resolve().parents[1]
RUNNER_PATH: Final = REPOSITORY_ROOT / "tools" / "run_lidar_odometry_accuracy_demo.sh"
RESULT_PREFIX: Final = "ODOMETRY_ACCURACY_RESULT="
VALID_RESULT_JSON: Final = (
    '{"input_topic":"/localization/odom","interpretation":"relative behavior",'
    '"metrics":{},"rejected_count":0,"sample_count":100,"schema_version":1,'
    '"status":"passed","trial":"stationary"}'
)


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
        "  'topic echo') printf 'header:\\n  frame_id: odom\\n' ;;\n"
        "  'run ed_uav_localization')\n"
        "    if [[ \"${FAKE_DEMO_WAIT_FOR_INT:-0}\" == \"1\" ]]; then\n"
        "      printf '%s\\n' \"$$\" >\"${FAKE_DEMO_PID:?}\"\n"
        "      trap 'printf \"%s\\n\" \"${FAKE_INTERRUPT_RESULT_LINE:?}\"; exit 1' INT\n"
        "      sleep 30\n"
        "    fi\n"
        "    printf '%s\\n' \"${FAKE_RESULT_LINE:?}\"\n"
        "    exit \"${FAKE_DEMO_STATUS:-0}\" ;;\n"
        "  *) exit 41 ;;\n"
        "esac\n",
    )
    return FakeWorkspace(root=root, runner=runner, log=log)


def run_runner(
    workspace: FakeWorkspace,
    arguments: tuple[str, ...] = (),
    extra_environment: Mapping[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    environment = {
        **os.environ,
        "FAKE_RUNTIME_LOG": str(workspace.log),
        "FAKE_RESULT_LINE": f"{RESULT_PREFIX}{VALID_RESULT_JSON}",
        "PATH": f"{workspace.root / 'fake-bin'}:{os.environ['PATH']}",
    }
    if extra_environment is not None:
        environment.update(extra_environment)
    return subprocess.run(
        [str(workspace.runner), *arguments],
        cwd=workspace.root,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def evidence_directory(workspace: FakeWorkspace) -> Path:
    directories = list((workspace.root / ".omo" / "evidence" / "lidar-odometry").iterdir())
    assert len(directories) == 1
    return directories[0]


def test_lidar_odometry_runner_has_a_fail_closed_real_hardware_boundary() -> None:
    # Given: the one-command real-lidar operator entry point.
    assert RUNNER_PATH.is_file(), "missing real-lidar odometry runner"
    source = RUNNER_PATH.read_text(encoding="utf-8")

    # When: its static safety contract is inspected.
    executable = RUNNER_PATH.stat().st_mode & stat.S_IXUSR

    # Then: it builds only localization, gates the external odometry chain, and never launches it.
    assert executable
    assert source.startswith("#!/usr/bin/env bash")
    assert "source /opt/ros/humble/setup.bash" in source
    assert source.index("source /opt/ros/humble/setup.bash") < source.index("set -euo pipefail")
    assert "colcon build --symlink-install --packages-up-to ed_uav_localization" in source
    assert 'source "$overlay_setup"' in source
    assert 'ODOM_TOPIC="${ODOM_TOPIC:-/localization/odom}"' in source
    assert 'ros2 topic type "$ODOM_TOPIC"' in source
    assert "nav_msgs/msg/Odometry" in source
    assert 'ros2 topic info "$ODOM_TOPIC"' in source
    assert 'ros2 topic echo --once "$ODOM_TOPIC"' in source
    assert "timeout " in source
    assert 'ros2 run ed_uav_localization odometry_accuracy_demo --odom-topic "$ODOM_TOPIC"' in source
    assert "--mode stationary --duration-sec 60 --min-samples 100" in source
    assert "ODOMETRY_ACCURACY_RESULT=" in source
    assert "result.json" in source
    assert "ODOM_TOPIC" in source
    assert "ros2 launch" not in source
    assert "livox_ros_driver2" not in source
    assert "fast_lio_ros2" not in source


def test_lidar_odometry_runner_builds_preflights_and_records_the_default_trial(tmp_path: Path) -> None:
    # Given: a sourceable temporary overlay and an available fake external odometry chain.
    workspace = create_fake_workspace(tmp_path)

    # When: the runner is invoked without arguments.
    result = run_runner(workspace)

    # Then: it builds the package, measures the default stationary trial, and preserves evidence.
    assert result.returncode == 0, result.stderr
    runtime_log = workspace.log.read_text(encoding="utf-8")
    assert "colcon:build --symlink-install --packages-up-to ed_uav_localization" in runtime_log
    assert "ros2:topic type /localization/odom" in runtime_log
    assert "ros2:topic info /localization/odom" in runtime_log
    assert "ros2:topic echo --once /localization/odom" in runtime_log
    assert (
        "ros2:run ed_uav_localization odometry_accuracy_demo --odom-topic /localization/odom "
        "--mode stationary --duration-sec 60 --min-samples 100"
    ) in runtime_log
    evidence = evidence_directory(workspace)
    assert (evidence / "command.txt").is_file()
    assert (evidence / "preflight.txt").is_file()
    assert (evidence / "result.txt").is_file()
    assert (evidence / "result.json").read_text(encoding="utf-8") == f"{VALID_RESULT_JSON}\n"
    assert f"LIDAR_ODOMETRY_RESULT={evidence / 'result.json'}" in result.stdout
    assert f"LIDAR_ODOMETRY_EVIDENCE={evidence}" in result.stdout


def test_lidar_odometry_runner_rejects_wrong_topics_before_starting_the_demo(tmp_path: Path) -> None:
    # Given: a topic with the wrong ROS message type.
    workspace = create_fake_workspace(tmp_path)

    # When: the preflight observes the mismatch.
    result = run_runner(workspace, extra_environment={"FAKE_TOPIC_TYPE": "geometry_msgs/msg/Pose"})

    # Then: it directs the operator to the external chain without producing measurement metrics.
    assert result.returncode != 0
    assert "Livox + ROS 2 FAST-LIO + localization chain" in result.stderr
    runtime_log = workspace.log.read_text(encoding="utf-8")
    assert "ros2:topic type /localization/odom" in runtime_log
    assert "ros2:run ed_uav_localization" not in runtime_log
    evidence = evidence_directory(workspace)
    assert not (evidence / "result.json").exists()


def test_lidar_odometry_runner_preserves_demo_failure_and_rejects_topic_flag_override(tmp_path: Path) -> None:
    # Given: a preflight-ready chain whose demo reports a failed trial.
    workspace = create_fake_workspace(tmp_path)

    # When: the demo exits nonzero after emitting its structured result.
    result = run_runner(workspace, extra_environment={"FAKE_DEMO_STATUS": "23"})

    # Then: the runner retains the demo exit status and its JSON evidence.
    assert result.returncode == 23
    evidence = evidence_directory(workspace)
    assert (evidence / "result.json").is_file()

    # When: a caller tries to split the runner and demo topic configuration.
    override = run_runner(workspace, ("--odom-topic", "/other/odom"))

    # Then: it fails before interacting with ROS and directs the caller to ODOM_TOPIC.
    assert override.returncode != 0
    assert "use ODOM_TOPIC" in override.stderr


@pytest.mark.parametrize(
    "invalid_result_line",
    (
        f"{RESULT_PREFIX}{{not-json",
        f"{RESULT_PREFIX}[]",
        f'{RESULT_PREFIX}{{"schema_version":1,"status":"passed"}}',
    ),
)
def test_lidar_odometry_runner_rejects_invalid_result_json(tmp_path: Path, invalid_result_line: str) -> None:
    # Given: a demo that exits successfully but emits an invalid structured result.
    workspace = create_fake_workspace(tmp_path)

    # When: the runner extracts the one prefixed line.
    result = run_runner(workspace, extra_environment={"FAKE_RESULT_LINE": invalid_result_line})

    # Then: it fails closed and retains only the unaccepted candidate as evidence.
    assert result.returncode != 0
    evidence = evidence_directory(workspace)
    assert not (evidence / "result.json").exists()
    assert (evidence / "result.json.candidate").is_file()
    assert "reason=invalid_result_json" in (evidence / "result.txt").read_text(encoding="utf-8")


def test_lidar_odometry_runner_explains_missing_overlay_when_build_is_skipped(tmp_path: Path) -> None:
    # Given: an already-skipped build whose expected install overlay is absent.
    workspace = create_fake_workspace(tmp_path)
    (workspace.root / "ros2_ws" / "install" / "setup.bash").unlink()

    # When: the runner is asked not to rebuild the workspace.
    result = run_runner(workspace, extra_environment={"ED_ODOMETRY_DEMO_SKIP_BUILD": "1"})

    # Then: it exits before ROS preflight with a stable reason and remediation.
    assert result.returncode != 0
    assert "rerun without ED_ODOMETRY_DEMO_SKIP_BUILD=1" in result.stderr
    evidence = evidence_directory(workspace)
    assert "reason=missing_install_overlay" in (evidence / "result.txt").read_text(encoding="utf-8")
    assert not workspace.log.exists()


def test_lidar_odometry_runner_postprocesses_demo_interrupt_without_orphaning_it(tmp_path: Path) -> None:
    # Given: a foreground fake demo that publishes an interrupted result only after SIGINT.
    workspace = create_fake_workspace(tmp_path)
    demo_pid_file = workspace.root / "demo.pid"
    environment = {
        **os.environ,
        "FAKE_RUNTIME_LOG": str(workspace.log),
        "FAKE_DEMO_PID": str(demo_pid_file),
        "FAKE_DEMO_WAIT_FOR_INT": "1",
        "FAKE_INTERRUPT_RESULT_LINE": (
            f'{RESULT_PREFIX}{{"input_topic":"/localization/odom","interpretation":"relative behavior",'
            '"metrics":null,"rejected_count":0,"sample_count":0,"schema_version":1,'
            '"status":"INTERRUPTED","trial":"stationary"}'
        ),
        "PATH": f"{workspace.root / 'fake-bin'}:{os.environ['PATH']}",
    }
    runner = subprocess.Popen(
        [str(workspace.runner)],
        cwd=workspace.root,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    demo_pid: int | None = None
    try:
        for _ in range(100):
            if demo_pid_file.is_file():
                demo_pid = int(demo_pid_file.read_text(encoding="utf-8"))
                break
            time.sleep(0.02)
        assert demo_pid is not None, "fake demo did not start"

        # When: Ctrl-C reaches the foreground process group.
        os.killpg(runner.pid, signal.SIGINT)
        stdout, stderr = runner.communicate(timeout=5)

        # Then: the interrupted JSON is accepted, the wrapper fails, and the demo is gone.
        assert runner.returncode != 0, stdout + stderr
        evidence = evidence_directory(workspace)
        result_json = json.loads((evidence / "result.json").read_text(encoding="utf-8"))
        assert result_json["status"] == "INTERRUPTED"
        assert f"LIDAR_ODOMETRY_RESULT={evidence / 'result.json'}" in stdout
        try:
            os.kill(demo_pid, 0)
        except ProcessLookupError:
            pass
        else:
            raise AssertionError("foreground demo remains after SIGINT")
    finally:
        if runner.poll() is None:
            os.killpg(runner.pid, signal.SIGKILL)
            runner.wait(timeout=5)
