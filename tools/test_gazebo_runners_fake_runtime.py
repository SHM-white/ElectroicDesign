from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER_NAMES = ("run_gazebo_sim.sh", "run_gazebo_smoke.sh", "run_gazebo_slam_nav.sh")


def test_gazebo_runners_forward_modes_and_create_evidence_with_fake_humble() -> None:
    # Given: isolated copies of the entry points and a fake shared Humble runner.
    with tempfile.TemporaryDirectory(prefix="ed-gazebo-runner-") as temp_dir:
        temp_root = Path(temp_dir)
        (temp_root / "tools").mkdir(parents=True)
        for runner_name in RUNNER_NAMES:
            shutil.copy2(REPOSITORY_ROOT / "tools" / runner_name, temp_root / "tools" / runner_name)
        fake_runner = temp_root / "tools" / "run_humble.sh"
        fake_runner.write_text(
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "printf '%s\\n' \"$*\" >\"${FAKE_HUMBLE_ARGS:?}\"\n"
            "cat >\"${FAKE_HUMBLE_STDIN:?}\"\n",
            encoding="utf-8",
        )
        fake_runner.chmod(0o755)

        # When: each wrapper runs against the fake runtime.
        for runner_name in RUNNER_NAMES:
            args_file = temp_root / f"{runner_name}.args"
            stdin_file = temp_root / f"{runner_name}.stdin"
            result = subprocess.run(
                [str(temp_root / "tools" / runner_name)],
                cwd=temp_root,
                env={
                    **os.environ,
                    "FAKE_HUMBLE_ARGS": str(args_file),
                    "FAKE_HUMBLE_STDIN": str(stdin_file),
                },
                check=False,
                capture_output=True,
                text=True,
            )

            # Then: the wrapper succeeds without Docker and forwards the fixed inner command.
            assert result.returncode == 0, result.stderr
            args = args_file.read_text()
            if runner_name == "run_gazebo_slam_nav.sh":
                assert "bash -s --" in args
                assert f"{temp_root} {temp_root}/.omo/evidence/gazebo/" in args
                assert "/workspace /workspace/.omo/evidence/gazebo/" in args
            else:
                assert "bash -s -- /workspace/.omo/evidence/gazebo/" in args
            assert "ed_uav_gazebo gazebo_simulation.launch.py" in stdin_file.read_text()


def test_slam_nav_inner_preamble_selects_host_workspace_in_fake_native_mode() -> None:
    # Given: a native workspace and its evidence directory, but no /workspace mount.
    with tempfile.TemporaryDirectory(prefix="ed-gazebo-native-") as temp_dir:
        temp_root = Path(temp_dir)
        host_workspace = temp_root / "workspace"
        host_evidence = host_workspace / ".omo" / "evidence" / "gazebo" / "native-run"
        (host_workspace / "ros2_ws").mkdir(parents=True)
        (host_workspace / "tools").mkdir()
        (host_workspace / "tools" / "run_humble.sh").touch()
        source = (REPOSITORY_ROOT / "tools" / "run_gazebo_slam_nav.sh").read_text(encoding="utf-8")
        inner_source = source.split("CONTAINER_SCRIPT' &\n", maxsplit=1)[1].split(
            "\nCONTAINER_SCRIPT\n", maxsplit=1
        )[0]
        preamble = inner_source.split("python3 - ", maxsplit=1)[0].replace(
            "source /opt/ros/humble/setup.bash", ":"
        )

        # When: the real inner preamble runs with native and container path pairs.
        result = subprocess.run(
            [
                "bash",
                "-s",
                "--",
                str(host_workspace),
                str(host_evidence),
                "/workspace",
                "/workspace/.omo/evidence/gazebo/native-run",
            ],
            input=preamble,
            cwd=temp_root,
            check=False,
            capture_output=True,
            text=True,
        )

        # Then: evidence belongs to the host workspace and Gazebo resources do not use /workspace.
        assert result.returncode == 0, result.stderr
        environment = (host_evidence / "environment.txt").read_text(encoding="utf-8")
        assert str(host_workspace / "ros2_ws" / "src" / "ed_uav_gazebo") in environment
        assert "=/workspace/" not in environment


def test_slam_nav_writes_failure_when_terminal_sigint_interrupts_acquisition() -> None:
    # Given: an outer runner with a fake Humble child still acquiring dependencies.
    with tempfile.TemporaryDirectory(prefix="ed-gazebo-sigint-") as temp_dir:
        temp_root = Path(temp_dir)
        tools_dir = temp_root / "tools"
        tools_dir.mkdir()
        shutil.copy2(REPOSITORY_ROOT / "tools" / "run_gazebo_slam_nav.sh", tools_dir)
        child_pid_path = temp_root / "acquisition.pid"
        fake_runner = tools_dir / "run_humble.sh"
        fake_runner.write_text(
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "trap '' INT\n"
            "sleep 30 &\n"
            "acquisition_pid=$!\n"
            "printf '%s\\n' \"$acquisition_pid\" >\"${FAKE_ACQUISITION_PID:?}\"\n"
            "wait \"$acquisition_pid\"\n",
            encoding="utf-8",
        )
        fake_runner.chmod(0o755)
        runner = subprocess.Popen(
            [str(tools_dir / "run_gazebo_slam_nav.sh")],
            cwd=temp_root,
            env={**os.environ, "FAKE_ACQUISITION_PID": str(child_pid_path)},
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        acquisition_pid: int | None = None
        try:
            for _ in range(100):
                if child_pid_path.is_file():
                    acquisition_pid = int(child_pid_path.read_text(encoding="utf-8"))
                    break
                time.sleep(0.02)
            assert acquisition_pid is not None, "fake acquisition child did not start"

            # When: an interactive terminal sends SIGINT to the outer process group.
            os.killpg(runner.pid, signal.SIGINT)
            stdout, stderr = runner.communicate(timeout=5)

            # Then: pre-launch interruption is a failed run with no surviving owned child.
            assert runner.returncode == 130, stdout + stderr
            evidence_dirs = list((temp_root / ".omo" / "evidence" / "gazebo").iterdir())
            assert len(evidence_dirs) == 1
            failed = evidence_dirs[0] / "FAILED"
            assert failed.read_text(encoding="utf-8") == "GAZEBO_SLAM_NAV_FAILED exit_code=130\n"
            assert not (evidence_dirs[0] / "SUCCESS").exists()
            try:
                os.kill(acquisition_pid, 0)
            except ProcessLookupError:
                pass
            else:
                raise AssertionError("fake acquisition child remains after outer SIGINT")
        finally:
            if runner.poll() is None:
                os.killpg(runner.pid, signal.SIGKILL)
                runner.wait(timeout=5)
            if acquisition_pid is not None:
                try:
                    os.kill(acquisition_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def test_nested_launch_pipeline_propagates_ros2_failure() -> None:
    # Given: the exact nested launch command and a ros2 executable that fails.
    with tempfile.TemporaryDirectory(prefix="ed-gazebo-launch-") as temp_dir:
        temp_root = Path(temp_dir)
        fake_ros2 = temp_root / "ros2"
        fake_ros2.write_text("#!/usr/bin/env bash\nexit 17\n", encoding="utf-8")
        fake_ros2.chmod(0o755)

        for runner_name in RUNNER_NAMES:
            source = (REPOSITORY_ROOT / "tools" / runner_name).read_text(encoding="utf-8")
            matches = re.findall(r"setsid bash -c \\\n\s+'([^']+)'", source)
            launch_command = next((command for command in matches if "ros2 launch" in command), None)
            assert launch_command is not None, f"missing nested launch command in {runner_name}"

            # When: the launch side of the pipeline exits nonzero.
            result = subprocess.run(
                ["bash", "-c", launch_command, "bash", str(temp_root / f"{runner_name}.log")],
                env={**os.environ, "PATH": f"{temp_root}:{os.environ['PATH']}"},
                check=False,
            )

            # Then: tee must not mask the launch failure.
            assert result.returncode == 17, runner_name
