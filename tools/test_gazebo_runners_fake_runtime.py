from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER_NAMES = ("run_gazebo_sim.sh", "run_gazebo_smoke.sh")


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
            assert "bash -s -- /workspace/.omo/evidence/gazebo/" in args_file.read_text()
            assert "ed_uav_gazebo gazebo_simulation.launch.py" in stdin_file.read_text()


def test_nested_launch_pipeline_propagates_ros2_failure() -> None:
    # Given: the exact nested launch command and a ros2 executable that fails.
    with tempfile.TemporaryDirectory(prefix="ed-gazebo-launch-") as temp_dir:
        temp_root = Path(temp_dir)
        fake_ros2 = temp_root / "ros2"
        fake_ros2.write_text("#!/usr/bin/env bash\nexit 17\n", encoding="utf-8")
        fake_ros2.chmod(0o755)

        for runner_name in RUNNER_NAMES:
            source = (REPOSITORY_ROOT / "tools" / runner_name).read_text(encoding="utf-8")
            match = re.search(r"setsid bash -c \\\n\s+'([^']+)'", source)
            assert match is not None, f"missing nested launch command in {runner_name}"

            # When: the launch side of the pipeline exits nonzero.
            result = subprocess.run(
                ["bash", "-c", match.group(1), "bash", str(temp_root / f"{runner_name}.log")],
                env={**os.environ, "PATH": f"{temp_root}:{os.environ['PATH']}"},
                check=False,
            )

            # Then: tee must not mask the launch failure.
            assert result.returncode == 17, runner_name
