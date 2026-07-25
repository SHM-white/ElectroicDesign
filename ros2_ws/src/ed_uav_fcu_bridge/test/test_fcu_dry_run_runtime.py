"""Runtime regressions for the installed FCU bridge and bounded dry-run launch."""

from __future__ import annotations

import os
import re
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

pytest.importorskip("ament_index_python")
from ament_index_python.packages import get_package_prefix  # noqa: E402


SOURCE_ENTRYPOINT = Path(__file__).resolve().parents[1] / "scripts" / "ed_uav_fcu_bridge"


def _installed_entrypoint() -> Path:
    prefix = Path(get_package_prefix("ed_uav_fcu_bridge"))
    return prefix / "lib" / "ed_uav_fcu_bridge" / "ed_uav_fcu_bridge"


def _ros_environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    return environment


def test_source_entrypoint_is_executable() -> None:
    # Given: the tracked source entrypoint installed by ament_cmake.
    mode = SOURCE_ENTRYPOINT.stat().st_mode

    # When: the source mode is inspected.
    executable = bool(mode & stat.S_IXUSR)

    # Then: the source entrypoint can be launched as the installed executable.
    assert executable, f"source entrypoint is not executable: {stat.filemode(mode)}"


def test_symlink_installed_entrypoint_is_executable() -> None:
    # Given: a --symlink-install package prefix.
    installed_entrypoint = _installed_entrypoint()

    # When: the installed bridge target is inspected.
    executable = os.access(installed_entrypoint, os.X_OK)

    # Then: the real installed executable can be launched by launch_ros.actions.Node.
    assert installed_entrypoint.is_symlink(), f"expected symlink: {installed_entrypoint}"
    assert executable, f"installed entrypoint is not executable: {installed_entrypoint}"


@pytest.mark.skipif(shutil.which("ros2") is None, reason="requires a sourced ROS 2 Humble environment")
def test_bounded_dry_run_exits_cleanly_without_bridge_traceback(tmp_path: Path) -> None:
    # Given: a real bounded dry-run launch with a caller-owned PTY path.
    pty_device = tmp_path / "fcu-pty"
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "ed_uav_bringup",
            "fcu_dry_run.launch.py",
            f"pty_device:={pty_device}",
            "duration_seconds:=1",
            "rate_hz:=20",
            "use_sim_time:=false",
        ],
        capture_output=True,
        check=False,
        env=_ros_environment(),
        text=True,
        timeout=25,
    )
    output = result.stdout + result.stderr

    # When: bounded shutdown completes after the fake FCU exits.
    # Then: launch and every child complete successfully without duplicate shutdown output.
    assert result.returncode == 0, output
    assert "FAKE FCU READY:" in output
    assert "ed_uav_fcu_bridge" in output
    assert "exit code 1" not in output
    assert "KeyboardInterrupt" not in output
    assert "process has died" not in output
    assert "exit code -2" not in output
    assert re.search(r"exit code -\d+", output) is None
    assert "rcl_shutdown already called" not in output
    assert "Traceback" not in output
    assert not pty_device.exists(), output
