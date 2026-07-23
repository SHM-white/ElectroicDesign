from __future__ import annotations

import subprocess
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
LAUNCH_CHECKER = BRINGUP_ROOT / "tools" / "verify_launch_surface.py"
BRINGUP_LAUNCH = BRINGUP_ROOT / "launch" / "bringup.launch.py"


def test_core_launch_exposes_only_the_p06_profiles_and_gates_competition_first() -> None:
    # Given: the core P06 bringup launch surface.
    # When: launch arguments and construction order are statically checked.
    result = subprocess.run(
        [sys.executable, str(LAUNCH_CHECKER), "--launch", str(BRINGUP_LAUNCH)],
        capture_output=True,
        check=False,
        text=True,
    )

    # Then: offline, camera-only, lidar, and competition paths remain explicit.
    assert result.returncode == 0, result.stderr
    assert result.stdout == "BRINGUP: GREEN\n"
