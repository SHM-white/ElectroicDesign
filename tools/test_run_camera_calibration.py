from pathlib import Path
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parent / "calibration" / "run_camera_calibration.sh"


def test_launcher_reports_required_role_when_standard_input_is_unavailable() -> None:
    # Given: the launcher is started without a role or interactive standard input.
    # When: the command reaches its role-selection boundary.
    completed = subprocess.run(
        [str(SCRIPT)],
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        check=False,
    )

    # Then: it explains how to start calibration instead of exiting silently.
    assert completed.returncode == 2
    assert "Standard input is unavailable" in completed.stderr
    assert "run_camera_calibration.sh [1|2]" in completed.stderr


@pytest.mark.parametrize(("choice", "role"), [("1", "narrow"), ("2", "wide")])
def test_launcher_maps_numeric_choice_to_camera_role(choice: str, role: str) -> None:
    # Given: one numeric camera-role choice and no later interactive input.
    # When: the launcher resolves that role before hardware selection.
    completed = subprocess.run(
        [str(SCRIPT), choice],
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        check=False,
    )

    # Then: the calibration CLI receives the corresponding internal role.
    assert f"Starting {role} camera calibration" in completed.stdout
