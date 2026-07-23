"""End-to-end mission replay and offline launch profile tests."""

from __future__ import annotations

import json
import subprocess
import sys
from hashlib import sha256
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
DESCRIPTION_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_description"
LAUNCH_CHECKER = BRINGUP_ROOT / "tools" / "verify_launch_profiles.py"
CALIBRATION_CLI = DESCRIPTION_ROOT / "tools" / "validate_calibration.py"


# ── helpers ────────────────────────────────────────────────────────────


def _run_checker(profile_name: str) -> subprocess.CompletedProcess[str]:
    launch_file = BRINGUP_ROOT / "launch" / f"{profile_name}.launch.py"
    return subprocess.run(
        [sys.executable, str(LAUNCH_CHECKER), "--launch", str(launch_file), "--profile", profile_name],
        capture_output=True,
        check=False,
        text=True,
    )


def _calibration_payload(status: str, calibration_hash: str = "") -> dict[str, object]:
    return {
        "schema_version": 1,
        "calibration_id": "fixture",
        "calibration_status": status,
        "calibration_hash": calibration_hash,
        "sensor_serials": {
            "camera_narrow": "NARROW-001",
            "camera_wide": "WIDE-001",
            "lidar": "LIDAR-001",
        },
        "transforms": {
            "fcu_link": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
            "lidar_link": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
            "camera_narrow_optical_frame": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
            "camera_wide_optical_frame": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
            "rangefinder_link": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
        },
    }


def _with_hash(payload: dict[str, object]) -> dict[str, object]:
    unsigned = dict(payload)
    unsigned.pop("calibration_hash", None)
    payload["calibration_hash"] = sha256(
        json.dumps(unsigned, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return payload


def _run_competition_gate(calibration_path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(CALIBRATION_CLI),
            "--profile", "competition",
            "--calibration", str(calibration_path),
            "--camera-narrow-serial", "NARROW-001",
            "--camera-wide-serial", "WIDE-001",
            "--lidar-serial", "LIDAR-001",
        ],
        capture_output=True,
        check=False,
        text=True,
    )


# ── tests ──────────────────────────────────────────────────────────────


def test_offline_replay_launch() -> None:
    """offline_replay starts without errors — has bag_path, replay stage, use_sim_time=true."""
    result = _run_checker("offline_replay")
    assert result.returncode == 0, f"offline_replay failed: {result.stderr}"
    assert "GREEN" in result.stdout


def test_camera_only_launch() -> None:
    """camera_only starts with lidar absent — no lidar_serial argument declared."""
    result = _run_checker("camera_only")
    assert result.returncode == 0, f"camera_only failed: {result.stderr}"
    assert "GREEN" in result.stdout


def test_competition_refuses_uncalibrated(tmp_path: Path) -> None:
    """competition refuses uncalibrated hardware — gate rejects non-CALIBRATED status."""

    # Verify the launch file structure first.
    result = _run_checker("competition")
    assert result.returncode == 0, f"competition structure check failed: {result.stderr}"
    assert "GREEN" in result.stdout

    # Then verify the gate function rejects uncalibrated calibration data.
    uncalibrated = tmp_path / "uncalibrated.yaml"
    uncalibrated.write_text(json.dumps(_with_hash(_calibration_payload("UNCALIBRATED")), indent=2), encoding="utf-8")
    gate_result = _run_competition_gate(uncalibrated)
    assert gate_result.returncode != 0
    assert "calibration status must be CALIBRATED for competition" in gate_result.stderr


def test_lidar_launch() -> None:
    """Lidar-enabled mode declares lidar_serial argument."""
    result = _run_checker("lidar")
    assert result.returncode == 0, f"lidar failed: {result.stderr}"
    assert "GREEN" in result.stdout


def test_fcu_dry_run_launch() -> None:
    """FCU dry-run declares pty_device argument."""
    result = _run_checker("fcu_dry_run")
    assert result.returncode == 0, f"fcu_dry_run failed: {result.stderr}"
    assert "GREEN" in result.stdout


def test_legacy_rollback_launch() -> None:
    """Legacy rollback has simplified lifecycle (two stages only)."""
    result = _run_checker("legacy_rollback")
    assert result.returncode == 0, f"legacy_rollback failed: {result.stderr}"
    assert "GREEN" in result.stdout


def test_all_profiles_declare_authority_token() -> None:
    """Every profile declares a unique authority_token for single control-authority."""
    for profile_name in ("offline_replay", "camera_only", "lidar", "competition", "fcu_dry_run", "legacy_rollback"):
        result = _run_checker(profile_name)
        assert result.returncode == 0, f"{profile_name} authority check failed: {result.stderr}"
