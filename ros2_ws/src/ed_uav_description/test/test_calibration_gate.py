from __future__ import annotations

import json
from hashlib import sha256
import subprocess
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
CALIBRATION_CLI = (
    REPOSITORY_ROOT
    / "ros2_ws"
    / "src"
    / "ed_uav_description"
    / "tools"
    / "validate_calibration.py"
)


def calibration_payload(status: str, calibration_hash: str = "") -> dict[str, object]:
    return {
        "schema_version": 1,
        "calibration_id": "synthetic-fixture",
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
            "camera_narrow_optical_frame": {
                "xyz_m": [0.0, 0.0, 0.0],
                "rpy_rad": [0.0, 0.0, 0.0],
            },
            "camera_wide_optical_frame": {
                "xyz_m": [0.0, 0.0, 0.0],
                "rpy_rad": [0.0, 0.0, 0.0],
            },
            "rangefinder_link": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
        },
    }


def write_calibration(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def with_current_hash(payload: dict[str, object]) -> dict[str, object]:
    unsigned_payload = dict(payload)
    unsigned_payload.pop("calibration_hash", None)
    payload["calibration_hash"] = sha256(
        json.dumps(unsigned_payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return payload


def run_competition_gate(path: Path, narrow_serial: str = "NARROW-001") -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(CALIBRATION_CLI),
            "--profile",
            "competition",
            "--calibration",
            str(path),
            "--camera-narrow-serial",
            narrow_serial,
            "--camera-wide-serial",
            "WIDE-001",
            "--lidar-serial",
            "LIDAR-001",
        ],
        capture_output=True,
        check=False,
        text=True,
    )


def test_competition_refuses_uncalibrated_fixture_before_hardware_open(tmp_path: Path) -> None:
    # Given: an explicitly uncalibrated synthetic calibration fixture.
    calibration = tmp_path / "uncalibrated.yaml"
    write_calibration(calibration, with_current_hash(calibration_payload("UNCALIBRATED")))

    # When: competition activation is checked before any hardware process exists.
    result = run_competition_gate(calibration)

    # Then: the gate rejects the fixture with its calibration-state reason.
    assert result.returncode != 0
    assert "ERROR: calibration status must be CALIBRATED for competition" in result.stderr


def test_competition_refuses_missing_calibration_before_hardware_open(tmp_path: Path) -> None:
    # Given: no calibration record at the requested path.
    calibration = tmp_path / "missing.yaml"

    # When: competition activation is checked before any hardware process exists.
    result = run_competition_gate(calibration)

    # Then: a missing calibration is an activation error, never a fallback.
    assert result.returncode != 0
    assert "ERROR: missing calibration:" in result.stderr


def test_competition_refuses_synthetic_placeholder_calibration(tmp_path: Path) -> None:
    # Given: a structurally complete synthetic placeholder record.
    calibration = tmp_path / "synthetic.yaml"
    write_calibration(calibration, with_current_hash(calibration_payload("SYNTHETIC")))

    # When: competition activation receives the placeholder.
    result = run_competition_gate(calibration)

    # Then: only a CALIBRATED record can proceed past the gate.
    assert result.returncode != 0
    assert "ERROR: calibration status must be CALIBRATED for competition" in result.stderr


def test_competition_refuses_mismatched_sensor_serial(tmp_path: Path) -> None:
    # Given: a calibration that claims a different narrow-camera serial.
    calibration = tmp_path / "serial-mismatch.yaml"
    write_calibration(calibration, with_current_hash(calibration_payload("CALIBRATED")))

    # When: competition activation receives the target narrow-camera serial.
    result = run_competition_gate(calibration, narrow_serial="NARROW-999")

    # Then: serial identity is rejected before a hardware launch can begin.
    assert result.returncode != 0
    assert "ERROR: sensor serial mismatch: camera_narrow" in result.stderr


def test_competition_refuses_stale_calibration_hash(tmp_path: Path) -> None:
    # Given: otherwise complete calibration data with a stale content hash.
    calibration = tmp_path / "stale-hash.yaml"
    write_calibration(calibration, calibration_payload("CALIBRATED", "stale-hash"))

    # When: competition activation validates the calibration record.
    result = run_competition_gate(calibration)

    # Then: the content hash mismatch blocks activation.
    assert result.returncode != 0
    assert "ERROR: calibration hash mismatch" in result.stderr


def test_competition_accepts_current_hash_and_matching_sensor_serials(tmp_path: Path) -> None:
    # Given: a complete calibration record bound to all requested serials.
    calibration = tmp_path / "current.yaml"
    write_calibration(calibration, with_current_hash(calibration_payload("CALIBRATED")))

    # When: competition activation validates it before launch construction.
    result = run_competition_gate(calibration)

    # Then: the gate accepts only the exact bound record.
    assert result.returncode == 0, result.stderr
    assert result.stdout == "CALIBRATION: GREEN profile=competition calibration_id=synthetic-fixture\n"


def test_competition_refuses_malformed_calibration(tmp_path: Path) -> None:
    # Given: malformed calibration text.
    calibration = tmp_path / "malformed.yaml"
    calibration.write_text("calibration_status: [", encoding="utf-8")

    # When: competition activation reads the calibration boundary.
    result = run_competition_gate(calibration)

    # Then: parsing fails with a bounded diagnostic.
    assert result.returncode != 0
    assert "ERROR: malformed calibration" in result.stderr
