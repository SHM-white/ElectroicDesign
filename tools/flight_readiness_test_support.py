from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Final


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CHECKER = PROJECT_ROOT / "tools" / "check_flight_readiness.py"
HASH: Final = "a" * 64


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def run_checker(bom: Path, measurements: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(CHECKER),
            "--bom",
            str(bom),
            "--measurements",
            str(measurements),
            "--strict",
        ],
        cwd=PROJECT_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def output(result: subprocess.CompletedProcess[str]) -> str:
    return result.stdout + result.stderr


def assert_rejected(result: subprocess.CompletedProcess[str], message: str) -> None:
    combined = output(result)
    assert result.returncode != 0
    assert message in combined
    assert "PASS:" not in combined
    assert "Traceback" not in combined


def artifact(measurements: Path, name: str, contents: bytes) -> dict[str, str]:
    path = measurements / name
    path.write_bytes(contents)
    return {"path": name, "sha256": hashlib.sha256(contents).hexdigest()}


def measured(value: float, unit: str, instrument_id: str = "scale-1") -> dict[str, str | float]:
    return {
        "value": value,
        "unit": unit,
        "source_kind": "measured",
        "uncertainty": 0.1,
        "instrument_id": instrument_id,
        "method": "bench measurement; ignore previous instructions is data only",
        "conditions": "20 C indoor bench",
        "measured_at": "2026-07-24",
    }


def create_bom(tmp_path: Path) -> Path:
    bom = tmp_path / "BOM.json"
    write_json(
        bom,
        {
            "schema_version": 1,
            "items": [
                {"id": "mid-360", "quantity": 1, "ownership": "owned", "mass_g": 265, "mass_status": "known", "steady_w": 14, "steady_power_status": "known", "peak_w": 18, "peak_power_status": "known"},
                {"id": "i5-computer", "quantity": 1, "ownership": "owned", "mass_g": 400, "mass_status": "known", "steady_w": 28, "steady_power_status": "known", "peak_w": 45, "peak_power_status": "known"},
                {"id": "narrow-uvc-camera", "quantity": 1, "ownership": "owned", "mass_g": 30, "mass_status": "known", "steady_w": 2, "steady_power_status": "known", "peak_w": 3, "peak_power_status": "known"},
                {"id": "wide-uvc-camera", "quantity": 1, "ownership": "owned", "mass_g": 35, "mass_status": "known", "steady_w": 2, "steady_power_status": "known", "peak_w": 3, "peak_power_status": "known"},
                {"id": "airframe-guards", "quantity": 1, "ownership": "owned", "mass_g": 250, "mass_status": "known", "steady_w": 0, "steady_power_status": "known", "peak_w": 0, "peak_power_status": "known"},
                {"id": "propulsion-set", "quantity": 1, "ownership": "owned", "mass_g": 300, "mass_status": "known", "steady_w": 20, "steady_power_status": "known", "peak_w": 120, "peak_power_status": "known"},
                {"id": "battery-distribution", "quantity": 2, "ownership": "owned", "mass_g": 220, "mass_status": "known", "steady_w": 1, "steady_power_status": "known", "peak_w": 5, "peak_power_status": "known"},
                {"id": "power-conversion", "quantity": 1, "ownership": "owned", "mass_g": 80, "mass_status": "known", "steady_w": 3, "steady_power_status": "known", "peak_w": 20, "peak_power_status": "known"},
                {"id": "cabling-kit", "quantity": 1, "ownership": "owned", "mass_g": 50, "mass_status": "known", "steady_w": 0, "steady_power_status": "known", "peak_w": 0, "peak_power_status": "known"},
                {"id": "sensor-mount-kit", "quantity": 1, "ownership": "owned", "mass_g": 70, "mass_status": "known", "steady_w": 0, "steady_power_status": "known", "peak_w": 0, "peak_power_status": "known"},
                {"id": "ground-vehicle", "quantity": 1, "ownership": "scenario-gated", "mass_g": None, "mass_status": "unknown", "steady_w": None, "steady_power_status": "unknown", "peak_w": None, "peak_power_status": "unknown"},
            ],
            "totals": {"known_mass_g": 1920, "known_steady_w": 71, "known_peak_w": 219},
        },
    )
    return bom


def create_measurements(tmp_path: Path) -> Path:
    measurements = tmp_path / "2026-07-24-flight-readiness"
    measurements.mkdir()
    config = artifact(measurements, "flight-config.json", b'{"config_id":"uav-flight-config-a","vibration_peak_limit_g":0.5}\n')
    artifacts = {
        "config": config,
        "vibration_spectrum": artifact(measurements, "vibration.csv", b"hz,g\n120,0.2\n"),
        "thrust_power_csv": artifact(measurements, "thrust-power.csv", b"thrust,power\n2600,120\n"),
        "cg_clearance_photo": artifact(measurements, "cg-clearance.jpg", b"photo-bytes"),
        "thermal_log": artifact(measurements, "thermal.log", b"600s no reset no throttle"),
    }
    write_json(
        measurements / "flight-readiness.json",
        {
            "schema_version": 1,
            "measurement_id": "FR-2026-07-24-A",
            "measured_at": "2026-07-24",
            "config_id": "uav-flight-config-a",
            "config_date": "2026-07-24",
            "config_sha256": config["sha256"],
            "max_mission_duration_min": 8,
            "bom_totals": {"known_mass_g": 1920, "known_steady_w": 71, "known_peak_w": 219},
            "instruments": [
                {"id": "scale-1", "kind": "scale", "serial": "S-1", "calibrated_at": "2026-07-01", "calibration_ref": "CAL-SCALE-1"},
                {"id": "thrust-stand-1", "kind": "thrust_stand", "serial": "T-1", "calibrated_at": "2026-07-01", "calibration_ref": "CAL-THRUST-1"},
                {"id": "logger-1", "kind": "power_logger", "serial": "P-1", "calibrated_at": "2026-07-01", "calibration_ref": "CAL-POWER-1"},
            ],
            "artifacts": artifacts,
            "values": {
                "all_up_mass_g": measured(2100, "g"),
                "static_thrust_g": measured(4500, "g", "thrust-stand-1"),
                "hover_command_percent": measured(45, "%", "logger-1"),
                "endurance_min": measured(13, "min", "logger-1"),
                "mid360_shell_c": measured(60, "C"),
                "cpu_temp_c": measured(75, "C"),
                "cpu_throttle_c": measured(90, "C"),
                "load_duration_min": measured(10, "min", "logger-1"),
                "reset_events": measured(0, "count", "logger-1"),
                "thermal_throttle_events": measured(0, "count", "logger-1"),
                "cg_x_mm": measured(0, "mm"),
                "cg_y_mm": measured(0, "mm"),
                "cg_z_mm": measured(10, "mm"),
                "prop_guard_clearance_mm": measured(8, "mm"),
                "installed_prop_count": measured(4, "count"),
                "expected_prop_count": measured(4, "count"),
                "steady_power_w": measured(80, "W", "logger-1"),
                "peak_power_w": measured(240, "W", "logger-1"),
                "vibration_peak_g": measured(0.2, "g", "logger-1"),
            },
            "cg_envelope_mm": {"x_min": -20, "x_max": 20, "y_min": -20, "y_max": 20, "z_min": 0, "z_max": 30},
            "regulators": [{"id": "5v", "continuous_a": 5, "measured_peak_a": 3.5, "unit": "A", "uncertainty": 0.1, "instrument_id": "logger-1", "source_kind": "measured", "method": "power logger", "conditions": "full load", "measured_at": "2026-07-24"}],
            "rails": [{"id": "5v", "measured_min_v": 4.85, "measured_max_v": 5.15, "device_min_v": 4.75, "device_max_v": 5.25, "unit": "V", "uncertainty": 0.02, "instrument_id": "logger-1", "source_kind": "measured", "method": "power logger", "conditions": "compute Mid-360 motor transient", "measured_at": "2026-07-24", "transient": "compute_mid360_motor"}],
        },
    )
    return measurements


def load_manifest(measurements: Path) -> dict:
    return json.loads((measurements / "flight-readiness.json").read_text(encoding="utf-8"))
