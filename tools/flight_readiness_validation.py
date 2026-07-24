"""Validation rules for offline flight-readiness manifests."""

from __future__ import annotations

import math
from datetime import date
from pathlib import Path
from typing import Final

from flight_readiness_evidence import HASH, Json, config_from_artifact, finite, read_json, require_list, require_map, validate_artifacts, validate_bom

MANIFEST: Final = "flight-readiness.json"
SENTINEL_IDS: Final = frozenset({"none", "null", "undefined"})
UNITS: Final = {"all_up_mass_g": "g", "static_thrust_g": "g", "hover_command_percent": "%", "endurance_min": "min", "mid360_shell_c": "C", "cpu_temp_c": "C", "cpu_throttle_c": "C", "load_duration_min": "min", "reset_events": "count", "thermal_throttle_events": "count", "cg_x_mm": "mm", "cg_y_mm": "mm", "cg_z_mm": "mm", "prop_guard_clearance_mm": "mm", "installed_prop_count": "count", "expected_prop_count": "count", "steady_power_w": "W", "peak_power_w": "W", "vibration_peak_g": "g"}


def parse_date(value: Json, field: str, errors: list[str]) -> date | None:
    if not isinstance(value, str):
        if "measured_at:" in field: errors.append(f"missing {field}"); return None
        errors.append(f"missing date: {field}"); return None
    try:
        return date.fromisoformat(value)
    except ValueError:
        errors.append(f"invalid date: {field}"); return None


def validate_instrument_id(value: Json, instruments: set[str], label: str, errors: list[str]) -> None:
    if not isinstance(value, str) or not value:
        errors.append(f"missing {label}instrument_id"); return
    if value.lower() in SENTINEL_IDS:
        errors.append(f"invalid {label}instrument_id"); return
    if value not in instruments: errors.append(f"unknown {label}instrument: {value}")


def validate_instruments(raw: Json, measured_at: date | None, errors: list[str]) -> set[str]:
    ids: set[str] = set()
    for item_raw in require_list(raw, "instruments", errors, nonempty=True):
        item = require_map(item_raw, "instrument", errors)
        if item is None: continue
        ident = item.get("id")
        if not isinstance(ident, str) or not ident:
            errors.append("missing instrument id"); continue
        if ident.lower() in SENTINEL_IDS: errors.append(f"invalid instrument id: {ident}")
        if ident in ids: errors.append(f"duplicate instrument id: {ident}")
        ids.add(ident)
        for field in ("kind", "serial", "calibration_ref"):
            if not isinstance(item.get(field), str) or not item.get(field): errors.append(f"missing instrument {field}: {ident}")
        calibrated_at = parse_date(item.get("calibrated_at"), f"instrument {ident} calibrated_at", errors)
        if calibrated_at is not None and measured_at is not None and calibrated_at > measured_at: errors.append(f"instrument calibrated after measurement: {ident}")
    return ids


def measured_value(values: dict[str, Json], instruments: set[str], name: str, measured_at: date | None, errors: list[str]) -> float:
    record = require_map(values.get(name), name, errors)
    if record is None: return math.nan
    value = finite(record.get("value"), name, errors, positive=name in {"all_up_mass_g", "static_thrust_g"}, signed=name in {"cg_x_mm", "cg_y_mm", "cg_z_mm"})
    if UNITS[name] == "count" and (not isinstance(record.get("value"), int) or isinstance(record.get("value"), bool)): errors.append(f"count must be an integer: {name}")
    if finite(record.get("uncertainty"), f"{name} uncertainty", errors) is None: return math.nan
    if record.get("source_kind") != "measured": errors.append(f"{'static thrust' if name == 'static_thrust_g' else name.replace('_', ' ')} source_kind must be measured")
    if record.get("unit") != UNITS[name]: errors.append(f"invalid unit: {name}")
    for field in ("method", "conditions"):
        if not isinstance(record.get(field), str) or not record.get(field): errors.append(f"missing {field}: {name}")
    sample_date = parse_date(record.get("measured_at"), f"{name}.measured_at", errors)
    if sample_date is not None and measured_at is not None and sample_date != measured_at: errors.append(f"measured_at must equal manifest measured_at: {name}")
    validate_instrument_id(record.get("instrument_id"), instruments, "", errors)
    return value if value is not None else math.nan


def validate_trace_records(raw: Json, label: str, instruments: set[str], measured_at: date | None, errors: list[str]) -> list[dict[str, Json]]:
    records, ids = [], set[str]()
    for record_raw in require_list(raw, label, errors, nonempty=True):
        record = require_map(record_raw, label[:-1], errors)
        if record is None: continue
        ident = record.get("id")
        if not isinstance(ident, str) or not ident: errors.append(f"missing {label[:-1]} id"); ident = "<unknown>"
        elif ident in ids: errors.append(f"duplicate {label[:-1]} id: {ident}")
        ids.add(str(ident)); records.append(record)
        if record.get("source_kind") != "measured": errors.append(f"{label[:-1]} source_kind must be measured: {ident}")
        validate_instrument_id(record.get("instrument_id"), instruments, f"{label[:-1]} ", errors)
        for field in ("unit", "method", "conditions"):
            if not isinstance(record.get(field), str) or not record.get(field): errors.append(f"missing {label[:-1]} {field}: {ident}")
        finite(record.get("uncertainty"), f"{label[:-1]} uncertainty: {ident}", errors)
        sample_date = parse_date(record.get("measured_at"), f"{label[:-1]} measured_at: {ident}", errors)
        if sample_date is not None and measured_at is not None and sample_date != measured_at: errors.append(f"{label[:-1]} measured_at must equal manifest measured_at: {ident}")
    return records


def check_regulators(raw: Json, instruments: set[str], measured_at: date | None, errors: list[str]) -> None:
    for record in validate_trace_records(raw, "regulators", instruments, measured_at, errors):
        reg_id = str(record.get("id", "<unknown>"))
        continuous = finite(record.get("continuous_a"), f"regulator continuous: {reg_id}", errors)
        peak = finite(record.get("measured_peak_a"), f"regulator peak: {reg_id}", errors)
        if record.get("unit") != "A": errors.append(f"invalid regulator unit: {reg_id}")
        if continuous is not None and peak is not None and continuous < peak * 1.3: errors.append(f"regulator margin below 30%: {reg_id}")


def check_rails(raw: Json, instruments: set[str], measured_at: date | None, errors: list[str]) -> None:
    for record in validate_trace_records(raw, "rails", instruments, measured_at, errors):
        rail_id = str(record.get("id", "<unknown>"))
        values = [finite(record.get(field), f"rail {field}: {rail_id}", errors) for field in ("measured_min_v", "measured_max_v", "device_min_v", "device_max_v")]
        if record.get("unit") != "V": errors.append(f"invalid rail unit: {rail_id}")
        if not isinstance(record.get("transient"), str) or not record.get("transient"): errors.append(f"missing rail transient: {rail_id}")
        if all(value is not None for value in values) and not values[2] <= values[0] <= values[1] <= values[3]: errors.append(f"rail outside device limits: {rail_id}")


def check_cg(readings: dict[str, float], envelope: dict[str, Json] | None, errors: list[str]) -> None:
    if envelope is None: return
    for axis in ("x", "y", "z"):
        low = finite(envelope.get(f"{axis}_min"), f"cg {axis}_min", errors, signed=True)
        high = finite(envelope.get(f"{axis}_max"), f"cg {axis}_max", errors, signed=True)
        if low is not None and high is not None and not low <= readings[f"cg_{axis}_mm"] <= high: errors.append("CG outside envelope")


def check_thresholds(readings: dict[str, float], mission: float, manifest: dict[str, Json], instruments: set[str], measured_at: date | None, errors: list[str]) -> None:
    checks = ((readings["static_thrust_g"] / readings["all_up_mass_g"] < 2.0, "thrust-to-weight below 2.0"), (readings["hover_command_percent"] > 50, "hover command above 50%"), (readings["endurance_min"] < mission * 1.5, "endurance below 1.5x mission duration"), (readings["mid360_shell_c"] > 70, "Mid-360 shell exceeds 70 C"), (readings["cpu_throttle_c"] - readings["cpu_temp_c"] < 10, "CPU thermal margin below 10 C"), (readings["load_duration_min"] < 10 or readings["reset_events"] != 0 or readings["thermal_throttle_events"] != 0, "compute/sensor load gate failed"), (readings["prop_guard_clearance_mm"] <= 0, "prop clearance must be positive"), (readings["installed_prop_count"] <= 0 or readings["installed_prop_count"] != readings["expected_prop_count"], "prop count mismatch"))
    for failed, message in checks:
        if failed: errors.append(message)
    check_cg(readings, require_map(manifest.get("cg_envelope_mm"), "cg_envelope_mm", errors), errors)
    check_regulators(manifest.get("regulators"), instruments, measured_at, errors); check_rails(manifest.get("rails"), instruments, measured_at, errors)


def validate_manifest(path: Path, measurement_dir: Path, bom_totals: tuple[float, float, float] | None, errors: list[str]) -> None:
    manifest = read_json(path, "readiness manifest", errors)
    if manifest is None: return
    measured_at = parse_date(manifest.get("measured_at"), "measured_at", errors)
    if measured_at is not None and not measurement_dir.name.startswith(measured_at.isoformat()): errors.append("measurement directory date mismatch")
    config_date = parse_date(manifest.get("config_date"), "config_date", errors)
    if config_date is not None and measured_at is not None and config_date != measured_at: errors.append("config_date must equal measured_at")
    if manifest.get("schema_version") != 1: errors.append("manifest schema_version must be 1")
    for field in ("measurement_id", "config_id"):
        if not isinstance(manifest.get(field), str) or not manifest.get(field): errors.append(f"missing {field}")
    config_sha = manifest.get("config_sha256")
    if not isinstance(config_sha, str) or not HASH.fullmatch(config_sha): errors.append("invalid config_sha256")
    instruments = validate_instruments(manifest.get("instruments"), measured_at, errors)
    verified, config_path = validate_artifacts(manifest.get("artifacts"), measurement_dir, errors)
    config = config_from_artifact(config_path, errors)
    if config is not None and config[0] != manifest.get("config_id"): errors.append("config_id mismatch")
    if isinstance(config_sha, str) and HASH.fullmatch(config_sha) and verified.get("config") != config_sha: errors.append("config_sha256 mismatch")
    values = require_map(manifest.get("values"), "values", errors)
    if values is None: return
    readings = {name: measured_value(values, instruments, name, measured_at, errors) for name in UNITS}
    mission = finite(manifest.get("max_mission_duration_min"), "max_mission_duration_min", errors) or math.inf
    if config is not None and readings["vibration_peak_g"] > config[1]: errors.append("vibration peak exceeds config limit")
    reconcile_power(manifest, readings, bom_totals, errors)
    if not math.isnan(readings["all_up_mass_g"]): check_thresholds(readings, mission, manifest, instruments, measured_at, errors)


def reconcile_power(manifest: dict[str, Json], readings: dict[str, float], bom_totals: tuple[float, float, float] | None, errors: list[str]) -> None:
    if bom_totals is None: return
    declared = require_map(manifest.get("bom_totals"), "bom_totals", errors)
    if declared is not None:
        for field, expected in (("known_mass_g", bom_totals[0]), ("known_steady_w", bom_totals[1]), ("known_peak_w", bom_totals[2])):
            actual = finite(declared.get(field), f"manifest {field}", errors)
            if actual is not None and not math.isclose(actual, expected, rel_tol=0, abs_tol=0.001): errors.append(f"BOM reconciliation mismatch: {field}")
    if readings["all_up_mass_g"] < bom_totals[0]: errors.append("measured all-up mass below BOM known mass")
    if readings["steady_power_w"] < bom_totals[1]: errors.append("measured steady power below BOM known steady power")
    if readings["peak_power_w"] < bom_totals[2]: errors.append("measured peak power below BOM known peak power")


def validate(bom: Path, measurements: Path) -> list[str]:
    errors: list[str] = []
    validate_manifest(measurements / MANIFEST, measurements.resolve(), validate_bom(bom, errors), errors)
    return errors
