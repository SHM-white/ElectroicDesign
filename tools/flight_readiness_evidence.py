"""BOM and artifact evidence parsing for flight-readiness checks."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Final, TypeAlias

Json: TypeAlias = None | bool | int | float | str | list["Json"] | dict[str, "Json"]
HASH = re.compile(r"[0-9a-f]{64}\Z")
REQUIRED_BOM_IDS: Final = frozenset({"mid-360", "i5-computer", "narrow-uvc-camera", "wide-uvc-camera", "airframe-guards", "propulsion-set", "battery-distribution", "power-conversion", "cabling-kit", "sensor-mount-kit"})
ZERO_POWER_IDS: Final = frozenset({"airframe-guards", "cabling-kit", "sensor-mount-kit"})
ACTIVE_UNKNOWN_OK: Final = frozenset({"scenario-gated"})
STATUSES: Final = frozenset({"known", "unknown"})
ARTIFACTS: Final = frozenset({"config", "thrust_power_csv", "cg_clearance_photo", "thermal_log", "vibration_spectrum"})


def read_json(path: Path, label: str, errors: list[str]) -> dict[str, Json] | None:
    try:
        raw: Json = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        errors.append(f"missing {label}: {path}"); return None
    except IsADirectoryError:
        errors.append(f"cannot read {label} JSON: {path} is a directory"); return None
    except NotADirectoryError:
        errors.append(f"cannot read {label} JSON: parent path is not a directory: {path}"); return None
    except PermissionError:
        errors.append(f"cannot read {label} JSON: permission denied: {path}"); return None
    except json.JSONDecodeError:
        errors.append(f"invalid JSON: {path}"); return None
    except UnicodeDecodeError:
        errors.append(f"invalid UTF-8: {path}"); return None
    if isinstance(raw, dict): return raw
    errors.append(f"JSON root must be an object: {path}"); return None


def require_map(value: Json, field: str, errors: list[str]) -> dict[str, Json] | None:
    if isinstance(value, dict): return value
    errors.append(f"{field} must be an object"); return None


def require_list(value: Json, field: str, errors: list[str], *, nonempty: bool = False) -> list[Json]:
    if isinstance(value, list):
        if nonempty and not value: errors.append(f"{field} must be nonempty")
        return value
    errors.append(f"{field} must be a list"); return []


def finite(value: Json, field: str, errors: list[str], *, positive: bool = False, signed: bool = False) -> float | None:
    if isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value)):
        number = float(value)
        if signed or number > 0 or (not positive and number >= 0): return number
    if positive:
        errors.append(f"{field} must be positive"); return None
    errors.append(f"invalid {'finite' if signed else 'nonnegative'} value: {field}"); return None


def validate_bom(path: Path, errors: list[str]) -> tuple[float, float, float] | None:
    if path.suffix.lower() in {".yaml", ".yml"}:
        errors.append("YAML BOM is unsupported in this dependency-free checker; provide JSON"); return None
    bom = read_json(path, "BOM", errors)
    if bom is None: return None
    if bom.get("schema_version") != 1: errors.append("BOM schema_version must be 1")
    known_mass, known_steady, known_peak, ids = 0.0, 0.0, 0.0, set[str]()
    for item_raw in require_list(bom.get("items"), "BOM items", errors, nonempty=True):
        item = require_map(item_raw, "BOM item", errors)
        if item is None: continue
        item_id = item.get("id")
        if not isinstance(item_id, str) or not item_id:
            errors.append("missing BOM item id"); item_id = "<unknown>"
        elif item_id in ids:
            errors.append(f"duplicate BOM item id: {item_id}")
        ids.add(str(item_id))
        quantity_raw = item.get("quantity")
        quantity = quantity_raw if isinstance(quantity_raw, int) and not isinstance(quantity_raw, bool) and quantity_raw > 0 else 0
        if quantity == 0: errors.append(f"invalid BOM quantity: {item_id}")
        active = str(item.get("ownership")) not in ACTIVE_UNKNOWN_OK
        values = validate_bom_item(item, str(item_id), active, errors)
        if values is not None:
            mass, steady, peak = values
            known_mass += mass * quantity; known_steady += steady * quantity; known_peak += peak * quantity
    for required_id in sorted(REQUIRED_BOM_IDS - ids):
        errors.append(f"missing required BOM item: {required_id}")
    validate_totals(require_map(bom.get("totals"), "BOM totals", errors), (known_mass, known_steady, known_peak), errors)
    return known_mass, known_steady, known_peak


def validate_bom_item(item: dict[str, Json], item_id: str, active: bool, errors: list[str]) -> tuple[float, float, float] | None:
    result: list[float] = []
    for value_field, status_field, label in (("mass_g", "mass_status", "mass"), ("steady_w", "steady_power_status", "steady power"), ("peak_w", "peak_power_status", "peak power")):
        status, value = item.get(status_field), item.get(value_field)
        if status not in STATUSES: errors.append(f"invalid BOM status: {status_field}: {item_id}")
        if active and status == "unknown": errors.append(f"BOM required active item has unknown {label}: {item_id}")
        if status == "unknown" and value is not None: errors.append(f"unknown {label} recorded as a value: {item_id}")
        if status == "known":
            if value == 0 and active and value_field == "mass_g": errors.append(f"active BOM mass must be positive: {item_id}")
            positive = value_field == "mass_g" and active
            measured = finite(value, f"BOM {label}: {item_id}", errors, positive=positive)
            if measured == 0 and active and value_field in {"steady_w", "peak_w"} and item_id not in ZERO_POWER_IDS:
                errors.append(f"powered BOM {label} must be positive: {item_id}")
            result.append(measured if measured is not None else 0.0)
        else:
            result.append(0.0)
    return result[0], result[1], result[2]


def validate_totals(totals: dict[str, Json] | None, expected: tuple[float, float, float], errors: list[str]) -> None:
    if totals is None: return
    for field, value in (("known_mass_g", expected[0]), ("known_steady_w", expected[1]), ("known_peak_w", expected[2])):
        actual = finite(totals.get(field), f"BOM total {field}", errors)
        if actual is not None and not math.isclose(actual, value, rel_tol=0, abs_tol=0.001): errors.append(f"BOM total mismatch: {field}")


def validate_artifact(record: Json, kind: str, measurement_dir: Path, errors: list[str]) -> tuple[str, Path] | None:
    artifact = require_map(record, f"artifact {kind}", errors)
    if artifact is None: return None
    rel, expected = artifact.get("path"), artifact.get("sha256")
    if not isinstance(rel, str) or Path(rel).is_absolute(): errors.append(f"invalid artifact path: {kind}"); return None
    candidate = measurement_dir / rel
    if candidate.is_symlink(): errors.append(f"artifact must be a regular file: {kind}"); return None
    target = candidate.resolve()
    if not target.is_relative_to(measurement_dir.resolve()): errors.append(f"artifact path escapes measurement directory: {kind}"); return None
    if target.is_symlink() or not target.is_file(): errors.append(f"artifact must be a regular file: {kind}"); return None
    if not isinstance(expected, str) or not HASH.fullmatch(expected): errors.append(f"invalid artifact hash: {kind}"); return None
    actual = hashlib.sha256(target.read_bytes()).hexdigest()
    if actual != expected: errors.append(f"artifact hash mismatch: {kind}")
    return actual, target


def validate_artifacts(raw: Json, measurement_dir: Path, errors: list[str]) -> tuple[dict[str, str], Path | None]:
    artifacts = require_map(raw, "artifacts", errors) or {}
    verified: dict[str, str] = {}; config_path: Path | None = None
    for kind in sorted(ARTIFACTS):
        result = validate_artifact(artifacts.get(kind), kind, measurement_dir, errors)
        if result is not None:
            verified[kind] = result[0]
            if kind == "config": config_path = result[1]
    return verified, config_path


def config_from_artifact(path: Path | None, errors: list[str]) -> tuple[str, float] | None:
    if path is None: return None
    try:
        raw: Json = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        errors.append("invalid config artifact JSON"); return None
    except UnicodeDecodeError:
        errors.append("invalid config artifact UTF-8"); return None
    if not isinstance(raw, dict): errors.append("config artifact must be an object"); return None
    config_id = raw.get("config_id")
    if not isinstance(config_id, str) or not config_id: errors.append("missing config artifact config_id"); return None
    limit = finite(raw.get("vibration_peak_limit_g"), "vibration_peak_limit_g", errors, positive=True)
    if limit is None:
        if raw.get("vibration_peak_limit_g") is None: errors.append("missing config vibration_peak_limit_g")
        return None
    return config_id, limit
