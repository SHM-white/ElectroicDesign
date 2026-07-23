"""Deterministic command-line replay for lidar transport contracts."""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias

from .contracts import (
    GenericCloudFieldError,
    MissingPointTiming,
    PointTimeRegression,
    assess_generic_point_cloud,
    validate_offset_times,
)
from .health import HealthState, evaluate_health

JsonScalar: TypeAlias = str | int | float | bool | None
JsonValue: TypeAlias = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]


@dataclass(frozen=True, slots=True)
class ReplayInputError(Exception):
    detail: str

    def __str__(self) -> str:
        return f"invalid replay input: {self.detail}"


@dataclass(frozen=True, slots=True)
class ReplayHealthFailure(Exception):
    code: str

    def __str__(self) -> str:
        return self.code


def _read_record(path: Path) -> dict[str, JsonValue]:
    try:
        decoded = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ReplayInputError(detail=str(error)) from error
    except json.JSONDecodeError as error:
        raise ReplayInputError(detail=str(error)) from error
    if not isinstance(decoded, dict):
        raise ReplayInputError(detail="root is not an object")
    return decoded


def _read_integer_list(record: dict[str, JsonValue], key: str) -> tuple[int, ...]:
    raw_values = record.get(key)
    if not isinstance(raw_values, list):
        raise ReplayInputError(detail=f"{key} is not a list")
    values = tuple(raw_values)
    if not all(isinstance(value, int) and not isinstance(value, bool) for value in values):
        raise ReplayInputError(detail=f"{key} has a non-integer value")
    return tuple(value for value in values if isinstance(value, int) and not isinstance(value, bool))


def _read_string_list(record: dict[str, JsonValue], key: str) -> tuple[str, ...]:
    raw_values = record.get(key)
    if not isinstance(raw_values, list):
        raise ReplayInputError(detail=f"{key} is not a list")
    values = tuple(raw_values)
    if not all(isinstance(value, str) for value in values):
        raise ReplayInputError(detail=f"{key} has a non-string value")
    return tuple(value for value in values if isinstance(value, str))


def _run_mid360(record: dict[str, JsonValue]) -> str:
    validate_offset_times(_read_integer_list(record, "offset_times_ns"))
    imu_stamp = record.get("imu_stamp_ns")
    if not isinstance(imu_stamp, int) or isinstance(imu_stamp, bool):
        raise ReplayInputError(detail="imu_stamp_ns is not an integer")
    return "MID360_LIO_ELIGIBLE"


def _run_generic(record: dict[str, JsonValue]) -> str:
    assessment = assess_generic_point_cloud(_read_string_list(record, "fields"))
    return "GENERIC_LIO_ELIGIBLE" if assessment.lio_eligible else "GENERIC_MONITORING_ONLY"


def _read_integer(record: dict[str, JsonValue], key: str) -> int:
    value = record.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise ReplayInputError(detail=f"{key} is not an integer")
    return value


def _run_health(record: dict[str, JsonValue]) -> str:
    driver_alive = record.get("driver_alive")
    if not isinstance(driver_alive, bool):
        raise ReplayInputError(detail="driver_alive is not a boolean")
    report = evaluate_health(
        HealthState(
            driver_alive=driver_alive,
            last_driver_steady_ns=_read_integer(record, "last_driver_steady_ns"),
            last_point_steady_ns=_read_integer(record, "last_point_steady_ns"),
            last_imu_steady_ns=_read_integer(record, "last_imu_steady_ns"),
        ),
        now_steady_ns=_read_integer(record, "now_steady_ns"),
        deadline_ns=_read_integer(record, "deadline_ns"),
    )
    if not report.active:
        raise ReplayHealthFailure(code=report.code)
    return report.code


def run_replay(path: Path) -> str:
    """Parse one synthetic sample and return its honest status code."""
    record = _read_record(path)
    kind = record.get("kind")
    match kind:
        case "mid360":
            return _run_mid360(record)
        case "generic":
            return _run_generic(record)
        case "health":
            return _run_health(record)
        case _:
            raise ReplayInputError(detail="kind must be mid360, generic, or health")


def main() -> int:
    """Run the deterministic replay surface with exactly one JSON sample path."""
    if len(sys.argv) != 2:
        print("usage: lidar_replay SAMPLE.json", file=sys.stderr)
        return 64
    try:
        result = run_replay(Path(sys.argv[1]))
    except MissingPointTiming as error:
        print(f"REPLAY: RED: LIDAR_POINT_TIME_MISSING: {error}", file=sys.stderr)
        return 1
    except PointTimeRegression as error:
        print(f"REPLAY: RED: LIDAR_POINT_TIME_REGRESSION: {error}", file=sys.stderr)
        return 1
    except GenericCloudFieldError as error:
        print(f"REPLAY: RED: LIDAR_POINT_FIELDS_INVALID: {error}", file=sys.stderr)
        return 1
    except ReplayHealthFailure as error:
        print(f"REPLAY: RED: {error}", file=sys.stderr)
        return 1
    except ReplayInputError as error:
        print(f"REPLAY: RED: {error}", file=sys.stderr)
        return 2
    print(f"REPLAY: GREEN: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
