#!/usr/bin/env python3
"""Validate external D-task YAML/JSON against schemas and typed models."""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Final, TypeAlias

import yaml
from jsonschema import Draft202012Validator
from jsonschema.exceptions import SchemaError, ValidationError as JsonSchemaValidationError
from pydantic import JsonValue, TypeAdapter, ValidationError
from typing_extensions import assert_never

from d_task_models import (
    DEPLOYMENT_ADAPTER,
    ESP32_FRAMES_ADAPTER,
    MISSION_ADAPTER,
    TARGET_ADAPTER,
    DocumentationDeployment,
    Esp32FrameWindow,
    FieldDeployment,
    MissionProfile,
    TargetProfile,
)


JSON_VALUE_ADAPTER: Final = TypeAdapter(JsonValue)
ConfigDocument: TypeAlias = (
    MissionProfile
    | TargetProfile
    | DocumentationDeployment
    | FieldDeployment
    | Esp32FrameWindow
)


class ConfigKind(str, Enum):
    MISSION = "mission"
    TARGET = "target"
    DEPLOYMENT = "deployment"
    ESP32_FRAMES = "esp32-frames"


@dataclass(frozen=True, slots=True)
class ConfigBoundaryError(ValueError):
    source: str
    reason: str

    def __str__(self) -> str:
        return f"{self.source}: {self.reason}"


def _schema_name(kind: ConfigKind) -> str:
    match kind:
        case ConfigKind.MISSION:
            return "mission_profile.schema.json"
        case ConfigKind.TARGET:
            return "target_revision.schema.json"
        case ConfigKind.DEPLOYMENT:
            return "deployment_preset.schema.json"
        case ConfigKind.ESP32_FRAMES:
            return "esp32_frames.schema.json"
        case unreachable:
            assert_never(unreachable)


def _parse_document(path: Path) -> JsonValue:
    try:
        source = path.read_text(encoding="utf-8")
        raw = yaml.safe_load(source)
        return JSON_VALUE_ADAPTER.validate_python(raw)
    except OSError as error:
        raise ConfigBoundaryError(str(path), f"cannot read input: {error}") from error
    except yaml.YAMLError as error:
        raise ConfigBoundaryError(str(path), f"malformed YAML/JSON: {error}") from error
    except ValidationError as error:
        raise ConfigBoundaryError(str(path), f"unsupported YAML/JSON value: {error}") from error


def load_config(kind: ConfigKind, path: Path) -> ConfigDocument:
    """Parse one untrusted config through JSON Schema and a typed model."""
    document = _parse_document(path)
    script_path = Path(__file__).resolve()
    source_schema_root = script_path.parents[1] / "contracts" / "d_task" / "schemas"
    installed_schema_root = (
        script_path.parents[2]
        / "share"
        / "ed_uav_interfaces"
        / "contracts"
        / "d_task"
        / "schemas"
    )
    schema_root = source_schema_root if source_schema_root.is_dir() else installed_schema_root
    schema_path = schema_root / _schema_name(kind)
    try:
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        Draft202012Validator(schema).validate(document)
        match kind:
            case ConfigKind.MISSION:
                return MISSION_ADAPTER.validate_python(document)
            case ConfigKind.TARGET:
                return TARGET_ADAPTER.validate_python(document)
            case ConfigKind.DEPLOYMENT:
                return DEPLOYMENT_ADAPTER.validate_python(document)
            case ConfigKind.ESP32_FRAMES:
                return ESP32_FRAMES_ADAPTER.validate_python(document)
            case unreachable:
                assert_never(unreachable)
    except OSError as error:
        raise ConfigBoundaryError(str(schema_path), f"cannot read schema: {error}") from error
    except json.JSONDecodeError as error:
        raise ConfigBoundaryError(str(schema_path), f"malformed schema: {error}") from error
    except (SchemaError, JsonSchemaValidationError, ValidationError) as error:
        raise ConfigBoundaryError(str(path), str(error)) from error


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print("usage: check_d_task_config.py KIND PATH", file=sys.stderr)
        return 64
    try:
        kind = ConfigKind(argv[1])
    except ValueError:
        print(f"D-TASK CONFIG: RED: unsupported kind: {argv[1]}", file=sys.stderr)
        return 64
    try:
        load_config(kind, Path(argv[2]))
    except ConfigBoundaryError as error:
        print(f"D-TASK CONFIG: RED: {error}", file=sys.stderr)
        return 1
    print(f"D-TASK CONFIG: GREEN: {kind.value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
