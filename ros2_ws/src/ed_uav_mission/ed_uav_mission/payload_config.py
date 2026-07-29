"""Strict package-local configuration for payload and touchdown adapters."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final, Literal

from ed_uav_description.yaml_boundary import load_strict_yaml
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, TypeAdapter


class PayloadBoundaryConfig(BaseModel):
    """Validated thresholds shared by release and touchdown boundaries."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract_version: Literal[1]
    freshness_timeout_s: FiniteFloat = Field(gt=0.0, le=2.0)
    actuator_timeout_s: FiniteFloat = Field(gt=0.0, le=5.0)
    minimum_standoff_m: FiniteFloat = Field(ge=0.0, le=10.0)
    contact_dwell_s: FiniteFloat = Field(ge=5.0, le=30.0)
    minimum_vehicle_speed_m_s: FiniteFloat = Field(gt=0.0, le=5.0)


@dataclass(frozen=True, slots=True)
class PayloadConfigReadError(Exception):
    path: Path
    detail: str

    def __str__(self) -> str:
        return f"cannot read payload adapter config {self.path}: {self.detail}"


PAYLOAD_BOUNDARY_SCHEMA: Final = TypeAdapter(PayloadBoundaryConfig)


def parse_payload_boundary_config_text(yaml_text: str) -> PayloadBoundaryConfig:
    """Parse strict YAML into payload boundary configuration."""
    document = load_strict_yaml(yaml_text, "<inline payload adapter>")
    return PAYLOAD_BOUNDARY_SCHEMA.validate_python(document)


def load_payload_boundary_config(path: Path) -> PayloadBoundaryConfig:
    """Read and parse one package-local payload adapter configuration."""
    try:
        source = path.read_text(encoding="utf-8")
    except OSError as error:
        raise PayloadConfigReadError(path=path, detail=str(error)) from error
    document = load_strict_yaml(source, str(path))
    return PAYLOAD_BOUNDARY_SCHEMA.validate_python(document)
