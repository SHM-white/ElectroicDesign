"""Model manifest validation using Pydantic."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ValidationError, field_validator


# ---------------------------------------------------------------------------
# Manifest schema
# ---------------------------------------------------------------------------


class ModelManifest(BaseModel):
    """Schema for a detector model manifest.

    Valid provider values are ``mock``, ``onnx``, and ``openvino``.
    ``input_shape`` must be a 3-element list [H, W, C].
    ``class_names`` must contain at least one entry.
    """

    version: str
    provider: str
    input_shape: list[int]
    class_names: list[str]

    @field_validator("provider")
    @classmethod
    def _check_provider(cls, v: str) -> str:
        allowed = {"mock", "onnx", "openvino"}
        if v not in allowed:
            raise ValueError(f"Unknown provider '{v}'; must be one of {sorted(allowed)}")
        return v

    @field_validator("input_shape")
    @classmethod
    def _check_input_shape(cls, v: list[int]) -> list[int]:
        if len(v) != 3:
            raise ValueError(f"input_shape must have exactly 3 elements, got {len(v)}")
        if any(dim <= 0 for dim in v):
            raise ValueError(f"All input_shape dimensions must be positive, got {v}")
        return v

    @field_validator("class_names")
    @classmethod
    def _check_class_names(cls, v: list[str]) -> list[str]:
        if len(v) < 1:
            raise ValueError("class_names must contain at least one entry")
        return v


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def validate_manifest(data: dict[str, Any]) -> ModelManifest:
    """Validate a raw manifest dict and return a validated ``ModelManifest``.

    Raises:
        ValidationError: When the manifest fails schema validation.
    """
    return ModelManifest.model_validate(data)


def is_valid_manifest(data: dict[str, Any]) -> bool:
    """Return ``True`` if *data* passes manifest validation."""
    try:
        validate_manifest(data)
    except ValidationError:
        return False
    return True
