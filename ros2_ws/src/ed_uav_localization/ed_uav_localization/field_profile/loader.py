"""YAML boundary parser for strict field profiles."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import yaml
from pydantic import ValidationError

from ed_uav_localization.field_profile.model import FieldProfile, PROFILE_SCHEMA


@dataclass(frozen=True, slots=True)
class FieldProfileError(ValueError):
    """Base error for a profile that cannot cross the configuration boundary."""

    source: str
    reason: str

    def __str__(self) -> str:
        return f"{self.source}: {self.reason}"


@dataclass(frozen=True, slots=True)
class FieldProfileYamlError(FieldProfileError):
    """A profile could not be parsed as a unique-key YAML document."""


@dataclass(frozen=True, slots=True)
class FieldProfileValidationError(FieldProfileError):
    """A parsed profile violated the strict field schema."""


def load_profile(path: Path) -> FieldProfile:
    """Load a profile from disk without caching its content or derived hash."""
    try:
        source = path.read_text(encoding="utf-8")
    except OSError as error:
        raise FieldProfileYamlError(str(path), f"cannot read profile: {error}") from error
    return load_profile_text(source, str(path))


def load_profile_text(source: str, source_name: str) -> FieldProfile:
    """Parse untrusted YAML into one of the validated field-profile models."""
    try:
        document = yaml.compose(source, Loader=yaml.SafeLoader)
    except yaml.YAMLError as error:
        raise FieldProfileYamlError(source_name, f"malformed YAML: {error}") from error
    if document is None:
        raise FieldProfileYamlError(source_name, "malformed YAML: document is empty")
    _reject_duplicate_mapping_keys(document, source_name)
    try:
        raw_profile = yaml.safe_load(source)
    except yaml.YAMLError as error:
        raise FieldProfileYamlError(source_name, f"malformed YAML: {error}") from error
    try:
        return PROFILE_SCHEMA.validate_python(raw_profile)
    except ValidationError as error:
        raise FieldProfileValidationError(source_name, str(error)) from error


def dump_profile(profile: FieldProfile) -> str:
    """Return a deterministic YAML representation of a validated profile."""
    return yaml.safe_dump(profile.model_dump(mode="json"), sort_keys=True)


def profile_hash(profile: FieldProfile) -> str:
    """Return a stable content hash for a validated profile."""
    canonical = json.dumps(
        profile.model_dump(mode="json"),
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _reject_duplicate_mapping_keys(node: yaml.Node, source_name: str) -> None:
    if isinstance(node, yaml.MappingNode):
        keys: set[str] = set()
        for key_node, value_node in node.value:
            if isinstance(key_node, yaml.ScalarNode):
                key = key_node.value
                if key in keys:
                    raise FieldProfileYamlError(source_name, f"duplicate YAML key: {key}")
                keys.add(key)
            _reject_duplicate_mapping_keys(key_node, source_name)
            _reject_duplicate_mapping_keys(value_node, source_name)
    elif isinstance(node, yaml.SequenceNode):
        for item in node.value:
            _reject_duplicate_mapping_keys(item, source_name)
