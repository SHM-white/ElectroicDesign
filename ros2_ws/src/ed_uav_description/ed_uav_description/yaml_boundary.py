"""Bounded YAML parsing for untrusted configuration files."""

from __future__ import annotations

from dataclasses import dataclass

import yaml

MAX_YAML_BYTES = 1_048_576


@dataclass(frozen=True, slots=True)
class StrictYamlError(ValueError):
    source: str
    reason: str

    def __str__(self) -> str:
        return f"{self.source}: {self.reason}"


def load_strict_yaml(source: str, source_name: str) -> object:
    """Parse one bounded, alias-free YAML document with unique mapping keys."""
    if len(source.encode("utf-8")) > MAX_YAML_BYTES:
        raise StrictYamlError(source_name, "YAML document exceeds 1 MiB")
    try:
        events = yaml.parse(source, Loader=yaml.SafeLoader)
        if any(isinstance(event, yaml.AliasEvent) for event in events):
            raise StrictYamlError(source_name, "YAML aliases are not allowed")
        document = yaml.compose(source, Loader=yaml.SafeLoader)
    except yaml.YAMLError as error:
        raise StrictYamlError(source_name, f"malformed YAML: {error}") from error
    if document is None:
        raise StrictYamlError(source_name, "YAML document is empty")
    _reject_duplicate_mapping_keys(document, source_name)
    try:
        return yaml.safe_load(source)
    except yaml.YAMLError as error:
        raise StrictYamlError(source_name, f"malformed YAML: {error}") from error


def _reject_duplicate_mapping_keys(node: yaml.Node, source_name: str) -> None:
    if isinstance(node, yaml.MappingNode):
        keys: set[str] = set()
        for key_node, value_node in node.value:
            if isinstance(key_node, yaml.ScalarNode):
                key = key_node.value
                if key in keys:
                    raise StrictYamlError(source_name, f"duplicate YAML key: {key}")
                keys.add(key)
            _reject_duplicate_mapping_keys(key_node, source_name)
            _reject_duplicate_mapping_keys(value_node, source_name)
    elif isinstance(node, yaml.SequenceNode):
        for item in node.value:
            _reject_duplicate_mapping_keys(item, source_name)
