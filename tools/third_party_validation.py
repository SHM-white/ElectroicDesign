#!/usr/bin/env python3
# Run: python3 tools/check_third_party.py --strict
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Final

from third_party_checkout import validate_checkout


REQUIRED_SOURCE_IDS: Final = frozenset(
    {"livox_ros_driver2", "livox_sdk2", "fast_lio_ros2", "ultralytics"}
)
INVOCATION_BOUNDARY_KINDS: Final = frozenset({"separate-library", "separate-process"})
SHA1: Final = re.compile(r"[0-9a-f]{40}")
SHA256: Final = re.compile(r"[0-9a-f]{64}")


def load_json(path: Path, errors: list[str]) -> dict | None:
    """Load one machine-readable manifest, recording parse failures."""
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        errors.append(f"missing manifest: {path}")
        return None
    except OSError as error:
        errors.append(f"cannot read manifest {path}: {error}")
        return None
    except json.JSONDecodeError as error:
        errors.append(f"malformed manifest {path}: {error.msg}")
        return None
    if not isinstance(value, dict):
        errors.append(f"manifest root must be an object: {path}")
        return None
    return value


def require_string(record: dict, field: str, label: str, errors: list[str]) -> str | None:
    """Return one nonempty string field or record a deterministic validation error."""
    value = record.get(field)
    if not isinstance(value, str) or not value:
        errors.append(f"{label} requires nonempty {field}")
        return None
    return value


def validate_dataset_manifest(dataset_manifest: dict, root: Path, errors: list[str]) -> None:
    """Require provenance fields for every future dataset or model artifact."""
    if dataset_manifest.get("schema_version") != 1:
        errors.append("dataset manifest requires schema_version 1")
    if not isinstance(dataset_manifest.get("policy"), dict):
        errors.append("dataset manifest requires policy object")
    for collection_name in ("datasets", "model_weights"):
        records = dataset_manifest.get(collection_name)
        if not isinstance(records, list):
            errors.append(f"dataset manifest requires {collection_name} array")
            continue
        for record in records:
            if not isinstance(record, dict):
                errors.append(f"dataset manifest {collection_name} entry must be an object")
                continue
            for field in ("id", "source_url", "license", "sha256"):
                require_string(record, field, f"{collection_name} entry", errors)
            digest = record.get("sha256")
            if isinstance(digest, str) and not SHA256.fullmatch(digest):
                errors.append(f"{collection_name} entry has invalid sha256")
    reference_archives = dataset_manifest.get("reference_archives")
    if not isinstance(reference_archives, list):
        errors.append("dataset manifest requires reference_archives array")
        return
    for record in reference_archives:
        if not isinstance(record, dict):
            errors.append("reference archive entry must be an object")
            continue
        for field in (
            "id",
            "local_path",
            "local_sha256",
            "upstream_url",
            "reviewed_revision",
            "license_status",
        ):
            require_string(record, field, "reference archive entry", errors)
        digest = record.get("local_sha256")
        if isinstance(digest, str) and not SHA256.fullmatch(digest):
            errors.append("reference archive entry has invalid local_sha256")
        local_path = record.get("local_path")
        if isinstance(local_path, str) and isinstance(digest, str):
            reference_path = root / local_path
            if Path(local_path).is_absolute() or not reference_path.resolve().is_relative_to(
                root.resolve()
            ):
                errors.append("reference archive path escapes workspace")
            elif not reference_path.is_file():
                errors.append(f"reference archive missing: {reference_path}")
            elif hashlib.sha256(reference_path.read_bytes()).hexdigest() != digest:
                errors.append(f"reference archive hash mismatch: {reference_path}")


def validate(root: Path, strict: bool) -> list[str]:
    """Validate immutable source, license, dataset, and boundary provenance."""
    errors: list[str] = []
    repos = load_json(root / "ros2_ws/dependencies.repos", errors)
    sources_manifest = load_json(root / "docs/provenance/third-party-sources.json", errors)
    dataset_manifest = load_json(root / "docs/provenance/dataset-manifest.json", errors)
    if repos is None or sources_manifest is None or dataset_manifest is None:
        return errors

    repositories = repos.get("repositories")
    sources = sources_manifest.get("sources")
    if not isinstance(repositories, dict):
        errors.append("dependencies.repos requires repositories object")
        return errors
    if sources_manifest.get("schema_version") != 1:
        errors.append("source manifest requires schema_version 1")
    if not isinstance(sources, list):
        errors.append("source manifest requires sources array")
        return errors
    validate_dataset_manifest(dataset_manifest, root, errors)

    source_ids: set[str] = set()
    source_by_id: dict[str, dict] = {}
    for source in sources:
        if not isinstance(source, dict):
            errors.append("source manifest entry must be an object")
            continue
        source_id = require_string(source, "id", "source entry", errors)
        if source_id is None:
            continue
        if source_id in source_ids:
            errors.append(f"duplicate source id: {source_id}")
            continue
        source_ids.add(source_id)
        source_by_id[source_id] = source

    missing_ids = REQUIRED_SOURCE_IDS - source_ids
    unexpected_ids = source_ids - REQUIRED_SOURCE_IDS
    for source_id in sorted(missing_ids):
        errors.append(f"required source missing: {source_id}")
    for source_id in sorted(unexpected_ids):
        errors.append(f"unexpected source id: {source_id}")

    repository_ids = set(repositories)
    for source_id in sorted(REQUIRED_SOURCE_IDS - repository_ids):
        errors.append(f"dependencies.repos missing required source: {source_id}")
    for source_id in sorted(repository_ids - REQUIRED_SOURCE_IDS):
        errors.append(f"dependencies.repos contains unexpected source: {source_id}")

    source_root = root / "ros2_ws/src"
    for source_id in sorted(REQUIRED_SOURCE_IDS & source_ids & repository_ids):
        source = source_by_id[source_id]
        repository = repositories[source_id]
        if not isinstance(repository, dict):
            errors.append(f"repository entry must be an object: {source_id}")
            continue
        if repository.get("type") != "git":
            errors.append(f"repository type must be git: {source_id}")
        revision = repository.get("version")
        if not isinstance(revision, str) or not SHA1.fullmatch(revision):
            errors.append(f"floating revision is forbidden: {source_id}")
            continue
        source_revision = require_string(source, "revision", source_id, errors)
        source_url = require_string(source, "repository_url", source_id, errors)
        repository_url = repository.get("url")
        if source_revision is not None and source_revision != revision:
            errors.append(f"source revision mismatch: {source_id}")
        if source_url is not None and source_url != repository_url:
            errors.append(f"source repository URL mismatch: {source_id}")

        checkout_name = require_string(source, "checkout_directory", source_id, errors)
        license = source.get("license")
        if not isinstance(license, dict):
            errors.append(f"{source_id} requires license metadata")
        else:
            for field in (
                "spdx",
                "repository_path",
                "source_url",
                "cache_path",
                "sha256",
                "retrieved_at",
            ):
                require_string(license, field, f"{source_id} license", errors)
            license_hash = license.get("sha256")
            cache_path = license.get("cache_path")
            if isinstance(license_hash, str) and not SHA256.fullmatch(license_hash):
                errors.append(f"{source_id} license has invalid sha256")
            if isinstance(cache_path, str):
                cached_license = root / cache_path
                if Path(cache_path).is_absolute() or not cached_license.resolve().is_relative_to(root.resolve()):
                    errors.append(f"{source_id} license cache path escapes workspace")
                elif not cached_license.is_file():
                    errors.append(f"missing cached license: {cached_license}")
                elif isinstance(license_hash, str):
                    observed_hash = hashlib.sha256(cached_license.read_bytes()).hexdigest()
                    if observed_hash != license_hash:
                        errors.append(f"license hash mismatch: {cached_license}")

        corresponding_source = source.get("corresponding_source")
        if not isinstance(corresponding_source, dict):
            errors.append(f"missing corresponding-source metadata: {source_id}")
        else:
            for field in ("repository_url", "revision", "availability", "archive_url"):
                require_string(corresponding_source, field, f"{source_id} corresponding-source", errors)
            if corresponding_source.get("repository_url") != source_url:
                errors.append(f"corresponding-source repository mismatch: {source_id}")
            if corresponding_source.get("revision") != revision:
                errors.append(f"corresponding-source revision mismatch: {source_id}")

        boundary = source.get("invocation_boundary")
        if not isinstance(boundary, dict):
            errors.append(f"{source_id} requires invocation boundary metadata")
        elif boundary.get("kind") not in INVOCATION_BOUNDARY_KINDS:
            errors.append(f"{source_id} must declare a supported invocation boundary")
        else:
            require_string(boundary, "description", f"{source_id} invocation boundary", errors)

        markers = source.get("forbidden_copy_markers")
        if not isinstance(markers, list) or not markers or not all(
            isinstance(marker, str) and marker for marker in markers
        ):
            errors.append(f"{source_id} requires forbidden copy markers")
            markers = []
        if strict and isinstance(checkout_name, str):
            checkout = source_root / "third_party" / checkout_name
            if checkout.exists():
                validate_checkout(checkout, source, errors)
        if strict and source_root.exists():
            for package in source_root.glob("ed_*"):
                if not package.is_dir():
                    continue
                for path in package.rglob("*"):
                    if not path.is_dir():
                        continue
                    parts = tuple(part.casefold() for part in path.parts)
                    if any(
                        marker.casefold() in part
                        for marker in markers
                        for part in parts
                    ):
                        errors.append(f"copied third-party source under ed_*: {path}")

    return errors
