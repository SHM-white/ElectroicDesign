"""Strict parsers for immutable dataset and runtime-model manifests."""

from __future__ import annotations

import re
from pathlib import Path

from .errors import ClassMapDriftError, DuplicateHashError, ManifestError, MissingMetadataError, SplitOverlapError
from .jsonio import JsonValue, load_json_mapping, sha256_bytes
from .models import (
    ClassDefinition,
    DatasetManifest,
    DatasetSample,
    DatasetSource,
    ModelArtifact,
    ModelManifest,
    Preprocessing,
    ResizeSpec,
    RuntimeSpec,
    TrainingProvider,
)


SCHEMA_VERSION = 1
ULTRALYTICS_REPOSITORY = "https://github.com/ultralytics/ultralytics.git"
ULTRALYTICS_REVISION = "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b"
ULTRALYTICS_LICENSE = "AGPL-3.0-only"
SHA1 = re.compile(r"[0-9a-f]{40}")
SHA256 = re.compile(r"[0-9a-f]{64}")
SPLITS = frozenset({"train", "val", "test"})


def _mapping(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise ManifestError(f"{label} must be an object")
    return value


def _list(value: JsonValue, label: str) -> list[JsonValue]:
    if not isinstance(value, list):
        raise ManifestError(f"{label} must be an array")
    return value


def _string(record: dict[str, JsonValue], field: str, label: str) -> str:
    value = record.get(field)
    if not isinstance(value, str) or not value.strip():
        raise MissingMetadataError(f"{label} requires nonempty {field}")
    return value


def _exact_keys(record: dict[str, JsonValue], expected: frozenset[str], label: str) -> None:
    received = frozenset(record)
    missing = expected - received
    if missing:
        raise MissingMetadataError(f"{label} requires {sorted(missing)[0]}")
    unexpected = received - expected
    if unexpected:
        raise ManifestError(f"{label} has unsupported fields: {sorted(unexpected)}")


def _sha256(value: str, label: str) -> str:
    if not SHA256.fullmatch(value):
        raise ManifestError(f"{label} requires lowercase SHA-256")
    return value


def _class_map(value: JsonValue, label: str) -> tuple[ClassDefinition, ...]:
    records = _list(value, label)
    classes: list[ClassDefinition] = []
    ids: set[int] = set()
    names: set[str] = set()
    for index, raw in enumerate(records):
        record = _mapping(raw, f"{label}[{index}]")
        _exact_keys(record, frozenset({"id", "name"}), f"{label}[{index}]")
        class_id = record.get("id")
        if not isinstance(class_id, int) or isinstance(class_id, bool) or class_id < 0:
            raise ManifestError(f"{label}[{index}] requires nonnegative integer id")
        name = _string(record, "name", f"{label}[{index}]")
        if class_id in ids or name in names:
            raise ManifestError(f"{label} requires unique class ids and names")
        ids.add(class_id)
        names.add(name)
        classes.append(ClassDefinition(class_id=class_id, name=name))
    if not classes:
        raise MissingMetadataError(f"{label} requires at least one class")
    return tuple(classes)


def _dataset_source(value: JsonValue) -> DatasetSource:
    record = _mapping(value, "source")
    _exact_keys(record, frozenset({"url", "revision", "license"}), "source")
    url = _string(record, "url", "source")
    revision = _string(record, "revision", "source")
    if not url.startswith("https://") or not SHA1.fullmatch(revision):
        raise ManifestError("source requires immutable HTTPS URL and revision")
    return DatasetSource(url=url, revision=revision, license_id=_string(record, "license", "source"))


def load_dataset_manifest(path: Path) -> DatasetManifest:
    """Parse an immutable train/val/test manifest from untrusted JSON."""
    root = load_json_mapping(path)
    _exact_keys(root, frozenset({"schema_version", "dataset_id", "source", "class_map", "samples"}), "dataset manifest")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise ManifestError("dataset manifest requires schema_version 1")
    class_map = _class_map(root.get("class_map"), "class_map")
    class_ids = frozenset(item.class_id for item in class_map)
    hashes: dict[str, str] = {}
    samples: list[DatasetSample] = []
    seen_ids: set[str] = set()
    for index, raw in enumerate(_list(root.get("samples"), "samples")):
        record = _mapping(raw, f"samples[{index}]")
        _exact_keys(record, frozenset({"id", "split", "sha256", "source_url", "license", "class_ids"}), f"samples[{index}]")
        sample_id = _string(record, "id", f"samples[{index}]")
        split = _string(record, "split", f"samples[{index}]")
        digest = _sha256(_string(record, "sha256", f"samples[{index}]"), f"samples[{index}].sha256")
        if split not in SPLITS:
            raise ManifestError(f"samples[{index}] has unsupported split: {split}")
        if sample_id in seen_ids:
            raise ManifestError(f"duplicate sample id: {sample_id}")
        prior_split = hashes.get(digest)
        if prior_split is not None:
            if prior_split != split:
                raise SplitOverlapError(f"split overlap for hash {digest}: {prior_split} and {split}")
            raise DuplicateHashError(f"duplicate hash in {split}: {digest}")
        values = _list(record.get("class_ids"), f"samples[{index}].class_ids")
        parsed_ids = tuple(item for item in values if isinstance(item, int) and not isinstance(item, bool))
        if len(parsed_ids) != len(values) or not parsed_ids or not set(parsed_ids).issubset(class_ids):
            raise ManifestError(f"samples[{index}] has unknown or invalid class_ids")
        hashes[digest] = split
        seen_ids.add(sample_id)
        samples.append(DatasetSample(sample_id, split, digest, _string(record, "source_url", f"samples[{index}]"), _string(record, "license", f"samples[{index}]"), parsed_ids))
    if frozenset(sample.split for sample in samples) != SPLITS:
        raise MissingMetadataError("samples require nonempty train, val, and test splits")
    return DatasetManifest(_string(root, "dataset_id", "dataset manifest"), _dataset_source(root.get("source")), class_map, tuple(samples))


def _preprocessing(value: JsonValue) -> Preprocessing:
    record = _mapping(value, "preprocessing")
    _exact_keys(record, frozenset({"color_space", "layout", "resize", "scale"}), "preprocessing")
    resize = _mapping(record.get("resize"), "preprocessing.resize")
    _exact_keys(resize, frozenset({"width", "height", "strategy"}), "preprocessing.resize")
    width, height = resize.get("width"), resize.get("height")
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ManifestError("preprocessing.resize requires positive width and height")
    scale = record.get("scale")
    if not isinstance(scale, int | float) or isinstance(scale, bool) or scale <= 0:
        raise ManifestError("preprocessing.scale requires a positive number")
    return Preprocessing(_string(record, "color_space", "preprocessing"), _string(record, "layout", "preprocessing"), ResizeSpec(width, height, _string(resize, "strategy", "preprocessing.resize")), float(scale))


def load_model_manifest(path: Path) -> ModelManifest:
    """Parse an ONNX or OpenVINO model manifest from untrusted JSON."""
    root = load_json_mapping(path)
    _exact_keys(root, frozenset({"schema_version", "model_id", "dataset_manifest_sha256", "class_map", "preprocessing", "runtime", "artifact", "training_provider"}), "model manifest")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise ManifestError("model manifest requires schema_version 1")
    runtime = _mapping(root.get("runtime"), "runtime")
    _exact_keys(runtime, frozenset({"format", "input_tensor", "output_tensor"}), "runtime")
    runtime_format = _string(runtime, "format", "runtime")
    if runtime_format not in {"onnx", "openvino"}:
        raise ManifestError("runtime.format must be onnx or openvino")
    artifact = _mapping(root.get("artifact"), "artifact")
    _exact_keys(artifact, frozenset({"path", "sha256"}), "artifact")
    artifact_path = _string(artifact, "path", "artifact")
    if Path(artifact_path).is_absolute() or ".." in Path(artifact_path).parts:
        raise ManifestError("artifact.path must remain relative to the manifest directory")
    provider = _mapping(root.get("training_provider"), "training_provider")
    _exact_keys(provider, frozenset({"repository_url", "revision", "license"}), "training_provider")
    training_provider = TrainingProvider(_string(provider, "repository_url", "training_provider"), _string(provider, "revision", "training_provider"), _string(provider, "license", "training_provider"))
    if training_provider != TrainingProvider(ULTRALYTICS_REPOSITORY, ULTRALYTICS_REVISION, ULTRALYTICS_LICENSE):
        raise ManifestError("training_provider must match the reviewed P04 Ultralytics source and AGPL license")
    return ModelManifest(
        _string(root, "model_id", "model manifest"),
        _sha256(_string(root, "dataset_manifest_sha256", "model manifest"), "dataset_manifest_sha256"),
        _class_map(root.get("class_map"), "class_map"),
        _preprocessing(root.get("preprocessing")),
        RuntimeSpec(runtime_format, _string(runtime, "input_tensor", "runtime"), _string(runtime, "output_tensor", "runtime")),
        ModelArtifact(artifact_path, _sha256(_string(artifact, "sha256", "artifact"), "artifact.sha256")),
        training_provider,
    )


def validate_model_against_dataset(model: ModelManifest, dataset: DatasetManifest, dataset_path: Path) -> None:
    """Prove a model contract is bound to one exact immutable dataset manifest."""
    observed_hash = sha256_bytes(dataset_path.read_bytes())
    if model.dataset_manifest_sha256 != observed_hash:
        raise ManifestError("model dataset_manifest_sha256 does not match dataset manifest")
    if model.class_map != dataset.class_map:
        raise ClassMapDriftError("class map drift between model and dataset")
