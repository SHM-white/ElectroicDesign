from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from yolo_contract.errors import (
    ClassMapDriftError,
    DuplicateHashError,
    ManifestError,
    MissingMetadataError,
    SplitOverlapError,
)
from yolo_contract.schema import (
    load_dataset_manifest,
    load_model_manifest,
    validate_model_against_dataset,
)


ULTRALYTICS_REVISION = "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b"


def digest(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def write_json(path: Path, contents: dict[str, object]) -> None:
    path.write_text(json.dumps(contents, indent=2) + "\n", encoding="utf-8")


def valid_dataset() -> dict[str, object]:
    return {
        "schema_version": 1,
        "dataset_id": "tiny-marker-set",
        "source": {
            "url": "https://example.invalid/datasets/tiny-marker-set.tar",
            "revision": "e" * 40,
            "license": "CC-BY-4.0",
        },
        "class_map": [{"id": 0, "name": "marker"}],
        "samples": [
            {
                "id": "train/one.jpg",
                "split": "train",
                "sha256": digest("train-image"),
                "source_url": "https://example.invalid/train/one.jpg",
                "license": "CC-BY-4.0",
                "class_ids": [0],
            },
            {
                "id": "val/two.jpg",
                "split": "val",
                "sha256": digest("validation-image"),
                "source_url": "https://example.invalid/val/two.jpg",
                "license": "CC-BY-4.0",
                "class_ids": [0],
            },
            {
                "id": "test/three.jpg",
                "split": "test",
                "sha256": digest("test-image"),
                "source_url": "https://example.invalid/test/three.jpg",
                "license": "CC-BY-4.0",
                "class_ids": [0],
            },
        ],
    }


def valid_model(dataset_path: Path, artifact_path: Path) -> dict[str, object]:
    return {
        "schema_version": 1,
        "model_id": "tiny-marker-onnx",
        "dataset_manifest_sha256": hashlib.sha256(dataset_path.read_bytes()).hexdigest(),
        "class_map": [{"id": 0, "name": "marker"}],
        "preprocessing": {
            "color_space": "RGB",
            "layout": "NCHW",
            "resize": {"width": 640, "height": 640, "strategy": "letterbox"},
            "scale": 0.00392156862745098,
        },
        "runtime": {
            "format": "onnx",
            "input_tensor": "images",
            "output_tensor": "output0",
        },
        "artifact": {
            "path": artifact_path.name,
            "sha256": hashlib.sha256(artifact_path.read_bytes()).hexdigest(),
        },
        "training_provider": {
            "repository_url": "https://github.com/ultralytics/ultralytics.git",
            "revision": ULTRALYTICS_REVISION,
            "license": "AGPL-3.0-only",
        },
    }


def test_accepts_immutable_dataset_and_bound_model_when_metadata_matches(tmp_path: Path) -> None:
    # Given: a fully attributed, disjoint dataset and bound ONNX model manifest.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model_path = tmp_path / "model.json"
    write_json(model_path, valid_model(dataset_path, artifact_path))

    # When: both manifests are parsed and the model is bound to the dataset.
    dataset = load_dataset_manifest(dataset_path)
    model = load_model_manifest(model_path)
    validate_model_against_dataset(model, dataset, dataset_path)

    # Then: immutable provenance, class identity, and preprocessing all hold.
    assert dataset.dataset_id == "tiny-marker-set"
    assert model.runtime_format == "onnx"


def test_rejects_split_overlap_when_one_hash_is_assigned_to_train_and_val(tmp_path: Path) -> None:
    # Given: the same sample hash is assigned to train and validation splits.
    manifest = valid_dataset()
    samples = manifest["samples"]
    assert isinstance(samples, list)
    duplicate = dict(samples[0])
    duplicate["id"] = "val/copied.jpg"
    duplicate["split"] = "val"
    samples.append(duplicate)
    path = tmp_path / "overlap.json"
    write_json(path, manifest)

    # When: the immutable manifest is parsed.
    # Then: cross-split reuse is rejected before training can start.
    with pytest.raises(SplitOverlapError, match="split overlap"):
        load_dataset_manifest(path)


def test_rejects_duplicate_hash_when_two_samples_share_one_split(tmp_path: Path) -> None:
    # Given: two distinct sample IDs share one content hash inside train.
    manifest = valid_dataset()
    samples = manifest["samples"]
    assert isinstance(samples, list)
    duplicate = dict(samples[0])
    duplicate["id"] = "train/copied.jpg"
    samples.append(duplicate)
    path = tmp_path / "duplicate.json"
    write_json(path, manifest)

    # When: the immutable manifest is parsed.
    # Then: duplicate content is rejected rather than silently reweighted.
    with pytest.raises(DuplicateHashError, match="duplicate hash"):
        load_dataset_manifest(path)


def test_rejects_missing_license_when_a_sample_has_no_attribution(tmp_path: Path) -> None:
    # Given: a sample record without its required license reference.
    manifest = valid_dataset()
    samples = manifest["samples"]
    assert isinstance(samples, list)
    sample = samples[0]
    assert isinstance(sample, dict)
    del sample["license"]
    path = tmp_path / "missing-license.json"
    write_json(path, manifest)

    # When: the immutable manifest is parsed.
    # Then: missing legal attribution is rejected.
    with pytest.raises(MissingMetadataError, match="license"):
        load_dataset_manifest(path)


def test_rejects_class_drift_when_model_classes_do_not_match_dataset(tmp_path: Path) -> None:
    # Given: a valid dataset but model metadata with a different class name.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model = valid_model(dataset_path, artifact_path)
    model["class_map"] = [{"id": 0, "name": "wrong-marker"}]
    model_path = tmp_path / "class-drift.json"
    write_json(model_path, model)

    # When: the model is checked against its pinned dataset manifest.
    # Then: class-map drift is rejected.
    with pytest.raises(ClassMapDriftError, match="class map drift"):
        validate_model_against_dataset(
            load_model_manifest(model_path), load_dataset_manifest(dataset_path), dataset_path
        )


def test_rejects_missing_preprocessing_when_model_metadata_is_incomplete(tmp_path: Path) -> None:
    # Given: model metadata that omits its required preprocessing contract.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model = valid_model(dataset_path, artifact_path)
    del model["preprocessing"]
    model_path = tmp_path / "missing-preprocessing.json"
    write_json(model_path, model)

    # When: the model manifest is parsed.
    # Then: deployment metadata cannot omit preprocessing.
    with pytest.raises(MissingMetadataError, match="preprocessing"):
        load_model_manifest(model_path)


def test_rejects_artifact_path_escape_when_model_metadata_is_untrusted(tmp_path: Path) -> None:
    # Given: untrusted model metadata that attempts to escape its manifest root.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model = valid_model(dataset_path, artifact_path)
    artifact = model["artifact"]
    assert isinstance(artifact, dict)
    artifact["path"] = "../outside.onnx"
    model_path = tmp_path / "path-escape.json"
    write_json(model_path, model)

    # When: the model manifest is parsed.
    # Then: it rejects the path before a provider can read outside its root.
    with pytest.raises(ManifestError, match="must remain relative"):
        load_model_manifest(model_path)
