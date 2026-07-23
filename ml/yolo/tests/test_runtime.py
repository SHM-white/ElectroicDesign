from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from yolo_contract.errors import ModelIntegrityError, ProviderFailureError
from yolo_contract.runtime import ImageRequest, MockDetectionProvider, MockProviderConfig
from yolo_contract.schema import load_dataset_manifest, load_model_manifest

from test_schema import valid_dataset, valid_model, write_json


def test_mock_provider_is_deterministic_and_vision_msgs_compatible(tmp_path: Path) -> None:
    # Given: a complete ONNX contract and a deterministic image identity.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model_path = tmp_path / "model.json"
    write_json(model_path, valid_model(dataset_path, artifact_path))
    dataset = load_dataset_manifest(dataset_path)
    model = load_model_manifest(model_path)
    request = ImageRequest(
        image_id="camera-frame-0001",
        image_sha256=hashlib.sha256(b"synthetic-image").hexdigest(),
        frame_id="camera_narrow_optical_frame",
    )
    provider = MockDetectionProvider(
        MockProviderConfig(model=model, dataset=dataset, model_root=tmp_path)
    )

    # When: the same request is detected twice.
    first = provider.detect(request)
    second = provider.detect(request)

    # Then: standard detection fields and all values are byte-stable.
    assert first == second
    assert first.contract == "vision_msgs/Detection2DArray-compatible/v1"
    assert first.frame_id == "camera_narrow_optical_frame"
    assert first.detections[0].class_name == "marker"
    assert first.detections[0].bbox.width == 0.5


def test_rejects_corrupt_model_hash_before_mock_detection(tmp_path: Path) -> None:
    # Given: model metadata whose recorded artifact hash does not match disk.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"corrupt artifact")
    write_json(dataset_path, valid_dataset())
    model = valid_model(dataset_path, artifact_path)
    artifact = model["artifact"]
    assert isinstance(artifact, dict)
    artifact["sha256"] = "0" * 64
    model_path = tmp_path / "corrupt-model.json"
    write_json(model_path, model)
    provider = MockDetectionProvider(
        MockProviderConfig(
            model=load_model_manifest(model_path),
            dataset=load_dataset_manifest(dataset_path),
            model_root=tmp_path,
        )
    )
    request = ImageRequest(
        image_id="camera-frame-0001",
        image_sha256=hashlib.sha256(b"synthetic-image").hexdigest(),
        frame_id="camera_narrow_optical_frame",
    )

    # When: the provider receives an otherwise valid detection request.
    # Then: integrity failure is surfaced instead of emitting a detection.
    with pytest.raises(ModelIntegrityError, match="model artifact hash mismatch"):
        provider.detect(request)


def test_surfaces_provider_failure_without_emitting_misleading_detection(tmp_path: Path) -> None:
    # Given: a configured mock provider failure and an otherwise valid model.
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model_path = tmp_path / "model.json"
    write_json(model_path, valid_model(dataset_path, artifact_path))
    provider = MockDetectionProvider(
        MockProviderConfig(
            model=load_model_manifest(model_path),
            dataset=load_dataset_manifest(dataset_path),
            model_root=tmp_path,
            failure_reason="synthetic provider failure",
        )
    )
    request = ImageRequest(
        image_id="camera-frame-0001",
        image_sha256=hashlib.sha256(b"synthetic-image").hexdigest(),
        frame_id="camera_narrow_optical_frame",
    )

    # When: the failed provider is asked to detect.
    # Then: it raises a typed failure rather than returning a false success.
    with pytest.raises(ProviderFailureError, match="synthetic provider failure"):
        provider.detect(request)
