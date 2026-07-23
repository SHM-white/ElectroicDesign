"""Provider-neutral ONNX/OpenVINO runtime and deterministic mock adapter."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from .errors import ModelIntegrityError, ProviderFailureError
from .jsonio import sha256_file
from .models import DatasetManifest, ModelManifest


ROS_DETECTION_CONTRACT = "vision_msgs/Detection2DArray-compatible/v1"


@dataclass(frozen=True, slots=True)
class ImageRequest:
    """A content-addressed camera frame presented to any detector provider."""

    image_id: str
    image_sha256: str
    frame_id: str


@dataclass(frozen=True, slots=True)
class BoundingBox2D:
    """Normalized center-positioned bounding box independent of ROS imports."""

    center_x: float
    center_y: float
    width: float
    height: float


@dataclass(frozen=True, slots=True)
class Detection2D:
    """One classified bounding box compatible with vision_msgs semantics."""

    class_id: int
    class_name: str
    score: float
    bbox: BoundingBox2D


@dataclass(frozen=True, slots=True)
class Detection2DArray:
    """A provider-neutral array later adaptable to vision_msgs/Detection2DArray."""

    contract: str
    image_id: str
    frame_id: str
    detections: tuple[Detection2D, ...]


class DetectionProvider(Protocol):
    """Provider seam for ONNX, OpenVINO, and deterministic test adapters."""

    def detect(self, request: ImageRequest) -> Detection2DArray:
        """Return detected pixel-normalized boxes or raise a typed provider error."""


@dataclass(frozen=True, slots=True)
class MockProviderConfig:
    """All immutable inputs required by the deterministic mock provider."""

    model: ModelManifest
    dataset: DatasetManifest
    model_root: Path
    failure_reason: str | None = None


class MockDetectionProvider:
    """Deterministic contract adapter with no training-framework dependency."""

    def __init__(self, config: MockProviderConfig) -> None:
        self._config = config

    def detect(self, request: ImageRequest) -> Detection2DArray:
        """Validate the artifact then emit one stable contract-only detection."""
        if self._config.failure_reason is not None:
            raise ProviderFailureError(self._config.failure_reason)
        artifact_path = (self._config.model_root / self._config.model.artifact.relative_path).resolve()
        if not artifact_path.is_relative_to(self._config.model_root.resolve()):
            raise ModelIntegrityError("model artifact path escapes model root")
        observed_hash = sha256_file(artifact_path)
        if observed_hash != self._config.model.artifact.sha256:
            raise ModelIntegrityError("model artifact hash mismatch")
        target = self._config.dataset.class_map[0]
        return Detection2DArray(
            contract=ROS_DETECTION_CONTRACT,
            image_id=request.image_id,
            frame_id=request.frame_id,
            detections=(
                Detection2D(
                    class_id=target.class_id,
                    class_name=target.name,
                    score=0.9,
                    bbox=BoundingBox2D(center_x=0.5, center_y=0.5, width=0.5, height=0.5),
                ),
            ),
        )
