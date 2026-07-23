"""Immutable data, model, and provider-neutral detector contracts."""

from .runtime import Detection2D, Detection2DArray, DetectionProvider, ImageRequest
from .schema import DatasetManifest, ModelManifest

__all__ = [
    "DatasetManifest",
    "Detection2D",
    "Detection2DArray",
    "DetectionProvider",
    "ImageRequest",
    "ModelManifest",
]
