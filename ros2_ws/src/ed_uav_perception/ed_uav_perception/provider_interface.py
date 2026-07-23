"""Abstract detector provider interface with mock and stub implementations."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)


class DetectorProvider(ABC):
    """Abstract interface for 2D object detection providers.

    All provider implementations must accept a numpy image array and return
    a list of vision_msgs Detection2D objects. Providers must NOT import
    Ultralytics in the ROS process.
    """

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection2D]:
        """Run detection on a single image frame.

        Args:
            image: Input image as numpy array (H, W, 3) in RGB uint8.

        Returns:
            List of Detection2D messages. Empty list if no objects found.
        """
        ...

    @property
    @abstractmethod
    def version(self) -> str:
        """Provider model version string."""
        ...

    @property
    @abstractmethod
    def provider_type(self) -> str:
        """Provider type identifier (e.g. 'mock', 'onnx', 'openvino')."""
        ...


# ---------------------------------------------------------------------------
# Mock provider - deterministic for testing
# ---------------------------------------------------------------------------


class MockDetectorProvider(DetectorProvider):
    """Deterministic mock provider returning fixed detections for testing.

    Detections are scaled relative to the input image dimensions so that the
    same image produces the same output, satisfying the determinism contract.
    """

    MOCK_CLASSES: tuple[tuple[str, float, float, float, float, float], ...] = (
        # (class_id, score, cx_norm, cy_norm, w_norm, h_norm)
        ("terminal_target", 0.95, 0.5, 0.4, 0.3, 0.25),
        ("obstacle", 0.82, 0.25, 0.6, 0.15, 0.2),
        ("marker", 0.71, 0.75, 0.7, 0.12, 0.12),
    )

    def __init__(self, model_version: str = "mock-1.0.0") -> None:
        self._version = model_version

    @property
    def version(self) -> str:
        return self._version

    @property
    def provider_type(self) -> str:
        return "mock"

    def detect(self, image: np.ndarray) -> list[Detection2D]:
        """Return deterministic detections scaled to image dimensions.

        Each detection's bounding box is scaled from normalized coordinates
        using the input image height and width. The same image always
        produces the same output.
        """
        h, w = image.shape[:2]
        detections: list[Detection2D] = []

        for class_id, score, cx_n, cy_n, bw_n, bh_n in self.MOCK_CLASSES:
            bbox = BoundingBox2D()
            bbox.center = Pose2D(
                position=Point2D(x=float(cx_n * w), y=float(cy_n * h)),
                theta=0.0,
            )
            bbox.size_x = float(bw_n * w)
            bbox.size_y = float(bh_n * h)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis = ObjectHypothesis(class_id=class_id, score=float(score))
            hypothesis.pose = PoseWithCovariance()

            det = Detection2D()
            det.bbox = bbox
            det.results = [hypothesis]
            det.id = f"{class_id}_{hash((w, h, class_id)) & 0xFFFF:04x}"
            detections.append(det)

        return detections


# ---------------------------------------------------------------------------
# ONNX stub - placeholder for future integration
# ---------------------------------------------------------------------------


class ONNXDetectorProvider(DetectorProvider):
    """Stub for future ONNX Runtime inference.

    This provider does NOT load model weights. It raises NotImplementedError
    on detect() calls until a real ONNX backend is wired in.
    """

    @property
    def version(self) -> str:
        return "onnx-stub-0.1.0"

    @property
    def provider_type(self) -> str:
        return "onnx"

    def detect(self, image: np.ndarray) -> list[Detection2D]:
        """Stub - not yet implemented."""
        raise NotImplementedError("ONNX inference backend not implemented")


# ---------------------------------------------------------------------------
# OpenVINO stub - placeholder for future integration
# ---------------------------------------------------------------------------


class OpenVINODetectorProvider(DetectorProvider):
    """Stub for future OpenVINO inference.

    This provider does NOT load model weights. It raises NotImplementedError
    on detect() calls until a real OpenVINO backend is wired in.
    """

    @property
    def version(self) -> str:
        return "openvino-stub-0.1.0"

    @property
    def provider_type(self) -> str:
        return "openvino"

    def detect(self, image: np.ndarray) -> list[Detection2D]:
        """Stub - not yet implemented."""
        raise NotImplementedError("OpenVINO inference backend not implemented")
