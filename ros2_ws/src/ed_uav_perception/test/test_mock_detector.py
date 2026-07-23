"""Tests for the mock detector provider and detector node integration."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_perception.provider_interface import (
    MockDetectorProvider,
    ONNXDetectorProvider,
    OpenVINODetectorProvider,
)
from ed_uav_perception.detector_node import DetectorNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_image(h: int = 480, w: int = 640) -> np.ndarray:
    """Create a dummy RGB uint8 image for testing."""
    rng = np.random.RandomState(42)
    return rng.randint(0, 255, (h, w, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# test_mock_detector_deterministic
# ---------------------------------------------------------------------------


def test_mock_detector_deterministic_same_image() -> None:
    """Same input image always yields the same output."""
    provider = MockDetectorProvider()
    img = _make_image()
    result_1 = provider.detect(img)
    result_2 = provider.detect(img)
    assert len(result_1) == len(result_2) > 0
    for d1, d2 in zip(result_1, result_2):
        assert d1.bbox.center.position.x == pytest.approx(d2.bbox.center.position.x)
        assert d1.bbox.center.position.y == pytest.approx(d2.bbox.center.position.y)
        assert d1.bbox.size_x == pytest.approx(d2.bbox.size_x)
        assert d1.bbox.size_y == pytest.approx(d2.bbox.size_y)
        assert d1.results[0].hypothesis.class_id == d2.results[0].hypothesis.class_id
        assert d1.results[0].hypothesis.score == pytest.approx(d2.results[0].hypothesis.score)


def test_mock_detector_deterministic_different_size() -> None:
    """Different image sizes produce scaled but proportionally consistent detections."""
    provider = MockDetectorProvider()
    img1 = _make_image(240, 320)
    img2 = _make_image(480, 640)
    r1 = provider.detect(img1)
    r2 = provider.detect(img2)
    assert len(r1) == len(r2)
    for d1, d2 in zip(r1, r2):
        # Centres scale linearly with image size (2× in each axis).
        assert d2.bbox.center.position.x == pytest.approx(d1.bbox.center.position.x * 2.0)
        assert d2.bbox.center.position.y == pytest.approx(d1.bbox.center.position.y * 2.0)
        assert d2.bbox.size_x == pytest.approx(d1.bbox.size_x * 2.0)
        assert d2.bbox.size_y == pytest.approx(d1.bbox.size_y * 2.0)


# ---------------------------------------------------------------------------
# test_detector_rejects_stale_image
# ---------------------------------------------------------------------------


def test_detector_rejects_stale_image() -> None:
    """Images older than the staleness threshold must not trigger inference."""
    import rclpy
    from sensor_msgs.msg import Image

    rclpy.init()

    try:
        provider = MockDetectorProvider()
        node = DetectorNode(provider=provider, node_name="test_stale_node")

        # Build an image stamped far in the past.
        img = Image()
        img.header.stamp.sec = 0
        img.header.stamp.nanosec = 0
        img.height = 240
        img.width = 320
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = 320 * 3
        img.data = np.zeros(320 * 240 * 3, dtype=np.uint8).tobytes()

        # Feed the stale image directly.
        node._image_callback(img)

        # No detections should have been produced.
        assert node.detection_count == 0
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# test_provider_isolation
# ---------------------------------------------------------------------------


class _CrashingProvider(MockDetectorProvider):
    """Provider that always raises to test crash isolation."""

    def detect(self, image: np.ndarray) -> list:
        raise RuntimeError("simulated provider crash")


def test_provider_isolation() -> None:
    """A crashing provider must not crash the detector node."""
    import rclpy
    from sensor_msgs.msg import Image

    rclpy.init()

    try:
        provider = _CrashingProvider()
        node = DetectorNode(provider=provider, node_name="test_isolation_node")

        img = Image()
        img.header.stamp.sec = 0
        img.header.stamp.nanosec = 0
        img.height = 240
        img.width = 320
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = 320 * 3
        img.data = np.zeros(320 * 240 * 3, dtype=np.uint8).tobytes()

        # Must not raise.
        node._image_callback(img)
        assert node.detection_count == 0
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Stub provider smoke tests
# ---------------------------------------------------------------------------


def test_onnx_stub_raises_not_implemented() -> None:
    provider = ONNXDetectorProvider()
    assert provider.provider_type == "onnx"
    with pytest.raises(NotImplementedError):
        provider.detect(_make_image())


def test_openvino_stub_raises_not_implemented() -> None:
    provider = OpenVINODetectorProvider()
    assert provider.provider_type == "openvino"
    with pytest.raises(NotImplementedError):
        provider.detect(_make_image())
