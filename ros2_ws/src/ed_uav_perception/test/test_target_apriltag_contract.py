import numpy as np

from ed_uav_perception import target_detector


def test_target_detector_selects_only_centered_tag36h11_id_zero(monkeypatch) -> None:
    calls = []
    sentinel = object()

    class Detector:
        def detect(self, image, target_tag_id=None):
            calls.append((image.shape, target_tag_id))
            return sentinel

    monkeypatch.setattr(target_detector, "_apriltag_detector", Detector())

    result = target_detector.detect_target(
        np.zeros((64, 64), dtype=np.uint8),
        target_detector.REVISION_APRILTAG,
    )

    assert result is sentinel
    assert calls == [((64, 64), target_detector.APRILTAG_ID)]
