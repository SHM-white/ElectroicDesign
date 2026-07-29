"""Render and process one prescribed target frame through the real pipeline.

Run from the repository root:
python3 ros2_ws/src/ed_uav_perception/test/target_pnp_driver.py OUTPUT_IMAGE
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import cv2

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from target_fixture import render_target  # noqa: E402


def main() -> int:
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import (
        AcceptedObservation,
        CameraModel,
        FrameContext,
        MotionContext,
        ObservationRequest,
        PoseLimits,
    )

    rendered = render_target()
    output = Path(sys.argv[1]) if len(sys.argv) == 2 else Path("target-render.png")
    if not cv2.imwrite(str(output), rendered.image):
        print(json.dumps({"status": "driver_error", "reason": "image_write_failed"}))
        return 2
    request = ObservationRequest(
        rendered.image,
        CameraModel(rendered.camera_matrix, rendered.distortion, 640, 480, "camera_optical", True),
        FrameContext(
            100.0,
            100.25 if os.environ.get("TARGET_DRIVER_STALE") == "1" else 100.05,
            42,
            "d2026-circle-cross-v1",
        ),
        MotionContext(100.02, 0, 0.18, 0.6, None),
        PoseLimits(),
    )
    result = observe_target(request)
    if not isinstance(result, AcceptedObservation):
        print(json.dumps({"status": "rejected", "reason": result.reject_reason.value}))
        return 1
    translation_error = float(
        ((result.pose.translation_m - rendered.tvec.reshape(3)) ** 2).sum() ** 0.5
    )
    estimated_rotation, _ = cv2.Rodrigues(result.pose.rotation_vector)
    expected_rotation, _ = cv2.Rodrigues(rendered.rvec)
    delta_rotation = estimated_rotation @ expected_rotation.T
    rotation_error = math.acos(
        max(-1.0, min(1.0, (float(delta_rotation.trace()) - 1.0) / 2.0))
    )
    print(
        json.dumps(
            {
                "status": "accepted",
                "frame_id": result.frame_id,
                "source_sequence": result.source_sequence,
                "candidate_count": result.candidate_count,
                "reprojection_rms_px": result.reprojection_rms_px,
                "quality": result.quality,
                "translation_m": result.pose.translation_m.tolist(),
                "translation_residual_m": translation_error,
                "rotation_residual_rad": rotation_error,
                "rotation_residual_deg": math.degrees(rotation_error),
                "render": str(output),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
