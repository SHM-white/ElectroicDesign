#!/usr/bin/env python3
"""Live camera landing-marker recognition test with real-time PnP overlay."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/ed_uav_perception"))
sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/ed_uav_camera"))

from ed_uav_perception.target_annotation import (
    AnnotationFrame,
    render_target_observation,
)
from ed_uav_perception.target_detector import detect_target, DetectionFailure
from ed_uav_perception.target_pose import estimate_target_pose
from ed_uav_perception.target_types import (
    AcceptedObservation,
    CameraModel,
    MotionContext,
    ObservationResult,
    PoseLimits,
    PoseRejection,
    RejectedObservation,
)

NARROW_CALIBRATION = (
    REPO_ROOT
    / "calibration_data/narrow_1280x720_20260729T155820Z/camera_info.yaml"
)
WIDE_CALIBRATION = (
    REPO_ROOT
    / "calibration_data/wide_1280x720_20260729T160215Z/camera_info.yaml"
)


def load_camera_model(yaml_path: Path) -> CameraModel:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)
    w = int(data["image_width"])
    h = int(data["image_height"])
    return CameraModel(k, d, w, h, "camera_narrow_optical_frame", True)


def find_camera_device() -> tuple[str, CameraModel]:
    by_id = Path("/dev/v4l/by-id")
    narrow = by_id / "usb-DHZJ-250122-ZW_W19_HD_Webcam-video-index0"
    wide = by_id / "usb-DHZJ-240708-XH_W19_HD_Webcam-video-index0"
    if narrow.exists():
        device = str(narrow.resolve())
        camera = load_camera_model(NARROW_CALIBRATION)
        print(f"Using narrow camera: {device}")
        return device, camera
    if wide.exists():
        device = str(wide.resolve())
        camera = load_camera_model(WIDE_CALIBRATION)
        print(f"Using wide camera: {device}")
        return device, camera
    raise SystemExit("No known camera found in /dev/v4l/by-id")


def open_camera(device_path: str, camera: CameraModel) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera: {device_path}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.height)
    return cap


def observe(frame_bgr: np.ndarray, camera: CameraModel, seq: int) -> ObservationResult:
    now = time.monotonic()
    detection = detect_target(frame_bgr, "d2026-circle-cross-v1")
    if isinstance(detection, DetectionFailure):
        return RejectedObservation(now, seq, camera.frame_id, "d2026-circle-cross-v1", detection.reason)
    motion = MotionContext(
        acquisition_sec=now,
        receipt_steady_sec=now,
        turn_class=0,
        heading_rad=0.0,
        yaw_rate_rad_s=0.0,
        speed_m_s=0.0,
        prior=None,
    )
    result = estimate_target_pose(detection, camera, motion, PoseLimits())
    if isinstance(result, PoseRejection):
        return RejectedObservation(now, seq, camera.frame_id, "d2026-circle-cross-v1", result.reason, result.candidate_count, result.reprojection_rms_px)
    return AcceptedObservation(now, seq, camera.frame_id, "d2026-circle-cross-v1", 0.02, result)


def main() -> int:
    headless = "--headless" in sys.argv
    device_path, camera = find_camera_device()
    cap = open_camera(device_path, camera)

    print(f"Camera opened: {camera.width}x{camera.height}")
    if headless:
        print("Headless mode: processing 30 frames")
    else:
        print("Press 'q' or Escape to quit")

    seq = 0
    accepted_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed", file=sys.stderr)
            break
        seq += 1
        result = observe(frame, camera, seq)
        if isinstance(result, AcceptedObservation):
            accepted_count += 1
            t = result.pose.translation_m
            print(f"[{seq:3d}] ACCEPTED  X={t[0]:+.3f} Y={t[1]:+.3f} Z={t[2]:+.3f}  quality={result.pose.quality:.3f}  rms={result.pose.reprojection_rms_px:.3f}px")
        else:
            print(f"[{seq:3d}] REJECTED  reason={result.reject_reason.value}")
        if not headless:
            annotation_frame = AnnotationFrame(frame, camera)
            annotated = render_target_observation(annotation_frame, result)
            cv2.imshow("Landing Marker PnP", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
        elif seq >= 30:
            break

    cap.release()
    if not headless:
        cv2.destroyAllWindows()
    print(f"\nDone: {accepted_count}/{seq} frames accepted")
    return 0 if accepted_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
