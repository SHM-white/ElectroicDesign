#!/usr/bin/env python3
"""Visual diagnosis: run tag detection on saved camera frames with annotation overlay.

Usage: python3 tools/test_sim_tag_detection.py [--frames 18-30]

Shows each frame with ArUco detection overlay and pose estimation results.
Press 'n' for next frame, 'q' to quit, any other key to step through.
"""
from __future__ import annotations

import sys
import glob
import os

import cv2
import numpy as np

# tag36h11 ID 0 detection with tuned parameters
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.15


def make_detector():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 201
    p.adaptiveThreshWinSizeStep = 4
    p.adaptiveThreshConstant = 3
    p.minMarkerPerimeterRate = 0.03
    p.maxMarkerPerimeterRate = 4.0
    p.polygonalApproxAccuracyRate = 0.05
    p.minCornerDistanceRate = 0.05
    p.minDistanceToBorder = 0
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return d, p


def detect_and_draw(img, dict_aruco, params):
    """Run detection and draw overlay. Returns annotated image and detection info."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    annotated = img.copy()

    # Try detection
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dict_aruco, parameters=params)

    # Draw rejected candidates in red
    if rejected:
        for r in rejected:
            pts = r.reshape(4, 2).astype(int)
            cv2.polylines(annotated, [pts], True, (0, 0, 255), 1)

    found = False
    if ids is not None and len(ids) > 0:
        for i, (cid, corner) in enumerate(zip(ids.flatten(), corners)):
            if cid == 0:  # Only tag ID 0
                found = True
                pts = corner.reshape(4, 2).astype(int)
                cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)
                # Draw corners
                for j, pt in enumerate(pts):
                    cv2.circle(annotated, tuple(pt), 4, (0, 255, 0), -1)
                    cv2.putText(annotated, str(j), tuple(pt + [5, -5]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                # Label
                cx, cy = pts.mean(axis=0).astype(int)
                cv2.putText(annotated, f"TAG 36h11 ID={cid}", (cx - 60, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Info overlay
    info_color = (0, 255, 0) if found else (0, 0, 255)
    status = "DETECTED" if found else f"NOT FOUND (rejected={len(rejected)})"
    cv2.putText(annotated, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, info_color, 2)

    # Image stats
    cv2.putText(annotated, f"{w}x{h} min={gray.min()} max={gray.max()} mean={gray.mean():.0f}",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return annotated, found, len(rejected) if rejected else 0


def parse_range(range_str, max_val):
    """Parse '18-30' or '18,20,22' into list of ints."""
    if not range_str:
        return list(range(1, max_val + 1))
    result = []
    for part in range_str.split(","):
        if "-" in part:
            a, b = part.split("-", 1)
            result.extend(range(int(a), int(b) + 1))
        else:
            result.append(int(part))
    return [x for x in result if 1 <= x <= max_val]


def main():
    # Parse args
    frame_range = None
    headless = "--headless" in sys.argv
    for arg in sys.argv[1:]:
        if arg.startswith("--frames"):
            idx = sys.argv.index(arg)
            if idx + 1 < len(sys.argv):
                frame_range = sys.argv[idx + 1]
            elif "=" in arg:
                frame_range = arg.split("=", 1)[1]

    # Find frame files
    frame_dir = os.path.join(os.path.dirname(__file__), "..", "camera_frames")
    if not os.path.isdir(frame_dir):
        # Try Docker path
        frame_dir = "/workspace/camera_frames"
    if not os.path.isdir(frame_dir):
        print(f"ERROR: camera_frames directory not found")
        return 1

    all_frames = sorted(glob.glob(os.path.join(frame_dir, "frame_*.png")))
    if not all_frames:
        print(f"ERROR: no frames found in {frame_dir}")
        return 1

    total = len(all_frames)
    selected = parse_range(frame_range, total)
    frames = [all_frames[i - 1] for i in selected if i <= total]

    print(f"Found {total} frames, testing {len(frames)} frames: {selected}")
    print(f"Headless: {headless}")
    print()

    dict_aruco, params = make_detector()

    detected_count = 0
    for idx, fpath in enumerate(frames):
        img = cv2.imread(fpath)
        if img is None:
            print(f"  {os.path.basename(fpath)}: FAILED TO LOAD")
            continue

        annotated, found, n_rej = detect_and_draw(img, dict_aruco, params)
        fname = os.path.basename(fpath)
        if found:
            detected_count += 1
            print(f"  {fname}: DETECTED  (rejected={n_rej})")
        else:
            print(f"  {fname}: not found (rejected={n_rej})")

        if not headless:
            cv2.imshow("Tag Detection Diagnosis", annotated)
            key = cv2.waitKey(0) & 0xFF
            if key in (ord("q"), 27):
                break

    print(f"\nResult: {detected_count}/{len(frames)} frames detected")

    if not headless:
        cv2.destroyAllWindows()
    return 0 if detected_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
