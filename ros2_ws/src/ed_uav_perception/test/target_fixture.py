"""Deterministic rendered fixtures for the prescribed D-task target."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True, slots=True)
class RenderedTarget:
    image: np.ndarray
    camera_matrix: np.ndarray
    distortion: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray


def render_target(
    *,
    inner_diameter_m: float = 0.30,
    include_inner: bool = True,
    cross_axes: tuple[str, ...] = ("x", "y"),
    line_width_m: float = 0.02,
    tvec: np.ndarray | None = None,
) -> RenderedTarget:
    """Render raw distorted pixels for two rings and a cross."""
    width, height = 640, 480
    camera_matrix = np.array(
        [[720.0, 0.0, 320.0], [0.0, 715.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.array([-0.08, 0.025, 0.0005, -0.0003, 0.0], dtype=np.float64)
    rvec = np.array([[0.04], [-0.06], [0.18]], dtype=np.float64)
    translation = (
        np.array([[0.04], [-0.03], [1.45]], dtype=np.float64)
        if tvec is None
        else tvec.astype(np.float64)
    )
    image = np.full((height, width, 3), 245, dtype=np.uint8)

    def project(points: np.ndarray) -> np.ndarray:
        pixels, _ = cv2.projectPoints(
            points, rvec, translation, camera_matrix, distortion
        )
        return np.rint(pixels.reshape(-1, 2)).astype(np.int32)

    for diameter in (0.50, inner_diameter_m) if include_inner else (0.50,):
        angles = np.linspace(0.0, 2.0 * np.pi, 361)
        radius = diameter / 2.0
        ring = np.column_stack(
            (radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles))
        )
        cv2.polylines(image, [project(ring)], True, (8, 8, 8), 10, cv2.LINE_AA)

    thickness_px = max(1, round(718.0 * line_width_m / float(translation[2, 0])))
    axis_points = {
        "x": np.array([[-0.25, 0.0, 0.0], [0.25, 0.0, 0.0]]),
        "y": np.array([[0.0, -0.25, 0.0], [0.0, 0.25, 0.0]]),
    }
    for axis in cross_axes:
        endpoints = axis_points[axis]
        pixels = project(endpoints.astype(np.float64))
        cv2.line(
            image,
            tuple(pixels[0]),
            tuple(pixels[1]),
            (8, 8, 8),
            thickness_px,
            cv2.LINE_AA,
        )

    return RenderedTarget(image, camera_matrix, distortion, rvec, translation)


def ring_correspondences(rendered: RenderedTarget) -> tuple[np.ndarray, np.ndarray]:
    """Return 16 exact ring points for solver quality-gate tests."""
    angles = np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
    points = []
    for radius in (0.25, 0.15):
        points.extend((radius * np.cos(angles), radius * np.sin(angles), np.zeros(8)))
    object_points = np.column_stack(
        (
            np.concatenate((0.25 * np.cos(angles), 0.15 * np.cos(angles))),
            np.concatenate((0.25 * np.sin(angles), 0.15 * np.sin(angles))),
            np.zeros(16),
        )
    ).astype(np.float64)
    image_points, _ = cv2.projectPoints(
        object_points,
        rendered.rvec,
        rendered.tvec,
        rendered.camera_matrix,
        rendered.distortion,
    )
    return object_points, image_points.reshape(-1, 2)
