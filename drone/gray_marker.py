"""中性灰色数字标记的几何中心检测。

本模块不识别具体编号，只从绿色地块背景中提取一至两个灰色字符，
输出整体外接框中心供导航阶段进行几何辅助校准。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class CharacterComponent:
    x: int
    y: int
    width: int
    height: int
    area: int
    label: int

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.width / 2.0, self.y + self.height / 2.0

    @property
    def right(self) -> int:
        return self.x + self.width

    @property
    def bottom(self) -> int:
        return self.y + self.height


@dataclass(frozen=True)
class GrayMarker:
    x: int
    y: int
    width: int
    height: int
    score: float
    characters: tuple[CharacterComponent, ...]

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.width / 2.0, self.y + self.height / 2.0

    @property
    def box(self) -> tuple[int, int, int, int]:
        return self.x, self.y, self.width, self.height


@dataclass(frozen=True)
class DetectionResult:
    markers: tuple[GrayMarker, ...]
    mask: np.ndarray
    dark_scene: bool
    delta_limit: int
    brightness_floor: float

    @property
    def best_marker(self) -> Optional[GrayMarker]:
        return self.markers[0] if self.markers else None


class GrayMarkerDetector:
    """用颜色中性度、绿色环带和字符几何提取数字中心。"""

    def detect(self, frame: np.ndarray) -> DetectionResult:
        if frame is None or frame.size == 0:
            raise ValueError("empty image")

        blue, green, red = cv2.split(frame)
        frame_i16 = frame.astype(np.int16)
        brightness = np.max(frame, axis=2)
        neutral_spread = np.max(frame_i16, axis=2) - np.min(frame_i16, axis=2)
        green_delta = green.astype(np.int16) - np.maximum(
            red, blue,
        ).astype(np.int16)
        dark_scene = float(np.percentile(brightness, 90)) < 110.0
        if dark_scene:
            delta_limits = (0, 1, 2, 3)
            spread_limit = 8
            brightness_floor = float(np.percentile(brightness, 35))
        else:
            delta_limits = (20,)
            spread_limit = 30
            brightness_floor = 70.0

        best: Optional[DetectionResult] = None
        best_rank = -1.0
        for delta_limit in delta_limits:
            mask = np.uint8(
                (green_delta <= delta_limit)
                & (neutral_spread <= spread_limit)
                & (brightness >= brightness_floor)
            ) * 255
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8),
            )
            if dark_scene:
                mask = cv2.morphologyEx(
                    mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8),
                )

            components, labels = self._extract_components(
                mask, green_delta, dark_scene,
            )
            markers = self._group_components(components, labels, mask)
            result = DetectionResult(
                tuple(markers), mask, dark_scene, delta_limit, brightness_floor,
            )
            best_marker_score = max(
                (marker.score for marker in markers), default=0.0,
            )
            pair_bonus = 0.12 * sum(
                len(marker.characters) == 2 for marker in markers
            )
            noise_penalty = 0.10 * max(0, len(markers) - 2) \
                + 0.025 * max(0, len(components) - 4)
            loosen_penalty = 0.06 * delta_limit if dark_scene else 0.0
            rank = best_marker_score + pair_bonus \
                - noise_penalty - loosen_penalty
            if rank > best_rank:
                best = result
                best_rank = rank

        assert best is not None
        return best

    def _extract_components(
        self,
        mask: np.ndarray,
        green_delta: np.ndarray,
        dark_scene: bool,
    ) -> tuple[list[CharacterComponent], np.ndarray]:
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        image_height, image_width = mask.shape
        image_area = image_height * image_width
        components: list[CharacterComponent] = []

        for label, stat in enumerate(stats[1:], 1):
            x, y, width, height, area = map(int, stat)
            if not 0.045 * image_height <= height <= 0.36 * image_height:
                continue
            if not 0.008 * image_width <= width <= 0.24 * image_width:
                continue
            if area < 0.00025 * image_area:
                continue
            aspect = width / max(1.0, height)
            fill = area / max(1.0, width * height)
            if not 0.04 <= aspect <= 1.30 or not 0.05 <= fill <= 0.85:
                continue
            center_x = x + width / 2.0
            center_y = y + height / 2.0
            if not 0.03 * image_width <= center_x <= 0.97 * image_width:
                continue
            if not 0.05 * image_height <= center_y <= 0.95 * image_height:
                continue
            if not self._has_green_support(
                green_delta, x, y, width, height, dark_scene,
            ):
                continue
            components.append(
                CharacterComponent(x, y, width, height, area, label),
            )

        components.sort(key=lambda item: item.area, reverse=True)
        return components[:24], labels

    @staticmethod
    def _has_green_support(
        green_delta: np.ndarray,
        x: int,
        y: int,
        width: int,
        height: int,
        dark_scene: bool,
    ) -> bool:
        image_height, image_width = green_delta.shape
        padding = max(8, int(height * 0.28))
        left = max(0, x - padding)
        top = max(0, y - padding)
        right = min(image_width, x + width + padding)
        bottom = min(image_height, y + height + padding)
        outer = green_delta[top:bottom, left:right]
        if outer.size == 0:
            return False
        ring = np.ones(outer.shape, dtype=bool)
        ring[y - top:y - top + height, x - left:x - left + width] = False
        if not np.any(ring):
            return False
        threshold = 2 if dark_scene else 12
        required_ratio = 0.12 if dark_scene else 0.18
        return float(np.mean(outer[ring] >= threshold)) >= required_ratio

    def _group_components(
        self,
        components: list[CharacterComponent],
        labels: np.ndarray,
        mask: np.ndarray,
    ) -> list[GrayMarker]:
        pair_candidates: list[GrayMarker] = []
        for index, left in enumerate(components):
            for right in components[index + 1:]:
                marker = self._make_pair(left, right)
                if marker is not None:
                    pair_candidates.append(marker)

        pair_candidates.sort(key=lambda marker: marker.score, reverse=True)
        markers: list[GrayMarker] = []
        used_labels: set[int] = set()
        for marker in pair_candidates:
            marker_labels = {component.label for component in marker.characters}
            if marker_labels & used_labels:
                continue
            markers.append(marker)
            used_labels.update(marker_labels)

        for component in components:
            if component.label not in used_labels:
                markers.append(self._make_single(component, labels, mask))

        markers = [marker for marker in markers if marker.score >= 0.34]
        image_height, image_width = mask.shape

        def marker_rank(marker: GrayMarker) -> float:
            center_x, center_y = marker.center
            distance = np.hypot(
                (center_x - image_width / 2.0) / image_width,
                (center_y - image_height / 2.0) / image_height,
            )
            return marker.score - 0.65 * distance

        markers.sort(key=marker_rank, reverse=True)
        if not markers:
            return []
        central = [
            marker for marker in markers
            if 0.15 * image_width <= marker.center[0] <= 0.85 * image_width
            and 0.15 * image_height <= marker.center[1] <= 0.85 * image_height
        ]
        return (central or markers)[:1]

    @staticmethod
    def _make_pair(
        first: CharacterComponent,
        second: CharacterComponent,
    ) -> Optional[GrayMarker]:
        left, right = sorted((first, second), key=lambda item: item.x)
        max_height = max(left.height, right.height)
        min_height = min(left.height, right.height)
        height_ratio = min_height / max(1.0, max_height)
        center_delta = abs(left.center[1] - right.center[1]) / max_height
        gap = max(0, right.x - left.right) / max_height
        overlap = max(0, left.right - right.x) / max_height
        if height_ratio < 0.58 or center_delta > 0.30:
            return None
        if gap > 0.85 or overlap > 0.28:
            return None

        x = min(left.x, right.x)
        y = min(left.y, right.y)
        right_edge = max(left.right, right.right)
        bottom = max(left.bottom, right.bottom)
        width = right_edge - x
        height = bottom - y
        combined_aspect = width / max(1.0, height)
        if not 0.22 <= combined_aspect <= 1.70:
            return None

        geometry_error = (
            (1.0 - height_ratio) + center_delta
            + 0.25 * gap + 0.30 * overlap
        )
        score = max(0.0, 1.0 - geometry_error)
        return GrayMarker(x, y, width, height, score, (left, right))

    @staticmethod
    def _make_single(
        component: CharacterComponent,
        labels: np.ndarray,
        mask: np.ndarray,
    ) -> GrayMarker:
        del labels
        fill = component.area / max(1.0, component.width * component.height)
        center_x, center_y = component.center
        image_height, image_width = mask.shape
        center_distance = np.hypot(
            (center_x - image_width / 2.0) / image_width,
            (center_y - image_height / 2.0) / image_height,
        )
        score = max(0.0, 0.58 + min(fill, 0.45) - 0.45 * center_distance)
        return GrayMarker(
            component.x, component.y, component.width, component.height,
            score, (component,),
        )
