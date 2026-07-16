#!/usr/bin/env python3
"""灰色数字中心检测与图片浏览 GUI。

不识别具体编号，只从绿色背景中提取中性灰色字符，并把同一编号的
一至两个字符聚合为一个中心点。

用法：
    python3 drone/test_gray_centers.py
    python3 drone/test_gray_centers.py --glob 'mission_vision_*.png'
    python3 drone/test_gray_centers.py --image IMG_20260715_175823.jpg
    python3 drone/test_gray_centers.py --save-dir tmp/gray_centers --no-gui

GUI 按键：
    ←/a、→/d    上一张/下一张
    Space        自动播放/暂停
    m            切换掩膜显示
    s            保存当前标注图
    q/Esc        退出
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class CharacterComponent:
    """单个灰色字符组件。"""

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
    """由一至两个字符组成的数字标记候选。"""

    x: int
    y: int
    width: int
    height: int
    score: float
    characters: tuple[CharacterComponent, ...]

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.width / 2.0, self.y + self.height / 2.0


@dataclass(frozen=True)
class DetectionResult:
    markers: tuple[GrayMarker, ...]
    mask: np.ndarray
    dark_scene: bool
    delta_limit: int
    brightness_floor: float


class GrayMarkerDetector:
    """用颜色中性度、绿色环带和字符几何提取数字中心。"""

    def detect(self, frame: np.ndarray) -> DetectionResult:
        if frame is None or frame.size == 0:
            raise ValueError("empty image")

        blue, green, red = cv2.split(frame)
        frame_i16 = frame.astype(np.int16)
        brightness = np.max(frame, axis=2)
        neutral_spread = np.max(frame_i16, axis=2) - np.min(frame_i16, axis=2)
        green_delta = (
            green.astype(np.int16)
            - np.maximum(red, blue).astype(np.int16)
        )
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
            # 暗场的 delta_limit 越大，绿色纹理越容易被当作中性灰色。
            # 只要严格掩膜已经找到高质量候选，就不应靠“候选更多”获胜。
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
            if component.label in used_labels:
                continue
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

        # 当前样本和飞行策略都把目标数字保持在主点附近。默认只保留
        # 中央 70% 区域内最可信的候选；没有中央候选时才回退到全图第一名。
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
            (1.0 - height_ratio)
            + center_delta
            + 0.25 * gap
            + 0.30 * overlap
        )
        score = max(0.0, 1.0 - geometry_error)
        return GrayMarker(x, y, width, height, score, (left, right))

    @staticmethod
    def _make_single(
        component: CharacterComponent,
        labels: np.ndarray,
        mask: np.ndarray,
    ) -> GrayMarker:
        fill = component.area / max(1.0, component.width * component.height)
        center_x, center_y = component.center
        image_height, image_width = mask.shape
        center_distance = np.hypot(
            (center_x - image_width / 2.0) / image_width,
            (center_y - image_height / 2.0) / image_height,
        )
        score = max(0.0, 0.58 + min(fill, 0.45) - 0.45 * center_distance)
        return GrayMarker(
            component.x,
            component.y,
            component.width,
            component.height,
            score,
            (component,),
        )


def draw_result(frame: np.ndarray, result: DetectionResult) -> np.ndarray:
    output = frame.copy()
    for index, marker in enumerate(result.markers, 1):
        color = (0, 255, 255) if len(marker.characters) == 2 else (0, 165, 255)
        cv2.rectangle(
            output,
            (marker.x, marker.y),
            (marker.x + marker.width, marker.y + marker.height),
            color,
            max(2, round(min(frame.shape[:2]) / 400)),
        )
        center_x, center_y = marker.center
        center = int(round(center_x)), int(round(center_y))
        cv2.drawMarker(
            output, center, (0, 0, 255), cv2.MARKER_CROSS,
            max(18, round(min(frame.shape[:2]) / 30)),
            max(2, round(min(frame.shape[:2]) / 500)),
        )
        label = f"C{index} ({center[0]},{center[1]}) {marker.score:.2f}"
        cv2.putText(
            output, label, (marker.x, max(24, marker.y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.55, min(frame.shape[:2]) / 1400),
            color, 2, cv2.LINE_AA,
        )

    status = (
        f"centers={len(result.markers)}  "
        f"scene={'dark' if result.dark_scene else 'bright'}  "
        f"delta<={result.delta_limit}  floor={result.brightness_floor:.0f}"
    )
    cv2.rectangle(output, (0, 0), (output.shape[1], 42), (0, 0, 0), -1)
    cv2.putText(
        output, status, (12, 29), cv2.FONT_HERSHEY_SIMPLEX,
        0.72, (80, 255, 80), 2, cv2.LINE_AA,
    )
    return output


def _fit_for_display(image: np.ndarray, max_width: int = 1500,
                     max_height: int = 900) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, max_width / width, max_height / height)
    if scale >= 1.0:
        return image
    return cv2.resize(
        image, (round(width * scale), round(height * scale)),
        interpolation=cv2.INTER_AREA,
    )


def collect_images(root: Path, patterns: Iterable[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(path for path in root.glob(pattern) if path.is_file())
    return sorted(set(paths), key=lambda path: path.name)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='只处理一张图片')
    parser.add_argument(
        '--glob', action='append', dest='patterns',
        help='相对当前目录的图片通配符，可重复指定',
    )
    parser.add_argument('--save-dir', type=Path, help='保存全部标注结果')
    parser.add_argument('--no-gui', action='store_true', help='不打开窗口')
    args = parser.parse_args()

    root = Path.cwd()
    if args.image:
        images = [args.image if args.image.is_absolute() else root / args.image]
    else:
        patterns = args.patterns or [
            'Vision Test - [[]h[]] help_screenshot_15.07.2026*.png',
            'IMG_20260715_175823.jpg',
            'mission_vision_216786120831.png',
            'mission_vision_294206335845.png',
            'mission_vision_295544256805.png',
            'mission_vision_343598988361.png',
        ]
        images = collect_images(root, patterns)

    if not images:
        print('未找到测试图片。')
        return 2

    detector = GrayMarkerDetector()
    results: list[tuple[Path, np.ndarray, DetectionResult, np.ndarray]] = []
    for path in images:
        frame = cv2.imread(str(path))
        if frame is None:
            print(f'跳过无法读取的图片: {path}')
            continue
        result = detector.detect(frame)
        annotated = draw_result(frame, result)
        results.append((path, frame, result, annotated))
        centers = [
            (round(marker.center[0], 1), round(marker.center[1], 1))
            for marker in result.markers
        ]
        print(f'{path.name}: {len(centers)} center(s) {centers}')

        if args.save_dir:
            args.save_dir.mkdir(parents=True, exist_ok=True)
            output_path = args.save_dir / f'{path.stem}_gray_centers.jpg'
            cv2.imwrite(str(output_path), annotated)

    if not results:
        return 2
    if args.no_gui:
        return 0

    window_name = 'Gray Digit Centers - arrows/a,d browse, m mask, s save, q quit'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    index = 0
    show_mask = False
    autoplay = False
    while True:
        path, frame, result, annotated = results[index]
        display = cv2.cvtColor(result.mask, cv2.COLOR_GRAY2BGR) \
            if show_mask else annotated.copy()
        title = f'{index + 1}/{len(results)}  {path.name}'
        cv2.putText(
            display, title, (12, display.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2, cv2.LINE_AA,
        )
        cv2.imshow(window_name, _fit_for_display(display))
        key = cv2.waitKey(700 if autoplay else 0) & 0xFF
        if autoplay and key == 255:
            index = (index + 1) % len(results)
            continue
        if key in (27, ord('q')):
            break
        if key in (ord('d'), 83):
            index = (index + 1) % len(results)
        elif key in (ord('a'), 81):
            index = (index - 1) % len(results)
        elif key == ord('m'):
            show_mask = not show_mask
        elif key == ord(' '):
            autoplay = not autoplay
        elif key == ord('s'):
            output = root / f'{path.stem}_gray_centers.jpg'
            cv2.imwrite(str(output), annotated)
            print(f'已保存: {output}')

    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
