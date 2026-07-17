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
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

try:
    from .gray_marker import DetectionResult, GrayMarkerDetector
except ImportError:
    from gray_marker import DetectionResult, GrayMarkerDetector


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
            'mission_vision_*.png'
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
