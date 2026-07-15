#!/usr/bin/env python3
"""
test_vision.py — 视觉识别独立测试脚本
======================================
不依赖飞控/状态机，独立测试相机 + 颜色识别 + 数字OCR。

用法:
    python3 test_vision.py                        # 自动选择: 有摄像头用摄像头, 无则用合成测试图
    python3 test_vision.py --camera 0             # 强制使用摄像头 0
    python3 test_vision.py --camera 0 --save      # 摄像头模式 + 保存截图
    python3 test_vision.py --image test_green.png # 识别单张图片
    python3 test_vision.py --generate             # 生成合成测试图并识别

按键 (摄像头/图片模式):
    q / ESC    退出
    s          保存当前帧为 screenshot_*.png
    g          切换: 显示绿色mask / 正常画面
    b          切换: 显示黑色边界mask
    r          切换: 显示灰色mask
    o          对当前帧执行 OCR 数字识别
    h          显示帮助

绿色/灰色占比实时打印在终端。
"""

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
import os
import sys
import time

# 确保能找到 drone 模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

import cv2
import numpy as np

from vision import BlockDetector, DigitReader


# ── 合成测试图生成 ──────────────────────────────────────

def generate_synthetic_test_image(with_digit: int = 21) -> np.ndarray:
    """生成一张模拟的「无人机俯视作业区」合成图，用于无摄像头时验证识别管线。"""
    h, w = 480, 640
    img = np.zeros((h, w, 3), dtype=np.uint8)

    # 背景: 淡灰色 (模拟灰色非播撒区)
    img[:] = (220, 220, 220)

    # 中央大块淡绿色播撒区
    green = (100, 230, 100)  # BGR 的淡绿色
    cv2.rectangle(img, (120, 80), (520, 400), green, -1)

    # 黑色边界线 (0.5cm ≈ 3px)
    cv2.rectangle(img, (120, 80), (520, 400), (0, 0, 0), 3)

    # 画数字
    if with_digit is not None:
        text = str(with_digit)
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 4.0
        thickness = 6
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
        tx = 320 - tw // 2
        ty = 240 + th // 2
        # 灰色数字
        cv2.putText(img, text, (tx, ty), font, font_scale, (200, 200, 200), thickness)

    return img


# ── 可视化叠加 ──────────────────────────────────────────

def draw_overlay(frame: np.ndarray, detector: BlockDetector,
                 green_ratio: float, gray_ratio: float,
                 ocr_result: int | None = None) -> np.ndarray:
    """在画面上叠加识别信息。"""
    out = frame.copy()
    h, w = out.shape[:2]

    # 顶部半透明状态栏
    overlay = out.copy()
    cv2.rectangle(overlay, (0, 0), (w, 35), (0, 0, 0), -1)
    out = cv2.addWeighted(out, 0.7, overlay, 0.3, 0)

    status = f"Green: {green_ratio:.1%}  Gray: {gray_ratio:.1%}"
    cv2.putText(out, status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

    if ocr_result is not None:
        ocr_text = f"OCR: {ocr_result}"
        cv2.putText(out, ocr_text, (w - 150, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

    return out


def show_mask(frame: np.ndarray, mask: np.ndarray, label: str) -> np.ndarray:
    """将二值 mask 转为可视化彩色图。"""
    colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    h, w = colored.shape[:2]
    cv2.putText(colored, f"[{label}]", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    return colored


# ── 主逻辑 ──────────────────────────────────────────────

def run_camera(camera_id: int, save_screenshots: bool = False):
    """摄像头实时识别循环。"""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"❌ 无法打开摄像头 {camera_id}")
        print("   尝试: python3 test_vision.py --generate (使用合成测试图)")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"✅ 摄像头已打开: {actual_w:.0f}x{actual_h:.0f}")
    _recognition_loop(cap, save_screenshots, is_live=True)


def run_mvs_camera(camera_id: int, save_screenshots: bool = False,
                   exposure_ms: float = 100.0, gain: float = 8.0):
    """使用海康 MVS SDK 启动工业相机实时识别。"""
    try:
        from mvs_camera import MvsCapture

        cap = MvsCapture(camera_id, auto_exposure=False,
                         exposure_us=exposure_ms * 1000.0, gain=gain)
    except (ImportError, RuntimeError) as exc:
        print(f"❌ 无法打开 MVS 相机 {camera_id}: {exc}")
        return

    print(f"✅ MVS 相机已打开: {cap.model} SN={cap.serial}")
    print(f"   曝光: {exposure_ms:.1f} ms  增益: {gain:.1f} dB")
    ret, frame = cap.read()
    if not ret or frame is None:
        print("❌ MVS 相机已连接，但未能读取首帧")
        cap.release()
        return

    cap.width = frame.shape[1]
    cap.height = frame.shape[0]
    print(f"   分辨率: {cap.width}x{cap.height}")
    _recognition_loop(cap, save_screenshots, is_live=True, initial_frame=frame)


def run_image(image_path: str):
    """单张图片识别。"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"❌ 无法读取图片: {image_path}")
        return

    detector = BlockDetector()
    reader = DigitReader()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_ratio = detector.calc_green_ratio(hsv)
    gray_ratio = detector.calc_gray_ratio(hsv)

    # 找到绿色区块, 在区块内做 OCR (避免全图噪点干扰)
    blocks = detector.find_green_blocks(hsv)
    if blocks:
        cx, cy, bw, bh, _ = blocks[0]
        block_roi = (cx - bw // 2, cy - bh // 2, bw, bh)
        ocr = reader.extract_digits(frame, block_roi=block_roi)
    else:
        ocr = reader.extract_digits(frame, detector=detector)

    print(f"图片: {image_path}")
    print(f"  绿色占比: {green_ratio:.1%}")
    print(f"  灰色占比: {gray_ratio:.1%}")
    print(f"  OCR 数字: {ocr}")

    out = draw_overlay(frame, detector, green_ratio, gray_ratio, ocr)
    cv2.imshow("Vision Test - Press any key to close", out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_synthetic(digit: int = 21):
    """合成测试图识别。"""
    print(f"生成合成测试图 (区块 {digit})...")
    frame = generate_synthetic_test_image(digit)
    _recognition_loop_static(frame)


def _recognition_loop_static(frame: np.ndarray):
    """对单帧进行识别并显示，按键可切换视图。"""
    detector = BlockDetector()
    reader = DigitReader()
    view_mode = 'normal'  # normal, green_mask, black_mask, gray_mask
    ocr_result = None

    while True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_ratio = detector.calc_green_ratio(hsv)
        gray_ratio = detector.calc_gray_ratio(hsv)

        if view_mode == 'normal':
            display = draw_overlay(frame, detector, green_ratio, gray_ratio, ocr_result)
        elif view_mode == 'green_mask':
            display = show_mask(frame, detector.detect_green_mask(hsv), 'GREEN MASK')
        elif view_mode == 'black_mask':
            display = show_mask(frame, detector.detect_black_mask(hsv), 'BLACK MASK')
        elif view_mode == 'gray_mask':
            display = show_mask(frame, detector.detect_gray_mask(hsv), 'GRAY MASK')
        else:
            display = frame

        # 找绿色区块轮廓
        blocks = detector.find_green_blocks(hsv)
        if view_mode == 'normal':
            for cx, cy, bw, bh, area in blocks[:3]:
                cv2.rectangle(display, (cx - bw // 2, cy - bh // 2),
                              (cx + bw // 2, cy + bh // 2), (0, 255, 0), 2)
                cv2.putText(display, f"{area:.0f}", (cx - 20, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        cv2.imshow("Vision Test - [h] help", display)
        key = cv2.waitKey(0) & 0xFF
        if key in (ord('q'), 27):  # q or ESC
            break
        elif key == ord('g'):
            view_mode = 'green_mask'
        elif key == ord('b'):
            view_mode = 'black_mask'
        elif key == ord('r'):
            view_mode = 'gray_mask'
        elif key == ord('n'):
            view_mode = 'normal'
        elif key == ord('o'):
            ocr_result = reader.extract_digits(frame, detector=detector)
            print(f"  OCR → {ocr_result}")
        elif key == ord('h'):
            _print_keys_help()

    cv2.destroyAllWindows()


def _recognition_loop(cap, save_screenshots: bool = False, is_live: bool = False,
                      initial_frame: np.ndarray | None = None):
    """通用识别循环，支持实时摄像头或图片序列。"""
    detector = BlockDetector()
    reader = DigitReader()
    view_mode = 'normal'
    ocr_result = None
    screenshot_count = 0
    last_ocr_time = 0.0
    frame_count = 0
    fps_timer = time.time()
    ocr_executor = ThreadPoolExecutor(max_workers=1,
                                      thread_name_prefix='vision-ocr')
    ocr_future: Future[int | None] | None = None

    print("按键: [q]退出 [g]绿色mask [b]黑色mask [r]灰色mask [n]正常 [o]OCR [s]截图 [h]帮助")
    print()

    frame = initial_frame
    while True:
        if frame is None:
            ret, frame = cap.read()
        else:
            ret = True
        if not ret:
            if is_live:
                print("⚠ 摄像头断流，重试中...")
                time.sleep(0.5)
                continue
            else:
                break

        frame_count += 1
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_ratio = detector.calc_green_ratio(hsv)
        gray_ratio = detector.calc_gray_ratio(hsv)

        # 每 2 秒最多提交一个 OCR；结果完成前继续显示新帧。
        now = time.time()
        if ocr_future is not None and ocr_future.done():
            ocr_result = ocr_future.result()
            ocr_future = None
            if ocr_result is not None:
                print(f"  [{now:.0f}] OCR → {ocr_result}")

        if now - last_ocr_time > 2.0 and ocr_future is None:
            ocr_future = ocr_executor.submit(
                reader.extract_digits, frame, detector=detector
            )
            last_ocr_time = now

        # 视图切换
        if view_mode == 'normal':
            display = draw_overlay(frame, detector, green_ratio, gray_ratio, ocr_result)
            # 找绿色区块轮廓
            blocks = detector.find_green_blocks(hsv)
            for cx, cy, bw, bh, area in blocks[:3]:
                cv2.rectangle(display, (cx - bw // 2, cy - bh // 2),
                              (cx + bw // 2, cy + bh // 2), (0, 255, 0), 2)
        elif view_mode == 'green_mask':
            display = show_mask(frame, detector.detect_green_mask(hsv), 'GREEN MASK')
        elif view_mode == 'black_mask':
            display = show_mask(frame, detector.detect_black_mask(hsv), 'BLACK MASK')
        elif view_mode == 'gray_mask':
            display = show_mask(frame, detector.detect_gray_mask(hsv), 'GRAY MASK')
        else:
            display = frame

        # FPS 显示
        if frame_count % 30 == 0:
            elapsed = time.time() - fps_timer
            fps = 30 / elapsed if elapsed > 0 else 0
            fps_timer = time.time()
            cv2.putText(display, f"FPS: {fps:.0f}", (display.shape[1] - 100, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.imshow("Vision Test - [h] help", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key == ord('g'):
            view_mode = 'green_mask'
        elif key == ord('b'):
            view_mode = 'black_mask'
        elif key == ord('r'):
            view_mode = 'gray_mask'
        elif key == ord('n'):
            view_mode = 'normal'
        elif key == ord('o'):
            if ocr_future is None:
                ocr_future = ocr_executor.submit(
                    reader.extract_digits, frame, detector=detector
                )
                last_ocr_time = now
        elif key == ord('s'):
            screenshot_count += 1
            filename = f"screenshot_{screenshot_count:03d}.png"
            cv2.imwrite(filename, frame)
            print(f"  💾 已保存: {filename}")
        elif key == ord('h'):
            _print_keys_help()

        frame = None

    ocr_executor.shutdown(wait=True, cancel_futures=True)
    cap.release()
    cv2.destroyAllWindows()
    print(f"\n总共处理 {frame_count} 帧")


def _print_keys_help():
    print("""
╔══════════════════════════════════════╗
║  按键说明                           ║
║  [q] [ESC]  退出                    ║
║  [g]        绿色 HSV mask           ║
║  [b]        黑色边界 HSV mask       ║
║  [r]        灰色 HSV mask           ║
║  [n]        正常画面                ║
║  [o]        手动触发 OCR            ║
║  [s]        保存当前帧截图          ║
║  [h]        显示此帮助              ║
╚══════════════════════════════════════╝
""")


# ── 入口 ────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="视觉识别独立测试")
    parser.add_argument('--camera', type=int, default=None, help='摄像头设备ID (如 0)')
    parser.add_argument('--mvs', type=int, default=None, help='海康 MVS 相机设备ID (如 0)')
    parser.add_argument('--exposure-ms', type=float, default=100.0,
                        help='MVS 手动曝光时间，毫秒 (默认100)')
    parser.add_argument('--gain', type=float, default=8.0,
                        help='MVS 手动增益，dB (默认8)')
    parser.add_argument('--image', type=str, default=None, help='识别单张图片')
    parser.add_argument('--generate', action='store_true', help='生成合成测试图')
    parser.add_argument('--digit', type=int, default=21, help='合成图的区块编号 (默认21)')
    parser.add_argument('--save', action='store_true', help='摄像头模式下允许截图保存')
    args = parser.parse_args()

    print("=" * 50)
    print("  视觉识别独立测试")
    print("=" * 50)

    if args.image:
        run_image(args.image)
    elif args.mvs is not None:
        run_mvs_camera(args.mvs, args.save, args.exposure_ms, args.gain)
    elif args.generate or (args.camera is None and not _has_camera()):
        if not args.generate:
            print("⚠ 未检测到摄像头，使用合成测试图。")
            print("  如有摄像头请用: python3 test_vision.py --camera 0")
            print()
        run_synthetic(args.digit)
    else:
        run_camera(args.camera or 0, args.save)


def _has_camera() -> bool:
    """快速检测是否有摄像头可用。"""
    cap = cv2.VideoCapture(0)
    ok = cap.isOpened()
    cap.release()
    return ok


if __name__ == '__main__':
    main()
