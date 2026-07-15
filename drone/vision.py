"""
vision.py — 视觉识别模块
Section 7: 视觉识别开发

功能:
- 相机管理 (海康USB3.0工业相机, 通过OpenCV VideoCapture)
- 颜色识别 (绿色/灰色/黑色 HSV阈值分割)
- 区块检测 (轮廓查找)
- 数字OCR (Tesseract)
- A标记检测
- 视觉偏移计算
"""

import cv2
import numpy as np
import logging
from itertools import combinations
from typing import Optional, List, Tuple

try:
    from .vision_result import VisionResult
except ImportError:
    from vision_result import VisionResult

logger = logging.getLogger('drone.vision')


# ── 相机管理 ──────────────────────────────────────────────

class Camera:
    """工业相机封装，支持 OpenCV UVC 或海康 MVS SDK。"""

    def __init__(self, device_id: int = 0,
                 width: int = 640, height: int = 480,
                 fps: int = 30, capture_backend: str = 'uvc',
                 exposure_ms: float = 50.0, gain: float = 4.0,
                 preview: bool = False):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.capture_backend = capture_backend
        self.exposure_ms = exposure_ms
        self.gain = gain
        self.preview = preview
        self.cap = None
        self.detector: Optional[BlockDetector] = None
        self.digit_reader: Optional[DigitReader] = None
        self.home_cross_detector: Optional[HomeCrossDetector] = None
        self._sequence = 0

    def open(self) -> bool:
        """打开相机"""
        try:
            if self.capture_backend == 'mvs':
                try:
                    from .mvs_camera import MvsCapture
                except ImportError:
                    from mvs_camera import MvsCapture
                self.cap = MvsCapture(
                    self.device_id,
                    auto_exposure=False,
                    exposure_us=self.exposure_ms * 1000.0,
                    gain=self.gain,
                )
            else:
                self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                logger.error(f"Cannot open camera {self.device_id}")
                self.cap.release()
                self.cap = None
                return False

            if self.capture_backend == 'uvc':
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            # 海康相机可能需要MJPG编码获得30fps
            # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

            actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            identity = ''
            if self.capture_backend == 'mvs':
                identity = f" model={self.cap.model} serial={self.cap.serial}"
            logger.info(
                "Camera opened via %s: %.0fx%.0f%s%s",
                self.capture_backend, actual_w, actual_h,
                f" @ {actual_fps:.0f}fps" if actual_fps > 0 else '', identity,
            )
            return True
        except Exception as e:
            logger.error(f"Camera open error: {e}")
            return False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """读取一帧"""
        if self.cap is None:
            return False, None
        ret, frame = self.cap.read()
        return ret, frame

    def convert_to_hsv(self, frame: np.ndarray) -> np.ndarray:
        """BGR转HSV"""
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def read_result(self) -> Optional[VisionResult]:
        """抓取一帧并在上位机完成工业相机识别。"""
        ret, frame = self.read()
        if not ret or frame is None:
            return None

        green_ratio = 0.0
        if self.detector is not None:
            hsv = self.convert_to_hsv(frame)
            green_ratio = self.detector.calc_green_ratio(hsv)

        digit = None
        if self.digit_reader is not None:
            digit = self.digit_reader.extract_digits(frame, detector=self.detector)

        cross_center = None
        cross_confidence = 0.0
        if self.home_cross_detector is not None:
            cross_center, cross_confidence = self.home_cross_detector.detect(frame)

        self._sequence += 1

        if self.preview:
            display = frame.copy()
            cv2.putText(
                display,
                f"Green: {green_ratio:.1%}  OCR: {digit if digit is not None else '-'}",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )
            if cross_center is not None:
                center = (int(cross_center[0]), int(cross_center[1]))
                cv2.drawMarker(
                    display, center, (0, 0, 255), cv2.MARKER_CROSS, 40, 3,
                )
                cv2.putText(
                    display, f"HOME {cross_confidence:.2f}",
                    (center[0] + 12, center[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                )
            cv2.imshow("Mission Vision - [q] quit [s] save", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                raise KeyboardInterrupt
            if key == ord('s'):
                filename = f"mission_vision_{cv2.getTickCount()}.png"
                cv2.imwrite(filename, frame)
                logger.info("Vision screenshot saved: %s", filename)

        return VisionResult(
            frame=frame,
            green_ratio=green_ratio,
            digit=digit,
            sequence=self._sequence,
            home_cross_center=cross_center,
            home_cross_confidence=cross_confidence,
        )

    def release(self):
        """释放相机"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.preview:
            cv2.destroyAllWindows()
        logger.info("Camera released")


# ── 颜色识别: 区块检测 ────────────────────────────────────

class BlockDetector:
    """
    颜色识别 + 区块检测

    赛题标准颜色:
    - 淡绿色播撒区: RGB(150, 250, 150)
    - 淡灰色非播撒区: RGB(240, 240, 240)
    - 黑色标志线: 0.5cm宽
    """

    def __init__(self,
                 green_lower: List[int] = None,
                 green_upper: List[int] = None,
                 gray_lower: List[int] = None,
                 gray_upper: List[int] = None,
                 black_lower: List[int] = None,
                 black_upper: List[int] = None,
                 min_contour_area: int = 500):
        self.green_lower = np.array(green_lower or [35, 40, 40])
        self.green_upper = np.array(green_upper or [85, 255, 255])
        self.gray_lower = np.array(gray_lower or [0, 0, 180])
        self.gray_upper = np.array(gray_upper or [180, 30, 255])
        self.black_lower = np.array(black_lower or [0, 0, 0])
        self.black_upper = np.array(black_upper or [180, 255, 50])
        self.min_contour_area = min_contour_area

    def detect_green_mask(self, hsv: np.ndarray) -> np.ndarray:
        """返回绿色区域二值mask"""
        return cv2.inRange(hsv, self.green_lower, self.green_upper)

    def detect_gray_mask(self, hsv: np.ndarray) -> np.ndarray:
        """返回灰色区域二值mask"""
        return cv2.inRange(hsv, self.gray_lower, self.gray_upper)

    def detect_black_mask(self, hsv: np.ndarray) -> np.ndarray:
        """返回黑色区域二值mask"""
        return cv2.inRange(hsv, self.black_lower, self.black_upper)

    def calc_green_ratio(self, hsv: np.ndarray) -> float:
        """计算画面中绿色像素占比 (用于边界跳变检测)"""
        mask = self.detect_green_mask(hsv)
        total = mask.size
        if total == 0:
            return 0.0
        return float(np.count_nonzero(mask)) / total

    def calc_gray_ratio(self, hsv: np.ndarray) -> float:
        """计算画面中灰色像素占比"""
        mask = self.detect_gray_mask(hsv)
        total = mask.size
        if total == 0:
            return 0.0
        return float(np.count_nonzero(mask)) / total

    def find_green_blocks(self, hsv: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """
        在画面中找到所有绿色区块轮廓

        Returns:
            [(cx, cy, w, h, area), ...] 各区块的中心和尺寸, 按面积从大到小排
        """
        mask = self.detect_green_mask(hsv)

        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        blocks = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            blocks.append((cx, cy, w, h, area))

        # 按面积从大到小排
        blocks.sort(key=lambda b: b[4], reverse=True)
        return blocks

    def find_largest_block(self, hsv: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """找最大的绿色区块"""
        blocks = self.find_green_blocks(hsv)
        return blocks[0] if blocks else None

    def find_block_boundary_lines(self, hsv: np.ndarray) -> Optional[np.ndarray]:
        """检测黑色边界线 (备选方案)。"""
        mask = self.detect_black_mask(hsv)
        return cv2.HoughLinesP(
            mask, 1, np.pi / 180,
            threshold=50, minLineLength=80, maxLineGap=20,
        )


class HomeCrossDetector:
    """检测起降点黑色十字，并返回其像素中心和几何置信度。

    检测不依赖十字的绝对物理尺寸：先提取低亮度连通域，再验证候选
    中心的水平、垂直四臂以及对角空白。这可排除单线、L形边界和数字。
    """

    def __init__(self, value_max: int = 70, min_area_ratio: float = 0.0005,
                 max_area_ratio: float = 0.25, min_confidence: float = 0.58):
        self.value_max = value_max
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.min_confidence = min_confidence

    def detect(self, frame: np.ndarray) -> Tuple[Optional[Tuple[float, float]], float]:
        if frame is None or frame.size == 0:
            return None, 0.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]),
                           np.array([180, 255, self.value_max]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        frame_area = frame.shape[0] * frame.shape[1]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best_center = None
        best_score = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if not self.min_area_ratio * frame_area <= area <= self.max_area_ratio * frame_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 15 or h < 15 or not 0.45 <= w / h <= 2.2:
                continue

            roi = mask[y:y + h, x:x + w]
            moments = cv2.moments(contour)
            if moments['m00'] <= 0:
                continue
            cx = int(round(moments['m10'] / moments['m00'])) - x
            cy = int(round(moments['m01'] / moments['m00'])) - y
            if not 0 <= cx < w or not 0 <= cy < h:
                continue

            arm = max(4, int(min(w, h) * 0.12))
            thickness = max(2, int(min(w, h) * 0.08))
            horizontal = roi[max(0, cy - thickness):min(h, cy + thickness + 1), :]
            vertical = roi[:, max(0, cx - thickness):min(w, cx + thickness + 1)]
            if horizontal.size == 0 or vertical.size == 0:
                continue

            left = np.mean(horizontal[:, :max(1, cx)] > 0)
            right = np.mean(horizontal[:, min(w - 1, cx + 1):] > 0)
            up = np.mean(vertical[:max(1, cy), :] > 0)
            down = np.mean(vertical[min(h - 1, cy + 1):, :] > 0)

            corner_h = max(1, cy - arm)
            corner_w = max(1, cx - arm)
            corners = [
                roi[:corner_h, :corner_w],
                roi[:corner_h, min(w, cx + arm):],
                roi[min(h, cy + arm):, :corner_w],
                roi[min(h, cy + arm):, min(w, cx + arm):],
            ]
            corner_density = np.mean([
                np.mean(part > 0) if part.size else 0.0 for part in corners
            ])
            arm_score = float(min(left, right, up, down))
            balance = min(w, h) / max(w, h)
            fill = area / max(1.0, float(w * h))
            fill_score = max(0.0, 1.0 - abs(fill - 0.35) / 0.35)
            score = (
                0.55 * arm_score + 0.20 * balance + 0.15 * fill_score
                + 0.10 * max(0.0, 1.0 - corner_density * 3.0)
            )

            if score > best_score:
                best_score = score
                best_center = (float(x + cx), float(y + cy))

        if best_center is None or best_score < self.min_confidence:
            return None, best_score
        return best_center, min(1.0, best_score)

# ── 数字识别 (OCR) ────────────────────────────────────────

def _preprocess_ocr(gray: np.ndarray, bgr: np.ndarray = None) -> list[np.ndarray]:
    """生成多种预处理候选图，供 OCR 逐一尝试。

    返回的候选按推荐优先级排列:
    1. 颜色差分 (R-G): 灰色数字 vs 绿色背景对比度最高 (~90级 vs 灰度~31级)
    2. 颜色差分 (B-G): 备选通道
    3. 灰度线性拉伸
    4. OTSU 二值化 (白底黑字)
    5. 灰度原图 (保底)
    """
    candidates = []

    # ── 颜色差分预处理: 灰色数字(R≈G≈B) vs 绿色背景(R<G, B<G) ──
    if bgr is not None:
        B = bgr[:, :, 0].astype(np.float32)
        G = bgr[:, :, 1].astype(np.float32)
        R = bgr[:, :, 2].astype(np.float32)

        for diff, name in [(R - G, 'R-G'), (B - G, 'B-G')]:
            # 归一化到 0-255, 128 对应差分为 0 (灰色数字)
            diff_norm = np.clip((diff + 128), 0, 255).astype(np.uint8)
            # OTSU 找到数字 (白底黑字)
            _, binary = cv2.threshold(diff_norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            if np.count_nonzero(binary) > binary.size * 0.5:
                binary = cv2.bitwise_not(binary)
            candidates.append(binary)

    # ── 灰度预处理 ──
    # 线性拉伸: 将 [p2, p98] 拉伸到 [0, 255]
    p2, p98 = np.percentile(gray, (2, 98))
    if p98 > p2 + 30:
        stretched = np.clip((gray.astype(np.float32) - p2) * 255.0 / (p98 - p2), 0, 255).astype(np.uint8)
        candidates.append(stretched)

    # OTSU 二值化, 确保白底黑字
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if np.count_nonzero(binary) > binary.size * 0.5:
        binary = cv2.bitwise_not(binary)
    candidates.append(binary)

    # 灰度原图 (保底)
    candidates.append(gray)

    return candidates


class DigitReader:
    """
    识别区块上的数字编号

    区块数字: 25cm高加粗黑体, 灰色(RGB 240,240,240)
    与灰色非播撒区颜色相同, 需要区分
    """

    def __init__(self):
        # 灰色数字的HSV阈值
        self.digit_lower = np.array([0, 0, 180])
        self.digit_upper = np.array([180, 25, 255])
        self._tesseract_available = False
        self._check_tesseract()

    def _check_tesseract(self):
        """检查tesseract是否可用"""
        try:
            import pytesseract
            self._tesseract_available = True
            logger.info("Tesseract OCR available")
        except ImportError:
            logger.warning("pytesseract not installed, OCR disabled")
            self._tesseract_available = False

    def extract_digits(self, frame: np.ndarray,
                        block_roi: Optional[Tuple[int, int, int, int]] = None,
                        detector = None) -> Optional[int]:
        """
        从画面中提取数字

        策略:
        1. 如果有 ROI, 先对 ROI 中心区域做 R-G + OTSU + --psm 8
           (灰色数字 vs 绿色背景对比度最高的方案)
        2. 再尝试全 ROI 的多预处理管线
        3. 无 ROI 时全图多预处理

        Args:
            frame: BGR图像
            block_roi: (x, y, w, h) 可选, 限定识别区域
            detector: BlockDetector 实例, 用于自动查找绿色区块 ROI

        Returns:
            识别到的数字(int) 或 None
        """
        if not self._tesseract_available:
            return None

        try:
            import pytesseract
        except ImportError:
            return None

        if block_roi is not None:
            x, y, w, h = block_roi
            roi = frame[y:y + h, x:x + w]
        elif detector is not None:
            # 自动检测最大的绿色区块作为 ROI
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blocks = detector.find_green_blocks(hsv)
            if blocks:
                cx, cy, bw, bh, _ = blocks[0]
                x, y = cx - bw // 2, cy - bh // 2
                roi = frame[max(0, y):min(frame.shape[0], y + bh),
                             max(0, x):min(frame.shape[1], x + bw)]
            else:
                roi = frame
        else:
            roi = frame

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        whitelist = '-c tessedit_char_whitelist=0123456789'

        # ── 管线 1: 区块内成对数字组件 ──
        if block_roi is not None or detector is not None:
            result = self._try_component_pair_ocr(roi, whitelist)
            if result is not None:
                return result

            # 组件不足时保留中心区域方案，兼容单数字区块。
            result = self._try_center_rg_ocr(roi, whitelist)
            if result is not None:
                return result

        # ── 管线 2: 全 ROI 多预处理 ──
        psm_modes = ['--psm 7', '--psm 8', '--psm 6']
        for preprocessed in _preprocess_ocr(gray, bgr=roi):
            for psm in psm_modes:
                result = self._ocr_single(preprocessed, f'{psm} {whitelist}')
                if result is not None:
                    return result

        return None

    def _try_component_pair_ocr(self, roi: np.ndarray,
                                whitelist: str) -> Optional[int]:
        """隔离同一基线上的两个灰色数字组件后执行 OCR。"""
        blue, green, red = cv2.split(roi)
        green_delta = (
            green.astype(np.int16)
            - np.maximum(red, blue).astype(np.int16)
        )
        brightness = np.max(roi, axis=2)
        mask = np.uint8((green_delta < 20) & (brightness > 70)) * 255

        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        roi_height, roi_width = roi.shape[:2]
        min_area = 0.002 * roi_height * roi_width
        candidates = []
        for label, stat in enumerate(stats[1:], 1):
            x, y, width, height, area = map(int, stat)
            center_x = x + width / 2
            center_y = y + height / 2
            if not 0.08 * roi_height <= height <= 0.25 * roi_height:
                continue
            if not 0.02 * roi_width <= width <= 0.2 * roi_width:
                continue
            if area < min_area:
                continue
            if not 0.2 * roi_width <= center_x <= 0.8 * roi_width:
                continue
            if not 0.3 * roi_height <= center_y <= 0.75 * roi_height:
                continue
            candidates.append((x, y, width, height, area, label))

        if len(candidates) < 2:
            return None

        def pair_score(pair) -> float:
            left, right = sorted(pair)
            max_height = max(left[3], right[3])
            center_delta = abs(
                (left[1] + left[3] / 2) - (right[1] + right[3] / 2)
            ) / max_height
            height_delta = abs(left[3] - right[3]) / max_height
            gap = max(0, right[0] - (left[0] + left[2])) / max_height
            return center_delta + height_delta + 0.2 * gap

        pair = min(combinations(candidates, 2), key=pair_score)
        if pair_score(pair) > 0.35:
            return None

        isolated = np.zeros_like(mask)
        for component in pair:
            isolated[labels == component[5]] = 255

        padding = 20
        left = max(0, min(component[0] for component in pair) - padding)
        top = max(0, min(component[1] for component in pair) - padding)
        right = min(
            roi_width,
            max(component[0] + component[2] for component in pair) + padding,
        )
        bottom = min(
            roi_height,
            max(component[1] + component[3] for component in pair) + padding,
        )
        digit_image = 255 - isolated[top:bottom, left:right]
        digit_image = cv2.resize(
            digit_image, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST
        )

        for psm in ('--psm 8', '--psm 13'):
            result = self._ocr_single(digit_image, f'{psm} {whitelist}')
            if result is not None and 10 <= result <= 28:
                return result
        return None

    def _try_center_rg_ocr(self, frame: np.ndarray, whitelist: str) -> Optional[int]:
        """对画面中心区域做 R-G + OTSU OCR, 专为灰色数字 vs 绿色背景优化。

        画面中心是无人机正下方, 目标区块编号应在此处。
        尝试多种裁剪比例, 从小到大: 太小会截断数字, 太大会引入相邻区块的噪点。
        """
        try:
            import pytesseract
        except ImportError:
            return None

        fh, fw = frame.shape[:2]
        # 从小比例到大比例: 0.10-0.22 覆盖典型数字尺寸
        for fraction in [0.18, 0.20, 0.15, 0.22, 0.12]:
            margin_h = int(fh * (1 - fraction) / 2)
            margin_w = int(fw * (1 - fraction) / 2)
            center = frame[margin_h:fh - margin_h, margin_w:fw - margin_w]
            if center.size == 0:
                continue

            R = center[:, :, 2].astype(np.float32)
            G = center[:, :, 1].astype(np.float32)
            rg_norm = np.clip((R - G + 128), 0, 255).astype(np.uint8)

            # 放大 2x 提高 Tesseract 精度
            rg_big = cv2.resize(rg_norm, None, fx=2, fy=2,
                                 interpolation=cv2.INTER_CUBIC)

            # OTSU 二值化
            _, binary = cv2.threshold(rg_big, 0, 255,
                                       cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            if np.count_nonzero(binary) > binary.size * 0.5:
                binary = cv2.bitwise_not(binary)

            # --psm 8 (single word) 对此场景最准
            for psm in ['--psm 8', '--psm 7']:
                result = self._ocr_single(binary, f'{psm} {whitelist}')
                if result is not None:
                    return result

        return None

    @staticmethod
    def _ocr_single(image: np.ndarray, config: str) -> Optional[int]:
        """对单张预处理图执行 OCR, 返回有效数字或 None。"""
        try:
            import pytesseract
        except ImportError:
            return None

        try:
            text = pytesseract.image_to_string(image, config=config).strip()
            if not text:
                return None
            digits_only = ''.join(c for c in text if c.isdigit())
            if not digits_only:
                return None
            num = int(digits_only)
            if 1 <= num <= 28:
                return num
            last2 = num % 100
            if 1 <= last2 <= 28:
                return last2
        except Exception:
            pass
        return None

    def find_a_marker(self, frame: np.ndarray) -> Optional[Tuple[int, int]]:
        """
        检测"A"标记 (区块21位置)

        A标记: 加粗黑体, 字符高25cm (赛题描述)

        Returns:
            (cx, cy) A标记中心坐标 或 None
        """
        if not self._tesseract_available:
            return None

        try:
            import pytesseract
        except ImportError:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # 找大面积的黑色连通区域
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP,
                                                cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return None

        indexed_contours = sorted(
            enumerate(contours), key=lambda item: cv2.contourArea(item[1]),
            reverse=True
        )
        for index, cnt in indexed_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if h <= 30 or w <= 15:
                continue
            if not 0.6 <= w / h <= 1.2:
                continue
            has_child_hole = hierarchy[0][index][2] >= 0
            is_inner_hole = hierarchy[0][index][3] >= 0
            if not has_child_hole and not is_inner_hole:
                continue

            roi = thresh[y:y + h, x:x + w]
            text = pytesseract.image_to_string(
                roi, config='--psm 10 -c tessedit_char_whitelist=A'
            ).strip()
            if text == 'A':
                return (x + w // 2, y + h // 2)

        return None


# ── 视觉偏移计算 ──────────────────────────────────────────

def calc_offset_to_block(frame_center: Tuple[int, int],
                          block_center: Tuple[int, int],
                          altitude_cm: float,
                          focal_length_px: float = 800.0) -> Tuple[float, float]:
    """
    根据像素偏差计算实际距离偏差(cm)

    相似三角形原理: 实际偏差 = 像素偏差 × (高度 / 焦距)

    Args:
        frame_center: (cx, cy) 画面中心(像素)
        block_center: (bx, by) 区块中心(像素)
        altitude_cm: 飞行高度(cm)
        focal_length_px: 相机焦距(像素), 需标定

    Returns:
        (dx_cm, dy_cm) X和Y方向的实际偏移
    """
    dx_px = block_center[0] - frame_center[0]
    dy_px = block_center[1] - frame_center[1]

    scale = altitude_cm / focal_length_px
    dx_cm = dx_px * scale
    dy_cm = dy_px * scale

    return dx_cm, dy_cm


def pixel_to_world(pixel_x: float, pixel_y: float,
                    altitude_cm: float,
                    focal_length_px: float = 800.0,
                    image_width: int = 640,
                    image_height: int = 480) -> Tuple[float, float]:
    """
    将像素坐标转换为世界坐标 (相对于相机正下方)

    Args:
        pixel_x, pixel_y: 像素坐标
        altitude_cm: 飞行高度
        focal_length_px: 焦距(像素)
        image_width, image_height: 图像尺寸

    Returns:
        (world_x_cm, world_y_cm) 相对相机正下方的世界偏移
    """
    cx_px = image_width / 2
    cy_px = image_height / 2
    scale = altitude_cm / focal_length_px

    wx = (pixel_x - cx_px) * scale
    wy = (pixel_y - cy_px) * scale

    return wx, wy


# ── 调试工具 ──────────────────────────────────────────────

def create_color_tuner_window():
    """
    创建HSV颜色阈值调试窗口 (Section 7.5)
    需要在GUI环境下运行
    """
    def nothing(x):
        pass

    cv2.namedWindow('Threshold Tuner')
    cv2.createTrackbar('H Low', 'Threshold Tuner', 35, 179, nothing)
    cv2.createTrackbar('S Low', 'Threshold Tuner', 40, 255, nothing)
    cv2.createTrackbar('V Low', 'Threshold Tuner', 40, 255, nothing)
    cv2.createTrackbar('H High', 'Threshold Tuner', 85, 179, nothing)
    cv2.createTrackbar('S High', 'Threshold Tuner', 255, 255, nothing)
    cv2.createTrackbar('V High', 'Threshold Tuner', 255, 255, nothing)

    def get_thresholds():
        return {
            'h_low': cv2.getTrackbarPos('H Low', 'Threshold Tuner'),
            's_low': cv2.getTrackbarPos('S Low', 'Threshold Tuner'),
            'v_low': cv2.getTrackbarPos('V Low', 'Threshold Tuner'),
            'h_high': cv2.getTrackbarPos('H High', 'Threshold Tuner'),
            's_high': cv2.getTrackbarPos('S High', 'Threshold Tuner'),
            'v_high': cv2.getTrackbarPos('V High', 'Threshold Tuner'),
        }

    return get_thresholds
