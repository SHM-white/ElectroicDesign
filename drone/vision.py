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
from typing import Optional, List, Tuple

logger = logging.getLogger('drone.vision')


# ── 相机管理 ──────────────────────────────────────────────

class Camera:
    """工业相机封装 (通过OpenCV UVC协议)"""

    def __init__(self, device_id: int = 0,
                 width: int = 640, height: int = 480,
                 fps: int = 30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.detector: Optional[BlockDetector] = None
        self.digit_reader: Optional[DigitReader] = None

    def open(self) -> bool:
        """打开相机"""
        try:
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                logger.error(f"Cannot open camera {self.device_id}")
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            # 海康相机可能需要MJPG编码获得30fps
            # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

            actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            logger.info(f"Camera opened: {actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.0f}fps")
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

    def release(self):
        """释放相机"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
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
        """
        检测黑色边界线 (备选方案)

        Returns:
            HoughLinesP结果 或 None
        """
        mask = self.detect_black_mask(hsv)
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180,
                                 threshold=50, minLineLength=80, maxLineGap=20)
        return lines


# ── 数字识别 (OCR) ────────────────────────────────────────

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
                        block_roi: Optional[Tuple[int, int, int, int]] = None) -> Optional[int]:
        """
        从画面中提取数字

        Args:
            frame: BGR图像
            block_roi: (x, y, w, h) 可选, 限定识别区域

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
        else:
            roi = frame

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # 二值化: 数字是亮的灰色, 背景是暗的绿色
        # 反转: 让数字变成白字黑底
        _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        config = '--psm 6 -c tessedit_char_whitelist=0123456789'
        try:
            text = pytesseract.image_to_string(thresh, config=config).strip()
            if text and text.isdigit():
                return int(text)
        except Exception as e:
            logger.warning(f"OCR error: {e}")

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
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            # 字高>字宽的竖长形状
            if h > 30 and w > 15 and h / w > 1.5:
                roi = thresh[y:y + h, x:x + w]
                try:
                    text = pytesseract.image_to_string(
                        roi, config='--psm 10 -c tessedit_char_whitelist=A'
                    ).strip()
                    if text == 'A':
                        return (x + w // 2, y + h // 2)
                except Exception:
                    pass

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
