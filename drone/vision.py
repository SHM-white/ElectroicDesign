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
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from itertools import combinations
from queue import Empty, Full, Queue
from threading import Event, Thread
from typing import Optional, List, Tuple

try:
    from .vision_result import VisionResult
except ImportError:
    from vision_result import VisionResult

logger = logging.getLogger('drone.vision')


# ── 实时预览 UI ───────────────────────────────────────────

def draw_mission_overlay(
        frame: np.ndarray, *, state_label: str = '', green_ratio: float = 0.0,
        green_blocks=None, digit: Optional[int] = None,
        ocr_enabled: bool = False, ocr_running: bool = False,
        start_marker: Optional[Tuple[int, int]] = None,
        home_cross: Optional[Tuple[float, float]] = None,
    home_confidence: float = 0.0, processing_fps: float = 0.0) -> np.ndarray:
    """绘制场地测试识别 UI，不修改输入帧。"""
    out = frame.copy()
    height, width = out.shape[:2]
    scale = max(0.55, min(1.0, width / 1440.0))
    line = max(1, int(round(2 * scale)))

    # 绿色区块候选框，最大候选用粗线突出显示。
    for index, block in enumerate((green_blocks or [])[:5]):
        cx, cy, block_w, block_h, area = block
        left = max(0, int(cx - block_w // 2))
        top = max(0, int(cy - block_h // 2))
        right = min(width - 1, int(left + block_w))
        bottom = min(height - 1, int(top + block_h))
        thickness = line + 1 if index == 0 else line
        cv2.rectangle(out, (left, top), (right, bottom), (40, 220, 40), thickness)
        cv2.putText(
            out, f"GREEN {index + 1}  {int(area)}px",
            (left + 5, max(88, top + 22)), cv2.FONT_HERSHEY_SIMPLEX,
            0.48 * scale, (40, 255, 40), line,
        )

    # 画面中心准星，便于人工移动时把编号放到视野中心。
    center = (width // 2, height // 2)
    cv2.drawMarker(
        out, center, (255, 220, 0), cv2.MARKER_CROSS,
        max(20, int(36 * scale)), line,
    )

    if start_marker is not None:
        marker = (int(start_marker[0]), int(start_marker[1]))
        cv2.circle(out, marker, max(16, int(28 * scale)), (0, 255, 255), line + 1)
        cv2.drawMarker(
            out, marker, (0, 255, 255), cv2.MARKER_TILTED_CROSS,
            max(24, int(42 * scale)), line + 1,
        )
        cv2.putText(
            out, "A / START 21", (marker[0] + 18, max(90, marker[1] - 18)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65 * scale, (0, 255, 255), line + 1,
        )

    if home_cross is not None:
        home = (int(home_cross[0]), int(home_cross[1]))
        cv2.drawMarker(
            out, home, (0, 80, 255), cv2.MARKER_CROSS,
            max(30, int(52 * scale)), line + 1,
        )
        cv2.putText(
            out, f"HOME {home_confidence:.2f}",
            (home[0] + 18, max(90, home[1] - 18)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65 * scale, (0, 80, 255), line + 1,
        )

    # 顶部半透明仪表栏。
    panel_height = min(height, max(68, int(82 * scale)))
    panel = out.copy()
    cv2.rectangle(panel, (0, 0), (width, panel_height), (12, 18, 24), -1)
    out = cv2.addWeighted(panel, 0.78, out, 0.22, 0)
    state_text = state_label or 'CAMERA'
    cv2.putText(
        out, f"STATE  {state_text}", (14, int(30 * scale)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.72 * scale, (255, 255, 255), line,
    )
    cv2.putText(
        out, f"GREEN  {green_ratio:.1%}", (14, int(63 * scale)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62 * scale, (40, 255, 40), line,
    )

    if digit is not None:
        ocr_text = f"OCR  {digit}"
        ocr_color = (0, 255, 255)
    elif ocr_running:
        ocr_text = "OCR  SCANNING..."
        ocr_color = (0, 200, 255)
    elif ocr_enabled:
        ocr_text = "OCR  WAITING"
        ocr_color = (170, 210, 255)
    else:
        ocr_text = "OCR  OFF"
        ocr_color = (150, 150, 150)
    (ocr_width, _), _ = cv2.getTextSize(
        ocr_text, cv2.FONT_HERSHEY_SIMPLEX, 0.72 * scale, line,
    )
    cv2.putText(
        out, ocr_text, (max(14, width - ocr_width - 18), int(30 * scale)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.72 * scale, ocr_color, line,
    )
    mode_parts = []
    if start_marker is not None:
        mode_parts.append('START FOUND')
    if home_cross is not None:
        mode_parts.append('HOME FOUND')
    mode_text = ' | '.join(mode_parts) or f"BLOCKS  {len(green_blocks or [])}"
    mode_text += f" | {processing_fps:.1f} FPS"
    (mode_width, _), _ = cv2.getTextSize(
        mode_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale, line,
    )
    cv2.putText(
        out, mode_text, (max(14, width - mode_width - 18), int(63 * scale)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale, (210, 210, 210), line,
    )

    cv2.putText(
        out, "[Q/ESC] quit   [S] save raw frame", (12, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX, 0.48 * scale, (230, 230, 230), line,
    )
    return out


@dataclass(frozen=True)
class _PreviewSnapshot:
    """预览线程独占的帧和绘制元数据快照。"""

    frame: np.ndarray
    state_label: str
    green_ratio: float
    green_blocks: tuple
    digit: Optional[int]
    ocr_enabled: bool
    ocr_running: bool
    start_marker: Optional[Tuple[int, int]]
    home_cross: Optional[Tuple[float, float]]
    home_confidence: float
    processing_fps: float


class _PreviewWorker:
    """异步绘制并显示最新帧；慢速 X11 不向识别线程施加反压。"""

    _WINDOW_NAME = "Mission Vision - [q] quit [s] save"

    def __init__(self, max_width: int):
        self.max_width = max_width
        self.quit_requested = Event()
        self._stop_requested = Event()
        self._snapshots: Queue[_PreviewSnapshot] = Queue(maxsize=1)
        self._thread = Thread(
            target=self._run,
            name='mission-preview',
            daemon=True,
        )
        self._started = False

    def start(self) -> None:
        if not self._started:
            self._started = True
            self._thread.start()

    def submit(self, snapshot: _PreviewSnapshot) -> None:
        """非阻塞覆盖旧帧，确保 UI 永远追踪最新识别结果。"""
        if self._stop_requested.is_set():
            return
        try:
            self._snapshots.put_nowait(snapshot)
            return
        except Full:
            pass
        try:
            self._snapshots.get_nowait()
        except Empty:
            pass
        try:
            self._snapshots.put_nowait(snapshot)
        except Full:
            # 消费线程恰好与覆盖操作竞争时，丢弃本帧即可。
            pass

    def stop(self, timeout: float = 0.5) -> None:
        self._stop_requested.set()
        if self._started:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning(
                    "Preview thread did not stop within %.1fs; X11 may be blocked",
                    timeout,
                )

    def _run(self) -> None:
        try:
            while not self._stop_requested.is_set():
                try:
                    snapshot = self._snapshots.get(timeout=0.1)
                except Empty:
                    continue

                display = draw_mission_overlay(
                    snapshot.frame,
                    state_label=snapshot.state_label,
                    green_ratio=snapshot.green_ratio,
                    green_blocks=snapshot.green_blocks,
                    digit=snapshot.digit,
                    ocr_enabled=snapshot.ocr_enabled,
                    ocr_running=snapshot.ocr_running,
                    start_marker=snapshot.start_marker,
                    home_cross=snapshot.home_cross,
                    home_confidence=snapshot.home_confidence,
                    processing_fps=snapshot.processing_fps,
                )
                if 0 < self.max_width < display.shape[1]:
                    preview_scale = self.max_width / display.shape[1]
                    display = cv2.resize(
                        display,
                        (
                            self.max_width,
                            max(1, int(round(display.shape[0] * preview_scale))),
                        ),
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow(self._WINDOW_NAME, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    self.quit_requested.set()
                elif key == ord('s'):
                    filename = f"mission_vision_{cv2.getTickCount()}.png"
                    cv2.imwrite(filename, snapshot.frame)
                    logger.info("Vision screenshot saved: %s", filename)
        except Exception:
            logger.exception("Preview UI thread failed")
        finally:
            try:
                cv2.destroyAllWindows()
            except Exception:
                logger.exception("Failed to destroy preview windows")


# ── 相机管理 ──────────────────────────────────────────────

class Camera:
    """工业相机封装，支持 OpenCV UVC 或海康 MVS SDK。"""

    def __init__(self, device_id: int = 0,
                 width: int = 640, height: int = 480,
                 fps: int = 30, capture_backend: str = 'uvc',
                 exposure_ms: float = 20.0, gain: float = 16.0,
                 preview: bool = False, preview_max_width: int = 720,
                 ocr_interval_s: float = 0.5):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.capture_backend = capture_backend
        self.exposure_ms = exposure_ms
        self.gain = gain
        self.preview = preview
        self.preview_max_width = preview_max_width
        self.ocr_interval_s = ocr_interval_s
        self.cap = None
        self.detector: Optional[BlockDetector] = None
        self.digit_reader: Optional[DigitReader] = None
        self.home_cross_detector: Optional[HomeCrossDetector] = None
        self._sequence = 0
        self._ocr_enabled = True
        self._start_marker_enabled = False
        self._cross_enabled = False
        self._ocr_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='mission-ocr',
        )
        self._ocr_future: Optional[Future] = None
        self._last_ocr_submit = 0.0
        self._last_digit: Optional[int] = None
        self._last_start_marker: Optional[Tuple[int, int]] = None
        self._expected_digit: Optional[int] = None
        self._digit_candidate: Optional[int] = None
        self._digit_candidate_count = 0
        self._digit_confirmations = 2
        self._state_label = ''
        self._fps_started = time.monotonic()
        self._fps_frames = 0
        self._processing_fps = 0.0
        self._last_fps_log = self._fps_started
        self._preview_worker = (
            _PreviewWorker(preview_max_width) if preview else None
        )

    def set_processing_modes(self, *, ocr: bool, home_cross: bool,
                             start_marker: bool = False,
                             expected_digit: Optional[int] = None,
                             state_label: str = '') -> None:
        """按状态启用重型视觉算法；抓帧、颜色检测和预览始终运行。"""
        if expected_digit != self._expected_digit:
            self._digit_candidate = None
            self._digit_candidate_count = 0
            self._last_digit = None
        self._ocr_enabled = ocr
        self._cross_enabled = home_cross
        self._start_marker_enabled = start_marker
        self._expected_digit = expected_digit
        self._state_label = state_label
        if not start_marker:
            self._last_start_marker = None

    def _recognize_start_or_digit(self, frame: np.ndarray):
        """按当前任务阶段执行A标记检测或数字OCR。"""
        if self._start_marker_enabled:
            marker = self.digit_reader.find_a_marker(frame)
            if marker is not None:
                return 21, marker
        digit = self.digit_reader.extract_digits(
            frame,
            detector=self.detector,
            expected_digit=self._expected_digit,
        )
        marker = None
        return digit, marker

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
                    timeout_ms=100,
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
        if (self._preview_worker is not None
            and self._preview_worker.quit_requested.is_set()):
            raise KeyboardInterrupt

        ret, frame = self.read()
        if not ret or frame is None:
            return None

        green_ratio = 0.0
        hsv = None
        if self.detector is not None:
            hsv = self.convert_to_hsv(frame)
            green_ratio = self.detector.calc_green_ratio(hsv)

        # OCR 在单独线程中运行。这里只消费一次新结果，绝不等待；同一个
        # OCR 结果不会被状态机重复计为多帧确认。
        digit = None
        start_marker = None
        if self._ocr_future is not None and self._ocr_future.done():
            try:
                candidate, candidate_marker = self._ocr_future.result()
                if candidate is None:
                    self._digit_candidate = None
                    self._digit_candidate_count = 0
                elif candidate == self._digit_candidate:
                    self._digit_candidate_count += 1
                else:
                    self._digit_candidate = candidate
                    self._digit_candidate_count = 1

                if self._digit_candidate_count >= self._digit_confirmations:
                    digit = candidate
                    start_marker = candidate_marker
                    self._last_digit = digit
                    self._last_start_marker = start_marker

                if digit is not None or start_marker is not None:
                    logger.info(
                        "Vision recognized: digit=%s, start_marker=%s",
                        digit, start_marker is not None,
                    )
                else:
                    logger.debug(
                        "Vision OCR candidate: digit=%s, confirmations=%d/%d",
                        candidate, self._digit_candidate_count,
                        self._digit_confirmations,
                    )
            except Exception:
                logger.exception("Asynchronous OCR failed")
            self._ocr_future = None
            # 从完成时开始冷却，避免耗时任务结束后立即再次满负荷运行。
            self._last_ocr_submit = time.monotonic()

        cross_center = None
        cross_confidence = 0.0
        if self._cross_enabled and self.home_cross_detector is not None:
            cross_center, cross_confidence = self.home_cross_detector.detect(frame)

        self._sequence += 1
        self._fps_frames += 1
        now = time.monotonic()
        fps_elapsed = now - self._fps_started
        if fps_elapsed >= 1.0:
            self._processing_fps = self._fps_frames / fps_elapsed
            self._fps_started = now
            self._fps_frames = 0
        if now - self._last_fps_log >= 5.0:
            logger.info("Vision processing rate: %.1f fps", self._processing_fps)
            self._last_fps_log = now

        if self.preview:
            green_blocks = []
            # 区块轮廓只在作业导航阶段有诊断意义；寻找A和返航时不再
            # 重复执行第二次绿色形态学/轮廓扫描。
            if (self._state_label == 'NAVIGATE'
                    and self.detector is not None and hsv is not None):
                green_blocks = self.detector.find_green_blocks(hsv)
            self._preview_worker.start()
            self._preview_worker.submit(_PreviewSnapshot(
                frame=frame.copy(),
                state_label=self._state_label,
                green_ratio=green_ratio,
                green_blocks=tuple(green_blocks),
                digit=self._last_digit,
                ocr_enabled=self._ocr_enabled,
                ocr_running=(
                    self._ocr_future is not None
                    and not self._ocr_future.done()
                ),
                start_marker=(
                    self._last_start_marker
                    if self._start_marker_enabled else None
                ),
                home_cross=cross_center,
                home_confidence=cross_confidence,
                processing_fps=self._processing_fps,
            ))

        # UI 已在独立线程中绘制和刷新；重型 OCR 也独立执行，主线程只
        # 负责抓帧、轻量识别和状态机所需结果。
        now = time.monotonic()
        if (self._ocr_enabled and self.digit_reader is not None
                and self._ocr_future is None
                and now - self._last_ocr_submit >= self.ocr_interval_s):
            self._ocr_future = self._ocr_executor.submit(
                self._recognize_start_or_digit,
                frame.copy(),
            )
            self._last_ocr_submit = now

        return VisionResult(
            frame=frame,
            green_ratio=green_ratio,
            digit=digit,
            sequence=self._sequence,
            home_cross_center=cross_center,
            home_cross_confidence=cross_confidence,
            start_marker_center=start_marker,
        )

    def release(self):
        """释放相机"""
        if self._preview_worker is not None:
            self._preview_worker.stop()
        self._ocr_executor.shutdown(wait=True, cancel_futures=True)
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
        """返回绿色区域二值mask，并兼容低照度下饱和度偏低的实拍画面。"""
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        if np.count_nonzero(mask) >= mask.size * 0.05:
            return mask
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        blue, green, red = cv2.split(bgr)
        green_i16 = green.astype(np.int16)
        low_light_green = np.uint8(
            (green_i16 >= np.maximum(red, blue).astype(np.int16) + 3)
            & (green_i16 >= 15)
        ) * 255
        return cv2.bitwise_or(mask, low_light_green)

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
    """以相对暗度和四臂几何检测起降十字。

    现场图整体亮度会随曝光显著变化，因此不用固定黑色阈值。检测器在
    多个暗像素分位数上寻找低凸度连通域，以行列投影的交点代替易偏移
    的轮廓质心，再验证交点四侧均有连续暗色笔画。
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

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_area = frame.shape[0] * frame.shape[1]
        best_center = None
        best_score = 0.0
        thresholds = sorted({
            int(min(self.value_max, np.percentile(gray, percentile)))
            for percentile in (3, 5, 8, 10)
        })
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        for threshold in thresholds:
            mask = np.uint8(gray <= threshold) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
            )
            for contour in contours:
                area = cv2.contourArea(contour)
                if not self.min_area_ratio * frame_area <= area <= \
                        self.max_area_ratio * frame_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if min(w, h) < 30 or not 0.35 <= w / h <= 2.85:
                    continue

                hull_area = cv2.contourArea(cv2.convexHull(contour))
                solidity = area / max(1.0, hull_area)
                fill = area / max(1.0, float(w * h))
                if not 0.10 <= fill <= 0.72 or solidity > 0.86:
                    continue

                component = np.zeros_like(mask)
                cv2.drawContours(component, [contour], -1, 255, -1)
                roi = component[y:y + h, x:x + w]
                row_density = np.mean(roi > 0, axis=1)
                col_density = np.mean(roi > 0, axis=0)

                def plateau_center(values: np.ndarray) -> int:
                    lo = int(len(values) * 0.15)
                    hi = max(lo + 1, int(len(values) * 0.85))
                    middle = values[lo:hi]
                    peak = float(np.max(middle))
                    indexes = np.flatnonzero(middle >= peak * 0.95)
                    return lo + int(round(float(np.mean(indexes))))

                cy = plateau_center(row_density)
                cx = plateau_center(col_density)
                thickness = max(2, int(min(w, h) * 0.04))
                horizontal = roi[
                    max(0, cy - thickness):min(h, cy + thickness + 1), :
                ]
                vertical = roi[
                    :, max(0, cx - thickness):min(w, cx + thickness + 1)
                ]
                arms = (
                    np.mean(horizontal[:, :max(1, cx)] > 0),
                    np.mean(horizontal[:, min(w - 1, cx + 1):] > 0),
                    np.mean(vertical[:max(1, cy), :] > 0),
                    np.mean(vertical[min(h - 1, cy + 1):, :] > 0),
                )
                arm_min = float(min(arms))
                if arm_min < 0.25:
                    continue
                balance = min(w, h) / max(w, h)
                fill_score = max(0.0, 1.0 - abs(fill - 0.30) / 0.50)
                score = (
                    0.35 * arm_min + 0.25 * float(np.mean(arms))
                    + 0.15 * balance + 0.15 * (1.0 - solidity)
                    + 0.10 * fill_score
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
    5. CLAHE + 自适应阈值 (低照度/局部阴影)
    6. 灰度原图 (保底)
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

    # OpenMV 参考方案依赖现场阈值；工业相机光照不均时改用局部阈值，
    # 并以小尺度闭运算连接因短曝光而断开的粗体数字笔画。
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    adaptive = cv2.adaptiveThreshold(
        clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 31, 7,
    )
    if np.count_nonzero(adaptive) > adaptive.size * 0.5:
        adaptive = cv2.bitwise_not(adaptive)
    adaptive = cv2.morphologyEx(
        adaptive, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8),
    )
    candidates.append(adaptive)

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
                        detector = None,
                        expected_digit: Optional[int] = None) -> Optional[int]:
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
            expected_digit: 状态机当前目标编号；提供时拒绝其他 OCR 结果

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
                # 没找到绿色区块时避免对1440x1080整帧执行大量OCR回退。
                roi = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
        else:
            roi = frame

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        whitelist = '-c tessedit_char_whitelist=0123456789'

        def is_expected(result: Optional[int]) -> bool:
            return result is not None and (
                expected_digit is None or result == expected_digit
            )

        # ── 管线 1: 区块内成对数字组件 ──
        if block_roi is not None or detector is not None:
            result = self._try_component_pair_ocr(
                roi, whitelist, expected_digit=expected_digit,
            )
            if is_expected(result):
                return result

            # 组件不足时保留中心区域方案，兼容单数字区块。
            result = self._try_center_rg_ocr(roi, whitelist)
            if is_expected(result):
                return result

        # ── 管线 2: 全 ROI 多预处理 ──
        psm_modes = ['--psm 7', '--psm 8']
        # 颜色差分失败后继续尝试灰度拉伸、全局阈值和局部阈值；这对
        # 白平衡漂移、阴影和低照度场景比只使用 R-G/B-G 更稳健。
        # 原始灰度图仍不送入 Tesseract，避免无效调用过多。
        for preprocessed in _preprocess_ocr(gray, bgr=roi)[:-1]:
            for psm in psm_modes:
                result = self._ocr_single(preprocessed, f'{psm} {whitelist}')
                if is_expected(result):
                    return result

        return None

    def _try_component_pair_ocr(self, roi: np.ndarray,
                                whitelist: str,
                                expected_digit: Optional[int] = None
                                ) -> Optional[int]:
        """隔离同一基线上的两个灰色数字组件后执行 OCR。

        亮图沿用原阈值；暗图按自身亮度分位数和 0~3 级颜色差生成多张
        掩膜。低照度下绝对亮度只有 20~60，固定 ``brightness > 70`` 会
        完全丢失数字，因此必须使用相对亮度，但仍由几何条件排除噪点。
        """
        blue, green, red = cv2.split(roi)
        green_delta = (
            green.astype(np.int16)
            - np.maximum(red, blue).astype(np.int16)
        )
        brightness = np.max(roi, axis=2)
        roi_height, roi_width = roi.shape[:2]
        masks = [
            np.uint8((green_delta < 20) & (brightness > 70)) * 255,
        ]
        repair_kernel = np.ones((3, 3), np.uint8)
        masks[0] = cv2.morphologyEx(
            masks[0], cv2.MORPH_CLOSE, repair_kernel,
        )
        if float(np.percentile(brightness, 90)) < 110:
            brightness_floor = float(np.percentile(brightness, 35))
            kernel = np.ones((2, 2), np.uint8)
            for delta_limit in (0, 1, 2, 3):
                adaptive = np.uint8(
                    (green_delta <= delta_limit)
                    & (brightness >= brightness_floor)
                ) * 255
                adaptive = cv2.morphologyEx(
                    adaptive, cv2.MORPH_CLOSE, repair_kernel,
                )
                masks.append(cv2.morphologyEx(
                    adaptive, cv2.MORPH_OPEN, kernel,
                ))

        fallback = None
        for mask_index, mask in enumerate(masks):
            result = self._ocr_component_mask(
                mask, whitelist, expected_digit=expected_digit,
                strict_geometry=mask_index == 0,
            )
            if result is None:
                continue
            if expected_digit is not None and result == expected_digit:
                return result
            if fallback is None:
                fallback = result
        return fallback

    def _ocr_component_mask(self, mask: np.ndarray, whitelist: str,
                            expected_digit: Optional[int] = None,
                            strict_geometry: bool = False,
                            ) -> Optional[int]:
        """从一张候选掩膜选择最佳数字对并调用现有 Tesseract。"""
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        roi_height, roi_width = mask.shape[:2]
        min_area_ratio = 0.002 if strict_geometry else 0.0005
        min_area = min_area_ratio * roi_height * roi_width
        candidates = []
        for label, stat in enumerate(stats[1:], 1):
            x, y, width, height, area = map(int, stat)
            center_x = x + width / 2
            center_y = y + height / 2
            min_height = 0.08 if strict_geometry else 0.06
            max_height = 0.25 if strict_geometry else 0.32
            min_width = 0.02 if strict_geometry else 0.012
            max_width = 0.20 if strict_geometry else 0.22
            min_center_x = 0.20 if strict_geometry else 0.10
            max_center_x = 0.80 if strict_geometry else 0.90
            min_center_y = 0.30 if strict_geometry else 0.15
            max_center_y = 0.75 if strict_geometry else 0.85
            if not min_height * roi_height <= height <= max_height * roi_height:
                continue
            if not min_width * roi_width <= width <= max_width * roi_width:
                continue
            if area < min_area:
                continue
            if not strict_geometry and not 0.1 <= width / height <= 1.0:
                continue
            if not min_center_x * roi_width <= center_x <= max_center_x * roi_width:
                continue
            if not min_center_y * roi_height <= center_y <= max_center_y * roi_height:
                continue
            candidates.append((x, y, width, height, area, label))

        # 参考多截面算法的离群剔除思想：优先保留面积大且靠近区块中心的
        # 组件，并限制组合规模。低照度噪声可能产生数十个组件，若直接对
        # 全部组件两两组合会造成 O(n²) 排序和大量无意义 OCR。
        def component_score(component) -> float:
            x, y, width, height, area, _ = component
            center_x = x + width / 2
            center_y = y + height / 2
            center_distance = (
                abs(center_x - roi_width / 2) / max(1.0, roi_width)
                + abs(center_y - roi_height / 2) / max(1.0, roi_height)
            )
            return float(area) / max(1.0, width * height) - center_distance

        candidates = sorted(
            candidates, key=component_score, reverse=True,
        )[:12]

        def pair_score(pair) -> float:
            left, right = sorted(pair)
            max_height = max(left[3], right[3])
            center_delta = abs(
                (left[1] + left[3] / 2) - (right[1] + right[3] / 2)
            ) / max_height
            height_delta = abs(left[3] - right[3]) / max_height
            gap = max(0, right[0] - (left[0] + left[2])) / max_height
            return center_delta + height_delta + 0.15 * gap

        ranked_pairs = sorted(combinations(candidates, 2), key=pair_score)
        fallback = None
        for pair in ranked_pairs[:4]:
            max_pair_score = 0.35 if strict_geometry else 0.45
            if pair_score(pair) > max_pair_score:
                break
            isolated = np.zeros_like(mask)
            for component in pair:
                isolated[labels == component[5]] = 255

            max_height = max(component[3] for component in pair)
            padding = 20 if strict_geometry else max(8, int(max_height * 0.05))
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
            digit_image = cv2.resize(
                255 - isolated[top:bottom, left:right], None,
                fx=2 if strict_geometry else 3,
                fy=2 if strict_geometry else 3,
                interpolation=cv2.INTER_NEAREST,
            )
            for psm in ('--psm 8', '--psm 13'):
                result = self._ocr_single(
                    digit_image, f'{psm} {whitelist}',
                )
                if result is None or not 10 <= result <= 28:
                    continue
                if expected_digit is not None and result == expected_digit:
                    return result
                if fallback is None:
                    fallback = result

        # 原实现只处理成对组件，1..9 只能依赖中心裁剪。借鉴参考代码的
        # 几何一致性筛选：仅在掩膜中恰有一个可信组件时尝试单字符 OCR，
        # 防止把两位数字的碎片分别误识别成单数字。
        if len(candidates) == 1 and (
                expected_digit is None or 1 <= expected_digit <= 9):
            x, y, width, height, _, label = candidates[0]
            padding = max(6, int(height * 0.12))
            left = max(0, x - padding)
            top = max(0, y - padding)
            right = min(roi_width, x + width + padding)
            bottom = min(roi_height, y + height + padding)
            isolated = np.zeros_like(mask)
            isolated[labels == label] = 255
            digit_image = cv2.resize(
                255 - isolated[top:bottom, left:right], None,
                fx=3, fy=3, interpolation=cv2.INTER_NEAREST,
            )
            result = self._ocr_single(
                digit_image, f'--psm 10 {whitelist}',
            )
            if result is not None and 1 <= result <= 9:
                if expected_digit is None or result == expected_digit:
                    return result
        return fallback

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
        fractions = [0.18, 0.20, 0.15, 0.22, 0.12]
        if float(np.percentile(frame, 90)) < 110:
            # 暗图中数字边缘更弱，扩大中心区域可避免裁掉第二位数字。
            fractions.extend([0.26, 0.30, 0.35])
        for fraction in fractions:
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
        # 固定阈值兼容明亮旧样本；图像分位数阈值兼容短曝光现场图。
        p60 = float(np.percentile(gray, 60))
        thresholds = [100]
        adaptive_value = int(np.clip(p60, 25, 90))
        if abs(adaptive_value - 100) > 5:
            thresholds.append(adaptive_value)

        frame_area = frame.shape[0] * frame.shape[1]
        for threshold in thresholds:
            _, binary = cv2.threshold(
                gray, threshold, 255, cv2.THRESH_BINARY_INV,
            )
            contours, hierarchy = cv2.findContours(
                binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE,
            )
            if hierarchy is None:
                continue

            indexed_contours = sorted(
                enumerate(contours),
                key=lambda item: cv2.contourArea(item[1]),
                reverse=True,
            )
            for index, cnt in indexed_contours[:100]:
                area = cv2.contourArea(cnt)
                x, y, w, h = cv2.boundingRect(cnt)
                if not 0.0002 * frame_area <= area <= 0.08 * frame_area:
                    continue
                if h <= 30 or w <= 15:
                    continue
                if not 0.45 <= w / h <= 1.2:
                    continue
                has_child_hole = hierarchy[0][index][2] >= 0
                is_inner_hole = hierarchy[0][index][3] >= 0
                if not has_child_hole and not is_inner_hole:
                    continue

                padding = max(3, int(min(w, h) * 0.06))
                roi = binary[
                    max(0, y - padding):min(binary.shape[0], y + h + padding),
                    max(0, x - padding):min(binary.shape[1], x + w + padding),
                ]
                text = pytesseract.image_to_string(
                    roi, config='--psm 10 -c tessedit_char_whitelist=A'
                ).strip().upper()
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
