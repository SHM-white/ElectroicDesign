"""Field-test node — standalone target-tracking validation without FCU.

Manually hold the drone over the AprilTag target; this node:
  1. Detects AprilTag corners and estimates 6-DOF pose (PnP)
  2. Smooths / predicts with a constant-velocity Kalman filter
  3. Computes visual-servo velocity commands (simulated, not sent)
  4. Tracks cumulative displacement from the initial lock-on position
  5. Displays side-by-side narrow + wide camera views with HUD
  6. Logs odometry + control data at a configurable interval

All display I/O runs in a daemon thread so slow X11 (remote SSH)
never blocks the perception / control loop.
"""

from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass, field
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from typing import Final

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ed_uav_perception.apriltag_detector import AprilTagDetector, TAG_SIZE_M, TAG_FAMILY
from ed_uav_perception.kalman_tracker import KalmanTracker
from ed_uav_perception.target_pose import estimate_target_pose
from ed_uav_perception.target_types import (
    CameraModel,
    CorrespondenceSet,
    FrameContext,
    MotionContext,
    PoseEstimate,
    PoseLimits,
)
from ed_uav_perception.visual_servo import (
    VelocityCommand,
    VisualServoController,
)

logger = logging.getLogger("ed_uav_mission.field_test")

# ── Display constants ──────────────────────────────────────────────────────
_WINDOW_NAME: Final = "Field Test — [Q/ESC] quit [R] reset origin [S] screenshot"

_COLOR_WHITE: Final = (255, 255, 255)
_COLOR_GREEN: Final = (60, 220, 60)
_COLOR_YELLOW: Final = (0, 220, 255)
_COLOR_RED: Final = (60, 60, 240)
_COLOR_CYAN: Final = (255, 200, 0)
_COLOR_MAGENTA: Final = (220, 60, 220)
_COLOR_PANEL_BG: Final = (12, 18, 24)
_COLOR_TAG_BORDER: Final = (0, 255, 0)
_COLOR_TAG_CORNERS: Final = (0, 200, 255)
_COLOR_KALMAN_PRED: Final = (255, 0, 255)
_COLOR_ORIGIN_ARROW: Final = (0, 180, 255)
_COLOR_SERVO_ARROW: Final = (0, 255, 200)
_COLOR_ODOM: Final = (100, 200, 255)


# ── Per-camera detection result ───────────────────────────────────────────

@dataclass
class _CameraDetection:
    frame: np.ndarray | None = None
    tag_detected: bool = False
    tag_corners_px: tuple[tuple[int, int], ...] | None = None
    tag_center_px: tuple[int, int] | None = None
    raw_pose_m: tuple[float, float, float] | None = None
    raw_reproj_rms_px: float = float("inf")


# ── Combined snapshot for display thread ──────────────────────────────────

@dataclass
class _FieldTestSnapshot:
    narrow: _CameraDetection
    wide: _CameraDetection
    # Kalman filtered (shared across cameras)
    kf_position_m: tuple[float, float, float] | None = None
    kf_velocity_m_s: tuple[float, float, float] | None = None
    kf_uncertainty_m: float = float("inf")
    kf_fresh: bool = False
    # Displacement from origin
    origin_set: bool = False
    displacement_m: tuple[float, float, float] | None = None
    # Odometry
    odom_position_m: tuple[float, float, float] | None = None
    odom_yaw_rad: float | None = None
    odom_linear_vel_m_s: tuple[float, float, float] | None = None
    odom_angular_vel_rad_s: float | None = None
    odom_topic: str = ""
    # Visual servo (simulated)
    servo_phase: str = "IDLE"
    servo_vx: float = 0.0
    servo_vy: float = 0.0
    servo_vz: float = 0.0
    servo_converged: bool = False
    # Performance
    fps: float = 0.0
    detection_latency_ms: float = 0.0


# ── Display worker ────────────────────────────────────────────────────────

class _FieldTestDisplayWorker:

    def __init__(self, max_width: int, headless_log_interval: float):
        self.max_width = max_width
        self.headless_log_interval = headless_log_interval
        self.quit_requested = Event()
        self.reset_origin_requested = Event()
        self._stop = Event()
        self._queue: Queue[_FieldTestSnapshot] = Queue(maxsize=1)
        self._thread = Thread(target=self._run, name="field-test-display", daemon=True)
        self._started = False
        self._headless = not os.environ.get("DISPLAY")
        self._last_headless_log: float = 0.0

    @property
    def is_headless(self) -> bool:
        return self._headless

    def start(self) -> None:
        if not self._started:
            self._started = True
            if self._headless:
                self._thread.start()

    def submit(self, snapshot: _FieldTestSnapshot) -> None:
        if self._stop.is_set():
            return
        try:
            self._queue.put_nowait(snapshot)
            return
        except Full:
            pass
        try:
            self._queue.get_nowait()
        except Empty:
            pass
        try:
            self._queue.put_nowait(snapshot)
        except Full:
            pass

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._started and self._thread.is_alive():
            self._thread.join(timeout=timeout)

    def _run(self) -> None:
        if self._headless:
            self._run_headless()
        else:
            self._run_display()

    def _run_headless(self) -> None:
        while not self._stop.is_set():
            try:
                snap = self._queue.get(timeout=0.5)
            except Empty:
                continue
            now = time.monotonic()
            if now - self._last_headless_log >= self.headless_log_interval:
                self._last_headless_log = now
                self._log_snapshot(snap)

    def _log_snapshot(self, snap: _FieldTestSnapshot) -> None:
        n_tag = "YES" if snap.narrow.tag_detected else "NO"
        w_tag = "YES" if snap.wide.tag_detected else "NO"
        kf_str = (
            f"({snap.kf_position_m[0]:+.3f}, {snap.kf_position_m[1]:+.3f}, {snap.kf_position_m[2]:.3f})"
            if snap.kf_position_m else "NONE"
        )
        logger.info(
            "[FIELD_TEST] narrow=%s wide=%s kf=%s servo=%s fps=%.1f",
            n_tag, w_tag, kf_str, snap.servo_phase, snap.fps,
        )

    def _run_display(self) -> None:
        try:
            os.environ.setdefault("GDK_SCALE", "1")
            os.environ.setdefault("GTK_CSD", "0")
            cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.startWindowThread()
            cv2.resizeWindow(_WINDOW_NAME, 1800, 720)
        except Exception:
            self._headless = True
            self._run_headless()
            return

        while not self._stop.is_set():
            try:
                snap = self._queue.get(timeout=0.1)
            except Empty:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    self.quit_requested.set()
                    break
                continue

            display = _render_side_by_side(snap)

            if 0 < self.max_width < display.shape[1]:
                scale = self.max_width / display.shape[1]
                display = cv2.resize(
                    display,
                    (self.max_width, max(1, int(round(display.shape[0] * scale)))),
                    interpolation=cv2.INTER_AREA,
                )

            cv2.imshow(_WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.quit_requested.set()
                break
            elif key == ord("r"):
                self.reset_origin_requested.set()
            elif key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = f"/tmp/field_test_{ts}.png"
                cv2.imwrite(path, display)

        try:
            cv2.destroyWindow(_WINDOW_NAME)
        except Exception:
            pass


# ── HUD rendering ─────────────────────────────────────────────────────────

def _render_camera_hud(
    frame: np.ndarray | None,
    det: _CameraDetection,
    role: str,
    snap: _FieldTestSnapshot,
) -> np.ndarray:
    """Render one camera's view with detection overlay and HUD."""
    if frame is None:
        out = np.zeros((720, 480, 3), dtype=np.uint8)
        cv2.putText(out, f"{role.upper()} NO SIGNAL", (80, 360),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, _COLOR_RED, 2, cv2.LINE_AA)
        return out

    out = frame.copy()
    h, w = out.shape[:2]
    sc = max(0.8, min(1.8, min(w, h) / 400.0))
    ln = max(2, int(round(3 * sc)))
    font = cv2.FONT_HERSHEY_SIMPLEX

    # ── Camera label ──
    label = f"[{'WIDE' if role == 'wide' else 'NARROW'}]"
    cv2.putText(out, label, (12, int(35 * sc)),
                font, 0.8 * sc, _COLOR_CYAN, ln + 1, cv2.LINE_AA)

    # ── AprilTag border ──
    if det.tag_detected and det.tag_corners_px is not None:
        pts = np.array(det.tag_corners_px, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, _COLOR_TAG_BORDER, ln + 1, cv2.LINE_AA)
        for corner in det.tag_corners_px:
            cv2.circle(out, corner, max(4, int(6 * sc)), _COLOR_TAG_CORNERS, -1, cv2.LINE_AA)
        if det.tag_center_px is not None:
            cv2.drawMarker(out, det.tag_center_px, _COLOR_TAG_CORNERS,
                           cv2.MARKER_CROSS, max(14, int(20 * sc)), ln)

    # ── Aircraft center (image center) + distance to tag center ──
    img_cx, img_cy = w // 2, h // 2
    cv2.drawMarker(out, (img_cx, img_cy), _COLOR_YELLOW,
                   cv2.MARKER_CROSS, max(20, int(28 * sc)), ln + 1)
    cv2.putText(out, "AC", (img_cx + 14, img_cy - 14),
                font, 0.5 * sc, _COLOR_YELLOW, ln, cv2.LINE_AA)
    if det.tag_detected and det.tag_center_px is not None:
        cv2.line(out, (img_cx, img_cy), det.tag_center_px,
                 _COLOR_YELLOW, max(1, int(2 * sc)), cv2.LINE_AA)

    # ── Kalman crosshair ──
    if snap.kf_position_m is not None and det.tag_center_px is not None and det.raw_pose_m is not None:
        cx, cy = det.tag_center_px
        kf_dx = snap.kf_position_m[0] - det.raw_pose_m[0]
        kf_dy = snap.kf_position_m[1] - det.raw_pose_m[1]
        arrow_px = max(40, int(100 * sc))
        kx = int(cx + kf_dx * arrow_px / max(snap.kf_position_m[2], 0.1))
        ky = int(cy + kf_dy * arrow_px / max(snap.kf_position_m[2], 0.1))
        kx = max(0, min(w - 1, kx))
        ky = max(0, min(h - 1, ky))
        cv2.circle(out, (kx, ky), max(8, int(12 * sc)), _COLOR_KALMAN_PRED, ln + 1, cv2.LINE_AA)
        cv2.putText(out, "KF", (kx + 12, max(20, ky - 12)),
                    font, 0.5 * sc, _COLOR_KALMAN_PRED, ln, cv2.LINE_AA)

    # ── Displacement arrow ──
    if snap.displacement_m is not None and snap.origin_set:
        img_cx, img_cy = w // 2, h // 2
        disp_scale = max(50, int(120 * sc))
        # displacement_m 是相机系 (x=右, y=下), 需映射到旋转后的显示帧:
        #   narrow(顺时针): 右→下, 下→左  ⇒ (dx,dy) = (-disp_y, +disp_x)
        #   wide(逆时针):   右→上, 下→右  ⇒ (dx,dy) = (+disp_y, -disp_x)
        if role == "narrow":
            dx_px = max(0, min(w - 1, int(img_cx - snap.displacement_m[1] * disp_scale)))
            dy_px = max(0, min(h - 1, int(img_cy + snap.displacement_m[0] * disp_scale)))
        else:
            dx_px = max(0, min(w - 1, int(img_cx + snap.displacement_m[1] * disp_scale)))
            dy_px = max(0, min(h - 1, int(img_cy - snap.displacement_m[0] * disp_scale)))
        cv2.arrowedLine(out, (img_cx, img_cy), (dx_px, dy_px),
                        _COLOR_ORIGIN_ARROW, max(2, int(3 * sc)), cv2.LINE_AA, tipLength=0.12)

    # ── Servo/move arrow ──
    # 指向"把 tag 移回画面中心"的方向（= 需要移动的方向）。
    # 优先用 PnP 相机系坐标 (x_right, y_down)：畸变校正、米制、含深度。
    # 移动方向 = (-x, -y)（把 tag 拉回光轴中心），再映射到旋转后的显示帧。
    # 实测：真实相机安装下 PnP 的 x/y 与显示方向整体相反，取反修正。
    #   narrow(顺时针): (du',dv') = (-y, x)；wide(逆时针): (du',dv') = (y, -x)
    # 不用 servo 的 vx/vy：下视相机下 vx 是高度误差（恒正），会误导箭头方向。
    img_cx, img_cy = w // 2, h // 2
    if det.raw_pose_m is not None:
        x, y = det.raw_pose_m[0], det.raw_pose_m[1]
        if role == "narrow":
            move_x, move_y = -y, x
        else:
            move_x, move_y = y, -x
        move_scale = max(60, int(300 * sc))
        svx = max(0, min(w - 1, int(img_cx + move_x * move_scale)))
        svy = max(0, min(h - 1, int(img_cy + move_y * move_scale)))
        cv2.arrowedLine(out, (img_cx, img_cy), (svx, svy),
                        _COLOR_SERVO_ARROW, max(2, int(2.5 * sc)), cv2.LINE_AA, tipLength=0.18)
    elif det.tag_detected and det.tag_center_px is not None:
        # 回退：PnP 失败时用像素中心偏移
        move_x = img_cx - det.tag_center_px[0]
        move_y = img_cy - det.tag_center_px[1]
        move_dist = math.hypot(move_x, move_y)
        if move_dist > 5:
            max_len = max(60, int(150 * sc))
            if move_dist > max_len:
                move_x = move_x / move_dist * max_len
                move_y = move_y / move_dist * max_len
            svx = max(0, min(w - 1, int(img_cx + move_x)))
            svy = max(0, min(h - 1, int(img_cy + move_y)))
            cv2.arrowedLine(out, (img_cx, img_cy), (svx, svy),
                            _COLOR_SERVO_ARROW, max(2, int(2.5 * sc)), cv2.LINE_AA, tipLength=0.18)

    # ── Bottom HUD lines ──
    line_h = int(26 * sc)
    hud_y = h - 12
    if det.tag_detected and det.raw_pose_m is not None:
        p = det.raw_pose_m
        horiz = math.sqrt(p[0] ** 2 + p[1] ** 2)
        total = math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
        cv2.putText(out, f"PnP X={p[0]:+.2f} Y={p[1]:+.2f} Z={p[2]:.2f}",
                    (12, hud_y), font, 0.55 * sc, _COLOR_WHITE, ln, cv2.LINE_AA)
        hud_y -= line_h
        cv2.putText(out, f"DIST horiz={horiz:.2f}m  3D={total:.2f}m",
                    (12, hud_y), font, 0.55 * sc, _COLOR_YELLOW, ln, cv2.LINE_AA)
        hud_y -= line_h
    elif det.tag_detected:
        cv2.putText(out, "TAG OK  PnP FAIL", (12, hud_y),
                    font, 0.55 * sc, _COLOR_YELLOW, ln, cv2.LINE_AA)
        hud_y -= line_h
    else:
        cv2.putText(out, "NO TAG", (12, hud_y),
                    font, 0.55 * sc, _COLOR_RED, ln, cv2.LINE_AA)
        hud_y -= line_h

    if snap.kf_position_m is not None:
        kf = snap.kf_position_m
        fresh = "FRESH" if snap.kf_fresh else "STALE"
        cv2.putText(out, f"KF X={kf[0]:+.2f} Y={kf[1]:+.2f} Z={kf[2]:+.2f} [{fresh}]",
                    (12, hud_y), font, 0.55 * sc, _COLOR_CYAN, ln, cv2.LINE_AA)

    return out


def _render_center_hud(snap: _FieldTestSnapshot) -> np.ndarray:
    """Render center info panel between the two camera views."""
    out = np.zeros((720, 400, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    sc = 1.0
    ln = 2
    y = 35
    line_h = 30

    lines: list[tuple[str, tuple[int, int, int]]] = []

    # KF state
    if snap.kf_position_m is not None:
        kf = snap.kf_position_m
        vel = snap.kf_velocity_m_s or (0.0, 0.0, 0.0)
        fresh = "FRESH" if snap.kf_fresh else "STALE"
        lines.append((f"KF X={kf[0]:+.3f}m", _COLOR_CYAN))
        lines.append((f"   Y={kf[1]:+.3f}m", _COLOR_CYAN))
        lines.append((f"   Z={kf[2]:+.3f}m [{fresh}]", _COLOR_CYAN))
        lines.append((f"   Vx={vel[0]:+.2f} Vy={vel[1]:+.2f}", _COLOR_CYAN))
        lines.append((f"   Vz={vel[2]:+.2f} sig={snap.kf_uncertainty_m:.3f}m", _COLOR_CYAN))
    else:
        lines.append(("KF NO DATA", (100, 100, 100)))

    # AC center → tag center distance (from narrow PnP pose)
    if snap.narrow.raw_pose_m is not None:
        p = snap.narrow.raw_pose_m
        horiz = math.sqrt(p[0] ** 2 + p[1] ** 2)
        total = math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
        lines.append((f"AC->TAG horiz={horiz:.3f}m", _COLOR_YELLOW))
        lines.append((f"        3D={total:.3f}m", _COLOR_YELLOW))
    else:
        lines.append(("AC->TAG NO POSE", (100, 100, 100)))

    # Displacement
    if snap.displacement_m is not None and snap.origin_set:
        d = snap.displacement_m
        dist = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        lines.append((f"DSP X={d[0]:+.3f}m", _COLOR_ORIGIN_ARROW))
        lines.append((f"    Y={d[1]:+.3f}m", _COLOR_ORIGIN_ARROW))
        lines.append((f"    Z={d[2]:+.3f}m |d|={dist:.3f}", _COLOR_ORIGIN_ARROW))
    elif not snap.origin_set:
        lines.append(("DSP ORIGIN NOT SET", _COLOR_YELLOW))

    # Odometry
    if snap.odom_position_m is not None:
        op = snap.odom_position_m
        yaw_deg = math.degrees(snap.odom_yaw_rad) if snap.odom_yaw_rad is not None else 0.0
        lines.append((f"ODOM X={op[0]:+.3f}m", _COLOR_ODOM))
        lines.append((f"     Y={op[1]:+.3f}m", _COLOR_ODOM))
        lines.append((f"     Z={op[2]:+.3f}m yaw={yaw_deg:+.1f}deg", _COLOR_ODOM))
        if snap.odom_linear_vel_m_s is not None:
            ov = snap.odom_linear_vel_m_s
            lines.append((f"     Vx={ov[0]:+.2f} Vy={ov[1]:+.2f} Vz={ov[2]:+.2f}", _COLOR_ODOM))
    else:
        lines.append(("ODOM NO DATA", (100, 100, 100)))
        if snap.odom_topic:
            lines.append((f"  topic: {snap.odom_topic}", (80, 80, 80)))

    # Servo
    conv = " CONV" if snap.servo_converged else ""
    lines.append((f"SRV {snap.servo_phase}{conv}", _COLOR_MAGENTA))
    lines.append((f"    vx={snap.servo_vx:+.3f} vy={snap.servo_vy:+.3f}", _COLOR_MAGENTA))
    lines.append((f"    vz={snap.servo_vz:+.3f} m/s", _COLOR_MAGENTA))

    # FPS
    lines.append((f"FPS {snap.fps:.1f} lat={snap.detection_latency_ms:.1f}ms", (160, 160, 160)))

    for text, color in lines:
        cv2.putText(out, text, (12, y), font, 0.55 * sc, color, ln, cv2.LINE_AA)
        y += line_h

    # Bottom bar
    cv2.putText(out, "[Q/ESC] quit  [R] origin  [S] save",
                (12, 700), font, 0.45, (150, 150, 150), 1, cv2.LINE_AA)

    return out


def _render_side_by_side(snap: _FieldTestSnapshot) -> np.ndarray:
    narrow_view = _render_camera_hud(snap.narrow.frame, snap.narrow, "narrow", snap)
    center_view = _render_center_hud(snap)
    wide_view = _render_camera_hud(snap.wide.frame, snap.wide, "wide", snap)

    h = max(narrow_view.shape[0], center_view.shape[0], wide_view.shape[0])

    def pad_to(img: np.ndarray, target_h: int) -> np.ndarray:
        if img.shape[0] < target_h:
            pad = np.zeros((target_h - img.shape[0], img.shape[1], 3), dtype=np.uint8)
            return np.vstack([img, pad])
        return img

    narrow_view = pad_to(narrow_view, h)
    center_view = pad_to(center_view, h)
    wide_view = pad_to(wide_view, h)

    return np.hstack([narrow_view, center_view, wide_view])


# ── ROS 2 Node ────────────────────────────────────────────────────────────

_ODOM_CANDIDATES: Final = (
    "/localization/odom",
    "/localization/lio/odom",
    "/fcu/optical_flow/odom",
    "/fast_lio/odometry",
)


class FieldTestNode(Node):

    def __init__(self) -> None:
        super().__init__("field_test")
        self.declare_parameter("target_tag_id", -1)
        self.declare_parameter("tag_size_m", TAG_SIZE_M)
        self.declare_parameter("tag_family", TAG_FAMILY)
        self.declare_parameter("max_display_width", 1280)
        self.declare_parameter("headless_log_interval_sec", 2.0)
        self.declare_parameter("log_interval_sec", 1.0)
        self.declare_parameter("camera_yaw_offset_rad", -math.pi / 2)
        self.declare_parameter("wide_camera_yaw_offset_rad", math.pi / 2)
        self.declare_parameter("kalman_process_noise_pos", 0.05)
        self.declare_parameter("kalman_process_noise_vel", 0.3)
        self.declare_parameter("kalman_max_predict_age_sec", 0.5)
        self.declare_parameter("use_direct_capture", False)
        self.declare_parameter("camera_device", "/dev/video2")
        self.declare_parameter("wide_camera_device", "/dev/video0")
        self.declare_parameter("odom_topic", "")

        tag_size = float(self.get_parameter("tag_size_m").value)
        tag_family = str(self.get_parameter("tag_family").value)
        target_tag_id = int(self.get_parameter("target_tag_id").value)
        max_width = int(self.get_parameter("max_display_width").value)
        headless_interval = float(self.get_parameter("headless_log_interval_sec").value)
        self._log_interval = float(self.get_parameter("log_interval_sec").value)
        self._camera_yaw_offset = float(self.get_parameter("camera_yaw_offset_rad").value)
        self._wide_camera_yaw_offset = float(self.get_parameter("wide_camera_yaw_offset_rad").value)
        self._use_direct_capture = bool(self.get_parameter("use_direct_capture").value)
        self._camera_device = str(self.get_parameter("camera_device").value)
        self._wide_camera_device = str(self.get_parameter("wide_camera_device").value)

        self._bridge = CvBridge()
        self._detector = AprilTagDetector(tag_size, tag_family)
        self._target_tag_id = target_tag_id if target_tag_id >= 0 else None
        self._kalman = KalmanTracker(
            process_noise_pos=float(self.get_parameter("kalman_process_noise_pos").value),
            process_noise_vel=float(self.get_parameter("kalman_process_noise_vel").value),
            max_predict_age_sec=float(self.get_parameter("kalman_max_predict_age_sec").value),
        )
        self._servo = VisualServoController()

        # 伺服结果缓存（wide 帧复用，避免箭头闪烁）
        self._servo_phase: str = "IDLE"
        self._servo_vx: float = 0.0
        self._servo_vy: float = 0.0
        self._servo_vz: float = 0.0
        self._servo_converged: bool = False

        self._camera_info: dict[str, CameraModel | None] = {"narrow": None, "wide": None}
        self._camera_lock = Lock()
        self._latest_detection: dict[str, _CameraDetection] = {
            "narrow": _CameraDetection(),
            "wide": _CameraDetection(),
        }

        self._odom_position_m: tuple[float, float, float] | None = None
        self._odom_yaw_rad: float | None = None
        self._odom_linear_vel_m_s: tuple[float, float, float] | None = None
        self._odom_angular_vel_rad_s: float | None = None
        self._odom_topic_active: str = ""
        self._odom_last_time: float = 0.0

        self._origin_position: np.ndarray | None = None

        self._frame_count: int = 0
        self._fps_start: float = time.monotonic()
        self._fps: float = 0.0
        self._last_log_time: float = 0.0

        self._display = _FieldTestDisplayWorker(max_width, headless_interval)
        self._display.start()

        self._cv_capture: dict[str, cv2.VideoCapture | None] = {"narrow": None, "wide": None}
        self._capture_threads: dict[str, Thread | None] = {"narrow": None, "wide": None}
        self._capture_stop = Event()

        if self._use_direct_capture:
            self._start_direct_capture()
        else:
            info_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            for role, ns in [("narrow", "/camera/narrow"), ("wide", "/camera/wide")]:
                self.create_subscription(
                    CameraInfo, f"{ns}/camera_info",
                    lambda msg, r=role: self._on_camera_info(msg, r), info_qos,
                )
                self.create_subscription(
                    Image, f"{ns}/image_raw",
                    lambda msg, r=role: self._on_image(msg, r), qos_profile_sensor_data,
                )

        odom_topic_param = str(self.get_parameter("odom_topic").value)
        if odom_topic_param:
            self.create_subscription(Odometry, odom_topic_param, self._on_odometry, qos_profile_sensor_data)
            self._odom_topic_active = odom_topic_param
            self.get_logger().info(f"Subscribing odom: {odom_topic_param}")
        else:
            for topic in _ODOM_CANDIDATES:
                self.create_subscription(Odometry, topic, self._on_odometry, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribing odom candidates: {_ODOM_CANDIDATES}")

        self._check_timer = self.create_timer(0.2, self._check_display_events)
        self._odom_check_timer = self.create_timer(2.0, self._check_odom_alive)

        if self._display.is_headless:
            self.get_logger().info("Field test HEADLESS mode, log every %.1fs", headless_interval)
        else:
            self.get_logger().info("Field test started. Press R to set origin, Q/ESC to quit.")

    # ── Direct capture ──────────────────────────────────────────────────

    def _start_direct_capture(self) -> None:
        narrow_calib = np.array([[1082.7, 0, 605.9], [0, 1082.1, 430.0], [0, 0, 1]], dtype=np.float64)
        narrow_dist = np.array([0.086, -0.207, -0.0004, 0.0006, -0.046], dtype=np.float64)
        wide_calib = np.array([[527.3, 0, 598.2], [0, 526.3, 385.7], [0, 0, 1]], dtype=np.float64)
        wide_dist = np.array([-0.003, 0.044, 0.0006, 0.002, -0.072], dtype=np.float64)

        for role, device, calib, dist, frame_id in [
            ("narrow", self._camera_device, narrow_calib, narrow_dist, "camera_narrow_optical_frame"),
            ("wide", self._wide_camera_device, wide_calib, wide_dist, "camera_wide_optical_frame"),
        ]:
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.get_logger().warn(f"Cannot open {role} camera: {device}")
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 20 if role == "narrow" else 15)

            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)

            self._cv_capture[role] = cap
            self._camera_info[role] = CameraModel(
                matrix=calib, distortion=dist,
                width=w, height=h, frame_id=frame_id, calibrated=True,
            )

            self.get_logger().info(f"Direct capture {role}: {device} {w}x{h}@{fps:.0f}fps")
            thread = Thread(target=self._capture_loop, args=(role,), daemon=True)
            self._capture_threads[role] = thread
            thread.start()

    def _capture_loop(self, role: str) -> None:
        cap = self._cv_capture[role]
        while not self._capture_stop.is_set() and rclpy.ok():
            if cap is None:
                time.sleep(0.01)
                continue
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            self._process_frame(frame, role)

    def destroy_node(self) -> None:
        self._capture_stop.set()
        for role in ("narrow", "wide"):
            cap = self._cv_capture[role]
            if cap is not None:
                cap.release()
        super().destroy_node()

    # ── Camera callbacks ────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo, role: str) -> None:
        matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(msg.d, dtype=np.float64)
        frame_id = msg.header.frame_id or f"camera_{role}_optical_frame"
        self._camera_info[role] = CameraModel(
            matrix=matrix, distortion=dist,
            width=msg.width, height=msg.height, frame_id=frame_id, calibrated=True,
        )

    def _on_image(self, msg: Image, role: str) -> None:
        if self._camera_info[role] is None:
            return
        try:
            if msg.encoding and msg.encoding.lower() in ['bgr8', 'rgb8', 'mono8', 'bgra8', 'rgba8']:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            elif msg.encoding and msg.encoding.lower() in ['mjpeg', 'jpg', 'jpeg']:
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    return
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except (CvBridgeError, Exception):
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    return
            except Exception:
                return
        self._process_frame(frame, role)

    # ── Frame processing ────────────────────────────────────────────────

    def _rotate_frame(self, frame: np.ndarray, role: str) -> np.ndarray:
        """Rotate image for display only (image-top → nose orientation)."""
        if role == "narrow":
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        else:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def _rotate_point(self, u: float, v: float, role: str, w: int, h: int) -> tuple[int, int]:
        """Map a pixel from the raw (unrotated) frame into the rotated display frame."""
        if role == "narrow":
            return (int(round(h - 1 - v)), int(round(u)))
        else:
            return (int(round(v)), int(round(w - 1 - u)))

    def _process_frame(self, frame: np.ndarray, role: str) -> None:
        t0 = time.monotonic()

        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_start
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start = now

        # 检测/PnP 在原始图像上进行（内参与原始图像匹配）
        raw_h, raw_w = frame.shape[:2]
        detection = self._detector.detect(frame, self._target_tag_id)

        det = _CameraDetection(frame=self._rotate_frame(frame, role))

        if isinstance(detection, CorrespondenceSet):
            det.tag_detected = True
            corners = detection.image_points
            # 角点从原始坐标系映射到旋转后的显示坐标系
            det.tag_corners_px = tuple(
                self._rotate_point(corners[i, 0], corners[i, 1], role, raw_w, raw_h)
                for i in range(4)
            )
            det.tag_center_px = (
                int(round(np.mean([c[0] for c in det.tag_corners_px]))),
                int(round(np.mean([c[1] for c in det.tag_corners_px]))),
            )

            frame_ctx = FrameContext(
                acquisition_sec=now, receipt_steady_sec=now,
                evaluation_steady_sec=now, source_sequence=0,
                target_revision="d2026-apriltag-v1",
            )
            motion_ctx = MotionContext(
                acquisition_sec=now, receipt_steady_sec=now,
                turn_class=0, heading_rad=None,
                yaw_rate_rad_s=0.0, speed_m_s=0.0, prior=None,
            )
            if self._camera_info[role] is not None:
                pose = estimate_target_pose(
                    detection, self._camera_info[role], motion_ctx, PoseLimits()
                )
            else:
                pose = None
            if isinstance(pose, PoseEstimate):
                det.raw_pose_m = (
                    float(pose.translation_m[0]),
                    float(pose.translation_m[1]),
                    float(pose.translation_m[2]),
                )
                det.raw_reproj_rms_px = pose.reprojection_rms_px

                # 只有主力相机(narrow)喂 Kalman，避免双相机坐标系混用导致状态抖动
                if role == "narrow":
                    depth_m = max(0.1, pose.translation_m[2])
                    self._kalman.update(
                        measurement=pose.translation_m,
                        measurement_noise_px=det.raw_reproj_rms_px,
                        now_sec=now, depth_m=depth_m,
                    )
            else:
                if role == "narrow":
                    self._kalman.predict(now)
        else:
            if role == "narrow":
                self._kalman.predict(now)

        detection_latency_ms = (time.monotonic() - t0) * 1000.0

        with self._camera_lock:
            self._latest_detection[role] = det

        kf_pos_m: tuple[float, float, float] | None = None
        kf_vel_m_s: tuple[float, float, float] | None = None
        kf_uncertainty: float = float("inf")
        kf_fresh = False

        if self._kalman.is_initialized:
            pos = self._kalman.position
            vel = self._kalman.velocity
            kf_pos_m = (float(pos[0]), float(pos[1]), float(pos[2]))
            kf_vel_m_s = (float(vel[0]), float(vel[1]), float(vel[2]))
            kf_uncertainty = self._kalman.position_uncertainty
            kf_fresh = self._kalman.is_fresh

        disp_m: tuple[float, float, float] | None = None
        if kf_pos_m is not None and self._origin_position is not None:
            d = np.array(kf_pos_m) - self._origin_position
            disp_m = (float(d[0]), float(d[1]), float(d[2]))

        servo_phase = "IDLE"
        servo_vx = servo_vy = servo_vz = 0.0
        servo_converged = False

        # 伺服只在主力相机(narrow)上计算，wide 只做检测显示。
        # 结果缓存到实例变量，供 wide 帧构建 snapshot 时复用，避免箭头闪烁。
        if role == "narrow" and kf_pos_m is not None and kf_fresh:
            cmd: VelocityCommand = self._servo.compute_command(
                target_x_m=kf_pos_m[0], target_y_m=kf_pos_m[1], target_z_m=kf_pos_m[2],
                current_timestamp_sec=now, camera_yaw_offset_rad=self._camera_yaw_offset,
            )
            servo_phase = cmd.phase.value.upper()
            servo_vx = cmd.vx_m_s
            servo_vy = cmd.vy_m_s
            servo_vz = cmd.vz_m_s
            servo_converged = cmd.converged
            self._servo_phase = servo_phase
            self._servo_vx = servo_vx
            self._servo_vy = servo_vy
            self._servo_vz = servo_vz
            self._servo_converged = servo_converged
        elif role == "wide":
            servo_phase = self._servo_phase
            servo_vx = self._servo_vx
            servo_vy = self._servo_vy
            servo_vz = self._servo_vz
            servo_converged = self._servo_converged

        if now - self._last_log_time >= self._log_interval:
            self._last_log_time = now
            self._log_status(det.tag_detected, det.raw_pose_m, kf_pos_m, kf_vel_m_s,
                             kf_uncertainty, kf_fresh, disp_m,
                             servo_phase, servo_vx, servo_vy, servo_vz, detection_latency_ms)

        with self._camera_lock:
            snap = _FieldTestSnapshot(
                narrow=self._latest_detection["narrow"],
                wide=self._latest_detection["wide"],
                kf_position_m=kf_pos_m,
                kf_velocity_m_s=kf_vel_m_s,
                kf_uncertainty_m=kf_uncertainty,
                kf_fresh=kf_fresh,
                origin_set=self._origin_position is not None,
                displacement_m=disp_m,
                odom_position_m=self._odom_position_m,
                odom_yaw_rad=self._odom_yaw_rad,
                odom_linear_vel_m_s=self._odom_linear_vel_m_s,
                odom_angular_vel_rad_s=self._odom_angular_vel_rad_s,
                odom_topic=self._odom_topic_active,
                servo_phase=servo_phase,
                servo_vx=servo_vx, servo_vy=servo_vy, servo_vz=servo_vz,
                servo_converged=servo_converged,
                fps=self._fps, detection_latency_ms=detection_latency_ms,
            )
        self._display.submit(snap)

    # ── Odometry ────────────────────────────────────────────────────────

    def _on_odometry(self, msg: Odometry) -> None:
        self._odom_last_time = time.monotonic()

        topic = ""
        for t in _ODOM_CANDIDATES:
            if self._odom_topic_active == "" or self._odom_topic_active == t:
                topic = t
                break
        if not topic and self._odom_topic_active:
            topic = self._odom_topic_active
        self._odom_topic_active = topic

        pos = msg.pose.pose.position
        self._odom_position_m = (float(pos.x), float(pos.y), float(pos.z))

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        vel = msg.twist.twist.linear
        self._odom_linear_vel_m_s = (float(vel.x), float(vel.y), float(vel.z))
        self._odom_angular_vel_rad_s = float(msg.twist.twist.angular.z)

    def _check_odom_alive(self) -> None:
        if self._odom_topic_active and self._odom_last_time > 0:
            age = time.monotonic() - self._odom_last_time
            if age > 2.0:
                self.get_logger().warn(f"Odom stale {age:.1f}s from {self._odom_topic_active}")

    # ── Origin ──────────────────────────────────────────────────────────

    def _reset_origin(self) -> None:
        if self._kalman.is_initialized:
            self._origin_position = self._kalman.position.copy()
            self.get_logger().info(
                "Origin set at (%.3f, %.3f, %.3f) m" % (
                    self._origin_position[0], self._origin_position[1], self._origin_position[2]
                )
            )
        else:
            self.get_logger().warn("Cannot set origin — KF not initialized")

    # ── Display events ──────────────────────────────────────────────────

    def _check_display_events(self) -> None:
        if self._display.quit_requested.is_set():
            self.get_logger().info("Quit requested")
            self.destroy_node()
            return
        if self._display.reset_origin_requested.is_set():
            self._display.reset_origin_requested.clear()
            self._reset_origin()

    # ── Logging ─────────────────────────────────────────────────────────

    def _log_status(self, tag_detected, raw_pose, kf_pos, kf_vel, kf_uncertainty,
                    kf_fresh, displacement, servo_phase, servo_vx, servo_vy, servo_vz,
                    detect_latency_ms) -> None:
        tag_str = "YES" if tag_detected else "NO"
        pose_str = f"({raw_pose[0]:+.3f}, {raw_pose[1]:+.3f}, {raw_pose[2]:.3f})" if raw_pose else "—"
        kf_str = f"({kf_pos[0]:+.3f}, {kf_pos[1]:+.3f}, {kf_pos[2]:+.3f})" if kf_pos else "—"
        vel_str = f"({kf_vel[0]:+.3f}, {kf_vel[1]:+.3f}, {kf_vel[2]:+.3f})" if kf_vel else "—"
        disp_str = f"({displacement[0]:+.3f}, {displacement[1]:+.3f}, {displacement[2]:+.3f})" if displacement else "—"
        fresh_str = "FRESH" if kf_fresh else "STALE"

        odom_str = "—"
        if self._odom_position_m is not None:
            op = self._odom_position_m
            yaw_deg = math.degrees(self._odom_yaw_rad) if self._odom_yaw_rad is not None else 0.0
            odom_str = f"({op[0]:+.3f}, {op[1]:+.3f}, {op[2]:+.3f}) yaw={yaw_deg:+.1f}° [{self._odom_topic_active}]"

        logger.info(
            "───────── Field Test ─────────\n"
            "  Tag: %s  PnP: %s\n"
            "  KF:  %s [%s] σ=%.4fm  vel: %s\n"
            "  Disp: %s\n"
            "  Odom: %s\n"
            "  Servo: %s vx=%.3f vy=%.3f vz=%.3f\n"
            "  FPS: %.1f lat=%.1fms",
            tag_str, pose_str,
            kf_str, fresh_str, kf_uncertainty, vel_str,
            disp_str, odom_str,
            servo_phase, servo_vx, servo_vy, servo_vz,
            self._fps, detect_latency_ms,
        )

    def destroy_node(self) -> None:
        self._display.stop(timeout=1.0)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FieldTestNode()
    try:
        if node._display.is_headless:
            rclpy.spin(node)
        else:
            _spin_with_display(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _spin_with_display(node: FieldTestNode) -> None:
    display = node._display
    os.environ.setdefault("GDK_SCALE", "1")
    os.environ.setdefault("GTK_CSD", "0")
    cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WINDOW_NAME, 1800, 720)

    while rclpy.ok() and not display.quit_requested.is_set():
        rclpy.spin_once(node, timeout_sec=0.0)

        try:
            snap = display._queue.get_nowait()
        except Empty:
            snap = None

        if snap is not None:
            frame = _render_side_by_side(snap)
            if 0 < display.max_width < frame.shape[1]:
                scale = display.max_width / frame.shape[1]
                frame = cv2.resize(
                    frame,
                    (display.max_width, max(1, int(round(frame.shape[0] * scale)))),
                    interpolation=cv2.INTER_AREA,
                )
            cv2.imshow(_WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            display.quit_requested.set()
        elif key == ord("r"):
            display.reset_origin_requested.set()
        elif key == ord("s") and snap is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = f"/tmp/field_test_{ts}.png"
            cv2.imwrite(path, frame)

    try:
        cv2.destroyWindow(_WINDOW_NAME)
    except Exception:
        pass


if __name__ == "__main__":
    main()
