"""Mission display node — real-time dual-camera feed with mission HUD.

Subscribes to both camera image topics (narrow + wide), mission status, FCU
state, target observations and the FlightCommand action feedback/result, then
renders a side-by-side view: narrow | center status panel | wide.

Camera frames are rotated for display only (image-top -> nose orientation):
  - narrow: 90 degrees clockwise
  - wide:   90 degrees counter-clockwise
(mirrors field_test_node so the two side views face the same direction)

Threading: in display mode cv2.namedWindow / cv2.imshow / cv2.waitKey run on
the MAIN thread via _spin_with_display (GTK3 requires the event loop on the
main thread); the worker thread is only started in headless mode for periodic
status logging.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread
from typing import Final

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ed_uav_interfaces.action import FlightCommand
from ed_uav_interfaces.msg import FcuState, MissionStatus, TargetObservation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

_WINDOW_NAME: Final = "Task3 Mission Display — [Q/ESC] quit [S] screenshot"

# Camera topics (raw frames; perception annotations are fused into the HUD)
_CAMERA_TOPICS: Final = {
    "narrow": "/camera/narrow/image_raw",
    "wide": "/camera/wide/image_raw",
}
_FEEDBACK_TOPIC: Final = "/fcu/flight_command/_action/feedback"
_RESULT_TOPIC: Final = "/fcu/flight_command/_action/result"

# ── HUD color palette (BGR) ────────────────────────────────────────────────
_COLOR_WHITE: Final = (255, 255, 255)
_COLOR_GREEN: Final = (60, 220, 60)
_COLOR_YELLOW: Final = (0, 220, 255)
_COLOR_RED: Final = (60, 60, 240)
_COLOR_ORANGE: Final = (0, 165, 255)
_COLOR_CYAN: Final = (255, 200, 0)
_COLOR_PANEL_BG: Final = (12, 18, 24)
_COLOR_CMD_ACTIVE: Final = (0, 255, 200)
_COLOR_MAGENTA: Final = (255, 0, 200)

# ── Mission state label mapping ────────────────────────────────────────────
_STATE_LABELS: Final[dict[int, str]] = {
    MissionStatus.STATE_PRE_ARM: "PRE_ARM",
    MissionStatus.STATE_TAKEOFF: "TAKEOFF",
    MissionStatus.STATE_SEARCHING: "SEARCHING",
    MissionStatus.STATE_ACCOMPANYING: "ACCOMPANYING",
    MissionStatus.STATE_PAYLOAD_DROP: "PAYLOAD_DROP",
    MissionStatus.STATE_LANDING_ON_VEHICLE: "LANDING_VEHICLE",
    MissionStatus.STATE_VEHICLE_DWELL: "VEHICLE_DWELL",
    MissionStatus.STATE_RETURNING_HOME: "RETURNING",
    MissionStatus.STATE_LANDING_HOME: "LANDING_HOME",
    MissionStatus.STATE_SUCCEEDED: "SUCCEEDED",
    MissionStatus.STATE_ABORTED: "ABORTED",
    MissionStatus.STATE_STABILITY_TEST: "STABILITY_TEST",
}

# ── FCU mode label mapping ─────────────────────────────────────────────────
_FCU_MODE_LABELS: Final[dict[int, str]] = {
    FcuState.MODE_STABILIZE: "STAB",
    FcuState.MODE_ALTITUDE_HOLD: "ALT_HOLD",
    FcuState.MODE_POSITION_HOLD: "POS_HOLD",
    FcuState.MODE_PROGRAM: "PROGRAM",
}

# ── FlightCommand action state/result mapping ─────────────────────────────
_CMD_STATE_LABELS: Final[dict[int, str]] = {
    FlightCommand.Feedback.STATE_QUEUED: "QUEUED",
    FlightCommand.Feedback.STATE_SENT: "SENT",
    FlightCommand.Feedback.STATE_ACKNOWLEDGED: "ACK",
    FlightCommand.Feedback.STATE_EXECUTING: "EXECUTING",
    FlightCommand.Feedback.STATE_TERMINAL: "TERMINAL",
}
_CMD_RESULT_LABELS: Final[dict[int, str]] = {
    FlightCommand.Result.RESULT_SUCCEEDED: "SUCCEEDED",
    FlightCommand.Result.RESULT_REJECTED: "REJECTED",
    FlightCommand.Result.RESULT_TIMEOUT: "TIMEOUT",
    FlightCommand.Result.RESULT_FCU_ERROR: "FCU_ERROR",
}

# correlation_id (set by mission_executor) -> friendly command name
_CMD_NAME_MAP: Final[dict[str, str]] = {
    "mission_takeoff": "TAKEOFF",
    "mission_move": "MOVE",
    "mission_hover": "HOVER",
    "landing_descend": "LAND DESCEND",
    "landing_land": "LAND",
    "landing_disarm": "DISARM",
    "d2026_target_track": "TARGET TRACK",
    "d2026_precision_land": "PRECISION LAND",
    "d2026_return_home": "RETURN HOME",
}


def _pretty_cmd_id(correlation_id: str) -> str:
    """Map a FlightCommand correlation_id to a short display name."""
    if correlation_id in _CMD_NAME_MAP:
        return _CMD_NAME_MAP[correlation_id]
    if correlation_id.startswith("stability_square_"):
        return f"SQUARE {correlation_id.rsplit('_', 1)[-1]}"
    if correlation_id.startswith("stability_circle_"):
        return f"CIRCLE {correlation_id.rsplit('_', 1)[-1]}"
    if correlation_id.startswith("d2026_"):
        return correlation_id.removeprefix("d2026_").replace("_", " ").upper()
    return correlation_id.replace("_", " ").upper() if correlation_id else "---"


# ── Frame snapshot for the display thread ──────────────────────────────────

@dataclass(frozen=True)
class _DisplaySnapshot:
    """Immutable snapshot of both camera frames plus all HUD metadata."""

    narrow_frame: np.ndarray | None
    wide_frame: np.ndarray | None
    mission_state: str
    mission_reason: str
    mission_complete: bool
    target_valid: bool
    target_x_m: float
    target_y_m: float
    target_z_m: float
    target_confidence: float
    target_quality: float
    fcu_altitude_m: float
    fcu_battery_v: float
    fcu_mode: str
    fcu_armed: bool
    fcu_comm_ok: bool
    fcu_position_m: tuple[float, float]
    task3_control_allowed: bool
    emergency_lock_active: bool
    fcu_cmd_id: str
    fcu_cmd_state: str
    fcu_cmd_active: bool
    fcu_cmd_result: str
    fcu_cmd_result_reason: str
    fps: float


# ── Display worker thread ──────────────────────────────────────────────────

class _DisplayWorker:
    """Async display worker — slow X11 never blocks the ROS callback chain.

    Follows the _PreviewWorker pattern from drone/vision.py:
      - daemon Thread for cv2.imshow / cv2.waitKey
      - Queue(maxsize=1) with non-blocking overwrite policy
      - old frames are dropped when the consumer cannot keep up
    """

    def __init__(self, max_width: int, headless_log_interval: float):
        self.max_width = max_width
        self.headless_log_interval = headless_log_interval
        self.quit_requested = Event()
        self._stop = Event()
        self._queue: Queue[_DisplaySnapshot] = Queue(maxsize=1)
        self._thread = Thread(
            target=self._run,
            name="mission-display",
            daemon=True,
        )
        self._started = False
        self._headless = not os.environ.get("DISPLAY")
        self._last_headless_log: float = 0.0

    @property
    def is_headless(self) -> bool:
        return self._headless

    def start(self) -> None:
        if not self._started:
            self._started = True
            # GTK3 事件循环必须在主线程: 非 headless 时窗口由 main() 的
            # _spin_with_display 驱动, 线程只用于 headless 日志模式
            if self._headless:
                self._thread.start()

    def submit(self, snapshot: _DisplaySnapshot) -> None:
        """Non-blocking overwrite — always tracks the latest snapshot."""
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
        if self._started:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                pass  # X11 may be blocked; non-fatal

    def _run(self) -> None:
        if self._headless:
            self._run_headless()
        else:
            self._run_display()

    def _run_headless(self) -> None:
        """Headless mode — periodically log status instead of showing a window."""
        while not self._stop.is_set():
            try:
                snap = self._queue.get(timeout=0.5)
            except Empty:
                continue
            now = time.monotonic()
            if now - self._last_headless_log >= self.headless_log_interval:
                self._last_headless_log = now
                import logging
                logger = logging.getLogger("ed_uav_mission.display")
                logger.info(
                    "[DISPLAY] state=%s alt=%.1fm bat=%.1fV tgt=%s "
                    "pos=(%.2f,%.2f)m qual=%.3f cam=(%s,%s) cmd=%s[%s] fps=%.1f",
                    snap.mission_state,
                    snap.fcu_altitude_m,
                    snap.fcu_battery_v,
                    "VALID" if snap.target_valid else "NONE",
                    snap.fcu_position_m[0],
                    snap.fcu_position_m[1],
                    snap.target_quality,
                    "N" if snap.narrow_frame is not None else "-",
                    "W" if snap.wide_frame is not None else "-",
                    snap.fcu_cmd_id,
                    snap.fcu_cmd_state,
                    snap.fps,
                )

    def _run_display(self) -> None:
        """Display mode — cv2.imshow in daemon thread."""
        try:
            cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(_WINDOW_NAME, 1280, 540)
        except Exception:
            # Fallback to headless if window creation fails
            self._headless = True
            self._run_headless()
            return

        while not self._stop.is_set():
            try:
                snap = self._queue.get(timeout=0.1)
            except Empty:
                # Still pump waitKey to keep the window responsive
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    self.quit_requested.set()
                    break
                continue

            display = _render_layout(snap, self.max_width)
            cv2.imshow(_WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):  # Q or ESC
                self.quit_requested.set()
                break
            elif key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = f"/tmp/task3_display_{ts}.png"
                cv2.imwrite(path, display)

        try:
            cv2.destroyWindow(_WINDOW_NAME)
        except Exception:
            pass


# ── HUD rendering ──────────────────────────────────────────────────────────

def _render_camera_view(frame: np.ndarray | None, role: str) -> np.ndarray:
    """Render one camera's (already rotated) view with a minimal overlay."""
    if frame is None:
        out = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(
            out, f"{role.upper()} NO SIGNAL", (int(640 * 0.28), 185),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, _COLOR_RED, 2, cv2.LINE_AA,
        )
        return out

    out = frame.copy()
    h, w = out.shape[:2]
    sc = max(0.8, min(1.8, min(w, h) / 400.0))
    ln = max(2, int(round(3 * sc)))
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Camera label
    cv2.putText(out, f"[{role.upper()}]", (12, int(35 * sc)),
                font, 0.8 * sc, _COLOR_CYAN, ln + 1, cv2.LINE_AA)

    # Aircraft center (image center) crosshair
    img_cx, img_cy = w // 2, h // 2
    cv2.drawMarker(out, (img_cx, img_cy), _COLOR_YELLOW,
                   cv2.MARKER_CROSS, max(20, int(28 * sc)), ln + 1)
    cv2.putText(out, "AC", (img_cx + 14, img_cy - 14),
                font, 0.5 * sc, _COLOR_YELLOW, ln, cv2.LINE_AA)

    return out


def _render_center_panel(snap: _DisplaySnapshot) -> np.ndarray:
    """Render the mission status / flight command panel between the cameras."""
    out = np.zeros((640, 400, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    sc = 1.0
    ln = 2
    y = 30
    line_h = 27

    def line(text: str, color: tuple[int, int, int]) -> None:
        nonlocal y
        cv2.putText(out, text, (12, y), font, 0.5 * sc, color, ln, cv2.LINE_AA)
        y += line_h

    # Mission state
    state_color = _COLOR_WHITE
    if snap.mission_state == "SUCCEEDED":
        state_color = _COLOR_GREEN
    elif snap.mission_state == "ABORTED":
        state_color = _COLOR_RED
    elif snap.mission_state in ("TAKEOFF", "SEARCHING"):
        state_color = _COLOR_YELLOW
    line(f"STATE  {snap.mission_state}", state_color)
    if snap.mission_reason:
        line(f"  >> {snap.mission_reason[:44]}", (180, 180, 180))

    # FCU status
    armed_str = "ARMED" if snap.fcu_armed else "DISARMED"
    comm_str = "OK" if snap.fcu_comm_ok else "LOST"
    line(f"FCU  {snap.fcu_mode}  {armed_str}  COMM={comm_str}",
        _COLOR_GREEN if snap.fcu_comm_ok else _COLOR_RED)

    # Task3 control authority / emergency lock
    ctrl_str = "CTRL=OK" if snap.task3_control_allowed else "CTRL=DENIED"
    lock_str = "LOCK" if snap.emergency_lock_active else "unlock"
    line(f"TASK3  {ctrl_str}  {lock_str}",
        _COLOR_GREEN if snap.task3_control_allowed else _COLOR_RED)

    # Altitude & battery
    if snap.fcu_altitude_m <= 0.3 or snap.fcu_battery_v <= 10.5:
        telemetry_color = _COLOR_RED
    elif snap.fcu_battery_v <= 11.0:
        telemetry_color = _COLOR_YELLOW
    else:
        telemetry_color = _COLOR_GREEN
    line(f"ALT  {snap.fcu_altitude_m:.2f}m   BAT  {snap.fcu_battery_v:.1f}V",
        telemetry_color)

    # Position (optical flow)
    line(f"POS  X={snap.fcu_position_m[0]:+.2f}m  Y={snap.fcu_position_m[1]:+.2f}m",
        _COLOR_CYAN)

    # Target info
    if snap.target_valid:
        line(f"TGT  X={snap.target_x_m:+.3f}m Y={snap.target_y_m:+.3f}m Z={snap.target_z_m:.3f}m",
            _COLOR_GREEN)
        line(f"     conf={snap.target_confidence:.3f}  qual={snap.target_quality:.3f}",
            _COLOR_GREEN)
    else:
        line("TGT  NO TARGET", _COLOR_RED)

    # Flight command being sent (from FlightCommand action feedback)
    cmd_color = _COLOR_CMD_ACTIVE if snap.fcu_cmd_active else _COLOR_WHITE
    line(f"CMD  {_pretty_cmd_id(snap.fcu_cmd_id)} [{snap.fcu_cmd_state}]", cmd_color)
    if snap.fcu_cmd_result:
        result_color = _COLOR_GREEN if snap.fcu_cmd_result == "SUCCEEDED" else _COLOR_RED
        suffix = f": {snap.fcu_cmd_result_reason[:36]}" if snap.fcu_cmd_result_reason else ""
        line(f"LAST {snap.fcu_cmd_result}{suffix}", result_color)

    # FPS
    line(f"FPS  {snap.fps:.1f}", (170, 170, 170))

    # Bottom bar
    bottom = "[Q/ESC] quit  [S] save"
    if snap.mission_complete:
        bottom += "   *** MISSION COMPLETE ***"
    cv2.putText(out, bottom, (12, 628), font, 0.42, (150, 150, 150), 1, cv2.LINE_AA)

    return out


def _render_layout(snap: _DisplaySnapshot, max_width: int) -> np.ndarray:
    """Compose narrow | center panel | wide side-by-side and scale down."""
    narrow_view = _render_camera_view(snap.narrow_frame, "narrow")
    center_view = _render_center_panel(snap)
    wide_view = _render_camera_view(snap.wide_frame, "wide")

    h = max(narrow_view.shape[0], center_view.shape[0], wide_view.shape[0])

    def pad_to(img: np.ndarray, target_h: int) -> np.ndarray:
        if img.shape[0] < target_h:
            pad = np.zeros((target_h - img.shape[0], img.shape[1], 3), dtype=np.uint8)
            return np.vstack([img, pad])
        return img

    narrow_view = pad_to(narrow_view, h)
    center_view = pad_to(center_view, h)
    wide_view = pad_to(wide_view, h)

    display = np.hstack([narrow_view, center_view, wide_view])

    if 0 < max_width < display.shape[1]:
        scale = max_width / display.shape[1]
        display = cv2.resize(
            display,
            (max_width, max(1, int(round(display.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    return display


# ── ROS 2 Node ─────────────────────────────────────────────────────────────

class MissionDisplayNode(Node):
    """ROS 2 node that renders both camera feeds with mission HUD.

    Subscribes:
      /camera/{narrow,wide}/image_raw       (sensor_msgs/Image)
      /d_task/mission_status                (MissionStatus)
      /d_task/target_observation            (TargetObservation)
      /fcu/state                            (FcuState)
      /fcu/flight_command/_action/feedback  (FlightCommand.FeedbackMessage)
      /fcu/flight_command/_action/result    (FlightCommand.ResultMessage)

    Displays a real-time OpenCV window with both cameras side by side,
    mission state panel, FCU telemetry overlay and the flight command
    currently being sent.
    """

    def __init__(self) -> None:
        super().__init__("mission_display")
        self.declare_parameter("max_display_width", 960)
        self.declare_parameter("headless_log_interval_sec", 5.0)

        max_width = int(self.get_parameter("max_display_width").value)
        headless_interval = float(self.get_parameter("headless_log_interval_sec").value)

        self._bridge = CvBridge()
        self._worker = _DisplayWorker(max_width, headless_interval)

        # Latest frames per camera (rotated for display)
        self._latest_frames: dict[str, np.ndarray | None] = {"narrow": None, "wide": None}

        # Latest state from subscriptions
        self._mission_state: str = "PRE_ARM"
        self._mission_reason: str = ""
        self._mission_complete: bool = False
        self._target_valid: bool = False
        self._target_x_m: float = 0.0
        self._target_y_m: float = 0.0
        self._target_z_m: float = 0.0
        self._target_confidence: float = 0.0
        self._target_quality: float = 0.0
        self._fcu_altitude_m: float = 0.0
        self._fcu_battery_v: float = 0.0
        self._fcu_mode: str = "STAB"
        self._fcu_armed: bool = False
        self._fcu_comm_ok: bool = False
        self._fcu_pos_x: float = 0.0
        self._fcu_pos_y: float = 0.0
        self._task3_control_allowed: bool = False
        self._emergency_lock_active: bool = False
        self._fcu_cmd_id: str = ""
        self._fcu_cmd_state: str = "---"
        self._fcu_cmd_active: bool = False
        self._fcu_cmd_result: str = ""
        self._fcu_cmd_result_reason: str = ""

        # FPS tracking (any camera frame arrival counts)
        self._frame_count: int = 0
        self._fps_start: float = time.monotonic()
        self._fps: float = 0.0

        # Subscriptions
        for role, topic in _CAMERA_TOPICS.items():
            self.create_subscription(
                Image,
                topic,
                lambda msg, r=role: self._on_image(msg, r),
                qos_profile_sensor_data,
            )
        self._target_sub = self.create_subscription(
            TargetObservation,
            "/d_task/target_observation",
            self._on_target_observation,
            20,
        )
        self._mission_sub = self.create_subscription(
            MissionStatus,
            "/d_task/mission_status",
            self._on_mission_status,
            20,
        )
        self._fcu_sub = self.create_subscription(
            FcuState,
            "/fcu/state",
            self._on_fcu_state,
            20,
        )
        self._feedback_sub = self.create_subscription(
            FlightCommand.FeedbackMessage,
            _FEEDBACK_TOPIC,
            self._on_feedback,
            20,
        )
        self._result_sub = self.create_subscription(
            FlightCommand.ResultMessage,
            _RESULT_TOPIC,
            self._on_result,
            20,
        )

        # Start display worker
        self._worker.start()

        if self._worker.is_headless:
            self.get_logger().info(
                "Mission display started in HEADLESS mode (no DISPLAY detected). "
                "Status will be logged periodically."
            )
        else:
            self.get_logger().info(
                "Mission display started with OpenCV window. "
                "Press Q/ESC to quit, S to save screenshot."
            )

        # Periodic check for quit request from display thread
        self._quit_timer = self.create_timer(0.5, self._check_quit)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _rotate_frame(self, frame: np.ndarray, role: str) -> np.ndarray:
        """Rotate image for display only (image-top -> nose orientation)."""
        if role == "narrow":
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def _build_snapshot(self) -> _DisplaySnapshot:
        return _DisplaySnapshot(
            narrow_frame=self._latest_frames["narrow"],
            wide_frame=self._latest_frames["wide"],
            mission_state=self._mission_state,
            mission_reason=self._mission_reason,
            mission_complete=self._mission_complete,
            target_valid=self._target_valid,
            target_x_m=self._target_x_m,
            target_y_m=self._target_y_m,
            target_z_m=self._target_z_m,
            target_confidence=self._target_confidence,
            target_quality=self._target_quality,
            fcu_altitude_m=self._fcu_altitude_m,
            fcu_battery_v=self._fcu_battery_v,
            fcu_mode=self._fcu_mode,
            fcu_armed=self._fcu_armed,
            fcu_comm_ok=self._fcu_comm_ok,
            fcu_position_m=(self._fcu_pos_x, self._fcu_pos_y),
            task3_control_allowed=self._task3_control_allowed,
            emergency_lock_active=self._emergency_lock_active,
            fcu_cmd_id=self._fcu_cmd_id,
            fcu_cmd_state=self._fcu_cmd_state,
            fcu_cmd_active=self._fcu_cmd_active,
            fcu_cmd_result=self._fcu_cmd_result,
            fcu_cmd_result_reason=self._fcu_cmd_result_reason,
            fps=self._fps,
        )

    def _on_image(self, msg: Image, role: str) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        self._latest_frames[role] = self._rotate_frame(frame, role)

        # Update FPS (any camera)
        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_start
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start = now

        self._worker.submit(self._build_snapshot())

    def _on_target_observation(self, msg: TargetObservation) -> None:
        self._target_valid = bool(msg.valid and msg.status == TargetObservation.STATUS_VALID)
        if self._target_valid:
            pos = msg.pose.pose.position
            self._target_x_m = float(pos.x)
            self._target_y_m = float(pos.y)
            self._target_z_m = float(pos.z)
        self._target_confidence = float(msg.confidence)
        self._target_quality = float(msg.quality)

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self._mission_state = _STATE_LABELS.get(msg.state, f"UNKNOWN({msg.state})")
        self._mission_reason = msg.reason
        self._mission_complete = msg.complete

    def _on_fcu_state(self, msg: FcuState) -> None:
        self._fcu_altitude_m = msg.altitude_m
        self._fcu_battery_v = msg.battery_voltage_v
        self._fcu_mode = _FCU_MODE_LABELS.get(msg.mode, f"MODE({msg.mode})")
        self._fcu_armed = msg.motors_armed
        self._fcu_comm_ok = msg.communication_ok
        self._fcu_pos_x = msg.optical_flow_position_m.x
        self._fcu_pos_y = msg.optical_flow_position_m.y
        self._task3_control_allowed = msg.task3_control_allowed
        self._emergency_lock_active = msg.emergency_lock_active

    def _on_feedback(self, msg: FlightCommand.FeedbackMessage) -> None:
        fb = msg.feedback
        self._fcu_cmd_id = fb.correlation_id
        self._fcu_cmd_state = _CMD_STATE_LABELS.get(
            fb.execution_state, f"STATE({fb.execution_state})"
        )
        self._fcu_cmd_active = fb.execution_state != FlightCommand.Feedback.STATE_TERMINAL

    def _on_result(self, msg: FlightCommand.ResultMessage) -> None:
        res = msg.result
        self._fcu_cmd_result = _CMD_RESULT_LABELS.get(
            res.result_code, f"CODE({res.result_code})"
        )
        self._fcu_cmd_result_reason = res.reason
        self._fcu_cmd_active = False

    def _check_quit(self) -> None:
        if self._worker.quit_requested.is_set():
            self.get_logger().info("Display quit requested — shutting down display node")
            self.destroy_node()

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:  # noqa: ANN201 — override
        self._worker.stop(timeout=1.0)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionDisplayNode()
    ros_thread: Thread | None = None
    try:
        if node._worker.is_headless:
            rclpy.spin(node)
        else:
            # 显示与处理解耦: ROS 回调在独立线程全速运行 (订阅/队列提交
            # 不被慢速 X11 渲染拖累); 主线程只负责取帧渲染。
            ros_thread = Thread(
                target=rclpy.spin, args=(node,), daemon=True,
                name="mission-display-ros",
            )
            ros_thread.start()
            _spin_with_display(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if ros_thread is not None:
            ros_thread.join(timeout=2.0)


def _spin_with_display(node: MissionDisplayNode) -> None:
    """Drive the OpenCV window from the main thread — GTK3 requires it.

    Mirrors field_test_node._spin_with_display; the display worker thread is
    not started in display mode, so cv2.imshow/cv2.waitKey stay on the thread
    that created the window.
    """
    worker = node._worker
    os.environ.setdefault("GDK_SCALE", "1")
    os.environ.setdefault("GTK_CSD", "0")
    os.environ.setdefault("GDK_BACKEND", "x11")
    cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WINDOW_NAME, 1280, 540)

    while rclpy.ok() and not worker.quit_requested.is_set():
        rclpy.spin_once(node, timeout_sec=0.0)

        try:
            snap = worker._queue.get_nowait()
        except Empty:
            snap = None

        if snap is not None:
            display = _render_layout(snap, worker.max_width)
            cv2.imshow(_WINDOW_NAME, display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            worker.quit_requested.set()
        elif key == ord("s") and snap is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = f"/tmp/task3_display_{ts}.png"
            cv2.imwrite(path, display)

    try:
        cv2.destroyWindow(_WINDOW_NAME)
    except Exception:
        pass


if __name__ == "__main__":
    main()
