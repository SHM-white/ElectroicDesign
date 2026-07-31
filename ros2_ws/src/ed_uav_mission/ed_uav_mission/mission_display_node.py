"""Mission display node — real-time annotated camera feed with HUD overlays.

Subscribes to the annotated image from the perception pipeline and mission
status topics, then renders a heads-up display overlay with:
  - AprilTag target detection bounding boxes (from perception pipeline)
  - Planned movement direction arrow (from target observation pose)
  - Key mission parameters as floating text (top-left HUD panel)

Uses a dedicated daemon thread for cv2.imshow to prevent slow X11 rendering
from blocking the main mission loop (critical for remote SSH sessions).

Threading model follows the _PreviewWorker pattern from the deprecated
drone/vision.py: daemon thread + Queue(maxsize=1) + non-blocking submit.
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
from ed_uav_interfaces.msg import FcuState, MissionStatus, TargetObservation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

_WINDOW_NAME: Final = "Task3 Mission Display — [Q/ESC] quit [S] screenshot"

# ── HUD color palette (BGR) ────────────────────────────────────────────────
_COLOR_WHITE: Final = (255, 255, 255)
_COLOR_GREEN: Final = (60, 220, 60)
_COLOR_YELLOW: Final = (0, 220, 255)
_COLOR_RED: Final = (60, 60, 240)
_COLOR_ORANGE: Final = (0, 165, 255)
_COLOR_CYAN: Final = (255, 200, 0)
_COLOR_PANEL_BG: Final = (12, 18, 24)
_COLOR_ARROW: Final = (0, 255, 200)
_COLOR_TARGET_BOX: Final = (0, 255, 0)
_COLOR_TARGET_CENTER: Final = (0, 0, 255)

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
}

# ── FCU mode label mapping ─────────────────────────────────────────────────
_FCU_MODE_LABELS: Final[dict[int, str]] = {
    FcuState.MODE_STABILIZE: "STAB",
    FcuState.MODE_ALTITUDE_HOLD: "ALT_HOLD",
    FcuState.MODE_POSITION_HOLD: "POS_HOLD",
    FcuState.MODE_PROGRAM: "PROGRAM",
}


# ── Frame snapshot for the display thread ──────────────────────────────────

@dataclass(frozen=True)
class _DisplaySnapshot:
    """Immutable snapshot of one frame plus all HUD metadata."""

    frame: np.ndarray
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
            self._thread.start()

    def submit(self, snapshot: _DisplaySnapshot) -> None:
        """Non-blocking overwrite — always tracks the latest frame."""
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
                # Import here to avoid circular dependency at module level
                import logging
                logger = logging.getLogger("ed_uav_mission.display")
                logger.info(
                    "[DISPLAY] state=%s alt=%.1fm bat=%.1fV tgt=%s "
                    "pos=(%.2f,%.2f)m qual=%.3f fps=%.1f",
                    snap.mission_state,
                    snap.fcu_altitude_m,
                    snap.fcu_battery_v,
                    "VALID" if snap.target_valid else "NONE",
                    snap.fcu_position_m[0],
                    snap.fcu_position_m[1],
                    snap.target_quality,
                    snap.fps,
                )

    def _run_display(self) -> None:
        """Display mode — cv2.imshow in daemon thread."""
        try:
            cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(_WINDOW_NAME, 960, 540)
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

            display = _render_hud(snap)

            # Scale down for remote sessions
            if 0 < self.max_width < display.shape[1]:
                scale = self.max_width / display.shape[1]
                display = cv2.resize(
                    display,
                    (self.max_width, max(1, int(round(display.shape[0] * scale)))),
                    interpolation=cv2.INTER_AREA,
                )

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

def _render_hud(snap: _DisplaySnapshot) -> np.ndarray:
    """Render the full HUD overlay onto a copy of the frame."""
    out = snap.frame.copy()
    h, w = out.shape[:2]
    scale = max(0.5, min(1.0, w / 1280.0))
    line = max(1, int(round(2 * scale)))

    # ── Target detection circle + center marker ──
    if snap.target_valid:
        cx, cy = w // 2, h // 2
        # Draw target offset arrow from center
        # target_x_m: right positive (optical X), target_y_m: down positive (optical Y)
        # Scale: roughly 200px per meter at typical altitude
        arrow_scale_px = max(80, int(200 * scale))
        tx = int(cx + snap.target_x_m * arrow_scale_px)
        ty = int(cy + snap.target_y_m * arrow_scale_px)
        tx = max(0, min(w - 1, tx))
        ty = max(0, min(h - 1, ty))

        # Arrow from center toward target
        cv2.arrowedLine(
            out, (cx, cy), (tx, ty),
            _COLOR_ARROW, max(2, int(3 * scale)),
            cv2.LINE_AA, tipLength=0.15,
        )
        # Target circle at the end of arrow
        cv2.circle(out, (tx, ty), max(12, int(20 * scale)), _COLOR_TARGET_CENTER, line + 1, cv2.LINE_AA)
        cv2.drawMarker(
            out, (tx, ty), _COLOR_TARGET_CENTER,
            cv2.MARKER_TILTED_CROSS, max(16, int(28 * scale)), line,
        )
        # Distance label
        dist_m = snap.target_z_m
        cv2.putText(
            out, f"D={dist_m:.2f}m",
            (tx + 14, max(20, ty - 14)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, _COLOR_TARGET_CENTER, line, cv2.LINE_AA,
        )

    # ── Top-left HUD panel ──
    hud_lines = _build_hud_lines(snap)
    panel_h = int((24 + len(hud_lines) * 26) * scale)
    panel_w = min(w - 8, max(320, int(480 * scale)))

    panel_overlay = out.copy()
    cv2.rectangle(panel_overlay, (4, 4), (4 + panel_w, 4 + panel_h), _COLOR_PANEL_BG, -1)
    out = cv2.addWeighted(panel_overlay, 0.80, out, 0.20, 0)

    for i, (text, color) in enumerate(hud_lines):
        y_pos = int((22 + i * 26) * scale)
        cv2.putText(
            out, text, (12, y_pos),
            cv2.FONT_HERSHEY_SIMPLEX, 0.50 * scale, color, line, cv2.LINE_AA,
        )

    # ── Bottom status bar ──
    bottom_text = "[Q/ESC] quit  [S] save"
    if snap.mission_complete:
        bottom_text += "  *** MISSION COMPLETE ***"
    cv2.putText(
        out, bottom_text, (8, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42 * scale, (200, 200, 200), line,
    )

    return out


def _build_hud_lines(snap: _DisplaySnapshot) -> list[tuple[str, tuple[int, int, int]]]:
    """Build (text, color) pairs for the HUD panel."""
    lines: list[tuple[str, tuple[int, int, int]]] = []

    # Mission state
    state_color = _COLOR_WHITE
    if snap.mission_state in ("SUCCEEDED",):
        state_color = _COLOR_GREEN
    elif snap.mission_state in ("ABORTED",):
        state_color = _COLOR_RED
    elif snap.mission_state in ("TAKEOFF", "SEARCHING"):
        state_color = _COLOR_YELLOW
    lines.append((f"STATE  {snap.mission_state}", state_color))

    if snap.mission_reason:
        lines.append((f"  >> {snap.mission_reason[:48]}", (180, 180, 180)))

    # FCU status
    armed_str = "ARMED" if snap.fcu_armed else "DISARMED"
    comm_str = "OK" if snap.fcu_comm_ok else "LOST"
    lines.append((
        f"FCU  {snap.fcu_mode}  {armed_str}  COMM={comm_str}",
        _COLOR_GREEN if snap.fcu_comm_ok else _COLOR_RED,
    ))

    # Altitude & battery — color reflects the most critical warning
    if snap.fcu_altitude_m <= 0.3 or snap.fcu_battery_v <= 10.5:
        telemetry_color = _COLOR_RED
    elif snap.fcu_battery_v <= 11.0:
        telemetry_color = _COLOR_YELLOW
    else:
        telemetry_color = _COLOR_GREEN
    lines.append((
        f"ALT  {snap.fcu_altitude_m:.2f}m   "
        f"BAT  {snap.fcu_battery_v:.1f}V",
        telemetry_color,
    ))

    # Position (optical flow)
    lines.append((
        f"POS  X={snap.fcu_position_m[0]:+.2f}m  Y={snap.fcu_position_m[1]:+.2f}m",
        _COLOR_CYAN,
    ))

    # Target info
    if snap.target_valid:
        lines.append((
            f"TGT  X={snap.target_x_m:+.3f}m Y={snap.target_y_m:+.3f}m Z={snap.target_z_m:.3f}m",
            _COLOR_GREEN,
        ))
        lines.append((
            f"     conf={snap.target_confidence:.3f}  qual={snap.target_quality:.3f}",
            _COLOR_GREEN,
        ))
    else:
        lines.append(("TGT  NO TARGET", _COLOR_RED))

    # FPS
    lines.append((f"FPS  {snap.fps:.1f}", (170, 170, 170)))

    return lines


# ── ROS 2 Node ─────────────────────────────────────────────────────────────

class MissionDisplayNode(Node):
    """ROS 2 node that renders annotated camera frames with mission HUD.

    Subscribes:
      /d_task/target_observation/annotated_image  (sensor_msgs/Image)
      /d_task/mission_status                      (MissionStatus)
      /fcu/state                                  (FcuState)

    Displays a real-time OpenCV window with:
      - Perception pipeline's AprilTag annotations
      - Target offset direction arrow
      - Mission state HUD panel
      - FCU telemetry overlay
    """

    def __init__(self) -> None:
        super().__init__("mission_display")
        self.declare_parameter("max_display_width", 960)
        self.declare_parameter("headless_log_interval_sec", 5.0)

        max_width = int(self.get_parameter("max_display_width").value)
        headless_interval = float(self.get_parameter("headless_log_interval_sec").value)

        self._bridge = CvBridge()
        self._worker = _DisplayWorker(max_width, headless_interval)

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

        # FPS tracking
        self._frame_count: int = 0
        self._fps_start: float = time.monotonic()
        self._fps: float = 0.0

        # Subscriptions
        self._image_sub = self.create_subscription(
            Image,
            "/d_task/target_observation/annotated_image",
            self._on_image,
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

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        # Update FPS
        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_start
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start = now

        snap = _DisplaySnapshot(
            frame=frame,
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
            fps=self._fps,
        )
        self._worker.submit(snap)

    def _on_target_observation(self, msg: TargetObservation) -> None:
        """Update target pose from the perception pipeline."""
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

        # Extract target info from TargetObservation if available via mission status
        # (TargetObservation comes from the annotated image subscriber)

    def _on_fcu_state(self, msg: FcuState) -> None:
        self._fcu_altitude_m = msg.altitude_m
        self._fcu_battery_v = msg.battery_voltage_v
        self._fcu_mode = _FCU_MODE_LABELS.get(msg.mode, f"MODE({msg.mode})")
        self._fcu_armed = msg.motors_armed
        self._fcu_comm_ok = msg.communication_ok
        self._fcu_pos_x = msg.optical_flow_position_m.x
        self._fcu_pos_y = msg.optical_flow_position_m.y

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
