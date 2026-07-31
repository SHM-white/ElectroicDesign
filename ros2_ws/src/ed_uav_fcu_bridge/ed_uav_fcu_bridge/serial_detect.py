"""Automatic USB-TTL serial device identification for the ED UAV bridge.

扫描 /dev/ttyUSB* 设备，通过发送探测指令并解析响应来自动识别：
  - H7 GPIO 板 (STM32H7 大疆电机开发板C): 115200 baud, 0xAA → 0xBB 响应
  - 凌霄飞控 IMU: 500000 baud, 主动发送匿名 V7 协议遥测帧

典型用法 (launch 文件中):
    from ed_uav_fcu_bridge.serial_detect import detect_or_fallback
    devices = detect_or_fallback()
    # devices = {'h7_gpio': '/dev/ttyUSB1', 'fcu': '/dev/ttyUSB0'}
"""

from __future__ import annotations

import glob
import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

__all__ = (
    "DeviceType",
    "DetectedDevice",
    "DetectionResult",
    "detect_serial_devices",
    "detect_or_fallback",
)

logger = logging.getLogger("ed_uav_fcu_bridge.serial_detect")


# ── 公共类型 ──────────────────────────────────────────────

class DeviceType(Enum):
    """识别到的设备类型"""
    H7_GPIO = "h7_gpio"
    FCU = "fcu"
    OPENMV = "openmv"
    UNKNOWN = "unknown"


@dataclass
class DetectedDevice:
    """检测到的单个设备信息"""
    device_path: str
    device_type: DeviceType
    baudrate: int
    probe_time_ms: float = 0.0


@dataclass
class DetectionResult:
    """检测结果汇总"""
    devices: List[DetectedDevice] = field(default_factory=list)
    h7_gpio: Optional[str] = None
    fcu: Optional[str] = None
    openmv: Optional[str] = None

    def as_dict(self) -> Dict[str, str]:
        """返回角色→设备路径字典 (仅含已识别设备)"""
        result: Dict[str, str] = {}
        if self.h7_gpio:
            result["h7_gpio"] = self.h7_gpio
        if self.fcu:
            result["fcu"] = self.fcu
        if self.openmv:
            result["openmv"] = self.openmv
        return result


# ── H7 GPIO 探测 ─────────────────────────────────────────

_H7_PROBE_CMD = bytes([0xAA, 0x00, 0x02, 0x01, 0x00, 0x03])
"""CONFIGURE pin 0 as INPUT — 无副作用的探测命令.

帧: AA [PIN=0] [CMD=0x02 CONFIGURE] [LEN=1] [0x00 INPUT] [XOR=0x03]
"""


def _parse_h7_response(buf: bytes) -> bool:
    """检查缓冲区中是否包含有效的 H7 GPIO 响应 (0xBB)."""
    idx = buf.find(0xBB)
    if idx < 0 or len(buf) - idx < 5:
        return False
    frame = buf[idx : idx + 5]
    xor = frame[1] ^ frame[2] ^ frame[3]
    return xor == frame[4]


def _probe_h7_gpio(port, timeout_s: float = 0.5) -> bool:
    """发送 H7 GPIO 探测命令，检查是否有 0xBB 响应."""
    port.reset_input_buffer()
    port.write(_H7_PROBE_CMD)
    port.flush()

    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        port.timeout = max(remaining, 0.01)
        try:
            chunk = port.read(64)
            if chunk:
                buf.extend(chunk)
                if _parse_h7_response(buf):
                    return True
        except Exception:
            break
    return False


# ── 凌霄 V7 帧解析 ──────────────────────────────────────

def _verify_v7_frame(frame: bytes) -> bool:
    """验证匿名 V7 协议帧校验和 (SC + AC)."""
    if len(frame) < 6 or frame[0] != 0xAA:
        return False
    frame_len = frame[3] + 6
    if len(frame) < frame_len:
        return False
    expected_sc = frame[frame_len - 2]
    expected_ac = frame[frame_len - 1]
    actual_sc = 0
    actual_ac = 0
    for b in frame[: frame_len - 2]:
        actual_sc = (actual_sc + b) & 0xFF
        actual_ac = (actual_ac + actual_sc) & 0xFF
    return actual_sc == expected_sc and actual_ac == expected_ac


def _count_v7_frames(buf: bytes) -> int:
    """统计缓冲区中有效 V7 帧的数量."""
    valid = 0
    pos = 0
    while pos < len(buf):
        try:
            idx = buf.index(0xAA, pos)
        except ValueError:
            break
        pos = idx
        if len(buf) - pos < 4:
            break
        frame_len = buf[pos + 3] + 6
        if frame_len > 261:
            pos += 1
            continue
        if len(buf) - pos < frame_len:
            break
        frame = bytes(buf[pos : pos + frame_len])
        if _verify_v7_frame(frame):
            valid += 1
            pos += frame_len
        else:
            pos += 1
    return valid


def _probe_fcu(port, listen_s: float = 1.0) -> bool:
    """被动监听串口，检查是否收到有效的凌霄 V7 遥测帧."""
    port.reset_input_buffer()
    deadline = time.monotonic() + listen_s
    buf = bytearray()
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        port.timeout = max(remaining, 0.01)
        try:
            chunk = port.read(256)
            if chunk:
                buf.extend(chunk)
                if _count_v7_frames(buf) > 0:
                    return True
        except Exception:
            break
    return False


# ── 设备扫描与识别 ────────────────────────────────────────

def _scan_ttyusb() -> List[str]:
    """扫描 /dev/ttyUSB* 设备."""
    devices = sorted(glob.glob("/dev/ttyUSB*"))
    logger.info("扫描到 %d 个 /dev/ttyUSB* 设备: %s", len(devices), devices)
    return devices


def _try_open(device: str, baudrate: int, timeout: float = 0.1):
    """尝试打开串口，失败返回 None."""
    try:
        import serial

        port = serial.Serial(
            port=device,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=0.5,
        )
        time.sleep(0.1)
        return port
    except Exception as exc:
        logger.debug("无法打开 %s @ %d: %s", device, baudrate, exc)
        return None


def identify_device(device: str, probe_timeout: float = 0.5) -> DetectedDevice:
    """识别单个串口设备.

    检测顺序:
      1. 115200 baud → H7 GPIO 探测 (发送 CONFIGURE 命令)
      2. 500000 baud → 凌霄飞控被动监听 (V7 遥测帧)
    """
    start = time.monotonic()

    # ── 探测 1: H7 GPIO (115200) ──
    port = _try_open(device, 115200, timeout=0.1)
    if port is not None:
        try:
            for _ in range(2):
                if _probe_h7_gpio(port, timeout_s=probe_timeout):
                    elapsed = (time.monotonic() - start) * 1000
                    logger.info(
                        "✓ %s → H7 GPIO 板 (115200, %.0fms)",
                        device,
                        elapsed,
                    )
                    return DetectedDevice(device, DeviceType.H7_GPIO, 115200, elapsed)
        finally:
            port.close()

    # ── 探测 2: 凌霄飞控 (500000) ──
    port = _try_open(device, 500000, timeout=0.1)
    if port is not None:
        try:
            if _probe_fcu(port, listen_s=max(probe_timeout, 1.0)):
                elapsed = (time.monotonic() - start) * 1000
                logger.info(
                    "✓ %s → 凌霄飞控 (500000, %.0fms)",
                    device,
                    elapsed,
                )
                return DetectedDevice(device, DeviceType.FCU, 500000, elapsed)
        finally:
            port.close()

    elapsed = (time.monotonic() - start) * 1000
    logger.warning("✗ %s → 未知设备 (%.0fms)", device, elapsed)
    return DetectedDevice(device, DeviceType.UNKNOWN, 0, elapsed)


def detect_serial_devices(
    probe_timeout: float = 0.5,
    known_h7: Optional[str] = None,
    known_fcu: Optional[str] = None,
) -> DetectionResult:
    """自动检测所有 /dev/ttyUSB* 设备并识别类型.

    Args:
        probe_timeout: 每个设备的探测超时 (秒)
        known_h7: 已知 H7 GPIO 路径 (跳过探测)
        known_fcu: 已知飞控路径 (跳过探测)

    Returns:
        DetectionResult 包含各设备路径
    """
    result = DetectionResult()
    devices = _scan_ttyusb()
    if not devices:
        logger.error("未找到任何 /dev/ttyUSB* 设备")
        return result

    for device in devices:
        if device == known_h7:
            result.h7_gpio = device
            result.devices.append(DetectedDevice(device, DeviceType.H7_GPIO, 115200))
            logger.info("✓ %s → H7 GPIO 板 (已知路径)", device)
            continue
        if device == known_fcu:
            result.fcu = device
            result.devices.append(DetectedDevice(device, DeviceType.FCU, 500000))
            logger.info("✓ %s → 凌霄飞控 (已知路径)", device)
            continue

        detected = identify_device(device, probe_timeout=probe_timeout)
        result.devices.append(detected)
        if detected.device_type == DeviceType.H7_GPIO and result.h7_gpio is None:
            result.h7_gpio = detected.device_path
        elif detected.device_type == DeviceType.FCU and result.fcu is None:
            result.fcu = detected.device_path
        elif detected.device_type == DeviceType.OPENMV and result.openmv is None:
            result.openmv = detected.device_path

    logger.info("=" * 50)
    logger.info("  串口设备自动检测结果:")
    logger.info("  H7 GPIO 板: %s", result.h7_gpio or "未检测到")
    logger.info("  凌霄飞控:   %s", result.fcu or "未检测到")
    logger.info("  OpenMV:     %s", result.openmv or "未检测到")
    logger.info("=" * 50)
    return result


def detect_or_fallback(
    default_h7: str = "/dev/ttyUSB1",
    default_fcu: str = "/dev/ttyUSB0",
    probe_timeout: float = 0.5,
) -> Dict[str, str]:
    """自动检测设备，失败时使用默认路径.

    Returns:
        {'h7_gpio': '/dev/ttyUSB...', 'fcu': '/dev/ttyUSB...'}
    """
    result = detect_serial_devices(probe_timeout=probe_timeout)
    devices: Dict[str, str] = {
        "h7_gpio": result.h7_gpio or default_h7,
        "fcu": result.fcu or default_fcu,
    }
    if result.openmv:
        devices["openmv"] = result.openmv
    if not result.h7_gpio:
        logger.warning("H7 GPIO 板未检测到，使用默认: %s", default_h7)
    if not result.fcu:
        logger.warning("凌霄飞控未检测到，使用默认: %s", default_fcu)
    return devices
