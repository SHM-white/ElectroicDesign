"""
config.py — 配置参数
Section 12.1: 三档速度配置 + 全局参数
"""

from typing import Dict, Any


# ── 三档速度配置 ──────────────────────────────────────────

SPEED_PROFILES: Dict[str, Dict[str, Any]] = {
    'debug': {
        'move_speed': 15,        # cm/s
        'ascent_speed': 15,      # cm/s
        'descent_speed': 10,     # cm/s
        'block_timeout': 10,     # 每块超时(秒)
        'laser_period': 2000,    # 激光闪烁周期(ms)
        'laser_count': 2,        # 闪烁次数
        'ocr_interval': 2,       # 每N个区块执行一次OCR校准(debug阶段更频繁)
    },
    'tuning': {
        'move_speed': 30,
        'ascent_speed': 25,
        'descent_speed': 15,
        'block_timeout': 5,
        'laser_period': 1500,
        'laser_count': 2,
        'ocr_interval': 3,
    },
    'competition': {
        'move_speed': 45,
        'ascent_speed': 30,
        'descent_speed': 25,
        'block_timeout': 5,
        'laser_period': 1200,
        'laser_count': 2,
        'ocr_interval': 4,
    },
}

# 当前使用的配置档位
CURRENT_PROFILE: str = 'debug'


# ── 飞行参数 ──────────────────────────────────────────────

TAKEOFF_HEIGHT_CM = 150          # 起飞目标高度(cm)
TAKEOFF_HEIGHT_TOLERANCE = 10    # 起飞高度容差(±cm)
TAKEOFF_TIMEOUT_S = 15           # 起飞超时(秒)

LAND_ALT_THRESHOLD_CM = 10       # 降落完成判定的高度阈值(cm)
LAND_TIMEOUT_S = 20              # 降落超时(秒)

MAX_MISSION_TIME_S = 360         # 任务总超时(秒) - 赛题要求

# 高度警戒
ALT_MIN_CM = 30                  # 最低安全高度
ALT_LOW_WARN_CM = 50             # 高度偏低警告
ALT_HIGH_WARN_CM = 250           # 高度偏高警告
ALT_MAX_CM = 300                 # 最高警戒高度
ALT_CRITICAL_LOW_CM = 10         # 临界低高度(触发紧急降落)

# 电机参数
UNLOCK_WAIT_S = 2                # 解锁后等待时间
MODE_SWITCH_WAIT_S = 1           # 模式切换后等待时间

# ── 定位融合参数 ──────────────────────────────────────────

GREEN_DROP_THRESHOLD = 0.4  # 绿色占比下降超过此值=跨边界
GREEN_HIGH = 0.6            # 绿色占比高=在区块内
GREEN_LOW = 0.2             # 绿色占比低=在灰色区域

OCR_INTERVAL_BLOCKS = 4     # 每N个区块尝试一次OCR绝对校准

BLOCK_SIZE_CM = 50          # 每块50cm×50cm

# ── 视觉参数 ──────────────────────────────────────────────

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_DEVICE_ID = 0

# 绿色HSV阈值 (现场微调参考值)
GREEN_HSV_LOWER = [35, 40, 40]
GREEN_HSV_UPPER = [85, 255, 255]

# 灰色HSV阈值
GRAY_HSV_LOWER = [0, 0, 180]
GRAY_HSV_UPPER = [180, 30, 255]

# 黑色HSV阈值 (边界线)
BLACK_HSV_LOWER = [0, 0, 0]
BLACK_HSV_UPPER = [180, 255, 50]

# 最小轮廓面积过滤
MIN_CONTOUR_AREA = 500

# ── 路径参数 ──────────────────────────────────────────────

# 作业区原点到起降点偏移
ORIGIN_OFFSET_X_CM = 100
ORIGIN_OFFSET_Y_CM = 100

# ── 硬件引脚 ──────────────────────────────────────────────

LASER_PIN = 17    # 激光笔GPIO (BCM编号)
LED_PIN = 27      # LED指示灯GPIO (BCM编号)

# 串口
SERIAL_PORT = '/dev/ttyUSB0'  # x86 USB-TTL默认; 树莓派用 /dev/serial0
SERIAL_BAUDRATE = 115200

# STM32H7 GPIO 开发板串口 (大疆电机开发板C)
H7_SERIAL_PORT = '/dev/ttyUSB1'
H7_SERIAL_BAUDRATE = 115200

# ── 返航参数 ──────────────────────────────────────────────

RETURN_HOME_SPEED_CMPS = 30  # 返航水平速度(cm/s)

# ── 异常处理 ──────────────────────────────────────────────

COMM_TIMEOUT_MS = 500         # MCU通信超时(毫秒)
COMM_MAX_RETRIES = 3           # 最大重试次数
MAX_BOUNDARY_MISSES = 3        # 连续边界检测失败上限
LOW_VOLTAGE_THRESHOLD = 10.5   # 3S电池低压阈值(V)
FLOW_LOST_TIMEOUT_S = 5        # 光流失锁超时(秒)

# ── 调试模式 ──────────────────────────────────────────────

DRY_RUN = False                # True=仅模拟不实际飞行
SAVE_LOGS = True               # 是否保存日志
LOG_DIR = 'logs/'


def get_config() -> Dict[str, Any]:
    """获取当前档位的完整配置"""
    cfg = dict(SPEED_PROFILES[CURRENT_PROFILE])
    cfg.update({
        'takeoff_height_cm': TAKEOFF_HEIGHT_CM,
        'takeoff_height_tolerance': TAKEOFF_HEIGHT_TOLERANCE,
        'takeoff_timeout_s': TAKEOFF_TIMEOUT_S,
        'land_alt_threshold_cm': LAND_ALT_THRESHOLD_CM,
        'land_timeout_s': LAND_TIMEOUT_S,
        'max_mission_time_s': MAX_MISSION_TIME_S,
        'alt_min_cm': ALT_MIN_CM,
        'alt_low_warn_cm': ALT_LOW_WARN_CM,
        'alt_high_warn_cm': ALT_HIGH_WARN_CM,
        'alt_max_cm': ALT_MAX_CM,
        'alt_critical_low_cm': ALT_CRITICAL_LOW_CM,
        'unlock_wait_s': UNLOCK_WAIT_S,
        'mode_switch_wait_s': MODE_SWITCH_WAIT_S,
        'green_drop_threshold': GREEN_DROP_THRESHOLD,
        'green_high': GREEN_HIGH,
        'green_low': GREEN_LOW,
        'ocr_interval_blocks': OCR_INTERVAL_BLOCKS,
        'block_size_cm': BLOCK_SIZE_CM,
        'camera_width': CAMERA_WIDTH,
        'camera_height': CAMERA_HEIGHT,
        'camera_fps': CAMERA_FPS,
        'camera_device_id': CAMERA_DEVICE_ID,
        'green_hsv_lower': GREEN_HSV_LOWER,
        'green_hsv_upper': GREEN_HSV_UPPER,
        'gray_hsv_lower': GRAY_HSV_LOWER,
        'gray_hsv_upper': GRAY_HSV_UPPER,
        'black_hsv_lower': BLACK_HSV_LOWER,
        'black_hsv_upper': BLACK_HSV_UPPER,
        'min_contour_area': MIN_CONTOUR_AREA,
        'origin_offset_x_cm': ORIGIN_OFFSET_X_CM,
        'origin_offset_y_cm': ORIGIN_OFFSET_Y_CM,
        'laser_pin': LASER_PIN,
        'led_pin': LED_PIN,
        'serial_port': SERIAL_PORT,
        'serial_baudrate': SERIAL_BAUDRATE,
        'return_home_speed_cmps': RETURN_HOME_SPEED_CMPS,
        'comm_timeout_ms': COMM_TIMEOUT_MS,
        'comm_max_retries': COMM_MAX_RETRIES,
        'max_boundary_misses': MAX_BOUNDARY_MISSES,
        'low_voltage_threshold': LOW_VOLTAGE_THRESHOLD,
        'flow_lost_timeout_s': FLOW_LOST_TIMEOUT_S,
        'dry_run': DRY_RUN,
        'save_logs': SAVE_LOGS,
        'log_dir': LOG_DIR,
    })
    return cfg
