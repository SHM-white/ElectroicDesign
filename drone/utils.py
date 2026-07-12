"""
utils.py — 工具函数
"""

import time
import os
import logging
import threading
from typing import Any, Optional
from datetime import datetime


def setup_logging(log_dir: str = 'logs/', verbose: bool = False,
                  save_logs: bool = True) -> logging.Logger:
    """
    设置日志系统

    Args:
        log_dir: 日志目录
        verbose: 是否打印到控制台
        save_logs: 是否保存日志文件

    Returns:
        logger实例
    """
    logger = logging.getLogger('drone')
    logger.setLevel(logging.DEBUG)

    # 文件处理器
    if save_logs:
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(log_dir, f'drone_{timestamp}.log')
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s'
        ))
        logger.addHandler(fh)

    # 控制台处理器
    if verbose:
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(logging.Formatter(
            '[%(levelname)s] %(message)s'
        ))
        logger.addHandler(ch)

    return logger


class RateLimiter:
    """频率控制器"""

    def __init__(self, hz: float):
        self.interval = 1.0 / hz
        self.last_call = 0.0

    def wait(self):
        """阻塞直到满足频率间隔"""
        elapsed = time.time() - self.last_call
        if elapsed < self.interval:
            time.sleep(self.interval - elapsed)
        self.last_call = time.time()

    def reset(self):
        self.last_call = time.time()


class Timer:
    """秒表计时器"""

    def __init__(self):
        self._start = time.time()

    def elapsed(self) -> float:
        """已过时间(秒)"""
        return time.time() - self._start

    def reset(self):
        self._start = time.time()

    def has_expired(self, timeout_s: float) -> bool:
        """是否超时"""
        return self.elapsed() > timeout_s


class MovingAverage:
    """滑动平均滤波器"""

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.values = []

    def add(self, value: float) -> float:
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return self.average()

    def average(self) -> float:
        if not self.values:
            return 0.0
        return sum(self.values) / len(self.values)

    def reset(self):
        self.values.clear()


def clamp(value: float, min_val: float, max_val: float) -> float:
    """将值限制在 [min_val, max_val] 范围内"""
    return max(min_val, min(value, max_val))


def safe_int(value: Any, default: int = 0) -> int:
    """安全转换为整数"""
    try:
        return int(value)
    except (ValueError, TypeError):
        return default


def safe_float(value: Any, default: float = 0.0) -> float:
    """安全转换为浮点数"""
    try:
        return float(value)
    except (ValueError, TypeError):
        return default
