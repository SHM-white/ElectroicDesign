#!/usr/bin/env python3
"""
main.py — 无人机自主任务控制器入口
G_植保飞行器项目 | x86迷你主机 + 凌霄飞控 + USB-TTL串口 + 海康工业相机
(兼容树莓派4B + 硬件UART)

用法:
    python3 drone/main.py [选项]
    python3 -m drone.main [选项]

选项:
    --profile {debug,tuning,competition}  速度配置档位 (默认: debug)
    --dry-run                             模拟模式, 不实际飞行
    --verbose                             输出日志到控制台
    --no-save-logs                        不保存日志文件
    --serial-port PORT                    串口设备路径 (默认: /dev/ttyUSB0)
    -h, --help                            显示帮助信息
"""

import argparse
import sys
import time
import logging

import config
from config import get_config
from utils import setup_logging, RateLimiter
from mcu_serial import MCUSerial
from vision import Camera
from localization import Localizer
from laser_led import LaserController
from state_machine import DroneStateMachine
import path_plan


def _parse_args() -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='G_植保飞行器 — 无人机自主任务控制器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
    python3 main.py --profile competition --verbose
    python3 main.py --dry-run --verbose
    python3 -m drone.main --profile tuning
        """,
    )
    parser.add_argument(
        '--profile',
        choices=['debug', 'tuning', 'competition'],
        default='debug',
        help='速度配置档位 (默认: debug)',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='模拟模式, 仅模拟不实际飞行',
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='输出日志到控制台',
    )
    parser.add_argument(
        '--no-save-logs',
        action='store_true',
        help='不保存日志文件',
    )
    parser.add_argument(
        '--serial-port',
        default=None,
        help='串口设备路径 (默认使用 config.py 中的 SERIAL_PORT)',
    )
    parser.add_argument(
        '--h7-serial',
        default=None,
        help='STM32H7 GPIO 开发板串口路径 (如 /dev/ttyUSB1)',
    )
    return parser.parse_args()


def main() -> int:
    """主入口: 初始化组件并运行任务状态机"""
    args = _parse_args()

    # ── 应用配置 ────────────────────────────────────────
    config.CURRENT_PROFILE = args.profile
    if args.dry_run:
        config.DRY_RUN = True
    if args.no_save_logs:
        config.SAVE_LOGS = False

    # ── 日志 ────────────────────────────────────────────
    logger = setup_logging(
        log_dir=config.LOG_DIR,
        verbose=args.verbose,
        save_logs=config.SAVE_LOGS,
    )

    # ── 启动横幅 ────────────────────────────────────────
    logger.info("=" * 50)
    logger.info("  G_植保飞行器 — 无人机自主任务控制器")
    logger.info("=" * 50)
    logger.info(f"  配置档位:  {config.CURRENT_PROFILE}")
    logger.info(f"  模拟模式:  {config.DRY_RUN}")
    logger.info(f"  保存日志:  {config.SAVE_LOGS}")
    logger.info("=" * 50)

    cfg = get_config()

    # ── 初始化硬件组件 ──────────────────────────────────
    serial_port = args.serial_port or cfg['serial_port']
    logger.info(f"  串口设备:  {serial_port}")
    mcu = MCUSerial(
        dry_run=config.DRY_RUN,
        port=serial_port,
        baudrate=cfg['serial_baudrate'],
    )
    if not mcu.connect():
        logger.error("MCU连接失败")
        return 1

    camera = Camera(
        device_id=cfg['camera_device_id'],
        width=cfg['camera_width'],
        height=cfg['camera_height'],
        fps=cfg['camera_fps'],
    )
    if not config.DRY_RUN:
        camera.open()

    localizer = Localizer(
        green_drop_threshold=cfg['green_drop_threshold'],
        green_high=cfg['green_high'],
        green_low=cfg['green_low'],
        ocr_interval=cfg['ocr_interval_blocks'],
        block_size_cm=cfg['block_size_cm'],
    )

    laser = None
    h7_serial = None
    if not config.DRY_RUN:
        if args.h7_serial:
            # 使用 STM32H7 GPIO 开发板控制激光/LED
            from h7_gpio_serial import H7GpioSerial
            from gpio_backend import H7GpioBackend

            h7_serial = H7GpioSerial(
                dry_run=config.DRY_RUN,
                port=args.h7_serial,
                baudrate=config.H7_SERIAL_BAUDRATE,
            )
            if h7_serial.connect():
                logger.info(f"H7 GPIO 开发板已连接: {args.h7_serial}")
                h7_backend = H7GpioBackend(h7_serial)
                laser = LaserController(pin=cfg['laser_pin'], backend=h7_backend)
            else:
                logger.error(f"H7 GPIO 开发板连接失败: {args.h7_serial}")
        else:
            laser = LaserController(pin=cfg['laser_pin'])

    # ── 创建状态机 ──────────────────────────────────────
    sm = DroneStateMachine(
        mcu=mcu,
        camera=camera,
        localizer=localizer,
        laser=laser,
        config=cfg,
    )

    logger.info("所有组件初始化完成, 等待启动信号...")

    # ── 主循环 (20Hz) ───────────────────────────────────
    rate = RateLimiter(20)
    exit_code = 0
    last_heartbeat = 0.0

    try:
        while not (sm.is_completed or sm.is_emergency):
            rate.wait()
            sm.run_iteration()
            # 每 500ms 发送一次心跳 (MCU 侧若 2s 无心跳则触发紧急降落)
            now = time.time()
            if now - last_heartbeat >= 0.5:
                mcu.send_heartbeat()
                last_heartbeat = now
    except KeyboardInterrupt:
        logger.info("Aborted by user (Ctrl+C)")
        exit_code = 130
    finally:
        # ── 清理资源 ────────────────────────────────────
        logger.info("正在清理资源...")
        if laser is not None:
            laser.cleanup()
        if h7_serial is not None:
            h7_serial.disconnect()
        camera.release()
        mcu.disconnect()

        # ── 输出统计 ────────────────────────────────────
        stats = sm.get_stats()
        logger.info("─" * 40)
        logger.info("任务统计:")
        logger.info(f"  状态转换次数: {stats['total_state_transitions']}")
        logger.info(f"  任务用时:     {stats['mission_time_s']:.1f}s")
        logger.info(f"  完成区块数:   {stats['blocks_completed']}")
        logger.info("─" * 40)
        logger.info("任务结束")

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
