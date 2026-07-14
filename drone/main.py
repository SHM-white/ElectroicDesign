#!/usr/bin/env python3
"""
main.py — 无人机自主任务控制器入口
G_植保飞行器项目 | x86迷你主机 + 凌霄飞控 + 工业相机/OpenMV
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

if __package__:
    from . import config, path_plan
    from .config import get_config
    from .utils import setup_logging, RateLimiter
    from .mcu_serial import MCUSerial
    from .localization import Localizer
    from .laser_led import LaserController, DummyLaser
    from .gpio_backend import auto_detect_backend, Ft232hBackend, DummyGpioBackend
    from .state_machine import DroneStateMachine
else:
    import config
    import path_plan
    from config import get_config
    from utils import setup_logging, RateLimiter
    from mcu_serial import MCUSerial
    from localization import Localizer
    from laser_led import LaserController, DummyLaser
    from gpio_backend import auto_detect_backend, Ft232hBackend, DummyGpioBackend
    from state_machine import DroneStateMachine


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
        '--auto-start',
        action='store_true',
        help='连接完成后自动开始任务 (真实飞行请谨慎使用)',
    )
    parser.add_argument(
        '--no-camera',
        action='store_true',
        help='不启用视觉, 仅依赖预设地图和光流位置',
    )
    parser.add_argument(
        '--vision-backend',
        choices=['industrial', 'openmv'],
        default=None,
        help='视觉后端 (默认使用 config.py 中的 VISION_BACKEND)',
    )
    parser.add_argument(
        '--openmv-port',
        default=None,
        help='OpenMV识别结果串口 (默认使用 config.py 中的 OPENMV_SERIAL_PORT)',
    )
    parser.add_argument(
        '--openmv-baudrate',
        type=int,
        default=None,
        help='OpenMV串口波特率 (默认使用 config.py 中的配置)',
    )
    parser.add_argument(
        '--no-laser',
        action='store_true',
        help='不初始化激光输出',
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
    cfg['auto_start'] = args.auto_start
    path_plan.init_grid(
        cfg['origin_offset_x_cm'],
        cfg['origin_offset_y_cm'],
    )

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

    camera = None
    if not config.DRY_RUN and not args.no_camera:
        vision_backend = args.vision_backend or cfg['vision_backend']
        logger.info("  视觉后端:  %s", vision_backend)
        if vision_backend not in ('industrial', 'openmv'):
            logger.error("不支持的视觉后端: %s", vision_backend)
            mcu.disconnect()
            return 2

        if vision_backend == 'openmv':
            try:
                if __package__:
                    from .openmv_vision import OpenMVVision
                else:
                    from openmv_vision import OpenMVVision
            except ImportError as exc:
                logger.error(f"OpenMV串口依赖不可用: {exc}")
                logger.error("请安装 pyserial，或使用 --no-camera")
                mcu.disconnect()
                return 2

            openmv_port = args.openmv_port or cfg['openmv_serial_port']
            openmv_baudrate = (
                args.openmv_baudrate or cfg['openmv_serial_baudrate']
            )
            used_ports = {serial_port}
            if args.h7_serial:
                used_ports.add(args.h7_serial)
            if openmv_port in used_ports:
                logger.error("OpenMV串口不能与 MCU/H7 共用: %s", openmv_port)
                mcu.disconnect()
                return 2

            camera = OpenMVVision(
                port=openmv_port,
                baudrate=openmv_baudrate,
                stale_timeout_s=cfg['openmv_stale_timeout_s'],
            )
            if not camera.open():
                logger.error("OpenMV连接失败; 请检查串口、波特率和接线")
                mcu.disconnect()
                return 2
        else:
            try:
                if __package__:
                    from .vision import Camera, BlockDetector, DigitReader
                else:
                    from vision import Camera, BlockDetector, DigitReader
            except ImportError as exc:
                logger.error(f"工业相机视觉依赖不可用: {exc}")
                logger.error("请安装 opencv-python、numpy，或使用 --no-camera")
                mcu.disconnect()
                return 2
            camera = Camera(
                device_id=cfg['camera_device_id'],
                width=cfg['camera_width'],
                height=cfg['camera_height'],
                fps=cfg['camera_fps'],
            )
            camera.detector = BlockDetector(
                green_lower=cfg['green_hsv_lower'],
                green_upper=cfg['green_hsv_upper'],
                gray_lower=cfg['gray_hsv_lower'],
                gray_upper=cfg['gray_hsv_upper'],
                black_lower=cfg['black_hsv_lower'],
                black_upper=cfg['black_hsv_upper'],
                min_contour_area=cfg['min_contour_area'],
            )
            camera.digit_reader = DigitReader()
            if not camera.open():
                logger.error("工业相机打开失败; 可使用 --no-camera 仅测试飞控链路")
                mcu.disconnect()
                return 2

    localizer = Localizer(
        green_drop_threshold=cfg['green_drop_threshold'],
        green_high=cfg['green_high'],
        green_low=cfg['green_low'],
        ocr_interval=cfg['ocr_interval_blocks'],
        block_size_cm=cfg['block_size_cm'],
    )

    laser = None
    h7_serial = None
    if config.DRY_RUN:
        laser = DummyLaser()
    elif not args.no_laser:
        if args.h7_serial:
            # 使用 STM32H7 GPIO 开发板控制激光/LED
            try:
                from .h7_gpio_serial import H7GpioSerial
                from .gpio_backend import H7GpioBackend
            except ImportError:
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
                laser = LaserController(pin=cfg['h7_laser_pin'], backend=h7_backend)
            else:
                logger.error(f"H7 GPIO 开发板连接失败: {args.h7_serial}")
        else:
            gpio_backend = auto_detect_backend()
            laser_pin = (
                cfg['ft232h_laser_pin']
                if isinstance(gpio_backend, Ft232hBackend)
                else cfg['laser_pin']
            )
            if isinstance(gpio_backend, DummyGpioBackend):
                logger.warning("未检测到可用GPIO, 激光输出将仅记录日志")
            laser = LaserController(pin=laser_pin, backend=gpio_backend)

    # ── 创建状态机 ──────────────────────────────────────
    sm = DroneStateMachine(
        mcu=mcu,
        camera=camera,
        localizer=localizer,
        laser=laser,
        config=cfg,
    )

    if cfg['auto_start'] or config.DRY_RUN:
        logger.info("所有组件初始化完成, 自动开始任务")
    else:
        logger.info("所有组件初始化完成, 等待启动信号...")

    # ── 主循环 (20Hz) ───────────────────────────────────
    rate = RateLimiter(20)
    exit_code = 0
    last_heartbeat = 0.0
    last_flow_query = 0.0
    last_status_query = 0.0

    try:
        while not sm.is_completed:
            rate.wait()
            sm.run_iteration()
            # 每 500ms 发送一次心跳 (MCU 侧若 2s 无心跳则触发紧急降落)
            now = time.time()
            if now - last_heartbeat >= 0.5:
                mcu.send_heartbeat()
                last_heartbeat = now
            if not config.DRY_RUN and now - last_flow_query >= 0.1:
                mcu.send_query(0x01)
                last_flow_query = now
            if not config.DRY_RUN and now - last_status_query >= 0.2:
                mcu.send_query(0x02)
                last_status_query = now
    except KeyboardInterrupt:
        logger.info("Aborted by user (Ctrl+C)")
        mcu.send_cmd_land()
        exit_code = 130
    finally:
        # ── 清理资源 ────────────────────────────────────
        logger.info("正在清理资源...")
        if laser is not None:
            laser.cleanup()
        if h7_serial is not None:
            h7_serial.disconnect()
        if camera is not None:
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

        if sm.emergency_reason and exit_code == 0:
            exit_code = 2

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
