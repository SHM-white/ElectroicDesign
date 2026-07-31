#!/usr/bin/env python3
"""
test_electromagnet.py — 电磁铁周期开关测试（间隔 5 秒）

沿用 C 板（STM32H7 大疆电机开发板C）旧激光头控制协议，通过 USB-TTL
串口发送 GPIO 命令帧控制电磁铁吸合/释放：

    Command:  0xAA [PIN] [CMD=0x01 SET_OUTPUT] [LEN=1] [VALUE] [XOR]
    Response: 0xBB [PIN] [CMD] [STATUS] [XOR]

C 板固件无需改动，仅将控制引脚从激光头（01 脚）换成电磁铁所接引脚。

用法:
    python3 drone/test_electromagnet.py                    # 默认引脚2, 3个周期
    python3 drone/test_electromagnet.py --pin 1            # 指定H7引脚(0-15)
    python3 drone/test_electromagnet.py --port /dev/ttyUSB1
    python3 drone/test_electromagnet.py --interval 5.0 --cycles 5
    python3 drone/test_electromagnet.py --cycles 0         # 无限循环, Ctrl+C停止
    python3 drone/test_electromagnet.py --dry-run          # 模拟模式(不连硬件)
"""

import argparse
import logging
import sys
import time
from datetime import datetime

# 兼容两种运行方式: 仓库根目录 python3 drone/test_electromagnet.py
# 与 drone/ 目录内 python3 test_electromagnet.py
try:
    from h7_gpio_protocol import cmd_set_output
    from h7_gpio_serial import H7GpioSerial
except ImportError:
    from .h7_gpio_protocol import cmd_set_output
    from .h7_gpio_serial import H7GpioSerial

logger = logging.getLogger('electromagnet_test')

# 默认参数
DEFAULT_PIN = 1          # H7 协议引脚编号; 激光头与电磁铁同脚 (01 脚), C板已封装PWM, 一般无需修改
DEFAULT_PORT = '/dev/ttyUSB1'   # 与 config.H7_SERIAL_PORT 一致
DEFAULT_BAUDRATE = 115200      # 与 config.H7_SERIAL_BAUDRATE 一致
DEFAULT_INTERVAL_S = 5.0       # 开关间隔 (秒)
DEFAULT_CYCLES = 3             # 完整周期数 (吸合+释放=1个周期), 0=无限


def set_electromagnet(serial: H7GpioSerial, pin: int, high: bool,
                      tag: str = '') -> bool:
    """
    通过 H7 串口协议设置电磁铁电平。

    Args:
        serial: 已连接的 H7GpioSerial
        pin: H7 协议引脚编号
        high: True=吸合(高电平), False=释放(低电平)
        tag: 日志标签 (如 init/cycle1/cleanup)

    Returns:
        True 帧发送成功 (不含响应状态判断, 响应可能因固件回执超时)
    """
    frame = cmd_set_output(pin, high)
    ok = serial.send_frame(frame)
    resp = serial.read_response(timeout_s=0.5)

    if resp is None:
        status = "无响应(超时)"
    else:
        status = f"status=0x{resp['status']:02X}"

    state = "吸合(ON)" if high else "释放(OFF)"
    ts = datetime.now().strftime('%H:%M:%S')
    logger.info("[%s] %-8s 电磁铁 -> %s | 帧=%s | %s",
                ts, f"[{tag}]", state, frame.hex().upper(), status)
    return ok


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='电磁铁周期开关测试 (间隔5秒, 沿用C板旧激光头串口协议)')
    parser.add_argument('--pin', type=int, default=DEFAULT_PIN,
                        help=f'H7 GPIO 引脚编号 0-15 (默认 {DEFAULT_PIN}, 按电磁铁实际接线修改)')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'串口设备路径 (默认 {DEFAULT_PORT})')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE,
                        help=f'波特率 (默认 {DEFAULT_BAUDRATE})')
    parser.add_argument('--interval', type=float, default=DEFAULT_INTERVAL_S,
                        help=f'开关间隔秒数 (默认 {DEFAULT_INTERVAL_S})')
    parser.add_argument('--cycles', type=int, default=DEFAULT_CYCLES,
                        help=f'完整周期数, 0=无限循环 (默认 {DEFAULT_CYCLES})')
    parser.add_argument('--dry-run', action='store_true',
                        help='模拟模式: 使用 DummySerial, 不连接真实硬件')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='输出 DEBUG 级日志')
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)s [%(name)s] %(message)s',
    )

    if not (0 <= args.pin <= 15):
        logger.error(f"引脚必须在 0-15 之间, 收到: {args.pin}")
        return 1

    serial = H7GpioSerial(dry_run=args.dry_run, port=args.port,
                          baudrate=args.baudrate)
    if not serial.connect():
        logger.error(f"串口连接失败: {args.port}")
        return 1

    logger.info(f"串口已连接: {args.port} @ {args.baudrate}bps (dry_run={args.dry_run})")
    logger.info(f"测试参数: pin={args.pin}, interval={args.interval}s, "
                f"cycles={'无限' if args.cycles == 0 else args.cycles}")

    cycle = 0
    try:
        # 初始置为释放态, 确保上电默认不吸合
        set_electromagnet(serial, args.pin, False, 'init')
        time.sleep(0.5)

        while True:
            cycle += 1
            if args.cycles and cycle > args.cycles:
                break

            logger.info("══════ 周期 %d ══════", cycle)
            # 吸合
            set_electromagnet(serial, args.pin, True, f'cycle{cycle}')
            time.sleep(args.interval)
            # 释放
            set_electromagnet(serial, args.pin, False, f'cycle{cycle}')

            if args.cycles and cycle >= args.cycles:
                break
            time.sleep(args.interval)

        logger.info(f"测试完成, 共 {cycle} 个周期。")
        return 0

    except KeyboardInterrupt:
        logger.warning("收到 Ctrl+C, 停止测试")
        return 0
    finally:
        # 无论正常结束还是中断, 确保电磁铁释放
        try:
            set_electromagnet(serial, args.pin, False, 'cleanup')
        except Exception:
            pass
        serial.disconnect()
        logger.info("串口已断开")


if __name__ == '__main__':
    sys.exit(main())
