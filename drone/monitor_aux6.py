#!/usr/bin/env python3
"""持续监控凌霄IMU原生V7遥控帧中的AUX6通道。"""

import argparse
import signal
import time

if __package__:
    from .mcu_serial import MCUSerial
else:
    from mcu_serial import MCUSerial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='持续监控凌霄飞控 AUX6 通道')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='串口设备')
    parser.add_argument('--baudrate', type=int, default=500000, help='串口波特率')
    parser.add_argument('--interval', type=float, default=0.1,
                        help='输出间隔，单位秒（默认0.1）')
    parser.add_argument('--threshold', type=int, default=1700,
                        help='任务启动阈值（默认1700）')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mcu = MCUSerial(dry_run=False, port=args.port, baudrate=args.baudrate)
    if not mcu.connect():
        print(f'无法打开串口: {args.port}')
        return 1

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print(f'监控 AUX6: {args.port} @ {args.baudrate} bps')
    print(f'启动阈值: > {args.threshold}；按 Ctrl+C 退出')
    print('-' * 64)

    last_print = 0.0
    last_value = None
    minimum = None
    maximum = None

    try:
        while running:
            mcu.poll()
            now = time.monotonic()
            if not mcu.has_rc_channels():
                if now - last_print >= args.interval:
                    print('\r等待首个有效 V7 0x40 遥控帧...', end='', flush=True)
                    last_print = now
                time.sleep(0.002)
                continue
            value = mcu.read_aux6()
            minimum = value if minimum is None else min(minimum, value)
            maximum = value if maximum is None else max(maximum, value)

            if value != last_value or now - last_print >= args.interval:
                state = 'START' if value > args.threshold else 'IDLE '
                stats = mcu.get_stats()
                print(
                    f'\rAUX6={value:4d}  {state}  '
                    f'min={minimum:4d} max={maximum:4d}  '
                    f'V7={stats["valid_frames"]:7d}  '
                    f'age={stats["last_rx_age_ms"]:6.1f}ms',
                    end='',
                    flush=True,
                )
                last_value = value
                last_print = now

            time.sleep(0.002)
    finally:
        print('\n停止监控。')
        mcu.disconnect()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
