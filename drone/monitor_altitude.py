#!/usr/bin/env python3
"""只读持续监控凌霄 IMU 原生 V7 高度遥测。"""

import argparse
import signal
import time

if __package__:
    from .mcu_serial import MCUSerial
else:
    from mcu_serial import MCUSerial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='持续监控凌霄飞控高度遥测')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='串口设备')
    parser.add_argument('--baudrate', type=int, default=500000, help='串口波特率')
    parser.add_argument('--interval', type=float, default=0.1,
                        help='输出间隔，单位秒（默认0.1）')
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

    print(f'只读监控高度: {args.port} @ {args.baudrate} bps')
    print('不会发送解锁、起飞或其他控制命令；按 Ctrl+C 退出')
    print('-' * 88)

    last_print = 0.0
    last_sequence = 0
    minimum = None
    maximum = None

    try:
        while running:
            mcu.poll()
            now = time.monotonic()
            altitude, sequence, age = mcu.read_altitude_sample()

            if sequence == 0:
                if now - last_print >= args.interval:
                    stats = mcu.get_stats()
                    print(
                        f'\r等待首个有效 V7 0x05 高度帧... '
                        f'V7={stats["valid_frames"]:7d}  '
                        f'link_age={stats["last_rx_age_ms"]:7.1f}ms',
                        end='', flush=True,
                    )
                    last_print = now
                time.sleep(0.002)
                continue

            if sequence != last_sequence:
                minimum = altitude if minimum is None else min(minimum, altitude)
                maximum = altitude if maximum is None else max(maximum, altitude)
                last_sequence = sequence

            if now - last_print >= args.interval:
                stats = mcu.get_stats()
                freshness = 'FRESH' if age <= 1.0 else 'STALE'
                print(
                    f'\r高度={altitude:6d}cm  {freshness}  '
                    f'min={minimum:6d} max={maximum:6d}  '
                    f'alt_seq={sequence:7d} age={age * 1000:7.1f}ms  '
                    f'V7={stats["valid_frames"]:7d} '
                    f'link_age={stats["last_rx_age_ms"]:7.1f}ms',
                    end='', flush=True,
                )
                last_print = now

            time.sleep(0.002)
    finally:
        print('\n停止高度监控。')
        mcu.disconnect()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
