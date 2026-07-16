"""
test_mcu_serial.py — 串口通信模块测试
验证 Section 3: 树莓派与MCU串口通信协议
"""

import sys
import os
import struct
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcu_serial import MCUSerial, DummySerial
from lx_protocol import (
    build_lx_frame,
    cmd_unlock, cmd_takeoff, cmd_move,
)


class TestMcuSerialInit(unittest.TestCase):
    """测试MCUSerial初始化"""

    def test_init_dry_run(self):
        mcu = MCUSerial(dry_run=True)
        self.assertTrue(mcu.dry_run)
        self.assertIsNone(mcu._ser)

    def test_connect_dry_run(self):
        mcu = MCUSerial(dry_run=True)
        self.assertTrue(mcu.connect())
        self.assertIsInstance(mcu._ser, DummySerial)

    def test_initial_values(self):
        mcu = MCUSerial(dry_run=True)
        # 初始值
        self.assertEqual(mcu.read_altitude(), 0)
        dx, dy = mcu.read_optical_flow()
        self.assertEqual(dx, 0.0)
        self.assertEqual(dy, 0.0)

    def test_real_link_has_no_synthetic_aux6_default(self):
        mcu = MCUSerial(dry_run=False)
        self.assertEqual(mcu.read_aux6(), 0)
        self.assertFalse(mcu.has_rc_channels())


class TestMcuSerialTxRx(unittest.TestCase):
    """测试串口发送接收"""

    def setUp(self):
        self.mcu = MCUSerial(dry_run=True)
        self.mcu.connect()

    def test_send_forward_frame(self):
        """测试发送指令转发帧"""
        imu_frame = cmd_unlock()
        result = self.mcu.send_imu_frame(imu_frame)
        self.assertTrue(result)

    def test_send_query_commands(self):
        """原生V7遥测无需查询；旧接口不应写入BB帧。"""
        before = self.mcu._tx_count
        self.assertTrue(self.mcu.send_query(0x01))
        self.assertTrue(self.mcu.send_query(0x02))
        self.assertEqual(self.mcu._tx_count, before)

    def test_of_position_injection(self):
        """测试光流数据注入与读取"""
        # 模拟V7 0x08位置偏移遥测帧
        pos_x = 1234
        pos_y = -567
        buf = build_lx_frame(
            0xFF, 0x08, struct.pack('<ii', pos_x, pos_y),
        )

        self.mcu._ser.inject_response(buf)
        self.mcu.poll()

        # 验证解析结果
        x, y, q = self.mcu.read_optical_flow_position()
        self.assertEqual(x, pos_x, f"pos_x: expected {pos_x}, got {x}")
        self.assertEqual(y, pos_y)
        self.assertEqual(q, 255)

    def test_of_incremental(self):
        """测试光流增量计算"""
        # 第一次注入: pos=(100, 0)
        buf1 = build_lx_frame(0xFF, 0x08, struct.pack('<ii', 100, 0))
        self.mcu._ser.inject_response(buf1)
        self.mcu.poll()

        dx, dy = self.mcu.read_optical_flow()
        self.assertEqual(dx, 100.0)
        self.assertEqual(dy, 0.0)

        # 第二次注入: pos=(120, -5)
        buf2 = build_lx_frame(0xFF, 0x08, struct.pack('<ii', 120, -5))
        self.mcu._ser.inject_response(buf2)
        self.mcu.poll()

        dx, dy = self.mcu.read_optical_flow()
        self.assertEqual(dx, 20.0)
        self.assertEqual(dy, -5.0)

        # 第三次读取 (无新数据) 应为0
        dx, dy = self.mcu.read_optical_flow()
        self.assertEqual(dx, 0.0)
        self.assertEqual(dy, 0.0)

    def test_flight_status_injection(self):
        """测试飞行状态数据注入"""
        status = build_lx_frame(0xFF, 0x06, bytes([3, 0, 0, 0, 0]))
        altitude = build_lx_frame(
            0xFF, 0x05, struct.pack('<iiB', 150, 0, 1),
        )
        self.mcu._ser.inject_response(status + altitude)
        self.mcu.poll()

        self.assertEqual(self.mcu.read_mode(), 3)
        self.assertEqual(self.mcu.read_locked(), 0)
        self.assertEqual(self.mcu.read_altitude(), 150)
        altitude_value, sequence, age = self.mcu.read_altitude_sample()
        self.assertEqual(altitude_value, 150)
        self.assertGreater(sequence, 0)
        self.assertLess(age, 1.0)
        self.assertTrue(self.mcu.has_recent_altitude())

    def test_garbage_data_handling(self):
        """测试垃圾数据不会导致崩溃"""
        garbage = bytes([0xFF, 0xFF, 0xAA, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
        self.mcu._ser.inject_response(garbage)
        # 不应抛异常
        self.mcu.poll()

    def test_mixed_data_parsing(self):
        """测试混合数据解析 (有效帧夹杂垃圾)"""
        # 注入垃圾 + 有效帧
        garbage = bytes([0xFF, 0x01, 0x02])
        valid = build_lx_frame(0xFF, 0x08, struct.pack('<ii', 500, 300))
        self.mcu._ser.inject_response(garbage + valid + garbage)
        self.mcu.poll()

        x, y, q = self.mcu.read_optical_flow_position()
        self.assertEqual(x, 500)
        self.assertEqual(y, 300)
        self.assertEqual(q, 255)

    def test_native_v7_command_is_not_wrapped(self):
        self.mcu.send_imu_frame(cmd_unlock())
        written = bytes(self.mcu._ser._buf)
        self.assertEqual(written, cmd_unlock())

    def test_parses_voltage_and_aux6(self):
        voltage = build_lx_frame(0xFF, 0x0D, struct.pack('<HH', 1240, 0))
        channels = [1500] * 10
        channels[9] = 1800
        rc = build_lx_frame(0xFF, 0x40, struct.pack('<10h', *channels))
        self.mcu._ser.inject_response(voltage + rc)
        self.mcu.poll()
        self.assertAlmostEqual(self.mcu.read_voltage(), 12.4)
        self.assertEqual(self.mcu.read_aux6(), 1800)
        self.assertTrue(self.mcu.has_rc_channels())

    def test_rejects_bad_v7_checksum(self):
        frame = bytearray(build_lx_frame(0xFF, 0x06, bytes([3, 1, 0, 0, 0])))
        frame[-1] ^= 0xFF
        self.mcu._ser.inject_response(bytes(frame))
        self.mcu.poll()
        self.assertEqual(self.mcu.read_mode(), 0)
        self.assertGreater(self.mcu.get_stats()['checksum_errors'], 0)

    def test_buffer_overflow_prevention(self):
        """测试接收缓冲区溢出保护"""
        # 注入大量垃圾数据
        self.mcu._ser.inject_response(b'\xFF' * 2000)
        self.mcu.poll()
        # 缓冲区应被截断
        self.assertLessEqual(len(self.mcu._rx_buf), self.mcu.RX_BUF_SIZE * 2)


class TestMcuCommands(unittest.TestCase):
    """测试高级指令发送"""

    def setUp(self):
        self.mcu = MCUSerial(dry_run=True)
        self.mcu.connect()

    def test_unlock_lock(self):
        self.assertTrue(self.mcu.send_cmd_unlock())
        self.assertTrue(self.mcu.send_cmd_lock())

    def test_mode_switch(self):
        for mode in [0, 1, 2, 3]:
            self.assertTrue(self.mcu.send_cmd_mode(mode))

    def test_takeoff_land(self):
        self.assertTrue(self.mcu.send_cmd_takeoff(150))
        self.assertEqual(self.mcu.read_altitude(), 150)
        self.assertTrue(self.mcu.send_cmd_land())
        self.assertEqual(self.mcu.read_altitude(), 0)

    def test_move(self):
        self.assertTrue(self.mcu.send_cmd_move(100, 30, 90))
        dx, dy = self.mcu.read_optical_flow()
        self.assertAlmostEqual(dx, 0.0, delta=0.01)
        self.assertAlmostEqual(dy, 100.0, delta=0.01)

    def test_ascend_descend(self):
        self.assertTrue(self.mcu.send_cmd_ascend(100, 25))
        self.assertTrue(self.mcu.send_cmd_descend(50, 15))

    def test_zero_reset(self):
        self.assertTrue(self.mcu.send_of_zero_reset())


class TestMcuStats(unittest.TestCase):
    """测试通信统计"""

    def setUp(self):
        self.mcu = MCUSerial(dry_run=True)
        self.mcu.connect()

    def test_tx_count(self):
        self.mcu.send_cmd_unlock()
        self.assertEqual(self.mcu._tx_count, 1)
        self.mcu.send_cmd_takeoff(150)
        self.assertEqual(self.mcu._tx_count, 2)

    def test_stats_dict(self):
        stats = self.mcu.get_stats()
        self.assertIn('tx_count', stats)
        self.assertIn('rx_count', stats)
        self.assertIn('comm_errors', stats)


if __name__ == '__main__':
    unittest.main(verbosity=2)
