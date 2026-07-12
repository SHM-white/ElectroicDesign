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
    parse_of_position, parse_flight_status,
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
        """测试发送查询命令"""
        for cmd in [0x01, 0x02, 0x03]:
            result = self.mcu.send_query(cmd)
            self.assertTrue(result)

    def test_of_position_injection(self):
        """测试光流数据注入与读取"""
        # 模拟MCU回传光流位置帧
        pos_x = 1234
        pos_y = -567
        quality = 200
        buf = bytes([0xCC, 0x01]) + struct.pack('<i', pos_x) + \
              struct.pack('<i', pos_y) + bytes([quality])

        self.mcu._ser.inject_response(buf)
        self.mcu.poll()

        # 验证解析结果
        x, y, q = self.mcu.read_optical_flow_position()
        self.assertEqual(x, pos_x, f"pos_x: expected {pos_x}, got {x}")
        self.assertEqual(y, pos_y)
        self.assertEqual(q, quality)

    def test_of_incremental(self):
        """测试光流增量计算"""
        # 第一次注入: pos=(100, 0)
        buf1 = bytes([0xCC, 0x01]) + struct.pack('<i', 100) + \
               struct.pack('<i', 0) + bytes([200])
        self.mcu._ser.inject_response(buf1)
        self.mcu.poll()

        dx, dy = self.mcu.read_optical_flow()
        self.assertEqual(dx, 100.0)
        self.assertEqual(dy, 0.0)

        # 第二次注入: pos=(120, -5)
        buf2 = bytes([0xCC, 0x01]) + struct.pack('<i', 120) + \
               struct.pack('<i', -5) + bytes([200])
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
        buf = bytes([0xCC, 0x02, 0x03, 0x00]) + struct.pack('<i', 150)
        self.mcu._ser.inject_response(buf)
        self.mcu.poll()

        self.assertEqual(self.mcu.read_mode(), 3)
        self.assertEqual(self.mcu.read_locked(), 0)
        self.assertEqual(self.mcu.read_altitude(), 150)

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
        valid = bytes([0xCC, 0x01]) + struct.pack('<i', 500) + \
                struct.pack('<i', 300) + bytes([150])
        self.mcu._ser.inject_response(garbage + valid + garbage)
        self.mcu.poll()

        x, y, q = self.mcu.read_optical_flow_position()
        self.assertEqual(x, 500)
        self.assertEqual(y, 300)
        self.assertEqual(q, 150)

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
        self.assertTrue(self.mcu.send_cmd_land())

    def test_move(self):
        self.assertTrue(self.mcu.send_cmd_move(100, 30, 90))

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
