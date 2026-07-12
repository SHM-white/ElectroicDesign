"""
test_lx_protocol.py — 凌霄协议帧构建与校验测试
验证 Section 4: 凌霄IMU API指令速查
"""

import sys
import os
import struct
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lx_protocol import (
    build_lx_frame, build_pi_frame, build_pi_query_frame,
    verify_lx_frame, parse_of_position, parse_flight_status,
    cmd_unlock, cmd_lock, cmd_mode,
    cmd_takeoff, cmd_land, cmd_move,
    cmd_ascend, cmd_descend,
    frame_to_hex,
)

# ── 已知正确值的参考帧 ────────────────────────────────────
# 从计划 Section 4.3 提取期望值


class TestLXFrameBuilding(unittest.TestCase):
    """测试凌霄API帧构建"""

    def test_unlock_frame(self):
        """验证解锁帧"""
        frame = cmd_unlock()
        # 期望: AA FF E0 0B 10 00 01 00 00 00 00 00 00 00 00 SC AC
        self.assertEqual(frame[0], 0xAA, "Frame header")
        self.assertEqual(frame[1], 0xFF, "Target address (broadcast)")
        self.assertEqual(frame[2], 0xE0, "Frame ID (command)")
        self.assertEqual(frame[3], 0x0B, "Payload length = 11")
        self.assertEqual(frame[4], 0x10, "CID=0x10")
        self.assertEqual(frame[5], 0x00, "CMD0")
        self.assertEqual(frame[6], 0x01, "CMD1 (unlock)")
        # 验证校验和
        self.assertTrue(verify_lx_frame(frame),
                        f"Checksum verification failed for {frame_to_hex(frame)}")

    def test_lock_frame(self):
        """验证加锁帧"""
        frame = cmd_lock()
        self.assertEqual(frame[4], 0x10)
        self.assertEqual(frame[5], 0x00)
        self.assertEqual(frame[6], 0x02, "CMD1 (lock)")
        self.assertTrue(verify_lx_frame(frame))

    def test_takeoff_default_height(self):
        """验证起飞帧 (默认高度)"""
        frame = cmd_takeoff(0)
        # height_cm=0 → 使用默认值
        self.assertEqual(frame[4], 0x10)
        self.assertEqual(frame[6], 0x05, "CMD1 (takeoff)")
        # 高度字段 u16小端 = 0x0000
        self.assertEqual(frame[7], 0x00)
        self.assertEqual(frame[8], 0x00)
        self.assertTrue(verify_lx_frame(frame))

    def test_takeoff_specific_height(self):
        """验证起飞帧 (指定高度150cm)"""
        frame = cmd_takeoff(150)
        # height_cm=150 = 0x0096
        self.assertEqual(frame[7], 0x96)
        self.assertEqual(frame[8], 0x00)
        self.assertTrue(verify_lx_frame(frame))

    def test_land_frame(self):
        """验证降落帧"""
        frame = cmd_land()
        self.assertEqual(frame[4], 0x10)
        self.assertEqual(frame[6], 0x06, "CMD1 (land)")
        self.assertTrue(verify_lx_frame(frame))

    def test_mode_switch(self):
        """验证模式切换帧"""
        modes = {0: "自稳", 1: "自稳+定高", 2: "定点", 3: "程控"}
        for mode, name in modes.items():
            frame = cmd_mode(mode)
            # AA FF E0 0B  01 01 01 [MODE] 00 00 00 00 00 00 00  SC AC
            self.assertEqual(frame[4], 0x01, f"Mode {name}: CMD at pos 4")
            self.assertEqual(frame[5], 0x01)
            self.assertEqual(frame[6], 0x01)
            self.assertEqual(frame[7], mode, f"Mode {name}: MODE byte")
            self.assertTrue(verify_lx_frame(frame),
                            f"Checksum failed for mode={name}")

    def test_invalid_mode_raises(self):
        """验证非法mode值抛出异常"""
        with self.assertRaises(ValueError):
            cmd_mode(4)
        with self.assertRaises(ValueError):
            cmd_mode(-1)

    def test_move_basic(self):
        """验证水平移动帧"""
        frame = cmd_move(100, 30, 90)
        # AA FF E0 0B  10 02 03 [D_LO D_HI] [S_LO S_HI] [A_LO A_HI] 00 00  SC AC
        self.assertEqual(frame[4], 0x10, "CID")
        self.assertEqual(frame[5], 0x02, "CMD0")
        self.assertEqual(frame[6], 0x03, "CMD1 (move)")

        # 距离: 100 = 0x0064 (小端)
        self.assertEqual(frame[7], 0x64)
        self.assertEqual(frame[8], 0x00)

        # 速度: 30 = 0x001E
        self.assertEqual(frame[9], 0x1E)
        self.assertEqual(frame[10], 0x00)

        # 方向: 90 = 0x005A
        self.assertEqual(frame[11], 0x5A)
        self.assertEqual(frame[12], 0x00)

        self.assertTrue(verify_lx_frame(frame))

    def test_move_boundary_values(self):
        """验证水平移动帧边界值"""
        # 最小值
        frame = cmd_move(0, 10, 0)
        self.assertTrue(verify_lx_frame(frame))
        # 最大值
        frame = cmd_move(10000, 300, 359)
        self.assertTrue(verify_lx_frame(frame))

    def test_move_invalid_raises(self):
        """验证非法移动参数抛出异常"""
        with self.assertRaises(ValueError):
            cmd_move(10001, 30, 90)  # 距离超限
        with self.assertRaises(ValueError):
            cmd_move(100, 5, 90)     # 速度低于下限
        with self.assertRaises(ValueError):
            cmd_move(100, 30, 360)   # 方向超限

    def test_ascend_frame(self):
        """验证上升帧"""
        frame = cmd_ascend(100, 25)
        self.assertEqual(frame[4], 0x10)
        self.assertEqual(frame[5], 0x02)
        self.assertEqual(frame[6], 0x01, "CMD1 (ascend)")
        self.assertTrue(verify_lx_frame(frame))

    def test_descend_frame(self):
        """验证下降帧"""
        frame = cmd_descend(50, 15)
        self.assertEqual(frame[6], 0x02, "CMD1 (descend)")
        self.assertTrue(verify_lx_frame(frame))

    def test_checksum_correctness(self):
        """验证校验和计算的正确性"""
        # 构建一个已知的简单帧手动验证
        # AA FF E0 01 00 = 5个字节
        frame = build_lx_frame(0xFF, 0xE0, bytes([0x00]))

        # 手动计算: AA+FF+E0+01+00 = 0x1+0x8A → sumcheck=8A
        # addcheck: 0+AA=AA, AA+54=FE, FE+DE=DC, DC+DD=B9 → addcheck=B9
        expected_sc = (0xAA + 0xFF + 0xE0 + 0x01 + 0x00) & 0xFF  # = 0x8A
        self.assertEqual(frame[-2], expected_sc)
        self.assertTrue(verify_lx_frame(frame))

    def test_payload_length_consistency(self):
        """验证所有帧的载荷长度一致性"""
        for cmd in [cmd_unlock(), cmd_lock(), cmd_mode(3),
                     cmd_takeoff(150), cmd_land(),
                     cmd_move(100, 30, 90), cmd_ascend(100, 25)]:
            declared_len = cmd[3]  # LEN字节
            actual_len = len(cmd) - 6  # 总长 - 固定头部 - 校验和
            self.assertEqual(declared_len, actual_len,
                             f"Length mismatch in {frame_to_hex(cmd)}")


class TestPIFrameBuilding(unittest.TestCase):
    """测试树莓派→MCU通信帧"""

    def test_build_pi_forward_frame(self):
        """测试TYPE=0x01指令转发帧"""
        imu_frame = cmd_unlock()
        pi_frame = build_pi_frame(imu_frame, frame_type=0x01)

        # AA [CMD_LEN] 01 [IMU_FRAME] [SUM_LO] [SUM_HI]
        self.assertEqual(pi_frame[0], 0xAA)
        self.assertEqual(pi_frame[1], len(imu_frame))
        self.assertEqual(pi_frame[2], 0x01, "TYPE=转发")

        # 校验和验证
        calc_sum = sum(pi_frame[:-2]) & 0xFFFF
        stored_sum = pi_frame[-2] | (pi_frame[-1] << 8)
        self.assertEqual(calc_sum, stored_sum)

    def test_build_pi_query_frame(self):
        """测试TYPE=0xBB查询帧"""
        # 请求光流位置
        frame = build_pi_query_frame(0x01)
        self.assertEqual(frame[0], 0xBB)
        self.assertEqual(frame[1], 0x01)

        # 请求飞行状态
        frame = build_pi_query_frame(0x02)
        self.assertEqual(frame[1], 0x02)

        # 重置光流零点
        frame = build_pi_query_frame(0x03)
        self.assertEqual(frame[1], 0x03)


class TestMcuFrameParsing(unittest.TestCase):
    """测试MCU回传帧解析"""

    def test_parse_of_position(self):
        """测试光流位置帧解析"""
        # 模拟: POS_X=1234cm, POS_Y=-567cm, QUALITY=200
        pos_x_packed = struct.pack('<i', 1234)
        pos_y_packed = struct.pack('<i', -567)
        buf = bytes([0xCC, 0x01]) + pos_x_packed + pos_y_packed + bytes([200])

        result = parse_of_position(buf)
        self.assertIsNotNone(result)
        self.assertEqual(result['pos_x'], 1234)
        self.assertEqual(result['pos_y'], -567)
        self.assertEqual(result['quality'], 200)

    def test_parse_of_position_short_buf(self):
        """测试缓冲区过短"""
        self.assertIsNone(parse_of_position(b'\xCC\x01'))
        self.assertIsNone(parse_of_position(b'\xCC\x01\x00\x00\x00'))

    def test_parse_of_position_wrong_header(self):
        """测试错误帧头"""
        buf = bytes([0xDD, 0x01]) + bytes(9)
        self.assertIsNone(parse_of_position(buf))

    def test_parse_flight_status(self):
        """测试飞行状态帧解析"""
        # MODE=3(程控), LOCKED=0(已解锁), ALT=150cm
        alt_packed = struct.pack('<i', 150)
        buf = bytes([0xCC, 0x02, 0x03, 0x00]) + alt_packed

        result = parse_flight_status(buf)
        self.assertIsNotNone(result)
        self.assertEqual(result['mode'], 3)
        self.assertEqual(result['locked'], 0)
        self.assertEqual(result['alt'], 150)


class TestChecksumVerification(unittest.TestCase):
    """校验和计算全面测试"""

    def test_verify_valid_frame(self):
        """测试自动生成的帧都能通过校验"""
        frames = [
            cmd_unlock(), cmd_lock(),
            cmd_takeoff(0), cmd_takeoff(150), cmd_land(),
            cmd_mode(0), cmd_mode(1), cmd_mode(2), cmd_mode(3),
            cmd_move(100, 30, 90), cmd_move(0, 10, 359),
            cmd_move(5000, 150, 180), cmd_move(10000, 300, 0),
            cmd_ascend(100, 25), cmd_descend(50, 15),
        ]
        for i, frame in enumerate(frames):
            self.assertTrue(verify_lx_frame(frame),
                            f"Frame {i} checksum failed: {frame_to_hex(frame)}")

    def test_verify_tampered_frame(self):
        """测试篡改后的帧校验会失败"""
        frame = bytearray(cmd_unlock())
        frame[4] ^= 0x01  # 翻转CID的1bit
        self.assertFalse(verify_lx_frame(bytes(frame)))

    def test_verify_short_frame(self):
        """测试过短帧"""
        self.assertFalse(verify_lx_frame(b'\xAA\xFF'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
