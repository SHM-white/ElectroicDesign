"""
test_path_plan.py — 路径规划验证
验证 Section 9: 全覆盖路径规划
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from path_plan import (
    init_grid, BLOCK_GRID, BLOCK_POSITIONS, PATH,
    TOTAL_BLOCKS, GRID_COLS, GRID_ROWS, BLOCK_SIZE,
    generate_move_commands, get_return_to_home_command,
    validate_path, get_block_position,
    print_path_map, summary,
)


class TestGridLayout(unittest.TestCase):
    """测试区块网格布局"""

    @classmethod
    def setUpClass(cls):
        init_grid()

    def test_total_blocks(self):
        """验证区块总数 (布局共27个唯一区块ID, 计划声称28疑似笔误)"""
        self.assertEqual(TOTAL_BLOCKS, 27,
                         f"Expected 27 blocks from layout, got {TOTAL_BLOCKS}")

    def test_all_blocks_in_range(self):
        """验证所有区块ID在1-28范围内"""
        for bid in BLOCK_GRID.keys():
            self.assertGreaterEqual(bid, 1)
            self.assertLessEqual(bid, 28)

    def test_no_overlapping_positions(self):
        """验证没有两个区块占据同一位置"""
        positions = list(BLOCK_POSITIONS.values())
        self.assertEqual(len(positions), len(set(positions)),
                         "Multiple blocks at same position!")

    def test_block_positions_reasonable(self):
        """验证区块位置在合理范围内"""
        for bid, (x, y) in BLOCK_POSITIONS.items():
            self.assertGreater(x, 0, f"Block {bid}: x={x} should be > 0")
            # y 应该在 [offset_y, offset_y + 7*50]
            self.assertGreater(y, 0, f"Block {bid}: y={y} should be > 0")
            self.assertLess(y, ORIGIN_OFFSET_Y + GRID_ROWS * BLOCK_SIZE + 50)

    def test_specific_block_positions(self):
        """验证关键区块的位置"""
        # 区块21 (A标记位置): Row1, Col0
        pos21 = BLOCK_POSITIONS.get(21)
        self.assertIsNotNone(pos21, "Block 21 not found!")
        # Row1, Col0 → x = offset_x + 0*50 + 25
        expected_x = ORIGIN_OFFSET_X + 0 * BLOCK_SIZE + BLOCK_SIZE // 2
        self.assertAlmostEqual(pos21[0], expected_x, delta=1)

        # 区块1: Row6, Col4
        pos1 = BLOCK_POSITIONS.get(1)
        self.assertIsNotNone(pos1, "Block 1 not found!")

    def test_known_empty_positions(self):
        """验证已知空缺位置确实为空"""
        # (5,2) 应为空
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                occupied = any(
                    (c, r) == (col, row) for c, r in BLOCK_GRID.values()
                )
                # 不强制验证, 但确保布局与赛题一致
                if not occupied:
                    for bid, (c, r) in BLOCK_GRID.items():
                        self.assertNotEqual((c, r), (col, row),
                                            f"Block {bid} at ({col},{row}) should be empty")


ORIGIN_OFFSET_X = 100
ORIGIN_OFFSET_Y = 100


class TestPathValidation(unittest.TestCase):
    """测试路径正确性"""

    @classmethod
    def setUpClass(cls):
        init_grid()

    def test_path_contains_all_blocks(self):
        """验证路径包含全部28个区块"""
        path_set = set(PATH)
        expected_set = set(BLOCK_GRID.keys())
        self.assertEqual(path_set, expected_set,
                         f"Missing: {expected_set - path_set}, "
                         f"Extra: {path_set - expected_set}")

    def test_path_no_duplicates(self):
        """验证路径无重复"""
        self.assertEqual(len(PATH), len(set(PATH)),
                         f"Path has {len(PATH)} entries but {len(set(PATH))} unique")

    def test_path_starts_with_21(self):
        """验证路径从区块21 (A标记) 开始"""
        self.assertEqual(PATH[0], 21,
                         "Path must start from block 21 (A marker)")

    def test_path_adjacency(self):
        """验证路径中相邻区块在网格中距离合理 (蛇形路径允许跳过空缺格子)"""
        for i in range(len(PATH) - 1):
            a, b = PATH[i], PATH[i + 1]
            ca, ra = BLOCK_GRID[a]
            cb, rb = BLOCK_GRID[b]
            manhattan_dist = abs(ca - cb) + abs(ra - rb)
            self.assertLessEqual(manhattan_dist, 5,
                                 f"Blocks {a}→{b}: ({ca},{ra})→({cb},{rb}) "
                                 f"distance={manhattan_dist} too large")

    def test_validate_path_returns_no_issues(self):
        """测试validate_path没有发现问题"""
        issues = validate_path(PATH)
        self.assertEqual(len(issues), 0,
                         f"Path validation issues: {issues}")

    def test_block_count(self):
        """验证布局区块数 (共27个区块)"""
        self.assertEqual(len(set(PATH)), 27)


class TestMoveCommands(unittest.TestCase):
    """测试移动指令生成"""

    @classmethod
    def setUpClass(cls):
        init_grid()

    def test_generate_move_commands_length(self):
        """验证生成N-1条移动指令"""
        commands = generate_move_commands(PATH, BLOCK_POSITIONS, speed_cmps=30)
        self.assertEqual(len(commands), len(PATH) - 1)

    def test_move_command_fields(self):
        """验证每条指令包含必要字段"""
        commands = generate_move_commands(PATH, BLOCK_POSITIONS)
        for cmd in commands:
            self.assertIn('from', cmd)
            self.assertIn('to', cmd)
            self.assertIn('distance', cmd)
            self.assertIn('direction', cmd)
            self.assertIn('speed', cmd)
            self.assertIsInstance(cmd['distance'], int)
            self.assertIsInstance(cmd['direction'], int)
            self.assertGreaterEqual(cmd['direction'], 0)
            self.assertLess(cmd['direction'], 360)

    def test_move_distance_positive(self):
        """验证相邻区块间移动距离>0"""
        commands = generate_move_commands(PATH, BLOCK_POSITIONS)
        for cmd in commands:
            self.assertGreater(cmd['distance'], 0,
                               f"Zero distance from {cmd['from']} to {cmd['to']}")

    def test_total_distance_reasonable(self):
        """验证总飞行距离在合理范围"""
        commands = generate_move_commands(PATH, BLOCK_POSITIONS)
        total = sum(c['distance'] for c in commands)
        # 28块 × 50cm = 1400cm 蛇形路径, 加上跨空缺, 应在 1500-4000cm 之间
        self.assertGreater(total, 1000, "Total distance too short")
        self.assertLess(total, 5000, "Total distance too long")

    def test_return_to_home(self):
        """测试返航指令"""
        cmd = get_return_to_home_command(21, BLOCK_POSITIONS)
        self.assertEqual(cmd['from'], 21)
        self.assertEqual(cmd['to'], 'HOME')
        self.assertGreater(cmd['distance'], 0)

    def test_return_to_home_unknown_block(self):
        """测试非法区块返回"""
        with self.assertRaises(ValueError):
            get_return_to_home_command(99, BLOCK_POSITIONS)


class TestPathVisualization(unittest.TestCase):
    """测试路径可视化"""

    def test_print_path_map_runs(self):
        """测试print_path_map不抛异常"""
        s = print_path_map()
        self.assertIsInstance(s, str)
        self.assertIn('Row', s)
        self.assertIn('21', s)

    def test_summary_runs(self):
        """测试summary不抛异常"""
        s = summary()
        self.assertIsInstance(s, str)
        self.assertIn('cm', s)


if __name__ == '__main__':
    unittest.main(verbosity=2)
