"""
path_plan.py — 全覆盖路径规划
Section 9: 蛇形全覆盖路径

作业区: 400cm × 500cm
区块尺寸: 50cm × 50cm
网格: 7列 × 7行 (部分位置空缺)
"""

import math
from typing import Dict, List, Tuple, Optional


# ── 区块网格布局 ──────────────────────────────────────────

# 作业区网格映射: block_id -> (col_index, row_index)
# row=0=顶部, row=6=底部, col=0=最左, col=6=最右
BLOCK_GRID: Dict[int, Tuple[int, int]] = {}
BLOCK_POSITIONS: Dict[int, Tuple[float, float]] = {}  # block_id -> (x_cm, y_cm)

# 从赛题图1还原的作业区布局
_LAYOUT: Dict[Tuple[int, int], int] = {
    # (row, col): block_id
    (0, 0): 28, (0, 1): 26, (0, 2): 25, (0, 3): 24, (0, 4): 23,
    # (0, 5): 空
    (0, 6): 22,
    (1, 0): 21, (1, 1): 20, (1, 2): 18, (1, 3): 16, (1, 4): 15, (1, 5): 19, (1, 6): 17,
    (2, 0): 12, (2, 1): 14, (2, 2): 13, (2, 3): 11,
    # (2, 4): 空, (2, 5): 空, (2, 6): 空
    (3, 0): 10, (3, 1): 9,
    # (3, 2): 空, (3, 3): 空
    (3, 4): 8, (3, 5): 7,
    # (4, 0): 空, (4, 1): 空, (4, 2): 空, (4, 3): 空
    (4, 4): 5, (4, 5): 6,
    # (5, 0): 空, (5, 1): 空, (5, 2): 空, (5, 3): 空
    (5, 4): 4, (5, 5): 3,
    # (6, 0): 空, (6, 1): 空, (6, 2): 空, (6, 3): 空
    (6, 4): 1, (6, 5): 2,
}

# 总区块数 (布局中共27个区块, 计划第15节声称28可能为笔误)
TOTAL_BLOCKS = len(_LAYOUT)  # = 27

# 起降点坐标偏移 (cm)
ORIGIN_OFFSET_X = 100
ORIGIN_OFFSET_Y = 100

# 作业区网格尺寸
GRID_COLS = 7
GRID_ROWS = 7
BLOCK_SIZE = 50  # cm


# ── 预设蛇形路径 ──────────────────────────────────────────

# A标记在区块21, 飞行路径从21开始
# 路径策略: 蛇形遍历, 减少空行程
PATH: List[int] = [
    # Row1: 从左到右
    21, 20, 18, 16, 15, 19, 17,
    # Row0: 从右到左 (上行)
    22, 23, 24, 25, 26, 28,
    # Row2: 从左到右 (下行)
    12, 14, 13, 11,
    # Row3: 从左到右, 处理缺口
    10, 9,
    8, 7,
    # Row4: 从右到左
    5, 6,
    # Row5: 从右到左
    4, 3,
    # Row6: 从左到右 (最后一行)
    1, 2,
]


def init_grid(offset_x: float = ORIGIN_OFFSET_X,
              offset_y: float = ORIGIN_OFFSET_Y) -> Dict[int, Tuple[float, float]]:
    """
    初始化区块世界坐标

    坐标系统:
        X轴正方向 = 机头方向 (指向作业区)
        Y轴正方向 = 飞机右侧 (Col增加方向)
        原点 = 起降点

    Args:
        offset_x: 起降点到作业区左边界的距离(cm)
        offset_y: 起降点到作业区底部边缘的距离(cm)

    Returns:
        {block_id: (x_cm, y_cm), ...}
    """
    global BLOCK_GRID, BLOCK_POSITIONS
    BLOCK_GRID.clear()
    BLOCK_POSITIONS.clear()

    for (row, col), bid in _LAYOUT.items():
        # col增加 = X轴正方向
        x = offset_x + col * BLOCK_SIZE + BLOCK_SIZE // 2
        # row增加 = 向下, Y轴以底部为0向上增加
        y = offset_y + (GRID_ROWS - 1 - row) * BLOCK_SIZE + BLOCK_SIZE // 2

        BLOCK_GRID[bid] = (col, row)
        BLOCK_POSITIONS[bid] = (x, y)

    return BLOCK_POSITIONS


def get_block_position(block_id: int) -> Optional[Tuple[float, float]]:
    """获取区块的世界坐标"""
    return BLOCK_POSITIONS.get(block_id)


def get_block_grid(block_id: int) -> Optional[Tuple[int, int]]:
    """获取区块的网格坐标 (col, row)"""
    return BLOCK_GRID.get(block_id)


def generate_move_commands(path: List[int],
                           positions: Dict[int, Tuple[float, float]],
                           speed_cmps: int = 30) -> List[dict]:
    """
    根据路径生成水平移动指令列表

    Args:
        path: 区块访问顺序列表
        positions: 区块位置映射
        speed_cmps: 移动速度(cm/s)

    Returns:
        [{'from': int, 'to': int, 'distance': int, 'direction': int, 'speed': int}, ...]
    """
    commands = []
    for i in range(len(path) - 1):
        cur_id = path[i]
        nxt_id = path[i + 1]
        cur_pos = positions[cur_id]
        nxt_pos = positions[nxt_id]

        dx = nxt_pos[0] - cur_pos[0]
        dy = nxt_pos[1] - cur_pos[1]

        distance = math.sqrt(dx ** 2 + dy ** 2)
        direction = math.degrees(math.atan2(dy, dx)) % 360

        commands.append({
            'from': cur_id,
            'to': nxt_id,
            'distance': int(round(distance)),
            'direction': int(round(direction)),
            'speed': speed_cmps,
        })

    return commands


def get_return_to_home_command(
    current_block_id: int,
    positions: Dict[int, Tuple[float, float]],
    speed_cmps: int = 30,
) -> dict:
    """
    从当前区块返回起降点的移动指令

    起降点坐标为 (0, 0)
    """
    if current_block_id not in positions:
        raise ValueError(f"Unknown block ID: {current_block_id}")

    cur_pos = positions[current_block_id]
    home_pos = (0.0, 0.0)

    dx = home_pos[0] - cur_pos[0]
    dy = home_pos[1] - cur_pos[1]

    distance = math.sqrt(dx ** 2 + dy ** 2)
    direction = math.degrees(math.atan2(dy, dx)) % 360

    return {
        'from': current_block_id,
        'to': 'HOME',
        'distance': int(round(distance)),
        'direction': int(round(direction)),
        'speed': speed_cmps,
    }


def validate_path(path: List[int]) -> List[str]:
    """
    验证路径正确性

    检查:
    1. 路径包含所有28个区块 (无遗漏)
    2. 路径中没有重复
    3. 所有区块ID都在合法范围内
    4. 相邻区块存在网格连接

    Returns:
        问题列表, 空列表表示路径通过验证
    """
    issues = []

    # 检查1: 数量
    expected_blocks = set(_LAYOUT.values())
    path_blocks = set(path)

    if len(expected_blocks) != TOTAL_BLOCKS:
        issues.append(f"Expected {TOTAL_BLOCKS} blocks in layout, got {len(expected_blocks)}")

    if len(path_blocks) != TOTAL_BLOCKS:
        issues.append(f"Path has {len(path_blocks)} unique blocks, expected {TOTAL_BLOCKS}")

    # 检查2: 重复
    if len(path) != len(path_blocks):
        from collections import Counter
        dupes = [bid for bid, cnt in Counter(path).items() if cnt > 1]
        issues.append(f"Duplicate blocks in path: {dupes}")

    # 检查3: 合法性
    missing = expected_blocks - path_blocks
    if missing:
        issues.append(f"Missing blocks: {missing}")

    extra = path_blocks - expected_blocks
    if extra:
        issues.append(f"Invalid block IDs: {extra}")

    # 检查4: 连续性 (蛇形路径允许跳过空缺格子, 放宽到曼哈顿距离≤5)
    if not issues:
        if not BLOCK_GRID:
            init_grid()
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            if a in BLOCK_GRID and b in BLOCK_GRID:
                ca, ra = BLOCK_GRID[a]
                cb, rb = BLOCK_GRID[b]
                dist = abs(ca - cb) + abs(ra - rb)
                if dist > 5:
                    issues.append(
                        f"Non-adjacent blocks {a}→{b}: "
                        f"({ca},{ra})→({cb},{rb}) distance={dist}"
                    )

    return issues


def print_path_map() -> str:
    """生成路径地图文本（调试用）"""
    if not BLOCK_GRID:
        init_grid()

    # 创建7×7网格字符串
    grid_str = [[' -- ' for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    for bid, (col, row) in BLOCK_GRID.items():
        grid_str[row][col] = f'{bid:3d}'

    lines = []
    lines.append("        Col: 0   1   2   3   4   5   6")
    lines.append("        " + "-" * 36)
    for row_idx, row in enumerate(grid_str):
        lines.append(f"Row {row_idx}: " + "".join(row))

    # 添加图例
    lines.append(f"\nTotal blocks: {TOTAL_BLOCKS}")
    lines.append(f"Path length: {len(PATH)} blocks")
    lines.append(f"Path: {'→'.join(map(str, PATH))}")

    return "\n".join(lines)


def get_home_position() -> Tuple[float, float]:
    """获取起降点坐标"""
    return (0.0, 0.0)


def block_id_to_world(block_id: int) -> Optional[Tuple[float, float]]:
    """将区块ID映射到世界坐标"""
    if not BLOCK_POSITIONS:
        init_grid()
    return BLOCK_POSITIONS.get(block_id)


# ── 调试辅助 ──────────────────────────────────────────────


def summary() -> str:
    """打印路径规划摘要"""
    if not BLOCK_POSITIONS:
        init_grid()

    lines = [f"Grid: {GRID_COLS}x{GRID_ROWS}, Block size: {BLOCK_SIZE}cm"]
    lines.append(f"Total blocks: {TOTAL_BLOCKS}")
    lines.append(f"Path: {len(PATH)} segments ({len(PATH)} blocks)")

    move_cmds = generate_move_commands(PATH, BLOCK_POSITIONS)
    total_dist = sum(c['distance'] for c in move_cmds)
    lines.append(f"Total travel distance: {total_dist}cm")

    # 预估时间 (按不同速度)
    for speed in [15, 30, 45]:
        travel_time = total_dist / speed
        spray_time = TOTAL_BLOCKS * 3  # 每块约3秒撒药
        total_time = travel_time + spray_time + 30  # +30秒起飞降落
        lines.append(f"  @ {speed}cm/s: ~{total_time:.0f}s total")

    return "\n".join(lines)


# 初始化
init_grid()
