# ⚠️ 已弃用 — 请勿使用此目录

> **此目录中的程序已不再使用，请勿运行或修改。**

## 原因

`drone/` 是早期的独立无人机任务控制器，设计为直接通过串口控制凌霄飞控。
当前比赛架构已完全替换为以下方案：

| 组件 | 路径 | 说明 |
|------|------|------|
| 任务逻辑测试 | `tools/test_stability_logic.sh` | 测试稳定性任务配置和运行器逻辑 |
| 完整仿真 | `tools/run_stability_test_sim.sh` | ROS2 + Gazebo 完整仿真 |
| 发送目标 | `tools/send_stability_goal.sh` | 向运行中的仿真发送任务目标 |
| ROS2 节点 | `ros2_ws/src/ed_uav_mission/` | 任务执行节点 |
| ROS2 仿真 | `ros2_ws/src/ed_uav_gazebo/` | Gazebo 仿真环境 |

## 当前启动方式

```bash
# 1. 测试任务逻辑（不需要 Gazebo）
./tools/test_stability_logic.sh

# 2. 启动完整 ROS2 + Gazebo 仿真
./tools/run_stability_test_sim.sh

# 3. 发送任务目标
./tools/send_stability_goal.sh
```

## 保留原因

此目录保留用于：
- 历史参考（路径规划、状态机逻辑）
- 部分算法可能在 ROS2 节点中复用
- 不要直接运行其中的脚本
