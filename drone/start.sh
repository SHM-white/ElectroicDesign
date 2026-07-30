#!/usr/bin/env bash
# ⚠️ 已弃用 — 此目录不再使用
echo "⚠️  drone/ 已弃用" >&2
echo "" >&2
echo "当前比赛流程使用 ROS2 仿真:" >&2
echo "  1. tools/test_stability_logic.sh     # 测试任务逻辑" >&2
echo "  2. tools/run_stability_test_sim.sh   # 启动完整仿真" >&2
echo "  3. tools/send_stability_goal.sh      # 发送任务目标" >&2
echo "" >&2
echo "详见 drone/DEPRECATED.md" >&2
exit 1
