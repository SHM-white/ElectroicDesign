# Gazebo Fortress 仿真

这是一条仅用于仿真的路径，用于验证 ROS 图连接、传感器传输、定位状态、任务编排以及仿真器 FCU 动作生命周期。它不是 V7 固件、HIL、硬件传感器或飞行安全测试。场地和任务文件均为合成文件，禁止用于竞赛激活。它始终不能替代串口硬件。

## 交互式 GUI

在 WSLg 中运行，并确保 `DISPLAY=:0`、`WAYLAND_DISPLAY`、`XDG_RUNTIME_DIR` 和 `/mnt/wslg` 可用：

```bash
./tools/run_gazebo_sim.sh
```

此命令会打开 Gazebo Fortress 和 RViz，并持续附着运行，直到按下 `Ctrl+C`。

## 有界冒烟测试

```bash
./tools/run_gazebo_smoke.sh
```

冒烟测试运行器会启动无头仿真器，检查 `/clock`，启用仿真控制器，验证真值里程计，并清理进程组。两条路径都不会打开串口硬件；仿真器拥有 `/fcu/flight_command`，并报告 `FcuState.SOURCE_SIMULATOR`。

仿真器使用 Gazebo Fortress 原生多旋翼控制和电机模型系统。它不声称具备 V7 协议、HIL 时序、真实传感器保真度或飞行就绪能力。
