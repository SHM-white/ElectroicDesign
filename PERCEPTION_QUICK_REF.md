# 感知重构 - 快速参考

## 🚀 启动命令
```bash
# 构建并启动仿真
./tools/run_competition.sh --build --simulation --enable-display --force-container

# 仅启动仿真
./tools/run_competition.sh --simulation --enable-display --force-container
```

## 🔍 调试命令
```bash
# 检查容器
docker ps --format "{{.Names}}"

# 话题列表
docker exec <container> /tmp/ros_topics.sh

# 话题频率
docker exec <container> ros2 topic hz /camera/wide/image_raw

# 节点状态
docker exec <container> ros2 node list

# 诊断信息
docker exec <container> ros2 topic echo /perception/narrow/diagnostics --once
```

## 📁 关键文件
- `single_camera_detector_node.py` - 单相机检测基类
- `target_fusion_node.py` - 双相机融合
- `perception_visualizer_node.py` - 可视化调试
- `sim.launch.py` - 仿真启动文件

## ⚠️ 待解决问题
1. `narrow_detector` 可能崩溃（exit code 1）- 需要进一步调试
2. WSL 和 Docker 之间的 DDS 发现不工作 - 需要配置 Fast DDS 或在容器内安装 rqt
3. 检测频率约 5-8 Hz（可优化）

## 📊 当前性能
| 指标 | 值 | 状态 |
|------|-----|------|
| 车辆遥测 | ~8 Hz | ✅ |
| 窄相机 FPS | ~5-8 Hz | ⚠️ |
| 广角相机 FPS | ~3-5 Hz | ⚠️ |
| 显示帧率 | 15 fps | ✅ |
| 显示分辨率 | 640px | ✅ |

## 🎯 下一步
1. 修复 narrow_detector 崩溃问题
2. 优化检测性能（降低分辨率/GPU加速）
3. 解决 WSL/Docker DDS 发现问题
