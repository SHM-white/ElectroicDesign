# Session 转交文档 - 2026-08-16 Part 3 (Tag 渲染与检测调试)

> **时间**: 2026-08-16
> **状态**: 部分完成，核心链路打通但时序问题未解决

---

## 一、本次 Session 目标

1. 调整 Gazebo 中小车靶上 tag 的渲染方式（禁用平滑算法）
2. 分析模拟相机链路是否能正常识别 tag

---

## 二、已完成的工作

### 2.1 Tag 纹理更新

| 项目 | 旧值 | 新值 |
|------|------|------|
| 纹理文件 | `tag36h11_0.png` 10×10 RGBA | 500×500 RGBA（cv2.aruco.drawMarker 生成） |
| 生成方式 | 手动构造（**错误**，模式与 OpenCV 不一致） | `cv2.aruco.drawMarker(DICT_APRILTAG_36h11, 0, 500, borderBits=1)` |
| 关键验证 | — | 在 Docker 内用 ArUco 检测通过 |

**教训**：不能手动构造 tag36h11 的 bit pattern，必须用 `cv2.aruco.drawMarker()` 生成，否则与 OpenCV 检测器的内部表示不一致。

### 2.2 ArUco 检测参数调优

文件：`ros2_ws/src/ed_uav_perception/ed_uav_perception/apriltag_detector.py`

通过逐参数消融实验找到的最小工作参数集（OpenCV 4.5.4 + tag36h11）：

| 参数 | 旧值 | 新值 | 是否关键 |
|------|------|------|---------|
| `adaptiveThreshWinSizeMax` | 23 | **201** | ✅ 必须 |
| `adaptiveThreshConstant` | 7 | **3** | ✅ 必须 |
| `minDistanceToBorder` | 3 | **0** | ✅ 必须 |
| `adaptiveThreshWinSizeStep` | 10 | **4** | ✅ 必须（或 polygonalApproxAccuracyRate=0.1） |
| 其他参数 | — | 保持默认 | ❌ |

**关键发现**：`cv2.aruco.drawMarker()` 生成的图像在默认参数下无法被 `cv2.aruco.detectMarkers()` 检测到。这可能是 OpenCV 4.5.4 的 bug 或已知行为。

### 2.3 相机位置修复

文件：`ros2_ws/src/ed_uav_gazebo/models/ed_quadrotor/model.sdf`

**根因**：相机传感器在无人机机体碰撞/视觉几何体内部，导致 Gazebo 渲染全黑/全灰图像。

| 传感器 | 旧位置 | 新位置 |
|--------|--------|--------|
| narrow_camera | (0.08, 0, -0.08) **机体内部** | (0.28, 0.22, -0.10) **机体外** |
| wide_camera | (-0.04, 0, -0.08) **机体内部** | (-0.28, -0.22, -0.10) **机体外** |
| rangefinder | (0, 0, -0.08) | (0, 0, -0.10) |

机体 box 尺寸：0.42×0.32×0.12（x: ±0.21, y: ±0.16, z: ±0.06）

### 2.4 QoS / 验证修复

| 文件 | 修改 | 原因 |
|------|------|------|
| `ed_uav_mission/d_task_ros.py` | 订阅 QoS → `qos_profile_sensor_data` | BEST_EFFORT/RELIABLE 不兼容 |
| `ed_uav_perception/target_observation_node.py` | 图像订阅 QoS → `VEHICLE_QOS` (RELIABLE depth=10) | 图像数据损坏（min=89 max=108 全灰） |
| `ed_uav_perception/target_pipeline.py` | `future_vehicle` 阈值 0.0→-0.5s | 仿真时序抖动 |
| `ed_uav_perception/target_input.py` | `vehicle_acquisition_regression` 阈值 0.0→-0.5s | 仿真时钟非严格单调 |
| `ed_uav_gazebo/sim_car_controller.py` | `frame_id` "world"→"vehicle_start" | 验证器要求 "vehicle_start" |
| `ed_uav_mission/d_task_reducer.py` | `valid=False` 不再中断任务 | 没看到 tag 是正常情况，不应 abort |

### 2.5 诊断工具

新增文件：
- `ros2_ws/src/ed_uav_gazebo/ed_uav_gazebo/camera_debug.py` — 相机帧保存 + 实时 ArUco 检测
- `tools/test_sim_tag_detection.py` — 离线批量检测 saved frames（带可视化）

---

## 三、验证结果

### 3.1 离线检测（Docker 内，1280×960 帧）

```
Frames 17-29: 12/13 检测成功（仅 frame_21 失败）
```

### 3.2 camera_debug 实时检测（仿真中）

```
Frames 17-29: 成功检测到 tag id=0（搜索阶段）
```

### 3.3 target_observation_node 实时检测

```
搜索阶段：❌ 收到全灰图像（min=89, max=108），无法检测
返航阶段：✅ 检测到 tag（quality=0.903），但已过搜索窗口
```

---

## 四、当前未解决的核心问题

### 问题 1：target_observation_node 在搜索阶段收到全灰图像

**现象**：
- camera_debug 订阅同一话题 `/camera/narrow/image_raw`，收到正常图像（min=0, max=255）
- target_observation_node 同时收到灰色图像（min=89, max=108）
- 两者 QoS 配置已统一为 RELIABLE depth=10
- 搜索结束后（~10秒），target_observation_node 才开始收到正常图像

**可能原因**：
1. Gazebo 相机传感器在高分辨率（1280×960）下初始化慢，前 ~60 帧为灰色
2. target_observation_node 的 `use_sim_time=true` 导致时钟同步问题
3. ros_gz_bridge 的 lazy 初始化行为（bridge 日志显示 `Lazy 0`）
4. 两个节点虽然订阅同一话题，但 ROS2 DDS 的 QoS 匹配可能因 `use_sim_time` 而有差异

**建议排查方向**：
1. 先确认相机分辨率：当前 `model.sdf` 中是 **640×480**（被我误改回去的），需要恢复到 **1280×960**（之前验证可识别的分辨率）
2. 测试 `use_sim_time=false` 对 target_observation_node 的影响
3. 在 bridge.yaml 中显式指定 QoS：
   ```yaml
   - ros_topic_name: /camera/narrow/image_raw
     ...
     publisher_qos:
       reliability: RELIABLE
       durability: VOLATILE
       depth: 10
   ```
4. 增加相机 update_rate（当前 narrow=15Hz, wide=10Hz）

### 问题 2：相机帧率低

用户观察到：
- 广角相机 ~2fps
- 窄角相机 ~3fps

**原因**：Gazebo Fortress 的 OGRE2 渲染引擎在高分辨率下性能差。

**建议**：
- 如果 1280×960 下帧率太低，可尝试 960×720 折中分辨率
- 或者只保留窄角相机用于 tag 检测，禁用广角相机减少渲染负担

---

## 五、当前文件修改清单

| 文件 | 修改状态 | 说明 |
|------|---------|------|
| `models/apriltag_marker/materials/textures/tag36h11_0.png` | ✅ 已更新 | 10×10→500×500 (drawMarker) |
| `models/ed_quadrotor/model.sdf` | ⚠️ 需要恢复 | 相机位置已改到机体外，但分辨率被误改为640×480 |
| `ed_uav_perception/apriltag_detector.py` | ✅ 已更新 | ArUco 参数调优 |
| `ed_uav_perception/target_observation_node.py` | ✅ 已更新 | QoS 修复 + 调试日志 |
| `ed_uav_perception/target_pipeline.py` | ✅ 已更新 | future_vehicle 阈值放宽 |
| `ed_uav_perception/target_input.py` | ✅ 已更新 | vehicle_acquisition_regression 阈值放宽 |
| `ed_uav_mission/d_task_ros.py` | ✅ 已更新 | QoS 修复 |
| `ed_uav_mission/d_task_reducer.py` | ✅ 已更新 | valid=False 不中断任务 |
| `ed_uav_gazebo/sim_car_controller.py` | ✅ 已更新 | frame_id 修复 |
| `ed_uav_gazebo/setup.py` | ✅ 已更新 | camera_debug 入口 |
| `ed_uav_gazebo/launch/sim.launch.py` | ✅ 已更新 | camera_debug 节点 |
| `ed_uav_gazebo/ed_uav_gazebo/camera_debug.py` | ✅ 新增 | 诊断工具 |
| `tools/test_sim_tag_detection.py` | ✅ 新增 | 离线检测工具 |

---

## 六、用户要求（原文）

> 调整一下gazebo内小车靶上的tag的渲染方式，不要使用平滑算法不然糊的要命，分辨率本身就低
> 同时分析一下现在的模拟相机链路可不可以正常识别tag，这个等把tag渲染方式改了再测试

> tag比之前清晰了点，但是为啥现在飞机又没法飞了，不飞我怎么看识别效果

> 这次编号17-29有完整tag，请你继续调试

> 是不是图片分辨率太低了，才640x480，我建议是提高模拟相机的分辨率

> 不是，我感觉单纯是虚拟相机加载慢吧，尤其是广角帧率很低就2fps不到，窄视角相机也就3fps，你别改相机分辨率万一改了识别效果又不行了，要么你就确保相机降分辨率后依旧可以稳定识别

---

## 七、下一步建议

1. **恢复相机分辨率到 1280×960**（`model.sdf` 第29行），这是之前验证可识别的分辨率
2. **解决 target_observation_node 灰图问题**：排查 QoS/use_sim_time/bridge lazy 初始化
3. **如果帧率确实太低**：尝试 960×720 折中，或只用窄角相机
4. **清理调试日志**：target_observation_node 和 camera_debug 中的大量日志输出
5. **运行 GUI 仿真验证**：确认 tag 在 Gazebo 中显示清晰，target_observation_node 能实时检测
