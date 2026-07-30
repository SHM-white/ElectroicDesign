# 协议采用风险文档

## 概述

本文档记录了从本地 `embedded/` 固件迁移到委托固件（`readonly/embedded/`）的关键风险和注意事项。

## 迁移决策

- **委托固件版本**：`readonly` 子模块提交 `203ea042a3498402e0c4c0e2035ae117b18e6091`（2026-07-30）
- **删除的本地代码**：`embedded/` 文件夹（包括 `car_esp32s3/`、`ground_station_esp32s3/`、`shared_protocol/`、`esp32s3_cam/`）
- **协议变更**：从大端序 EDU1 协议迁移到小端序 0x4454 协议

## 关键风险

### 1. 二进制不兼容

| 属性 | 旧协议（已删除） | 新协议（委托） |
|------|------------------|----------------|
| 魔数 | `EDU1` | `0x4454` |
| 字节序 | 大端序 | 小端序 |
| 发送方 ID | 8字节 ASCII 字符串 | 4字节 uint32 |
| 最大载荷 | 256 字节 | 64 字节 |
| HMAC 标签 | 16 字节 | 8 字节 |

**风险**：任何使用旧协议的固件或工具都无法与新 ROS 桥接器通信。

### 2. HMAC 标签长度限制

新协议使用 8 字节 HMAC-SHA256 标签（取前 8 字节），而非完整的 32 字节。

**风险**：认证强度降低，但对于比赛场景足够。

**缓解措施**：使用至少 32 字节的随机密钥。

### 3. 不可用的航向和偏航率字段

委托车辆协议不提供 `heading_rad` 和 `yaw_rate_rad_s` 字段。

**映射**：
- `heading_rad` → `0.0`
- `yaw_rate_rad_s` → `0.0`
- `frame_id` → `vehicle_start`

**风险**：视觉定位模块无法获得车辆朝向信息。

**缓解措施**：视觉模块需要独立计算朝向，或使用其他传感器。

### 4. 不可用的视觉有效状态

委托协议不提供 `MISSION_VISION_VALID` 状态位。

**当前行为**：该标志位始终为 0，直到 ROS 获得定义明确的视觉健康输入。

**风险**：HMI 无法显示视觉系统状态。

**缓解措施**：需要在未来添加视觉健康输入接口。

### 5. 任务身份绑定

委托协议要求 `MISSION_STATUS` 同时绑定车辆 `boot_id` 和 HMI `boot_id`。

**要求**：
- 配置的 `mission_id`、`mission_profile_id`、`deployment_preset_id` 必须与活跃的 `ed_uav_mission` 配置匹配
- 选择服务只在接受匹配身份时才能接受任务选择

**风险**：配置不匹配会导致任务选择被拒绝。

### 6. 闭网通信要求

所有 UDP 通信必须在闭网环境中进行，禁止广播。

**要求**：
- WPA2 热点
- MAC-DHCP 保留地址
- AP 客户端隔离策略允许车辆到 HMI 的单播

**风险**：网络配置错误会导致通信失败。

## 部署检查清单

### 硬件验证

- [ ] 车辆固件按 `readonly/embedded/BUILD.md` 编译和烧录
- [ ] 地面站固件按 `readonly/embedded/BUILD.md` 编译和烧录
- [ ] 屏幕显示和触摸方向验证
- [ ] 车辆接线和传感器校准
- [ ] 闭网通信测试

### 软件验证

- [ ] ROS 桥接器测试通过（`python3 -m pytest test/`）
- [ ] 委托协议测试通过（`readonly/tests/run_tests.ps1`）
- [ ] 委托草图编译通过（`readonly/tests/compile_sketches.ps1`）
- [ ] 车辆遥测到达 ROS
- [ ] HMI 接收新鲜的预启动/选择/解锁/运行/故障状态
- [ ] 重放、错误 HMAC、错误 CRC、错误源端点和陈旧数据被拒绝
- [ ] 物理车辆安全停车保持锁定
- [ ] HMI 动作无法解锁、启动或直接控制无人机

### 配置验证

- [ ] 车辆 Sender ID：`0x43415231`（"CAR1"）
- [ ] HMI Sender ID：`0x484D4931`（"HMI1"）
- [ ] ROS Sender ID：`0x524F5331`（"ROS1"）
- [ ] HMAC 密钥：至少 32 字节随机值
- [ ] 车辆 IP：`192.168.20.2`
- [ ] HMI IP：`192.168.20.3`
- [ ] ROS IP：`192.168.20.1`
- [ ] 车辆端口：`42001`
- [ ] HMI 端口：`42002`
- [ ] ROS 端口：`42000`

## 故障排查

### 通信失败

1. 检查网络配置（IP、端口、WPA2）
2. 检查 HMAC 密钥是否匹配
3. 检查 Sender ID 是否正确
4. 检查固件版本是否一致

### 任务选择被拒绝

1. 检查车辆 `boot_id` 是否匹配
2. 检查 HMI `boot_id` 是否匹配
3. 检查任务编号是否为 1 或 2
4. 检查 ROS 任务配置是否匹配

### 安全停车

1. 检查车辆传感器（灰度、编码器）
2. 检查电机驱动
3. 检查电池电压
4. 检查通信超时

## 参考文档

- [委托协议规范](readonly/embedded/shared_protocol/PROTOCOL_V1.md)
- [车辆使用说明](readonly/embedded/car_esp32s3/CAR_ESP32S3_USER_GUIDE.md)
- [地面站使用说明](readonly/embedded/ground_station_esp32s3/GROUND_STATION_USER_GUIDE.md)
- [硬件清单](readonly/embedded/HARDWARE_CHECKLIST.md)
- [构建说明](readonly/embedded/BUILD.md)
