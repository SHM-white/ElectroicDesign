# 比赛流程模拟使用指南

## 快速开始

### 1. 完整模拟（推荐）
```bash
# 启动热点 + 连接真实小车和地面站 + 自动选择任务1
sudo ./tools/run_competition_sim.sh --auto-task 1

# 启动热点 + 等待地面站手动选择任务
sudo ./tools/run_competition_sim.sh

# 不启动无人机（仅测试小车和地面站通信）
sudo ./tools/run_competition_sim.sh --auto-task 1 --no-drone

# 无人机使用模拟模式（不实际飞行）
sudo ./tools/run_competition_sim.sh --auto-task 1 --dry-run
```

### 2. 仅运行桥接器（热点已启动）
```bash
# 直接运行 Python 桥接器
python3 tools/competition_sim_bridge.py --auto-task 1

# 使用自定义 HMAC 密钥
python3 tools/competition_sim_bridge.py --auto-task 1 --key-file /path/to/key.hex

# 详细输出
python3 tools/competition_sim_bridge.py --auto-task 1 --verbose
```

### 3. 仅测试通信（诊断模式）
```bash
# 启动热点 + 运行通信诊断
sudo ./tools/ed_comm.sh run

# 测试网络连通性
sudo ./tools/ed_comm.sh test

# 查看热点状态
sudo ./tools/ed_comm.sh status
```

## 比赛流程说明

小车无独立启动按钮，上电即自动启动。完整比赛流程：

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   地面站    │     │    ROS      │     │    小车     │
│  (HMI)      │     │  (桥接器)   │     │  (CAR)      │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       │  1. 上电启动      │                   │  1. 上电启动
       │  发送 HEARTBEAT   │                   │  自动开始循迹
       │                   │                   │  发送 CAR_TELEMETRY
       │                   │                   │
       │  2. 显示 PRESTART │                   │
       │  用户选择任务      │                   │
       │  发送 TASK_SELECTION                  │
       │ ─────────────────>│                   │
       │                   │                   │
       │  3. 收到回执      │                   │
       │  SELECTION_ACKED  │                   │
       │ <─────────────────│                   │
       │                   │                   │
       │  4. ARMED_READY   │                   │
       │ <─────────────────│                   │
       │                   │                   │
       │  5. CAR_RUNNING   │                   │  小车已在运行
       │ <─────────────────│<──────────────────│  (跳过等待启动)
       │                   │                   │
       │                   │  6. 事件上报      │
       │                   │  START → B → D    │
       │                   │  → A → COMPLETE   │
       │                   │ <─────────────────│
       │                   │                   │
       │  7. COMPLETE      │                   │
       │ <─────────────────│                   │
       │                   │                   │
```

> **注意**: 小车无启动按钮，上电后自动开始循迹运行。桥接器会跳过等待启动信号，直接进入运行状态。

## 事件序列

小车在赛道上运行时会依次报告以下事件：

| 事件 | 说明 | 赛道位置 |
|------|------|----------|
| START | 按钮启动 | 起点 |
| B | 到达 B 点 | 赛道 B 标记 |
| D | 到达 D 点 | 赛道 D 标记 |
| A | 到达 A 点 | 赛道 A 标记 |
| COMPLETE | 完成一圈 | 回到起点 |

## 故障处理

| 故障码 | 说明 | 处理方式 |
|--------|------|----------|
| 0x0001 | WiFi 超时 | 检查热点和网络连接 |
| 0x0002 | 循迹丢失 | 检查赛道和传感器 |
| 0x0010 | 按钮卡住 | 检查硬件按钮 |
| 0x0020 | 电机故障 | 检查电机驱动 |
| 0x0040 | 数据陈旧 | 检查 UDP 通信 |
| 0x0100 | 未确认选择 | 重新选择任务 |

## 环境变量

| 变量 | 说明 | 默认值 |
|------|------|--------|
| ED_HOTSPOT_SSID | 热点名称 | ED-UAV |
| ED_HOTSPOT_PASSWORD | WPA2 密码 | (开放) |
| ED_HOTSPOT_CHANNEL | 信道 | 6 |
| ED_CAR_MAC | 小车 MAC | (自动) |
| ED_HMI_MAC | 地面站 MAC | (自动) |
| ED_AUTH_KEY_FILE | HMAC 密钥文件 | (示例密钥) |

## 网络配置

| 节点 | IP 地址 | UDP 端口 |
|------|---------|----------|
| ROS (本机) | 192.168.20.1 | 42000 |
| 小车 | 192.168.20.2 | 42001 |
| 地面站 | 192.168.20.3 | 42002 |

## 故障排除

### 1. 热点启动失败
```bash
# 检查无线网卡
nmcli device status | grep wifi

# 手动指定接口
ED_HOTSPOT_IFACE=wlan0 sudo ./tools/run_competition_sim.sh
```

### 2. 设备无法连接
```bash
# 检查热点状态
sudo ./tools/ed_comm.sh status

# 测试连通性
sudo ./tools/ed_comm.sh test

# 查看 DHCP 租约
cat /var/lib/misc/dnsmasq.leases
```

### 3. 端口被占用
```bash
# 查看占用进程
ss -ulnp | grep 42000

# 终止占用进程
kill <PID>
```

### 4. HMAC 密钥不匹配
```bash
# 检查 ESP32 配置文件
cat readonly/embedded/car_esp32s3/config_local.h | grep AUTH_KEY
cat readonly/embedded/ground_station_esp32s3/config_local.h | grep AUTH_KEY

# 使用匹配的密钥文件
sudo ./tools/run_competition_sim.sh --key-file /path/to/matching_key.hex
```

## 相关文件

- `tools/dtask_lib.py` - DTask 协议 Python 实现
- `tools/competition_sim_bridge.py` - 比赛流程模拟桥接器
- `tools/run_competition_sim.sh` - 主启动脚本
- `tools/ed_comm.sh` - 热点和通信管理
- `tools/diagnostics/vehicle_comm_diagnostic.py` - 通信诊断工具
- `readonly/embedded/shared_protocol/PROTOCOL_V1.md` - 协议规范
