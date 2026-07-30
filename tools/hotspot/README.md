# ED UAV 热点管理工具

## 硬件要求

> **重要**：NUC 内置无线网卡**不能同时开启热点和连接 Wi-Fi**。
> 必须插入一个 **USB 无线网卡**作为 AP 热点，内置网卡保持连接互联网（可选）。

```
┌──────────────────────────────────────────┐
│  NUC                                     │
│  ├─ 内置 wlan0 → STA（连互联网，可选）    │
│  └─ USB   wlan1 → AP  （ED-UAV 热点）    │
└──────────────────────────────────────────┘
```

脚本会自动检测多个无线接口：已连接的做 STA，空闲的做 AP。
也可通过 `ED_HOTSPOT_IFACE=wlan1` 手动指定 AP 接口。

## 快速开始

```bash
# 1. 创建热点（首次使用）
sudo ./tools/hotspot/setup_hotspot.sh create

# 2. 查看状态
sudo ./tools/hotspot/setup_hotspot.sh status

# 3. 测试连通性
sudo ./tools/hotspot/setup_hotspot.sh test

# 4. 运行通信诊断
python3 tools/diagnostics/vehicle_comm_diagnostic.py
```

## 命令一览

| 命令 | 说明 |
|------|------|
| `create` | 创建热点 + 防火墙 + DHCP + 开机自启 |
| `remove` | 删除全部热点配置 |
| `start` | 启动热点 |
| `stop` | 关闭热点 |
| `status` | 查看当前状态和已连接客户端 |
| `enable` | 设置开机自动启动 |
| `disable` | 取消开机自启 |
| `test` | 测试网络连通性 |

## 配置方式

### 方式一：环境变量

```bash
export ED_HOTSPOT_SSID="ED-UAV"
export ED_CAR_MAC="AA:BB:CC:DD:EE:FF"
export ED_HMI_MAC="11:22:33:44:55:66"
sudo -E ./tools/hotspot/setup_hotspot.sh create
```

### 方式二：配置文件

```bash
cp tools/hotspot/hotspot.example.env tools/hotspot/hotspot.local.env
# 编辑 hotspot.local.env
source tools/hotspot/hotspot.local.env
sudo -E ./tools/hotspot/setup_hotspot.sh create
```

## 网络拓扑

```
┌───────────────────────────────────────┐
│  NUC (AP)   192.168.20.1:42000       │
│              SSID: ED-UAV             │
└────────┬──────────────────┬───────────┘
         │ Wi-Fi            │ Wi-Fi
┌────────┴─────────┐ ┌──────┴──────────┐
│  CAR (STA)       │ │  HMI (STA)      │
│  192.168.20.2    │ │  192.168.20.3   │
│  :42001          │ │  :42002         │
└──────────────────┘ └─────────────────┘
```

## DHCP 静态绑定

设置 ESP32 的 MAC 地址后，热点会为设备分配固定 IP：

| 设备 | IP | MAC 来源 |
|------|----:|----------|
| NUC | .1 | （热点自身） |
| CAR | .2 | `ED_CAR_MAC` |
| HMI | .3 | `ED_HMI_MAC` |

未设置 MAC 的设备会从 `192.168.20.10~50` 获得动态 IP。

## 注意事项

- ESP32 只支持 2.4GHz，`ED_HOTSPOT_BAND` 必须为 `bg`
- 客户端互通已通过 iptables FORWARD 规则启用
- 比赛场地应提前扫描 Wi-Fi 信道，选择干扰最少的 1/6/11
- 正式环境必须启用 WPA2 密码
- 如需外网访问（如 apt update），NUC 需通过有线连接外网，脚本会自动配置 NAT
