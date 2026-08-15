# D 题组合运行链路

## 三端网络

实机固定在同一封闭热点网段：NUC/ROS `192.168.20.1:42000`、小车
`192.168.20.2:42001`、地面站 HMI `192.168.20.3:42002`。三端使用同一 DTask UDP v1
帧格式和 HMAC 密钥，每个发送端带独立的随机 `boot_id` 与单调序号。

```text
小车 ──CAR_TELEMETRY/HEARTBEAT──▶ NUC
  └──同一个已签名包──────────────▶ HMI

HMI ──TASK_SELECTION/HEARTBEAT──▶ NUC

NUC ──MISSION_STATUS/HEARTBEAT──▶ HMI
  └──MISSION_STATUS/HEARTBEAT───▶ 小车
```

小车必须直接双发遥测。NUC 不能把小车原始包转发给 HMI，因为 UDP 源端点会变成 NUC，HMI 应按
端点绑定规则拒绝该包。心跳只维持对端在线状态，不会刷新 ROS 对小车业务遥测的 0.75 s 新鲜度。

## 启动时序

1. 小车发出当前 `boot_id` 的 READY 遥测，HMI 和 NUC 建立同一车辆会话。
2. HMI 发送任务 1/2/3 选择；NUC 将启动文件从 YAML 读取到的任务、场地和靶标身份补入
   `/d_task/pre_arm/select_mission`。
3. 任务执行器接受后，NUC 回传同时绑定 `selection_id`、小车 `boot_id` 和 HMI `boot_id` 的确认。
4. 飞控解锁后，权威状态进入 ARMED_READY；任务 1/2/3 使用完全相同的路径，不再有 Task 3 专用
   AUX 或 capability 门禁。
5. 小车物理启动键产生一次 START 事件，NUC 恰好一次发送 `/mission/execute`。重放、旧启动纪元和
   乱序事件不会重复派发任务。
6. 任务状态持续回传 HMI 和小车；小车遥测转换为 `/d_task/vehicle/telemetry`，供任务和视觉相对运动使用。

## 飞行与安全边界

任务执行器通过 `/fcu/flight_command` 调用凌霄 V7 串口桥。SROS2 和调用方网络准入由部署环境负责，
进程内不再检查 keystore、enclave 或 capability 报告，也不要求摇杆居中或 AUX 模式窗口。

实飞代码内只保留一个人工紧急入口：完整新鲜的 V7 RC 帧中 AUX1（第 5 通道）达到
`1800..2000 us` 时，串口所有者立即写一次锁浆命令、抢占正在执行的动作并永久锁存；同一进程内
不能软件解锁。正常任务结束时的 LAND/DISARM 是飞行流程动作，不是额外启动门禁。

## 一键入口

```bash
# 纯 Gazebo 闭环
./tools/run_competition.sh --simulation --task 1 --no-display

# 实机链路；本机必须已有 ROS 2 Humble、本地 HMAC、MID-360 清单和相机计划
./tools/run_competition.sh --real --build
```

实机入口不会用容器网络或模拟节点冒充硬件；缺少本地密钥、雷达清单或标定时会明确退出。
