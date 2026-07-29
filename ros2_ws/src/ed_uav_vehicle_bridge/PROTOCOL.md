# Vehicle/HMI UDP v1 协议

所有整数域均使用网络字节序。数据报采用单播，长度上限为
306 字节。不会解码认证边界之后的任何字段。

| 偏移 | 大小 | 字段 |
| --- | ---: | --- |
| 0 | 4 | 魔数 `EDU1` |
| 4 | 1 | 版本 `1` |
| 5 | 1 | 消息类型：telemetry `1`、selection `2`、ACK `3`、status `4` |
| 6 | 2 | 载荷长度，最大值 256 |
| 8 | 8 | ASCII 发送方 ID，右侧以 NUL 填充 |
| 16 | 8 | 随机且非零的启动纪元（boot epoch） |
| 24 | 4 | 模 `uint32` 序列号 |
| 28 | 4 | 发送方单调毫秒数，仅供参考 |
| 32 | N | 显式类型化的载荷 |
| 32+N | 2 | CRC16-CCITT-FALSE，覆盖报头和载荷 |
| 34+N | 16 | HMAC-SHA256 的前 16 个字节，覆盖报头、载荷和 CRC |

接收方会将每个发送方 ID 及当前启动纪元绑定到其已配置的
源 IP/port。只接受正向的模序列差值 `1..1024`，淘汰之前的 32 个
启动纪元，并使用本地单调接收时间执行 0.75 秒遥测新鲜度门限。
发送方时间绝不能用于建立新鲜度。

载荷中的字符串使用 UTF-8 编码；每个字符串均以一个字节的长度前缀开头，
并受合约边界限制。
遥测固定字段为 `>HBBffffBB`：contract/flags/motion、displacement、
wheel speed、`heading_rad`、有符号的 `yaw_rate_rad_s`、turn class 和 route stage。
Selection 字段为 `>HQQB`；ACK 字段为 `>HQQBBB`；mission-status 字段
为 `>HIBBB`。字符串按 D-task 合约顺序跟随这些固定字段。

黄金向量密钥：字节范围 `00` 到 `1f`；type `1`；sender `CAR-01`；epoch
`0102030405060708`；sequence `fffffffe`；source millis `10203040`；payload
`010203`；CRC `4450`：

```text
45445531010100034341522d303100000102030405060708fffffffe102030400102034450affe1d99aa17475115930f8a10f67f52
```

已提交的示例配置有意设置为不可用。部署时必须提供数字形式的保留对端地址、
自有 UDP 端口、sender ID，以及一个本地文件，其中至少包含 32 个随机密钥字节的十六进制编码。
