# 共享 UDP v1 Arduino 库

这是桥接器经认证 UDP v1 封装的可移植 Arduino/主机实现。它使用明确的网络字节序
序列化，绝不会将数据包强制转换为 C 结构体。

- 最大载荷：256 bytes；最大数据报：306 bytes。
- 头部：`EDU1`、版本、消息类型、载荷长度、8-byte 发送方 ID、非零启动纪元、
  序列号和发送方单调毫秒数。
- CRC16-CCITT-FALSE 覆盖头部和载荷。
- HMAC-SHA256 的前 16 bytes 覆盖头部、载荷和 CRC。密钥必须至少包含 32 bytes。
- 接收方接受前向模序列差值 `1..1024`，使之前的 32 个纪元失效，并使用本地
  接收时间执行 750 ms 过期门限。

固定桥接向量在 `tests/protocol_test.cpp` 中测试。除 C++ 标准库外，本库没有运行时
依赖，适用于 Arduino CLI 的 `--libraries` 用法。
