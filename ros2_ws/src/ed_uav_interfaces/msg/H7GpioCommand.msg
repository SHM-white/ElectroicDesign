# H7 GPIO 开发板输出命令 (STM32H7 大疆电机开发板C, 0xAA 串口协议)。
# 沿用旧激光头控制协议帧, C 板固件无需改动; 控制电磁铁/激光/LED 等引脚。
uint8 CMD_SET_OUTPUT=0x01
uint8 CMD_CONFIGURE=0x02
uint8 CMD_PULSE=0x03
uint8 command
uint8 pin
uint8 value
uint16 period_ms
