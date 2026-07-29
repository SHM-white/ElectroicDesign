# 竞赛 BOM

本 BOM 是规划清单，不是质量、功率或适航就绪声明。`unknown` 不会被刻意转换为测量值 0。数值均按单件记录；已知汇总值按数量乘以各已知值。每一行均对应 [`BOM.json`](BOM.json)。

| 项目 | 数量 | 所有权 | 采购状态 | 备件状态 | 质量 / 稳态 / 峰值 | 连接器或接口 | Evidence |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Livox Mid-360 激光雷达 | 1 | owned | owned | spare-needed | unknown / unknown / unknown | RJ45；待测量 | [[C-BOM-OWNED]] |
| 第 12 代 i5 机载计算机 | 1 | owned | owned | spare-needed | unknown / unknown / unknown | 电源/存储接口待确认 | [[C-BOM-OWNED]] |
| 窄视场 USB 2.0 UVC 摄像头 | 1 | owned | owned | spare-needed | unknown / unknown / unknown | USB 2.0 | [[C-BOM-OWNED]] |
| 宽视场 USB 2.0 UVC 摄像头 | 1 | owned | owned | spare-needed | unknown / unknown / unknown | USB 2.0 | [[C-BOM-OWNED]] |
| 机架和全套桨叶防护罩 | 1 | replacement-required | replacement-required | spare-needed | unknown / unknown / unknown | 待核验机械接口 | [[C-BOM-UNKNOWN]] |
| 电机、ESC 和螺旋桨 | 1 | replacement-required | replacement-required | spare-needed | unknown / unknown / unknown | 待核验电机/ESC 接口 | [[C-BOM-UNKNOWN]] |
| 飞行电池和受保护配电 | 2 | missing | procure | spare-needed | unknown / unknown / unknown | 待核验电池/配电接口 | [[C-BOM-UNKNOWN]] |
| 计算机和传感器电源转换 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 待核验输入/输出接口 | [[C-BOM-UNKNOWN]] |
| 机载存储 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 待核验存储接口 | [[C-BOM-UNKNOWN]] |
| 电源、USB、以太网和串口线缆 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 按线缆记录连接器 | [[C-BOM-UNKNOWN]] |
| 激光雷达、摄像头和计算机安装座 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 待核验机械接口 | [[C-BOM-UNKNOWN]] |
| 标定靶和测量工具 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 不适用工具接口 | [[C-BOM-UNKNOWN]] |
| 机载显示和按键界面 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 待核验电源/数据接口 | [[C-2026-NO-LAPTOP]] |
| 无线链路 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 待核验无线接口 | [[C-2026-CAMERA-WIRELESS]] |
| 赛场备件包 | 1 | missing | procure | spare-needed | unknown / unknown / unknown | 取决于所选硬件 | [[C-BOM-UNKNOWN]] |
| 地面车辆 | 1 | scenario-gated | scenario-gated | scenario-gated | unknown / unknown / unknown | 规则要求前不选用 | [[C-BOM-SCENARIO-GATE]] |
| 载荷机构 | 1 | scenario-gated | scenario-gated | scenario-gated | unknown / unknown / unknown | 规则要求前不选用 | [[C-BOM-SCENARIO-GATE]] |

| 汇总项 | 数值 | Evidence |
| --- | --- | --- |
| 行项目 | 17 | [[C-BOM-UNKNOWN]] |
| 计划数量 | 18 | [[C-BOM-UNKNOWN]] |
| 已知质量 | 0 g，涉及 0 个已测量行项目 | [[C-BOM-UNKNOWN]] |
| 已知稳态功率 | 0 W，涉及 0 个已测量行项目 | [[C-BOM-UNKNOWN]] |
| 已知峰值功率 | 0 W，涉及 0 个已测量行项目 | [[C-BOM-UNKNOWN]] |
| 质量 / 稳态 / 峰值未知的行项目 | 17 / 17 / 17 | [[C-BOM-UNKNOWN]] |

在完成 P24-P29 测量前，所有实体数值仍为未知。已知数值总和为 0，仅因为没有已测量的行项目，并不表示任何部件的质量或功率为 0。[[C-BOM-UNKNOWN]]
