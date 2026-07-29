# 竞赛 BOM

本 BOM 是规划清单，不是质量、功率或适航就绪声明。`unknown` 不会被刻意转换为测量值 0。数值均按单件记录；已知汇总值按数量乘以各已知值。每一行均对应 [`BOM.json`](BOM.json)。

| Item | Quantity | Ownership | Procurement status | Spare status | Mass / steady / peak | Connector or interface | Evidence |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Livox Mid-360 | 1 | owned | owned | spare-needed | unknown / unknown / unknown | RJ45; measurement pending | [[C-BOM-OWNED]] |
| 12th-generation i5 onboard computer | 1 | owned | owned | spare-needed | unknown / unknown / unknown | power/storage interfaces pending | [[C-BOM-OWNED]] |
| Narrow USB 2.0 UVC camera | 1 | owned | owned | spare-needed | unknown / unknown / unknown | USB 2.0 | [[C-BOM-OWNED]] |
| Wide USB 2.0 UVC camera | 1 | owned | owned | spare-needed | unknown / unknown / unknown | USB 2.0 | [[C-BOM-OWNED]] |
| Airframe and full prop guards | 1 | replacement-required | replacement-required | spare-needed | unknown / unknown / unknown | mechanical interface to verify | [[C-BOM-UNKNOWN]] |
| Motors, ESCs, and propellers | 1 | replacement-required | replacement-required | spare-needed | unknown / unknown / unknown | motor/ESC interfaces to verify | [[C-BOM-UNKNOWN]] |
| Flight battery and protected distribution | 2 | missing | procure | spare-needed | unknown / unknown / unknown | battery/distribution interface to verify | [[C-BOM-UNKNOWN]] |
| Compute and sensor power conversion | 1 | missing | procure | spare-needed | unknown / unknown / unknown | input/output interfaces to verify | [[C-BOM-UNKNOWN]] |
| Onboard storage | 1 | missing | procure | spare-needed | unknown / unknown / unknown | storage interface to verify | [[C-BOM-UNKNOWN]] |
| Power, USB, Ethernet, and serial cabling | 1 | missing | procure | spare-needed | unknown / unknown / unknown | connectors documented per cable | [[C-BOM-UNKNOWN]] |
| Lidar, camera, and compute mounts | 1 | missing | procure | spare-needed | unknown / unknown / unknown | mechanical interfaces to verify | [[C-BOM-UNKNOWN]] |
| Calibration target and measurement tools | 1 | missing | procure | spare-needed | unknown / unknown / unknown | tooling interfaces not applicable | [[C-BOM-UNKNOWN]] |
| Embedded display and button UI | 1 | missing | procure | spare-needed | unknown / unknown / unknown | power/data interface to verify | [[C-2026-NO-LAPTOP]] |
| Wireless link | 1 | missing | procure | spare-needed | unknown / unknown / unknown | radio interface to verify | [[C-2026-CAMERA-WIRELESS]] |
| Field spares kit | 1 | missing | procure | spare-needed | unknown / unknown / unknown | depends on selected hardware | [[C-BOM-UNKNOWN]] |
| Ground vehicle | 1 | scenario-gated | scenario-gated | scenario-gated | unknown / unknown / unknown | not selected until rules require it | [[C-BOM-SCENARIO-GATE]] |
| Payload mechanism | 1 | scenario-gated | scenario-gated | scenario-gated | unknown / unknown / unknown | not selected until rules require it | [[C-BOM-SCENARIO-GATE]] |

| Aggregate | Value | Evidence |
| --- | --- | --- |
| Line items | 17 | [[C-BOM-UNKNOWN]] |
| Planned quantity | 18 | [[C-BOM-UNKNOWN]] |
| Known mass | 0 g across 0 measured line items | [[C-BOM-UNKNOWN]] |
| Known steady power | 0 W across 0 measured line items | [[C-BOM-UNKNOWN]] |
| Known peak power | 0 W across 0 measured line items | [[C-BOM-UNKNOWN]] |
| Unknown mass / steady / peak line items | 17 / 17 / 17 | [[C-BOM-UNKNOWN]] |

在完成 P24-P29 测量前，所有实体数值仍为未知。已知数值总和为 0，仅因为没有已测量的行项目，并不表示任何部件的质量或功率为 0。[[C-BOM-UNKNOWN]]
