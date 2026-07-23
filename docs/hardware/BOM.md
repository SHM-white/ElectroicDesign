# Competition BOM

This BOM is a planning inventory, not a mass, power, or flight-readiness claim. `unknown` is deliberately not converted to a measured zero. Values remain per unit; aggregate known totals multiply each known value by quantity. Every row maps to [`BOM.json`](BOM.json).

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

All physical values remain unknown until P24-P29 measurements. The numeric known-value sums are zero only because there are no measured line items, not because any component has zero mass or power. [[C-BOM-UNKNOWN]]
