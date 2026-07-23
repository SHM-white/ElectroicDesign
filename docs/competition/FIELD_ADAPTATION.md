# Field Adaptation Guide

Use this procedure when an official task appears. It is intentionally parameter-driven and does not encode a 2026 story.

| Step | Operator decision | Reusable capability or gate | Evidence |
| --- | --- | --- | --- |
| 1 | Record the official task, equipment notice, and rule revision before selecting hardware | the stored 2026 equipment notice confirms context but does not replace task rules | [[C-2026-CAMERA-WIRELESS]] [[C-2026-LIDAR]] |
| 2 | Enter arena dimensions, takeoff pose, zones, landmarks, and mission target data into a field profile | historical coverage/patrol/inspection patterns are examples only | [[C-HIST-2021]] |
| 3 | Select coverage, inspection, target-visit, or air-ground reporting plugins from the released objective | no hard-coded route or target semantics | [[C-HIST-2023]] |
| 4 | Use the two independent UVC cameras as monocular sensing lanes; calibrate each resolution and mount before activation | camera capability is retained, calibration remains physical-work pending | [[C-2026-CAMERA-WIRELESS]] |
| 5 | Keep the operator flow on onboard compute with embedded display/buttons and wireless reporting as applicable | no PC is required during the scored flow | [[C-2026-NO-LAPTOP]] |
| 6 | Set lidar to disabled until current rules explicitly allow it and target measurements qualify it | camera-only is the default safe profile | [[C-2026-LIDAR]] |
| 7 | Enable vehicle, payload, or spares only when the released scenario selects them | purchase/installation remains scenario-gated | [[C-BOM-SCENARIO-GATE]] |

Physical calibration, USB enumeration, propulsion sizing, mass, power, thermal, and endurance measurements remain P24-P29 work; this guide does not turn catalogue or plan values into measurements. [[C-BOM-UNKNOWN]]
