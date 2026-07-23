# 2026 Scenario Envelope

This is an adaptation envelope, not a guessed task implementation. `confirmed-context` means the local official 2026 equipment notice supports component availability, but the notice is not a released task rule. `unknown` remains a hard gate.

| Class | Fact or decision | Consequence | Evidence |
| --- | --- | --- | --- |
| Confirmed context | The stored official 2026-07-21 equipment notice lists a camera, short-range wireless communication, and wireless image transmission | retain camera and wireless paths without treating the equipment list as a task rule | [[C-2026-CAMERA-WIRELESS]] |
| Confirmed context | A laptop ground-station dependency is imprudent; a comparable 2025 task explicitly required embedded controls/display and prohibited a PC | scored flow uses embedded display, buttons, and onboard compute | [[C-2026-NO-LAPTOP]] |
| Inferred | Coverage, inspection, recognition, target visit, and air-ground reporting are reusable capability families | select a mission plugin only after rules define the arena | [[C-HIST-2021]] |
| Inferred | A vehicle and payload are useful only for air-ground or delivery/fire variants | keep both as disabled scenario-gated BOM items | [[C-HIST-2022]] |
| Unknown | Lidar permission, required sensing modality, field dimensions, scoring, vehicle, and payload rules | do not mark lidar allowed or procure story-specific hardware | [[C-2026-LIDAR]] |
| Rejected input | The local `2026电赛控制类猜题.pdf` is titled as a guess and contains its own disclaimer | do not classify it as official or derive requirements from it | [[C-2026-GUESS-REJECTED]] |

The equipment notice does not state lidar permission or a required sensing modality. Lidar permission therefore remains unknown and rule-gated; otherwise the camera-only profile remains available. [[C-2026-CAMERA-WIRELESS]] [[C-2026-LIDAR]]
