# D-task Contract Documents

All files crossing this boundary are checked against the committed Draft 2020-12
schema and then parsed into a frozen Pydantic model. Versions, units, frames,
freshness limits, route order, target geometry, and owners are mandatory.

The committed `examples/` are credential-free. Copy real field values only into
`deployment_preset.local.yaml` beside this README; that path is gitignored.
There is no default serial, IP address, firmware, or ESP-NOW peer. A missing
local manifest, placeholder token, or RFC 5737 documentation address blocks a
field preset.

Validate a document from the repository root:

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_d_task_config.py \
  deployment ros2_ws/src/ed_uav_interfaces/contracts/d_task/examples/deployment_preset.example.yaml
```
