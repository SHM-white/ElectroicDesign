# 场地配置

每个 YAML 文件在任务使用前都会由 `ed_uav_localization` 解析。
坐标使用 REP-103 `map` ENU 坐标系中的米，航向角使用弧度。
该 schema 会拒绝未声明的字段、旧单位、重复 ID、无效的多边形拓扑、相互重叠的禁飞区，
以及无法支持完整平面位姿的场地几何信息。

`historical_2021_example.yaml` 是根据旧资料整理的、已阻止使用的历史示例。
它不是赛场测量数据，也不是可激活的配置。`unknown_arena.yaml` 有意不包含几何信息，
在录入正式场地数据前会保持阻止使用。

使用以下命令验证目录：

```bash
python3 tools/validate_field_profile.py --all config/fields
```
