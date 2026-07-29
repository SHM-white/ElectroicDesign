# D-task 合约文档

所有跨越此边界的文件都会先根据已提交的 Draft 2020-12
schema 进行校验，再解析为冻结的 Pydantic 模型。版本、单位、坐标系、
新鲜度限制、路线顺序、目标几何信息和所有者均为必填项。

已提交的 `examples/` 不包含凭据。真实现场值只能复制到本 README
旁的 `deployment_preset.local.yaml` 中；该路径已被 gitignore 忽略。
系统没有默认串口、IP 地址、固件或 ESP-NOW 对端。缺少本地清单、存在占位符
令牌或使用 RFC 5737 文档地址，都会阻止现场预设生效。

请从仓库根目录校验文档：

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_d_task_config.py \
  deployment ros2_ws/src/ed_uav_interfaces/contracts/d_task/examples/deployment_preset.example.yaml
```
