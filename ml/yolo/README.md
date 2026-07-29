# 隔离的 YOLO 合约

此目录是唯一由 P12 负责的 YOLO 训练与导出边界。它将 Ultralytics Git 源固定为
`7a159ea24ec94c47cf25c75785e0a56e47ba4e7b`，与
`docs/provenance/third-party-sources.json` 中的 P04 一致。固定的源代码采用
`AGPL-3.0-only`；内部竞赛使用并不会免除对分发、远程访问、模型工件或对应源代码的审查义务。

ROS 产品不会导入此软件包或 Ultralytics。P12 导出与提供方无关的值合约，结构设计为今后可适配
`vision_msgs/Detection2DArray`；该 ROS 运行时集成由 P15 负责。

## 合约文件

- `schemas/dataset-manifest.schema.json` 描述不可变的源、许可证、哈希、类别映射以及
  train/val/test 样本记录。
- `schemas/model-manifest.schema.json` 描述绑定数据集的模型、预处理、ONNX/OpenVINO 运行时元数据、
  工件摘要，以及固定的训练提供方来源信息。
- `src/yolo_contract/schema.py` 是可执行的严格解析器。它会拒绝未知元数据、数据集划分重叠、重复哈希、
  署名缺失、类别漂移、未固定的提供方、缺失的预处理信息以及路径逃逸。

此处不包含任何已批准的数据集、模型权重、训练模型、ONNX 图或 OpenVINO IR。测试只使用临时文本工件
验证哈希处理；这些工件不是模型权重，并会由 pytest 删除。

## 命令

隔离环境通过 `pyproject.toml` 有意固定：

```bash
uv sync --project ml/yolo --frozen
```

根据 P04 固定的源生成经过审查的锁文件后，使用：

```bash
cd ml/yolo
ed-yolo train --dataset DATASET.json --model MODEL.json --dry-run
ed-yolo validate --dataset DATASET.json --model MODEL.json --dry-run
ed-yolo export --model MODEL.json --format onnx --output OUT.onnx --dry-run
ed-yolo detect-mock --dataset DATASET.json --model MODEL.json \
  --image-id FRAME --image-sha256 SHA256 --frame-id camera_narrow_optical_frame
```

`train`、`validate` 和 `export` 必须使用 `--dry-run`；它们只验证计划，不会下载权重、训练、计算
mAP/latency 或写入导出文件。`detect-mock` 会验证模型工件摘要，并输出一个确定性的、与提供方无关的
检测结果。工件损坏或配置的提供方失败时会返回错误，不会输出误导性的检测结果。

清理安全的验收驱动程序会针对临时合成清单执行以下完全相同的 CLI 命令，并在之后删除所有夹具数据：

```bash
PYTHONPATH=ml/yolo/src ./.venv/bin/python ml/yolo/tests/manual_cli_smoke.py
```

## 运行时边界

`runtime.format` 只能是 `onnx` 或 `openvino`。其预处理元数据是必需的，包括颜色空间、张量布局、
缩放宽度/高度/策略以及缩放比例。任何未来的 ONNX 或 OpenVINO 提供方都必须实现 `DetectionProvider`，
不得将原始边界框转换为车辆位姿。相机标定和终端几何仍由 P15 负责。
