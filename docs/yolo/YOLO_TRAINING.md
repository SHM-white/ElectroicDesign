# YOLO 训练与部署操作手册

> 来源：`ml/yolo/`（契约层）、`ros2_ws/src/ed_uav_perception/`（运行时）、
> `THIRD_PARTY_NOTICES.md`（许可证固定）、`docs/legal/OPEN_SOURCE.md`（发布门槛）。

---

## 1. 许可，AGPL-3.0 义务

Ultralytics 固定为：

| 字段 | 值 |
|---|---|
| 代码库 | `https://github.com/ultralytics/ultralytics.git` |
| 修订版本 | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| 许可证 | **AGPL-3.0-only** |
| SHA-256 | `0d96a4ff68ad6d4b6f1f30f713b18d5184912ba8dd389f86aa7710db079abcb0` |

### AGPL 影响

- **分发**：如果分发使用 Ultralytics 训练的模型权重，也必须分发对应的 Ultralytics 源代码。
- **远程访问**：如果模型作为网络服务使用（例如推理 API），必须向用户提供源代码。
- **模型工件**：ONNX/OpenVINO 导出物是衍生作品，承担 AGPL 义务。
- **项目措施**：ML 环境（`ml/yolo/`）与 ROS 进程**隔离**。`ed_uav_perception` 不导入 Ultralytics。推理只使用与提供方无关的 ONNX/OpenVINO 运行时。

### 发布门槛

在分发任何模型工件前，审阅：
`docs/legal/OPEN_SOURCE.md` — 工件组成、GPL/AGPL 条件、修改、源代码提供机制、模型/数据集许可证。

---

## 2. 项目布局

```
ml/yolo/
├── pyproject.toml                    # 软件包：ed-yolo-contract v0.1.0
├── schemas/
│   ├── dataset-manifest.schema.json  # 数据集清单的 JSON Schema
│   └── model-manifest.schema.json    # 模型清单的 JSON Schema
├── src/yolo_contract/
│   ├── __init__.py                   # 公共 API 表面
│   ├── models.py                     # 冻结数据类（DatasetManifest、ModelManifest 等）
│   ├── schema.py                     # 严格解析器、分区验证
│   ├── runtime.py                    # 与提供方无关的 ONNX/OpenVINO 协议
│   ├── cli.py                        # 试运行 CLI（train、validate、export、detect-mock）
│   ├── errors.py                     # 类型化错误层级
│   └── jsonio.py                     # JSON 加载 + SHA-256 哈希
└── tests/
    ├── test_schema.py                # 清单解析器测试
    ├── test_runtime.py               # 运行时确定性测试
    └── test_cli.py                   # CLI 集成测试
```

**不存在训练权重、数据集文件或实际训练脚本。**契约层定义模式和验证，训练执行属于未来工作。

---

## 3. 数据集采集

### 3.1 图像来源

| 来源 | 用途 | 分辨率 |
|---|---|---|
| `/camera/narrow/image_raw` | 窄视场检测 | 按运行时配置 |
| `/camera/wide/image_raw` | 宽视场边界/定位 | 按运行时配置 |
| 手动采集（USB 摄像头） | 离线构建数据集 | 匹配目标分辨率 |

### 3.2 采集指南

- 使用与目标运行模式**相同的分辨率**采集
- 包含不同光照（室内、室外、阴影、直射阳光）
- 包含不同角度（相对正视方向 0°–60°）
- 包含部分遮挡（目标的 10%–50%）
- 包含负样本（看不到目标）
- 初始训练每类至少 **200 张图像**

### 3.3 存储

```
datasets/
└── marker-v1/
    ├── images/
    │   ├── train/    # 70%
    │   ├── val/      # 15%
    │   └── test/     # 15%
    └── labels/
        ├── train/    # YOLO format: class_id cx cy w h (normalized)
        ├── val/
        └── test/
```

---

## 4. 标注指南

### 4.1 工具

使用 [CVAT](https://cvat.ai/) 或 [Label Studio](https://labelstud.io/) 进行标注。以 YOLO 格式导出。

### 4.2 边界框规则

| 规则 | 描述 |
|---|---|
| 紧密贴合 | 边界框应位于对象边界内 1–2 个像素处 |
| 完整对象 | 包含整个对象，即使对象部分被遮挡 |
| 遮挡部分 | 按对象完全可见时的样子绘制边界框 |
| 截断对象 | 可见部分 >30% 时标注；<30% 时跳过 |
| 重叠边界框 | 允许，每个对象都有自己的边界框 |

### 4.3 类别定义

在数据集清单的 `class_map` 中定义类别：

```json
{
  "class_map": [
    {"id": 0, "name": "marker"},
    {"id": 1, "name": "terminal_target"},
    {"id": 2, "name": "obstacle"}
  ]
}
```

**约束**：类别 ID 必须从 0 开始连续排列。类别名称必须使用小写字母和下划线。

---

## 5. 不可变的训练/验证/测试分区

### 5.1 分区要求

摘自 `schema.py::load_dataset_manifest()`：

| 要求 | 强制方式 |
|---|---|
| 三个分区均存在 | 任一分区为空则为 `MissingMetadataError` |
| 分区之间无哈希重叠 | SHA-256 出现在多个分区时为 `SplitOverlapError` |
| 分区内无重复哈希 | `DuplicateHashError` |
| 每个样本都有许可证 | `MissingMetadataError` |
| 每个样本都有 SHA-256 | 模式验证失败 |

### 5.2 数据集清单格式

```json
{
  "schema_version": 1,
  "dataset_id": "marker-v1-2026-07",
  "source": {
    "url": "https://github.com/org/datasets/marker-v1",
    "revision": "abc123...",
    "license": "CC-BY-4.0"
  },
  "class_map": [
    {"id": 0, "name": "marker"}
  ],
  "samples": [
    {
      "id": "img_001",
      "split": "train",
      "sha256": "a1b2c3...",
      "source_url": "https://...",
      "license": "CC-BY-4.0",
      "class_ids": [0]
    }
  ]
}
```

### 5.3 验证

```bash
# 验证数据集清单
cd ml/yolo
python -m yolo_contract validate --dataset datasets/marker-v1/dataset.json
```

模式解析器拒绝：
- 浮动修订版本（必须固定为 SHA-1）
- 缺少许可证归属信息
- 分区重叠（同一图像同时位于 train 和 val）
- 样本 URL 中的路径遍历

---

## 6. 训练流程

### 6.1 环境设置

```bash
# 创建独立的 ML 环境（与 ROS 分离）
cd ml/yolo
python -m venv .venv
source .venv/bin/activate
pip install -e ".[train]"
```

这只会在独立的 `ml/yolo` 环境中安装 Ultralytics（AGPL-3.0）。

### 6.2 训练数据配置（YOLO 格式）

创建 `data.yaml`：

```yaml
path: /absolute/path/to/datasets/marker-v1
train: images/train
val: images/val
test: images/test
names:
  0: marker
```

### 6.3 训练命令

```bash
# YOLOv8n（nano），推荐用于 Intel i5 推理
yolo detect train \
  model=yolov8n.pt \
  data=data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=8 \
  device=cpu \
  project=runs/train \
  name=marker-v1
```

| 参数 | 值 | 理由 |
|---|---|---|
| `model` | `yolov8n.pt` | Nano，适合 Intel i5 CPU 推理预算 |
| `imgsz` | 640 | 标准 YOLO 输入，匹配预处理契约 |
| `batch` | 8 | 针对 16 GB 内存的保守设置 |
| `device` | `cpu` | 目标平台没有 GPU |
| `epochs` | 100 | 从此处开始；欠拟合时增加 |

### 6.4 监控

```bash
# TensorBoard
tensorboard --logdir runs/train

# 或检查结果
cat runs/train/marker-v1/results.csv
```

### 6.5 验收标准

| 指标 | 目标 |
|---|---|
| mAP@50 | ≥ 0.85 |
| mAP@50-95 | ≥ 0.60 |
| 推理延迟（CPU、i5） | 每帧 < 100 ms |
| 假阳性率 | 测试集上 < 5% |

---

## 7. 模型清单

### 7.1 清单格式

训练后创建 `model-manifest.json`：

```json
{
  "schema_version": 1,
  "model_id": "marker-yolov8n-v1",
  "dataset_manifest_sha256": "<sha256 of dataset.json>",
  "class_map": [
    {"id": 0, "name": "marker"}
  ],
  "preprocessing": {
    "color_space": "RGB",
    "layout": "NCHW",
    "resize": {"width": 640, "height": 640, "strategy": "letterbox"},
    "scale": 0.00392156862745098
  },
  "runtime": {
    "format": "onnx",
    "input_tensor": "images",
    "output_tensor": "output0"
  },
  "artifact": {
    "path": "models/marker-yolov8n-v1.onnx",
    "sha256": "<sha256 of .onnx file>"
  },
  "training_provider": {
    "repository_url": "https://github.com/ultralytics/ultralytics.git",
    "revision": "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b",
    "license": "AGPL-3.0-only"
  }
}
```

### 7.2 验证

```bash
python -m yolo_contract validate --model model-manifest.json
```

拒绝以下情况：
- `training_provider` 与固定的 Ultralytics 修订版本不匹配
- 缺少或不匹配的 `dataset_manifest_sha256`
- 工件 SHA-256 与实际文件不匹配
- 路径越界（工件路径位于清单目录之外）

---

## 8. 导出到 ONNX / OpenVINO

### 8.1 ONNX 导出

```bash
yolo export \
  model=runs/train/marker-v1/weights/best.pt \
  format=onnx \
  imgsz=640 \
  simplify=True
```

输出：`best.onnx`

### 8.2 OpenVINO 导出

```bash
yolo export \
  model=runs/train/marker-v1/weights/best.pt \
  format=openvino \
  imgsz=640
```

输出：包含 `.xml` + `.bin` 文件的 `best_openvino_model/` 目录。

### 8.3 Intel i5 优化

对于 Intel i5 目标，优先使用 OpenVINO：

```bash
# 安装 OpenVINO 运行时
pip install openvino

# 基准测试
yolo benchmark model=best_openvino_model imgsz=640 device=cpu
```

### 8.4 导出验收标准

| 门槛 | 标准 |
|---|---|
| ONNX 加载 | `onnxruntime.InferenceSession` 成功 |
| OpenVINO 加载 | `openvino.Core.compile_model` 成功 |
| 输出形状匹配 | YOLOv8 为 `[1, num_classes+4, num_anchors]` |
| 数值一致性 | 测试图像上的 ONNX 输出与 PyTorch 输出相差不超过 1e-4 |
| 工件 SHA-256 | 匹配 `model-manifest.json` 的 `artifact.sha256` |

---

## 9. ROS 运行时集成

### 9.1 提供方架构

来源：`ed_uav_perception/provider_interface.py`

```python
class DetectorProvider(ABC):
    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection2D]: ...
    version: str
    provider_type: str
```

| 提供方 | 状态 | 导入 |
|---|---|---|
| `MockDetectorProvider` | ✅ 已实现 | 无（确定性模拟） |
| `ONNXDetectorProvider` | ❌ 存根（`NotImplementedError`） | `onnxruntime` |
| `OpenVINODetectorProvider` | ❌ 存根（`NotImplementedError`） | `openvino` |

### 9.2 检测流水线

```
/camera/narrow/image_raw
  → DetectorNode._image_callback()
    → stale check (age > 0.5 s → reject)
    → rate limit (10 Hz)
    → CvBridge.imgmsg_to_cv2() → RGB numpy
    → DetectorProvider.detect(image)
    → Publish Detection2DArray to /perception/detections
```

### 9.3 模型加载（未来）

```python
# ONNX 提供方（待实现）
class ONNXDetectorProvider(DetectorProvider):
    def __init__(self, model_path: str, manifest: ModelManifest):
        import onnxruntime as ort
        self.session = ort.InferenceSession(model_path)
        self.manifest = manifest

    def detect(self, image: np.ndarray) -> list[Detection2D]:
        # 预处理：调整大小、归一化、转置为 NCHW
        # 执行推理
        # 后处理：NMS，将边界框缩放到图像坐标
        ...
```

### 9.4 关键约束

**提供方不得在 ROS 进程中导入 Ultralytics。**ML 环境是隔离的。ROS 进程只使用 ONNX Runtime 或 OpenVINO Runtime。

---

## 10. CLI 参考

入口点：`ed-yolo`（映射到 `yolo_contract.cli:main`）

**所有命令都要求 `--dry-run`，不会执行实际训练/导出。**

```bash
# 训练（仅试运行验证）
ed-yolo train --dataset dataset.json --model model.json --dry-run

# 验证清单一致性
ed-yolo validate --dataset dataset.json --model model.json --dry-run

# 导出（仅试运行验证）
ed-yolo export --model model.json --format onnx --output out.onnx --dry-run

# 模拟检测（确定性执行，无需模型）
ed-yolo detect-mock --dataset dataset.json --model model.json \
  --image-id FRAME --image-sha256 SHA256 --frame-id camera_narrow_optical_frame
```

---

## 11. 验收标准摘要

| 门槛 | 标准 | 工具 |
|---|---|---|
| 数据集不可变性 | 无分区重叠，SHA-256 唯一 | `schema.py` 解析器 |
| 模型清单完整性 | 工件 SHA-256 匹配，提供方固定值匹配 | `schema.py` 解析器 |
| 训练质量 | mAP@50 ≥ 0.85，mAP@50-95 ≥ 0.60 | `yolo benchmark` |
| 导出正确性 | ONNX/OpenVINO 可加载，输出形状匹配，数值一致 | 导出基准测试 |
| CPU 延迟 | Intel i5 上 < 100 ms/帧 | `yolo benchmark` |
| 许可证合规 | AGPL-3.0 源代码提供已准备 | `check_third_party.py --strict` |
| ROS 隔离 | `ed_uav_perception` 中无 Ultralytics 导入 | 在 `ros2_ws/src/` 中搜索 `ultralytics` |
