# 模型发布检查清单

> **状态**：契约层已实现（Task 12）
> **负责人**：`ml/yolo`
> **许可证**：AGPL-3.0-only（Ultralytics）

---

## 1. 概览

本文定义 YOLO 模型工件的发布检查清单。系统在采用 AGPL-3.0 许可证的 Ultralytics 训练环境与 ROS 运行时之间执行严格隔离，并在每个边界进行完整的来源跟踪和哈希校验。

### 隔离架构

```
ml/yolo/                          ros2_ws/src/ed_uav_perception/
├── pyproject.toml                ├── provider_interface.py
├── src/yolo_contract/            │   ├── ONNXDetectorProvider (stub)
│   ├── schema.py                 │   └── OpenVINODetectorProvider (stub)
│   ├── runtime.py                ├── detector_node.py
│   └── cli.py                    └── localizer.py
└── tests/
```

**关键规则**：`ros2_ws/` 永不导入 `ultralytics`。ROS 运行时只通过 `DetectionProvider` 协议消费与提供方无关的 ONNX/OpenVINO 输出。

### 当前状态

| 组件 | 文件 | 状态 |
|---|---|---|
| 模式验证 | `ml/yolo/src/yolo_contract/schema.py` | **已实现** |
| 哈希校验 | `ml/yolo/src/yolo_contract/jsonio.py` | **已实现** |
| MockDetectionProvider | `ml/yolo/src/yolo_contract/runtime.py` | **已实现** |
| CLI（仅试运行） | `ml/yolo/src/yolo_contract/cli.py` | **已实现** |
| ONNXDetectorProvider | `ros2_ws/src/ed_uav_perception/provider_interface.py` | **Stub** |
| OpenVINODetectorProvider | `ros2_ws/src/ed_uav_perception/provider_interface.py` | **Stub** |
| 实际训练 | `ml/yolo/` | **有意阻止** |
| 实际导出 | `ml/yolo/` | **有意阻止** |
| 模型权重 | — | **未批准或下载任何权重** |

---

## 2. AGPL-3.0 合规

Ultralytics YOLOv8 采用 AGPL-3.0 许可证。任何分发或网络使用都会产生具体义务。

### 2.1 许可证义务

| 义务 | 要求 | 缓解措施 |
|---|---|---|
| **源代码披露** | 必须向接收方提供完整源代码 | 固定确切的上游提交 |
| **相同许可证** | 组合著作必须采用 AGPL-3.0 | 将 YOLO 保持在 `ml/yolo/` 中隔离 |
| **对应源代码** | 必须提供所有链接组件的源代码 | 独立进程边界 |
| **网络使用** | 如果通过网络使用，必须提供源代码 | 仅限内部使用（竞赛） |

### 2.2 内部使用例外

内部使用（竞赛准备、测试）**不会**触发分发义务。但是，活动后发布完整系统时，YOLO 组件必须符合 AGPL。

### 2.3 隔离策略

项目通过以下方式保持 AGPL 隔离：

1. **独立进程**：YOLO 推理作为独立 ROS 节点运行
2. **标准接口**：通过 `vision_msgs/Detection2DArray` 通信
3. **不导入**：ROS 进程永不导入 `ultralytics`
4. **权重分离**：训练权重按 AGPL-3.0 单独发布

摘自 `docs/legal/OPEN_SOURCE.md`：

> Ultralytics 记录为 AGPL-3.0-only。除分发问题外，AGPL 还可能要求向通过网络与修改后的受涵盖程序交互的用户提供对应源代码。项目因此保持使用隔离，固定上游源，并在形成特定任务的来源记录前阻止下载模型权重。

### 2.4 验证

```bash
# Verify no ultralytics import in ROS workspace
grep -r "import ultralytics" ros2_ws/src/  # Should return nothing
grep -r "from ultralytics" ros2_ws/src/    # Should return nothing
```

---

## 3. 版本固定

Ultralytics 在三个位置固定为同一个提交，并进行交叉校验。

### 3.1 三处锁定

| 位置 | 文件 | 行 |
|---|---|---|
| pip 安装源 | `ml/yolo/pyproject.toml` | 第 8 行 |
| VCS 导入 | `ros2_ws/dependencies.repos` | 第 16 行 |
| 运行时常量 | `ml/yolo/src/yolo_contract/schema.py` | 第 25-27 行 |

### 3.2 固定值

摘自 `ml/yolo/src/yolo_contract/schema.py`：

```python
SCHEMA_VERSION = 1
ULTRALYTICS_REPOSITORY = "https://github.com/ultralytics/ultralytics.git"
ULTRALYTICS_REVISION = "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b"
ULTRALYTICS_LICENSE = "AGPL-3.0-only"
```

摘自 `ml/yolo/pyproject.toml`：

```toml
[project]
name = "ed-yolo-contract"
version = "0.1.0"
dependencies = [
  "ultralytics @ git+https://github.com/ultralytics/ultralytics.git@7a159ea24ec94c47cf25c75785e0a56e47ba4e7b",
]
```

### 3.3 强制执行

模式解析器在加载时强制执行该固定值：

```python
# 摘自 schema.py（第 170-172 行）
if training_provider != TrainingProvider(
    ULTRALYTICS_REPOSITORY, ULTRALYTICS_REVISION, ULTRALYTICS_LICENSE
):
    raise ManifestError(
        "training_provider must match the reviewed P04 Ultralytics source and AGPL license"
    )
```

任何 `training_provider` 不同的模型清单都会被拒绝。

---

## 4. 哈希校验链

### 4.1 数据集清单哈希

数据集清单中的每个样本都有 SHA-256 哈希：

```json
{
  "id": "sample_001",
  "split": "train",
  "sha256": "abc123...",
  "source_url": "https://...",
  "license": "CC-BY-4.0",
  "class_ids": [0, 1]
}
```

模式解析器验证：
- 同一分区内没有重复哈希（`DuplicateHashError`）
- 分区之间没有哈希重叠（`SplitOverlapError`）
- 所有哈希均为小写 SHA-256（正则表达式：`[0-9a-f]{64}`）

### 4.2 模型清单哈希

模型清单通过 `dataset_manifest_sha256` 绑定到数据集：

```json
{
  "model_id": "terminal_v1",
  "dataset_manifest_sha256": "def456...",
  "artifact": {
    "path": "best.onnx",
    "sha256": "ghi789..."
  }
}
```

验证：
- `dataset_manifest_sha256` 必须匹配实际数据集文件哈希
- `artifact.sha256` 必须匹配实际模型文件哈希
- `artifact.path` 必须是相对路径（不得通过 `..` 越界）

### 4.3 哈希函数

摘自 `ml/yolo/src/yolo_contract/jsonio.py`：

```python
def sha256_bytes(contents: bytes) -> str:
    """Return a lowercase SHA-256 digest."""
    return hashlib.sha256(contents).hexdigest()

def sha256_file(path: Path) -> str:
    """Hash one existing artifact without retaining its contents."""
    try:
        return sha256_bytes(path.read_bytes())
    except OSError as error:
        raise ModelIntegrityError(f"cannot read model artifact: {path}") from error
```

### 4.4 第三方许可证验证

摘自 `tools/third_party_validation.py`：
- 许可证文件的 SHA-256 验证
- `docs/provenance/licenses/` 中的缓存许可证
- 带精确 SHA 的参考归档
- 检出完整性验证

---

## 5. 导出流程

### 5.1 ONNX 导出

```bash
# 试运行（仅验证计划，不执行实际导出）
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m yolo_contract.cli export \
  --model ml/yolo/models/v1/model-manifest.json \
  --format onnx \
  --output ml/yolo/models/v1/best.onnx \
  --dry-run
```

导出元数据要求：
- 输入形状：`[640, 640, 3]`（来自预处理）
- 色彩空间：`rgb`（来自预处理）
- 布局：`NCHW`（来自预处理）
- 缩放：`0.00392156862745098`（1/255，来自预处理）

### 5.2 OpenVINO 导出

```bash
# 试运行（仅验证计划，不执行实际导出）
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m yolo_contract.cli export \
  --model ml/yolo/models/v1/model-manifest.json \
  --format openvino \
  --output ml/yolo/models/v1/openvino/ \
  --dry-run
```

Intel i5 的 OpenVINO 优化：
- 目标：`CPU`（Intel i5-1240P）
- 精度：`FP16`（半精度）
- 批大小：`1`（单图像推理）

### 5.3 导出 CLI

摘自 `ml/yolo/src/yolo_contract/cli.py`：

```python
exporter = commands.add_parser("export")
exporter.add_argument("--model", required=True, type=Path)
exporter.add_argument("--format", required=True, choices=("onnx", "openvino"))
exporter.add_argument("--output", required=True, type=Path)
exporter.add_argument("--dry-run", action="store_true")
```

**注意**：所有导出命令都要求 `--dry-run`。在训练获批准前，实际导出会被有意禁用。

### 5.4 验证

导出后验证：

```bash
# 根据数据集验证模型
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m yolo_contract.cli validate \
  --dataset ml/yolo/datasets/v1/dataset-manifest.json \
  --model ml/yolo/models/v1/model-manifest.json \
  --dry-run
```

---

## 6. 发布前测试

### 6.1 数据集验证

- [ ] 已验证不可变的 train/val/test 分区（无重叠）
- [ ] 清单中记录了所有图像哈希
- [ ] 所有分区中的类别映射一致
- [ ] 为每张图像记录了来源/许可证
- [ ] 分区之间没有重复图像

### 6.2 模型训练

- [ ] 训练脚本使用固定的 Ultralytics 提交
- [ ] 已记录超参数
- [ ] 已保留训练日志
- [ ] 记录了验证指标（mAP50、mAP50-95）
- [ ] 未检测到过拟合（train/val 差距 < 10%）

### 6.3 导出验证

- [ ] ONNX 导出成功
- [ ] ONNX 推理结果与 PyTorch 在容差范围内一致
- [ ] OpenVINO 导出成功（Intel 目标）
- [ ] 导出元数据包含输入形状和类别名称
- [ ] 清单中记录了模型哈希

### 6.4 集成测试

- [ ] 模拟提供方返回确定性检测结果
- [ ] ONNX 提供方能正确加载并推理
- [ ] 终端几何生成有效位姿
- [ ] ROS 进程中没有 `ultralytics` 导入
- [ ] 无模型权重时所有测试通过

### 6.5 测试套件

摘自 `ml/yolo/tests/`：

| 测试文件 | 测试内容 | 覆盖范围 |
|---|---|---|
| `test_schema.py` | 225 行 | 清单解析、分区重叠、重复哈希、类别漂移 |
| `test_runtime.py` | 102 行 | 哈希拒绝、提供方失败、确定性 |
| `test_cli.py` | 119 行 | 试运行、模拟检测、格式错误输入 |
| `manual_cli_smoke.py` | 86 行 | 可安全清理的验收驱动程序 |

---

## 7. 模型清单格式

### 7.1 模式

摘自 `ml/yolo/schemas/model-manifest.schema.json`：

```json
{
  "schema_version": 1,
  "model_id": "terminal_v1",
  "dataset_manifest_sha256": "abc123...",
  "class_map": [
    {"id": 0, "name": "terminal_target"},
    {"id": 1, "name": "obstacle"},
    {"id": 2, "name": "marker"}
  ],
  "preprocessing": {
    "color_space": "rgb",
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
    "path": "best.onnx",
    "sha256": "ghi789..."
  },
  "training_provider": {
    "repository_url": "https://github.com/ultralytics/ultralytics.git",
    "revision": "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b",
    "license": "AGPL-3.0-only"
  }
}
```

### 7.2 运行时契约

摘自 `ml/yolo/src/yolo_contract/runtime.py`：

```python
ROS_DETECTION_CONTRACT = "vision_msgs/Detection2DArray-compatible/v1"

class DetectionProvider(Protocol):
    def detect(self, request: ImageRequest) -> Detection2DArray:
        """Return detected pixel-normalized boxes or raise a typed provider error."""
```

与提供方无关的输出格式：
- `contract`：版本字符串
- `image_id`：按内容寻址的图像标识符
- `frame_id`：摄像头坐标系
- `detections`：`Detection2D` 元组（class_id、class_name、score、bbox）

---

## 8. ROS 集成

### 8.1 提供方接口

摘自 `ros2_ws/src/ed_uav_perception/ed_uav_perception/provider_interface.py`：

```python
class DetectorProvider(ABC):
    @abstractmethod
    def detect(self, request: ImageRequest) -> Detection2DArray:
        ...

class ONNXDetectorProvider(DetectorProvider):
    """Stub — not yet implemented."""
    def detect(self, request: ImageRequest) -> Detection2DArray:
        raise NotImplementedError("ONNX provider not yet implemented")

class OpenVINODetectorProvider(DetectorProvider):
    """Stub — not yet implemented."""
    def detect(self, request: ImageRequest) -> Detection2DArray:
        raise NotImplementedError("OpenVINO provider not yet implemented")
```

### 8.2 模型验证

摘自 `ros2_ws/src/ed_uav_perception/ed_uav_perception/model_validator.py`：

用于 ROS 侧模型加载的基于 Pydantic 的清单验证。验证：
- 模式版本
- 类别映射一致性
- 预处理参数
- 运行时格式
- 工件路径安全性

### 8.3 定位器

摘自 `ros2_ws/src/ed_uav_perception/ed_uav_perception/localizer.py`：

> YOLO 检测结果绝不作为位姿来源。摄像头校准和终端几何仍属于 P15 工作。

---

## 9. 版本固定摘要

| 组件 | 固定位置 | 固定值 |
|---|---|---|
| Ultralytics 提交 | `ml/yolo/pyproject.toml` | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| Ultralytics 提交 | `ros2_ws/dependencies.repos` | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| Ultralytics 提交 | `ml/yolo/src/yolo_contract/schema.py` | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| Livox 驱动 | `ros2_ws/dependencies.repos` | `13eb05e` |
| FAST-LIO | `ros2_ws/dependencies.repos` | 已固定 |
| ONNX 操作集 | `ml/yolo/pyproject.toml` | 17 |
| 输入形状 | `ml/yolo/src/yolo_contract/schema.py` | `[640, 640, 3]` |

---

## 10. 验收标准

来自 Task 12：

| 标准 | 验证 |
|---|---|
| 模式拒绝未知元数据 | `test_schema.py` — 未知字段测试 |
| 模式拒绝分区重叠 | `test_schema.py` — `SplitOverlapError` 测试 |
| 模式拒绝重复哈希 | `test_schema.py` — `DuplicateHashError` 测试 |
| 模式拒绝类别漂移 | `test_schema.py` — `ClassMapDriftError` 测试 |
| 模式拒绝未固定的提供方 | `test_schema.py` — 提供方不匹配测试 |
| 模拟提供方验证哈希 | `test_runtime.py` — 哈希拒绝测试 |
| 模拟提供方报告失败 | `test_runtime.py` — 提供方失败测试 |
| CLI 试运行验证计划 | `test_cli.py` — 试运行测试 |
| ROS 工作区中没有 ultralytics | `grep -r "import ultralytics" ros2_ws/src/` |
| 无权重时所有测试通过 | `pytest ml/yolo/tests/` |

---

## 11. 参考资料

- `ml/yolo/src/yolo_contract/schema.py` — 清单解析器和验证
- `ml/yolo/src/yolo_contract/runtime.py` — 提供方接口和模拟适配器
- `ml/yolo/src/yolo_contract/cli.py` — 导出 CLI
- `ml/yolo/src/yolo_contract/jsonio.py` — 哈希校验函数
- `ml/yolo/src/yolo_contract/errors.py` — 类型化错误层级
- `ml/yolo/schemas/model-manifest.schema.json` — 模型清单的 JSON Schema
- `ml/yolo/schemas/dataset-manifest.schema.json` — 数据集清单的 JSON Schema
- `ml/yolo/pyproject.toml` — Ultralytics 版本固定
- `ros2_ws/src/ed_uav_perception/ed_uav_perception/provider_interface.py` — ROS 提供方存根
- `ros2_ws/src/ed_uav_perception/ed_uav_perception/model_validator.py` — 清单验证
- `docs/legal/OPEN_SOURCE.md` — 开源使用边界
- `docs/provenance/third-party-sources.json` — 机器可读的来源记录
