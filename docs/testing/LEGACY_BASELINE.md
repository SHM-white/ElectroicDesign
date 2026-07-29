# 旧版测试基线

## 采集

该基线在 task-1 行为变更之前、使用仓库虚拟环境采集。逐字命令输出保存在 `.omo/evidence/task-1/characterization/` 下：

- `git-status.txt` 记录初始未跟踪的 `往年赛题/` 源材料。
- `protected-files.sha256`、`protected-files.diff` 和 `protected-files.numstat` 记录受保护启动脚本的状态。
- `tool-versions.txt` 记录 Python 3.12.3、pytest 7.4.4、Git 2.43.0 和 GNU sha256sum 9.4。
- `pytest-collect.txt` 包含变更前收集的全部 479 个节点 ID。
- `pytest-full.txt` 包含变更前的完整运行结果和失败详情。

## 受保护文件

采集到的 diff 和 numstat 均为空。在某个任务明确负责这些文件之前，受保护文件必须保持以下 SHA-256 值：

| 文件 | SHA-256 |
| --- | --- |
| `drone/start.sh` | `9658f7ea196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |
| `drone/debug_start.sh` | `af24ba8afbffa6483ade8dd87a78a2d2f688243c5d57c486924dce45b00af85d` |
| `drone/field_test.sh` | `dda7ecb3348be65dc01356eb626420c4c3794c4aef75baa359a6fcff3bb1432b` |

## 原始 Pytest 结果

原始命令 `./.venv/bin/python -m pytest` 收集了 479 个节点，最终 471 个通过、8 个失败。`drone/test/test_all.py` 产生了 225 个重复的聚合节点；它仍可通过 `unittest` 运行，但会被 pytest 忽略。`test_vision_regression.py` 仍包含在 pytest 中。

原始失败项如下：

1. `drone/test/test_gray_marker.py::TestGrayMarkerDetector::test_detects_dark_saved_sample_center`
2. `drone/test/test_home_cross.py::TestHomeCrossDetector::test_detects_complete_saved_home_sample`
3. `drone/test/test_home_cross.py::TestHomeCrossDetector::test_detects_saved_home_sequence`
4. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_all_saved_field_frames_are_readable`
5. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_expected_digit_rejects_low_light_noise_ocr`
6. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_finds_a_marker_in_both_block_21_samples`
7. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_finds_a_marker_in_low_light_start_samples`
8. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_recognizes_saved_low_light_two_digit_samples`

`field_data` 标记标识所有依赖已保存场地图像的测试。第 1、2、3、4、5、7 和 8 项失败是因为缺少 `mission_vision_*.png` 输入。第 6 项使用可用图像；当运行严格场地数据测试面时，它仍属于算法回归结果。检查器只报告图像可用性和哈希值；它绝不会把算法结果转换为夹具通过。

## 当前外部数据门禁

`drone/test/fixtures/field-images.json` 明确记录每个缺失的 `mission_vision_*.png` 夹具，并保留其 OCR/标记期望值。使用以下命令检查状态：

```bash
./.venv/bin/python tools/check_field_fixtures.py \
  --manifest drone/test/fixtures/field-images.json \
  --expect-current-state
```

只有当文件系统与清单完全一致时，命令才会成功。它会打印每个声明的缺失项；已恢复、已更改、不可读或本应存在但缺失的夹具都会产生非零不匹配，而不是错误成功。
