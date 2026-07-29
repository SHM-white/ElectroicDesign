# 开源使用边界

本文是工程合规记录，不构成法律意见。发布或交付活动前，必须由合格审阅者评估实际分发、部署、修改、接收方和司法辖区。

## 当前范围

内部学术竞赛开发是获批准的工作范围。如果项目之后分发源代码、二进制文件、容器、镜像、模型工件，或提供受许可约束的网络服务访问，该范围不会免除许可证义务。确切的上游修订版本、许可证快照、通知和源位置记录在来源清单中，并由 `python3 tools/check_third_party.py --strict` 检查。

## 调用边界

Livox ROS Driver 2 and FAST-LIO remain independently imported and launched from `ros2_ws/src/third_party`. Project-owned `ed_*` packages communicate only through declared ROS interfaces and must not copy their sources. Future Ultralytics training/export runs remain in the isolated `ml/yolo` environment; ROS runtime code consumes provider-neutral outputs and must not import Ultralytics.

这些独立进程边界保留了技术所有权和可追溯性。但仅凭这些边界，不能判断组合工件是否构成任何许可证下的衍生作品或组合作品；该问题需要基于具体事实进行法律审阅。

## 著作权左派审阅要点

FAST-LIO is recorded as GPL-2.0-only. Distribution of the covered program or a modified/combined artifact can create notice, license-text, and corresponding-source obligations. Ultralytics is recorded as AGPL-3.0-only. In addition to distribution questions, AGPL can require an offer of corresponding source to users who interact with a modified covered program over a network. The project therefore keeps its use isolated, pins the upstream source, and blocks model-weight downloads until task-specific provenance exists.

义务触发后，应保留适用的通知，并按要求的渠道和时限提供受涵盖版本及其修改的对应源代码。仅在活动结束后发布材料可以支持合规工作，但不能追溯性地绕过已经适用的义务。MIT 许可的 Livox 材料仍须保留适用的版权和许可通知。

## 发布门槛

在任何再分发、竞赛交付、公共服务或镜像/容器交接前，审阅确切的工件组成、相关 GPL/AGPL 条件、所有修改、源代码提供机制、模型和数据集许可证以及接收方访问权限。不得将本文表述为法律结论。
