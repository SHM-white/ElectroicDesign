# OpenCode 配置指南

## 概述

本项目配置了 OpenCode，使用小米 MiMo-V2.5-Pro 作为主模型，DeepSeek V4 Flash 作为子代理模型。配置旨在提高主模型调用不同模型子代理的积极性，实现高效的任务委派和并行处理。

## 快速开始

### 1. 设置环境变量

#### Windows PowerShell
```powershell
$env:XIAOMI_API_KEY = "your_xiaomi_api_key"
$env:DEEPSEEK_API_KEY = "your_deepseek_api_key"
```

#### Linux/macOS
```bash
export XIAOMI_API_KEY="your_xiaomi_api_key"
export DEEPSEEK_API_KEY="your_deepseek_api_key"
```

### 2. 验证配置

运行环境设置脚本：
```bash
# Windows PowerShell
.\setup_env.ps1

# Linux/macOS
source setup_env.sh
```

### 3. 启动 OpenCode

重启 OpenCode 以加载新配置。

## 配置详情

### 模型配置

| 模型 | 提供商 | 用途 | 思考预算 |
|------|--------|------|----------|
| MiMo-V2.5-Pro | 小米 | 主构建和规划代理 | 8000 tokens |
| DeepSeek V4 Flash | DeepSeek | 各种子代理任务 | 10000 tokens |

### 代理配置

#### 主代理
- **build**：主构建代理，负责协调和完成软件工程任务
- **plan**：规划代理，负责分析复杂任务并制定执行计划

#### 子代理
- **general**：通用子代理，用于委派具体任务
- **explore**：探索子代理，用于代码探索和搜索
- **test**：测试子代理，专门用于运行测试和验证功能
- **docs**：文档子代理，专门用于编写和更新文档
- **deploy**：部署子代理，专门用于部署和发布任务

### 技能配置

| 技能 | 触发条件 | 用途 |
|------|----------|------|
| delegate-tasks | 需要委派具体任务给子代理时 | 鼓励任务委派 |
| subagent-orchestrator | 需要同时协调多个子代理时 | 并行处理任务 |
| wsl-powershell-shell | 需要在PowerShell中运行Linux命令时 | WSL环境支持 |

### 指令文件

| 文件 | 内容 |
|------|------|
| AGENTS.md | 代理协作指南 |
| DELEGATION.md | 任务委派指南 |
| SKILLS.md | 技能使用指南 |
| MODEL_CONFIG.md | 模型配置指南 |
| TROUBLESHOOTING.md | 故障排除指南 |

## 使用建议

### 任务委派策略

1. **探索-实现模式**：先委派探索代理分析，再委派通用代理实现
2. **并行处理模式**：同时委派多个子代理处理不同任务
3. **渐进式委派模式**：先委派小规模试点任务，逐步扩大范围

### 最佳实践

1. **清晰的任务描述**：为子代理提供明确、具体的任务描述
2. **适当的上下文**：提供足够的背景信息，但避免信息过载
3. **结果整合**：主模型负责整合子代理的结果并做出最终决策
4. **错误处理**：为子代理任务设置合理的超时和错误处理机制

## 故障排除

### 常见问题

1. **API密钥无效**：检查环境变量是否正确设置
2. **模型不可用**：确认模型标识符和提供商配置
3. **配置未生效**：重启 OpenCode 应用
4. **代理权限不足**：检查代理权限配置

### 调试技巧

1. 启用详细日志：在配置中添加 `"logLevel": "DEBUG"`
2. 测试单个组件：单独测试API连接和代理执行
3. 使用简化配置：创建最小配置文件测试

## 更新和维护

### 定期检查

1. **API更新**：关注提供商API更新
2. **模型版本**：检查模型版本更新
3. **配置优化**：根据使用情况优化配置

### 备份策略

1. **配置备份**：定期备份配置文件
2. **环境变量**：记录环境变量设置
3. **恢复计划**：准备配置恢复方案

## 相关链接

- [OpenCode 官方文档](https://opencode.ai/docs)
- [配置参考](https://opencode.ai/config.json)
- [GitHub 仓库](https://github.com/anomalyco/opencode)

## 支持

如遇到问题，请：
1. 查看 [故障排除指南](TROUBLESHOOTING.md)
2. 搜索 [GitHub Issues](https://github.com/anomalyco/opencode/issues)
3. 提交新的 Issue 并附上详细信息