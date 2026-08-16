# 模型配置指南

## 模型架构

### 主模型：MiMo-V2.5-Pro
- **提供商**：小米
- **用途**：主要构建和规划代理
- **特性**：支持思考模式，预算8000 tokens
- **优势**：高层次决策、复杂逻辑处理、整体协调

### 子代理模型：DeepSeek V4 Flash
- **提供商**：DeepSeek
- **用途**：各种子代理任务
- **特性**：支持思考模式，预算10000 tokens
- **优势**：快速执行、专门任务处理、并行处理能力

## 配置详解

### 提供商配置

#### 小米提供商
```json
{
  "xiaomi": {
    "api": "openai",
    "options": {
      "baseURL": "https://api.xiaomi.com/v1",
      "apiKey": "{env:XIAOMI_API_KEY}"
    },
    "models": {
      "mimo-v2.5-pro": {
        "reasoning": true,
        "options": {
          "thinking": {
            "type": "enabled",
            "budget_tokens": 8000
          }
        }
      }
    }
  }
}
```

#### DeepSeek提供商
```json
{
  "deepseek": {
    "api": "openai",
    "options": {
      "baseURL": "https://api.deepseek.com/v1",
      "apiKey": "{env:DEEPSEEK_API_KEY}"
    },
    "models": {
      "deepseek-v4-flash": {
        "reasoning": true,
        "options": {
          "thinking": {
            "type": "enabled",
            "budget_tokens": 10000
          }
        }
      }
    }
  }
}
```

### 代理配置

#### 主代理配置
- **模型**：`xiaomi/mimo-v2.5-pro`
- **模式**：`primary`
- **职责**：决策、协调、整合

#### 子代理配置
- **模型**：`deepseek/deepseek-v4-flash`
- **模式**：`subagent`
- **职责**：执行、实现、专门任务

## 环境变量配置

### 必需的环境变量
```bash
# 小米API密钥
export XIAOMI_API_KEY="your_xiaomi_api_key"

# DeepSeek API密钥
export DEEPSEEK_API_KEY="your_deepseek_api_key"
```

### 环境变量设置方法

#### Windows PowerShell
```powershell
$env:XIAOMI_API_KEY="your_xiaomi_api_key"
$env:DEEPSEEK_API_KEY="your_deepseek_api_key"
```

#### Linux/macOS
```bash
export XIAOMI_API_KEY="your_xiaomi_api_key"
export DEEPSEEK_API_KEY="your_deepseek_api_key"
```

#### 永久设置
将环境变量添加到：
- Windows：系统环境变量或用户环境变量
- Linux/macOS：`~/.bashrc`、`~/.zshrc` 或 `~/.profile`

## 模型优化

### 思考模式配置

#### 主模型思考模式
- **预算**：8000 tokens
- **用途**：复杂决策和逻辑推理
- **触发**：自动启用

#### 子代理思考模式
- **预算**：10000 tokens
- **用途**：详细任务执行和推理
- **触发**：自动启用

### 性能优化建议

#### 主模型优化
1. **专注于高层次任务**：避免处理具体实现细节
2. **合理使用思考模式**：复杂决策时启用
3. **控制上下文长度**：避免信息过载

#### 子代理优化
1. **任务专业化**：根据任务类型选择合适代理
2. **并行处理**：同时执行多个独立任务
3. **结果缓存**：重用已有结果避免重复计算

## 故障排除

### 常见问题

#### API密钥错误
**症状**：认证失败、401错误
**解决**：
1. 检查环境变量是否正确设置
2. 验证API密钥是否有效
3. 确认提供商配置是否正确

#### 模型不可用
**症状**：模型未找到、404错误
**解决**：
1. 检查模型标识符是否正确
2. 确认提供商是否支持该模型
3. 验证API端点是否可访问

#### 思考模式问题
**症状**：思考模式未启用、预算不足
**解决**：
1. 检查模型配置中的思考模式设置
2. 调整思考预算tokens
3. 验证模型是否支持思考模式

### 调试技巧

#### 启用详细日志
在 `opencode.json` 中添加：
```json
{
  "logLevel": "DEBUG"
}
```

#### 测试连接
使用简单的测试任务验证API连接：
```
请帮我测试API连接是否正常
```

## 最佳实践

### 配置管理
1. **版本控制**：将配置文件纳入版本控制
2. **环境分离**：为不同环境使用不同配置
3. **密钥安全**：不要将API密钥提交到版本控制

### 模型选择
1. **任务匹配**：根据任务特性选择合适模型
2. **成本考虑**：平衡性能和成本
3. **可用性**：确保模型服务稳定可用

### 性能监控
1. **响应时间**：监控模型响应时间
2. **成功率**：跟踪任务完成成功率
3. **资源使用**：监控API调用次数和token使用

## 更新和维护

### 定期检查
1. **API更新**：关注提供商API更新
2. **模型版本**：检查模型版本更新
3. **配置优化**：根据使用情况优化配置

### 备份策略
1. **配置备份**：定期备份配置文件
2. **环境变量**：记录环境变量设置
3. **恢复计划**：准备配置恢复方案