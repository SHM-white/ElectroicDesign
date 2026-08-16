#!/bin/bash

# OpenCode 环境变量设置脚本
# 使用方法: source setup_env.sh

echo "设置 OpenCode 环境变量..."

# 检查是否已设置环境变量
if [ -z "$XIAOMI_API_KEY" ]; then
    echo "警告: XIAOMI_API_KEY 未设置"
    echo "请设置小米 API 密钥: export XIAOMI_API_KEY='your_api_key'"
else
    echo "✓ XIAOMI_API_KEY 已设置"
fi

if [ -z "$DEEPSEEK_API_KEY" ]; then
    echo "警告: DEEPSEEK_API_KEY 未设置"
    echo "请设置 DeepSeek API 密钥: export DEEPSEEK_API_KEY='your_api_key'"
else
    echo "✓ DEEPSEEK_API_KEY 已设置"
fi

# 检查配置文件
if [ -f "opencode.json" ]; then
    echo "✓ 找到配置文件: opencode.json"
else
    echo "警告: 未找到 opencode.json 配置文件"
fi

# 检查技能目录
if [ -d ".opencode/skills" ]; then
    echo "✓ 找到技能目录: .opencode/skills"
    echo "可用技能:"
    ls -1 .opencode/skills/
else
    echo "警告: 未找到技能目录 .opencode/skills"
fi

echo ""
echo "环境设置完成!"
echo "请确保已设置所有必需的环境变量，然后重启 opencode。"