# OpenCode 环境变量设置脚本 (Windows PowerShell)
# 使用方法: .\setup_env.ps1

Write-Host "设置 OpenCode 环境变量..." -ForegroundColor Cyan

# 检查是否已设置环境变量
if (-not $env:XIAOMI_API_KEY) {
    Write-Host "警告: XIAOMI_API_KEY 未设置" -ForegroundColor Yellow
    Write-Host "请设置小米 API 密钥: `$env:XIAOMI_API_KEY = 'your_api_key'" -ForegroundColor Yellow
} else {
    Write-Host "✓ XIAOMI_API_KEY 已设置" -ForegroundColor Green
}

if (-not $env:DEEPSEEK_API_KEY) {
    Write-Host "警告: DEEPSEEK_API_KEY 未设置" -ForegroundColor Yellow
    Write-Host "请设置 DeepSeek API 密钥: `$env:DEEPSEEK_API_KEY = 'your_api_key'" -ForegroundColor Yellow
} else {
    Write-Host "✓ DEEPSEEK_API_KEY 已设置" -ForegroundColor Green
}

# 检查配置文件
if (Test-Path "opencode.json") {
    Write-Host "✓ 找到配置文件: opencode.json" -ForegroundColor Green
} else {
    Write-Host "警告: 未找到 opencode.json 配置文件" -ForegroundColor Yellow
}

# 检查技能目录
if (Test-Path ".opencode\skills") {
    Write-Host "✓ 找到技能目录: .opencode\skills" -ForegroundColor Green
    Write-Host "可用技能:" -ForegroundColor Cyan
    Get-ChildItem -Path ".opencode\skills" -Directory | ForEach-Object {
        Write-Host "  - $($_.Name)" -ForegroundColor White
    }
} else {
    Write-Host "警告: 未找到技能目录 .opencode\skills" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "环境设置完成!" -ForegroundColor Green
Write-Host "请确保已设置所有必需的环境变量，然后重启 opencode。" -ForegroundColor Cyan