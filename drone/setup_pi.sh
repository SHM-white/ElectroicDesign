#!/usr/bin/env bash
# ==============================================================================
# setup_pi.sh — Raspberry Pi 一键环境配置脚本
# G_植保飞行器 项目 | 树莓派4B + 凌霄飞控 + 工业相机/OpenMV
#
# 适用: 干净的 Raspberry Pi OS Lite (64-bit, Bookworm)
#
# 用法:
#   chmod +x setup_pi.sh
#   sudo ./setup_pi.sh
#
# 配置内容:
#   1. 系统更新 + 基础工具
#   2. 串口配置 (释放 /dev/serial0 给 MCU 通信)
#   3. Python3 + OpenCV + 相关依赖
#   4. 海康相机 MVS SDK (可选, 需手动下载)
#   5. 项目代码部署
#   6. GPIO 权限配置
#   7. 开机自启服务
#   8. 运行测试验证
# ==============================================================================

set -e  # 遇错即停

# ── 颜色输出 ──────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── 配置变量 ──────────────────────────────────────────
PROJECT_DIR="/home/pi/drone"
PROJECT_REPO="https://github.com/SHM-white/ElectroicDesign.git"  # 修改为实际仓库地址
PYTHON="python3"

# ── 检查root权限 ──────────────────────────────────────
if [ "$(id -u)" -ne 0 ]; then
    error "请使用 sudo 运行此脚本: sudo ./setup_pi.sh"
    exit 1
fi

REAL_USER="${SUDO_USER:-pi}"
REAL_HOME=$(eval echo "~$REAL_USER")

info "================================================"
info "  G_植保飞行器 - 树莓派一键环境配置"
info "================================================"
info "用户: $REAL_USER"
info "项目目录: $PROJECT_DIR"
info ""

# ==============================================================================
# 第1步: 系统更新 + 基础工具
# ==============================================================================
info "[1/8] 系统更新与基础工具安装..."

apt-get update -qq
apt-get upgrade -y -qq

apt-get install -y -qq \
    git vim tmux htop i2c-tools \
    build-essential cmake pkg-config \
    libjpeg-dev libtiff5-dev libjasper-dev libpng-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev \
    libfontconfig1-dev libcairo2-dev \
    libgdk-pixbuf2.0-dev libpango1.0-dev \
    libgtk2.0-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libhdf5-dev libhdf5-serial-dev \
    libqt5gui5 libqt5webkit5 libqt5test5 \
    libusb-1.0-0-dev \
    v4l-utils \
    tesseract-ocr

info "[1/8] 基础工具安装完成"

# ==============================================================================
# 第2步: 串口配置
# ==============================================================================
info "[2/8] 配置串口 (释放 GPIO14/15 给 MCU 通信)..."

# 禁用蓝牙串口占用
if ! grep -q "dtoverlay=disable-bt" /boot/firmware/config.txt 2>/dev/null; then
    echo "" >> /boot/firmware/config.txt
    echo "# 释放 UART (GPIO14/15) 给 MCU 通信" >> /boot/firmware/config.txt
    echo "enable_uart=1" >> /boot/firmware/config.txt
    echo "dtoverlay=disable-bt" >> /boot/firmware/config.txt
fi

# 禁用串口控制台 (systemd)
systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true
systemctl disable hciuart 2>/dev/null || true

# 移除 console=serial0 内核参数
if grep -q "console=serial0" /boot/firmware/cmdline.txt 2>/dev/null; then
    sed -i 's/console=serial0,[0-9]* //' /boot/firmware/cmdline.txt
fi

info "[2/8] 串口配置完成 (重启后生效)"

# ==============================================================================
# 第3步: Python 环境
# ==============================================================================
info "[3/8] 配置 Python 虚拟环境..."

# 安装 pip + venv
apt-get install -y -qq $PYTHON-pip $PYTHON-venv

# 安装 OpenCV (预编译版, 避免从源码编译 4小时+)
apt-get install -y -qq $PYTHON-opencv

# 安装系统级 Python 包 (供虚拟环境共享)
pip3 install --break-system-packages --no-cache-dir \
    pyserial \
    numpy \
    pytesseract \
    RPi.GPIO 2>/dev/null || pip3 install --no-cache-dir \
    pyserial \
    numpy \
    pytesseract \
    RPi.GPIO

# 创建虚拟环境作为备用
if [ ! -d "$REAL_HOME/drone-venv" ]; then
    sudo -u "$REAL_USER" $PYTHON -m venv "$REAL_HOME/drone-venv"
    sudo -u "$REAL_USER" "$REAL_HOME/drone-venv/bin/pip" install --no-cache-dir \
        pyserial numpy opencv-python-headless pytesseract RPi.GPIO
    info "[3/8] 虚拟环境创建完成: $REAL_HOME/drone-venv"
fi

info "[3/8] Python 环境配置完成"

# ==============================================================================
# 第4步: 海康相机 MVS SDK (可选)
# ==============================================================================
info "[4/8] 海康相机 SDK..."

HIK_SDK_URL="https://www.hikrobotics.com/cn/machinevision/service/download"
echo ""
warn "海康相机 MVS SDK 需要手动从官网下载 Linux ARM64 版本:"
warn "  $HIK_SDK_URL"
warn ""
warn "如果相机通过 UVC 协议工作 (大多数海康USB相机支持),"
warn "OpenCV VideoCapture 可直接使用, 无需 SDK."
warn "跳过 SDK 安装。"

info "[4/8] 相机配置完成 (使用 UVC 模式)"
info "OpenMV模式无需OpenCV识别，只需pyserial接收结果:"
info "  python3 -m drone.main --vision-backend openmv --openmv-port /dev/ttyUSB0"

# ==============================================================================
# 第5步: 部署项目代码
# ==============================================================================
info "[5/8] 部署项目代码..."

if [ -d "$PROJECT_DIR" ]; then
    warn "项目目录已存在: $PROJECT_DIR, 备份后覆盖..."
    cp -r "$PROJECT_DIR" "${PROJECT_DIR}.bak.$(date +%Y%m%d_%H%M%S)" 2>/dev/null || true
fi

mkdir -p "$PROJECT_DIR"
mkdir -p "$PROJECT_DIR/logs"
mkdir -p "$PROJECT_DIR/test"
mkdir -p "$PROJECT_DIR/openmv"

# 复制当前项目文件 (从脚本所在目录)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/lx_protocol.py" ]; then
    info "从本地 $SCRIPT_DIR 复制项目文件..."
    cp "$SCRIPT_DIR"/*.py "$PROJECT_DIR/" 2>/dev/null || true
    cp "$SCRIPT_DIR/test"/*.py "$PROJECT_DIR/test/" 2>/dev/null || true
    cp -r "$SCRIPT_DIR/openmv"/. "$PROJECT_DIR/openmv/" 2>/dev/null || true
elif [ -n "$PROJECT_REPO" ] && [ "$PROJECT_REPO" != "https://github.com/your-org/drone.git" ]; then
    info "从 Git 仓库克隆..."
    sudo -u "$REAL_USER" git clone "$PROJECT_REPO" "$PROJECT_DIR" || warn "Git clone 失败, 请手动部署"
else
    warn "未找到项目文件来源, 请手动将 drone/*.py 复制到 $PROJECT_DIR/"
fi

# 设置权限
chown -R "$REAL_USER":"$REAL_USER" "$PROJECT_DIR"
chmod -R 755 "$PROJECT_DIR"

# 创建 __init__.py
touch "$PROJECT_DIR/__init__.py"

info "[5/8] 项目代码部署完成: $PROJECT_DIR"

# ==============================================================================
# 第6步: GPIO 权限
# ==============================================================================
info "[6/8] 配置 GPIO 权限..."

# 将 pi 用户加入 gpio 组
usermod -a -G gpio "$REAL_USER" 2>/dev/null || true
usermod -a -G dialout "$REAL_USER" 2>/dev/null || true
usermod -a -G video "$REAL_USER" 2>/dev/null || true

# 创建 udev 规则 (如有需要)
if [ ! -f /etc/udev/rules.d/99-gpio.rules ]; then
    cat > /etc/udev/rules.d/99-gpio.rules << 'UDEV'
# GPIO 权限
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c '\
    chown -R root:gpio /sys/class/gpio && \
    chmod -R 770 /sys/class/gpio && \
    chown -R root:gpio /sys/devices/virtual/gpio && \
    chmod -R 770 /sys/devices/virtual/gpio'"
UDEV
    udevadm control --reload-rules 2>/dev/null || true
fi

info "[6/8] GPIO 权限配置完成"

# ==============================================================================
# 第7步: 开机自启服务
# ==============================================================================
info "[7/8] 配置开机自启服务..."

cat > /etc/systemd/system/drone.service << EOF
[Unit]
Description=G Drone Auto Mission Controller
Documentation=https://github.com/your-org/drone
After=multi-user.target network.target
Wants=network.target

[Service]
Type=simple
User=$REAL_USER
WorkingDirectory=$PROJECT_DIR
ExecStart=/usr/bin/python3 $PROJECT_DIR/main.py
Restart=on-failure
RestartSec=5
StandardOutput=append:$PROJECT_DIR/logs/stdout.log
StandardError=append:$PROJECT_DIR/logs/stderr.log

# 安全限制
NoNewPrivileges=yes
PrivateTmp=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload

echo ""
warn "开机自启服务已创建但未启用。"
warn "如需启用, 请运行:"
warn "  sudo systemctl enable drone.service"
warn "  sudo systemctl start  drone.service"
warn ""
warn "⚠ 建议先完成调试再启用自启, 避免意外起飞!"
info "[7/8] 开机自启配置完成"

# ==============================================================================
# 第8步: 验证
# ==============================================================================
info "[8/8] 运行验证..."

PASSED=0
FAILED=0

echo ""
info "--- 检查串口 ---"
if [ -e /dev/serial0 ]; then
    ls -la /dev/serial0
    PASSED=$((PASSED + 1))
else
    warn "/dev/serial0 不存在, 请重启后重试"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 检查视觉设备 ---"
if command -v v4l2-ctl &> /dev/null; then
    v4l2-ctl --list-devices 2>/dev/null || warn "未检测到视频设备"
    PASSED=$((PASSED + 1))
else
    warn "v4l2-ctl 未安装, 无法检测相机"
fi

echo ""
info "--- 检查 Tesseract OCR ---"
if command -v tesseract &> /dev/null; then
    TESS_VERSION=$(tesseract --version 2>&1 | head -1)
    info "Tesseract: $TESS_VERSION"
    PASSED=$((PASSED + 1))
else
    warn "Tesseract 未安装"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 运行 Python 单元测试 ---"
if [ -f "$PROJECT_DIR/test/test_lx_protocol.py" ]; then
    cd "$PROJECT_DIR/test"
    if $PYTHON -m pytest test_lx_protocol.py test_path_plan.py -q 2>/dev/null; then
        PASSED=$((PASSED + 1))
    elif $PYTHON -m unittest discover -s . -p "test_*.py" -q 2>/dev/null; then
        PASSED=$((PASSED + 1))
    else
        warn "部分测试失败 (如果在非树莓派环境运行属正常)"
        FAILED=$((FAILED + 1))
    fi
else
    warn "测试文件未找到, 跳过"
fi

echo ""
echo "================================================"
info "  配置完成!"
echo ""
info "  通过:  $PASSED 项"
if [ $FAILED -gt 0 ]; then
    warn "  失败:  $FAILED 项"
fi
echo ""
warn "  重新启动以应用串口配置:"
warn "    sudo reboot"
echo ""
info "  重启后验证:"
info "    1. ls -la /dev/serial0     # 应指向 ttyAMA0"
info "    2. lsusb                   # 确认工业相机或OpenMV串口"
info "    3. python3 -c 'import cv2; print(cv2.__version__)'  # OpenCV"
info "    4. cd $PROJECT_DIR/test && python3 -m unittest discover -v"
echo ""
info "  手动测试飞行 (⚠ 务必在安全环境下):"
info "    cd $PROJECT_DIR"
info "    python3 test/test_move.py"
echo "================================================"
