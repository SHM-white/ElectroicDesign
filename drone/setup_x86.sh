#!/usr/bin/env bash
# ==============================================================================
# setup_x86.sh — x86 迷你工控机 一键环境配置脚本
# G_植保飞行器 项目 | x86 迷你PC + 凌霄飞控 + 海康工业相机
#
# 适用: 干净的 Ubuntu 22.04 / 24.04 LTS (amd64)
#
# 用法:
#   chmod +x setup_x86.sh
#   sudo ./setup_x86.sh
#
# 配置内容:
#   1. 系统更新 + 基础工具
#   2. USB-TTL 串口 udev 规则 + 相机检测
#   3. Python3 + OpenCV + 依赖
#   4. 可选 GPIO (FT232H USB 适配器)
#   5. 项目代码部署
#   6. systemd 开机自启服务 (未启用, 需手动)
#   7. 验证测试
# ==============================================================================

set -e  # 遇错即停

# ── 颜色输出 ──────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── 配置变量 ──────────────────────────────────────────
PROJECT_DIR="$HOME/drone"
PYTHON="python3"

# ── 检查root权限 ──────────────────────────────────────
if [ "$(id -u)" -ne 0 ]; then
    error "请使用 sudo 运行此脚本: sudo ./setup_x86.sh"
    exit 1
fi

REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME=$(eval echo "~$REAL_USER")

info "================================================"
info "  G_植保飞行器 - x86 工控机一键环境配置"
info "================================================"
info "用户:     $REAL_USER"
info "主目录:   $REAL_HOME"
info "项目目录: $PROJECT_DIR"
info "系统:     $(lsb_release -ds 2>/dev/null || echo 'Ubuntu')"
info "架构:     $(uname -m)"
info ""

# ==============================================================================
# [1/8] 系统更新 + 基础工具
# ==============================================================================
info "[1/8] 系统更新与基础工具安装..."

apt-get update -qq
apt-get upgrade -y -qq

info "[1/8] 安装完成"

# ==============================================================================
# [2/8] 安装基础软件包
# ==============================================================================
info "[2/8] 安装基础软件包..."

apt-get install -y -qq \
    git vim tmux htop \
    build-essential cmake pkg-config \
    tesseract-ocr \
    v4l-utils \
    i2c-tools \
    usbutils \
    python3 python3-pip python3-venv \
    python3-opencv

info "[2/8] 基础软件包安装完成"

# ==============================================================================
# [3/8] Python 环境配置
# ==============================================================================
info "[3/8] 配置 Python 环境..."

# 安装系统级 pip 包 (供全局使用)
pip3 install --break-system-packages --no-cache-dir \
    pyserial \
    numpy \
    opencv-python-headless \
    pytesseract \
    2>/dev/null || pip3 install --no-cache-dir \
    pyserial \
    numpy \
    opencv-python-headless \
    pytesseract

# 创建虚拟环境作为备用
if [ ! -d "$REAL_HOME/drone-venv" ]; then
    sudo -u "$REAL_USER" $PYTHON -m venv "$REAL_HOME/drone-venv"
    sudo -u "$REAL_USER" "$REAL_HOME/drone-venv/bin/pip" install --no-cache-dir \
        pyserial numpy opencv-python-headless pytesseract
    info "[3/8] 虚拟环境创建完成: $REAL_HOME/drone-venv"
fi

info "[3/8] Python 环境配置完成"

# ==============================================================================
# [4/8] 可选 GPIO (FT232H USB适配器)
# ==============================================================================
info "[4/8] GPIO 配置..."

echo ""
info "x86 平台无原生 GPIO, 可选以下方案控制激光/LED:"
info "  方案A: 飞控端 PWM/IO 直连 (推荐, 无需额外硬件)"
info "  方案B: FT232H USB-to-GPIO 适配器 (Adafruit FT232H 等)"
info ""
info "如使用方案B, 安装 pyftdi:"
info "  pip3 install pyftdi"
info ""

# 预装 pyftdi (注释掉, 按需取消注释)
# pip3 install --break-system-packages --no-cache-dir pyftdi 2>/dev/null || \
#     pip3 install --no-cache-dir pyftdi

# 添加用户到 dialout 和 video 组 (USB 设备权限)
usermod -a -G dialout "$REAL_USER" 2>/dev/null || true
usermod -a -G video "$REAL_USER" 2>/dev/null || true

info "[4/8] GPIO 配置完成 (如用 FT232H, 请手动取消注释 pyftdi 安装)"

# ==============================================================================
# [5/8] USB-TTL 串口配置
# ==============================================================================
info "[5/8] 配置 USB-TTL 串口..."

# 创建 udev 规则: 将 USB-TTL 设备 (CP210x / CH340 / FT232 等) 固定为 /dev/drone_mcu
cat > /etc/udev/rules.d/99-usb-serial.rules << 'UDEV'
# USB-TTL 串口 → /dev/drone_mcu 符号链接
# 匹配常见 USB-TTL 芯片: CP210x, CH340, FT232, PL2303
# 如需精确匹配特定设备, 替换 idVendor/idProduct 为 lsusb 查到的值:
#   lsusb | grep -i serial

# CP210x (Silicon Labs)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="drone_mcu", MODE="0666"
# CH340/CH341 (WCH)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="drone_mcu", MODE="0666"
# FT232 (FTDI)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="drone_mcu", MODE="0666"
# PL2303 (Prolific)
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="drone_mcu", MODE="0666"

# 通用规则: 任何 USB 串口设备
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", MODE="0666"
UDEV

udevadm control --reload-rules 2>/dev/null || true
udevadm trigger 2>/dev/null || true

# 检查当前插入的 USB 串口
echo ""
if ls /dev/ttyUSB* 2>/dev/null >/dev/null; then
    info "检测到 USB-TTL 设备:"
    ls -la /dev/ttyUSB* 2>/dev/null || true
else
    warn "未检测到 /dev/ttyUSB* 设备"
    warn "请插入 USB-TTL 模块后重新运行: sudo udevadm trigger"
fi

info "[5/8] USB-TTL 串口配置完成"

# ==============================================================================
# [6/8] 相机配置
# ==============================================================================
info "[6/8] 相机配置..."

echo ""
if command -v v4l2-ctl &> /dev/null; then
    info "检测视频设备:"
    v4l2-ctl --list-devices 2>/dev/null || warn "未检测到视频设备"
else
    warn "v4l2-ctl 未安装"
fi

echo ""
info "海康机器人工业相机说明:"
info "  海康 USB3.0 工业相机通常支持 UVC 协议,"
info "  OpenCV VideoCapture(0) 可直接打开, 无需额外 SDK。"
info "  如使用 GigE 网口相机, 需安装 MVS SDK (从海康官网下载 amd64 版本):"
info "    https://www.hikrobotics.com/cn/machinevision/service/download"
info ""

info "[6/8] 相机配置完成"

# ==============================================================================
# [7/8] 部署项目代码
# ==============================================================================
info "[7/8] 部署项目代码..."

if [ -d "$PROJECT_DIR" ]; then
    warn "项目目录已存在: $PROJECT_DIR, 备份后覆盖..."
    cp -r "$PROJECT_DIR" "${PROJECT_DIR}.bak.$(date +%Y%m%d_%H%M%S)" 2>/dev/null || true
fi

mkdir -p "$PROJECT_DIR"
mkdir -p "$PROJECT_DIR/logs"
mkdir -p "$PROJECT_DIR/test"

# 复制当前项目文件 (从脚本所在目录)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/lx_protocol.py" ]; then
    info "从本地 $SCRIPT_DIR 复制项目文件..."
    cp "$SCRIPT_DIR"/*.py "$PROJECT_DIR/" 2>/dev/null || true
    cp "$SCRIPT_DIR/test"/*.py "$PROJECT_DIR/test/" 2>/dev/null || true
else
    warn "未在脚本目录找到项目文件, 请手动将 drone/*.py 复制到 $PROJECT_DIR/"
fi

# 设置权限
chown -R "$REAL_USER":"$REAL_USER" "$PROJECT_DIR"
chmod -R 755 "$PROJECT_DIR"

# 创建 __init__.py
touch "$PROJECT_DIR/__init__.py"

info "[7/8] 项目代码部署完成: $PROJECT_DIR"

# ==============================================================================
# [8/8] systemd 开机自启服务
# ==============================================================================
info "[8/8] 配置 systemd 服务..."

cat > /etc/systemd/system/drone.service << EOF
[Unit]
Description=G Drone Auto Mission Controller
Documentation=https://github.com/SHM-white/ElectroicDesign
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
warn "══════════════════════════════════════════════════"
warn "  systemd 服务已创建但 未启用！"
warn ""
warn "  如需启用开机自启, 请完成调试后运行:"
warn "    sudo systemctl enable drone.service"
warn "    sudo systemctl start  drone.service"
warn ""
warn "  ⚠ 务必先在安全环境下完成全部调试再启用自启!"
warn "  ⚠ 意外自启可能导致无人机意外起飞！"
warn "══════════════════════════════════════════════════"
info "[8/8] systemd 服务配置完成"

# ==============================================================================
# [9/9] 验证
# ==============================================================================
info "[9/9] 运行验证..."

PASSED=0
FAILED=0

echo ""
info "--- 检查 Python3 ---"
if command -v python3 &> /dev/null; then
    PY_VER=$(python3 --version)
    info "Python3: $PY_VER"
    PASSED=$((PASSED + 1))
else
    warn "Python3 未安装"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 检查 OpenCV ---"
if python3 -c "import cv2; print('OpenCV', cv2.__version__)" 2>/dev/null; then
    PASSED=$((PASSED + 1))
else
    warn "OpenCV 导入失败"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 检查 NumPy ---"
if python3 -c "import numpy; print('NumPy', numpy.__version__)" 2>/dev/null; then
    PASSED=$((PASSED + 1))
else
    warn "NumPy 导入失败"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 检查 pyserial ---"
if python3 -c "import serial; print('pySerial', serial.__version__)" 2>/dev/null; then
    PASSED=$((PASSED + 1))
else
    warn "pySerial 导入失败"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 检查 pytesseract ---"
if python3 -c "import pytesseract; print('pytesseract OK')" 2>/dev/null; then
    PASSED=$((PASSED + 1))
else
    warn "pytesseract 导入失败"
    FAILED=$((FAILED + 1))
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
info "--- 检查串口 ---"
if ls /dev/ttyUSB* 2>/dev/null >/dev/null; then
    info "检测到 USB 串口设备:"
    ls -la /dev/ttyUSB* 2>/dev/null
    PASSED=$((PASSED + 1))
elif [ -e /dev/drone_mcu ]; then
    info "/dev/drone_mcu 符号链接存在:"
    ls -la /dev/drone_mcu
    PASSED=$((PASSED + 1))
else
    warn "未检测到 USB 串口设备, 请插入 USB-TTL 模块"
    warn "设备通常会出现在 /dev/ttyUSB0"
    FAILED=$((FAILED + 1))
fi

echo ""
info "--- 检查相机 ---"
VIDEO_DEVICES=$(v4l2-ctl --list-devices 2>/dev/null | grep -c '/dev/video' || true)
if [ "$VIDEO_DEVICES" -gt 0 ] 2>/dev/null; then
    v4l2-ctl --list-devices 2>/dev/null
    info "检测到 $VIDEO_DEVICES 个视频设备"
    PASSED=$((PASSED + 1))
else
    warn "未检测到 /dev/video* 设备, 请连接相机"
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
        warn "部分测试失败 (无硬件环境下属正常)"
        FAILED=$((FAILED + 1))
    fi
else
    warn "测试文件未找到, 跳过"
fi

# ── 汇总 ──────────────────────────────────────────────
echo ""
echo "================================================"
info "  配置完成!"
echo ""
info "  通过:  $PASSED 项"
if [ $FAILED -gt 0 ]; then
    warn "  失败:  $FAILED 项"
fi
echo ""
info "  后续步骤:"
info "    1. 插入 USB-TTL 模块, 确认设备出现:"
info "       ls -la /dev/ttyUSB*"
info ""
info "    2. 检查 config.py 中的串口配置:"
info "       grep SERIAL_PORT $PROJECT_DIR/config.py"
info "       应显示: SERIAL_PORT = '/dev/ttyUSB0'"
info ""
info "    3. 连接相机并验证 OpenCV:"
info "       python3 -c 'import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened())'"
info ""
info "    4. 手动测试飞行 (⚠ 务必在安全环境下):"
info "       cd $PROJECT_DIR"
info "       python3 test/test_move.py"
info ""
info "    5. 调试通过后启用开机自启 (可选):"
info "       sudo systemctl enable drone.service"
info "       sudo systemctl start  drone.service"
info ""
warn "  ⚠ 无人机项目涉及安全风险, 请务必:" 
warn "  ⚠  - 先在 DRY_RUN=True 模式下调试"
warn "  ⚠  - 拆桨测试确认传感器和执行器正确"
warn "  ⚠  - 在空旷场地, 确保紧急停机方案就绪"
echo "================================================"
