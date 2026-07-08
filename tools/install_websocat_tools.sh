#!/bin/bash
#
# ElectronBot_SIM 固件烧录工具一键安装脚本
# 安装 esptool、websocat、minicom，配置串口权限
#
# 用法:
#   bash tools/install_websocat_tools.sh
#

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo ""
echo "=============================="
echo "  ElectronBot 工具安装"
echo "=============================="
echo ""

# ---- 1. 系统更新 + apt 工具 ----
log_info "更新软件源..."
sudo apt update -qq

log_info "安装系统工具..."
sudo apt install -y wget minicom

# ---- 2. esptool ----
log_info "安装 esptool（固件烧录）..."
pip install esptool -q
esptool.py version 2>&1 | head -1

# ---- 3. websocat（参考 https://blog.csdn.net/sunyuhua_keyboard/article/details/135837535） ----
log_info "安装 websocat（WebSocket 客户端）..."

WEBSOCAT_BIN="websocat_amd64-linux"

if [ -f "${SCRIPT_DIR}/${WEBSOCAT_BIN}" ]; then
    log_info "  使用本地二进制: tools/${WEBSOCAT_BIN}"
    chmod +x "${SCRIPT_DIR}/${WEBSOCAT_BIN}"
    sudo cp "${SCRIPT_DIR}/${WEBSOCAT_BIN}" /usr/local/bin/websocat
else
    log_info "  从 GitHub 下载..."
    cd /tmp
    wget -q --show-progress "https://github.com/vi/websocat/releases/download/v1.8.0/${WEBSOCAT_BIN}" -O "${WEBSOCAT_BIN}"
    chmod +x "${WEBSOCAT_BIN}"
    sudo mv "${WEBSOCAT_BIN}" /usr/local/bin/websocat
fi
websocat --version

# ---- 4. 串口权限 ----
log_info "配置串口权限..."
if groups "$USER" | grep -qE '\bdialout\b'; then
    log_info "  dialout 组已配置，跳过"
else
    sudo usermod -a -G dialout,plugdev "$USER"
    log_warn "  已添加 $USER 到 dialout/plugdev 组，需重新登录生效"
fi

# ---- 5. 验证 ----
echo ""
echo "=============================="
echo "  安装验证"
echo "=============================="

ok()  { echo -e "  ${GREEN}✓${NC} $1"; }
fail(){ echo -e "  ${RED}✗${NC} $1 — 请手动排查"; }

command -v esptool.py &>/dev/null && ok "esptool"     || fail "esptool"
command -v websocat    &>/dev/null && ok "websocat"    || fail "websocat"
command -v minicom     &>/dev/null && ok "minicom"     || fail "minicom"

if groups "$USER" | grep -qE '\bdialout\b'; then
    ok "串口权限 (dialout)"
else
    echo -e "  ${YELLOW}⚠${NC}  串口权限未生效 — 请重新登录或执行: newgrp dialout"
fi

echo ""
echo "=============================="
echo "  常用命令"
echo "=============================="
echo "  查看串口:         ls /dev/ttyUSB* /dev/ttyACM*"
echo "  烧录固件:         esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 921600 write_flash 0x0 firmware.bin"
echo "  串口监视:         minicom -D /dev/ttyUSB0 -b 115200"
echo "  WebSocket 测试:   websocat ws://192.168.x.x:8080/ws"
echo ""
echo -e "${GREEN}安装完成！${NC}"
