#!/bin/bash
# ============================================================
# ElectronBot_SIM VLA LLM 环境安装脚本
# 用途: 安装 ollama + qwen2.5:7b，用于 Phase 7 VLA 本地推理
# 硬件要求: CPU 8核+ / 内存 16GB+ / (可选) NVIDIA GPU 8GB+
# ============================================================
set -euo pipefail

# ── 颜色输出 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${CYAN}▶ $*${NC}"; }

# ── 配置 ──
OLLAMA_MODEL="${OLLAMA_MODEL:-qwen2.5:7b}"
OLLAMA_INSTALL_URL="https://ollama.com/install.sh"

# ═══════════════════════════════════════════════════════════════
# 1. 检查系统环境
# ═══════════════════════════════════════════════════════════════
log_step "Step 1/5: 检查系统环境"

# 检查架构
ARCH=$(uname -m)
log_info "系统架构: $ARCH"

# 检查内存
MEM_GB=$(free -g | awk '/^Mem:/{print $2}')
log_info "可用内存: ${MEM_GB}GB"
if [ "$MEM_GB" -lt 16 ]; then
    log_warn "内存不足 16GB，运行 7B 模型可能较慢"
fi

# 检查 NVIDIA GPU
if command -v nvidia-smi &>/dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    GPU_MEM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null | head -1 | sed 's/ MiB//')
    log_info "GPU: $GPU_NAME (${GPU_MEM} MiB)"
    if [ "$GPU_MEM" -lt 8000 ]; then
        log_warn "GPU 显存 < 8GB，建议使用 CPU 模式: OLLAMA_LLM_LIBRARY=off ollama serve"
    fi
else
    log_warn "未检测到 NVIDIA GPU，将使用 CPU 推理（速度较慢）"
fi

# 检查磁盘
DISK_AVAIL_GB=$(df -BG . | awk 'NR==2{print $4}' | sed 's/G//')
log_info "可用磁盘: ${DISK_AVAIL_GB}GB"
if [ "$DISK_AVAIL_GB" -lt 10 ]; then
    log_error "磁盘空间不足 10GB（需要约 5GB 存放模型），请清理后重试"
    exit 1
fi

# ═══════════════════════════════════════════════════════════════
# 2. 安装 ollama
# ═══════════════════════════════════════════════════════════════
log_step "Step 2/5: 安装 ollama"

if command -v ollama &>/dev/null; then
    OLLAMA_VER=$(ollama --version 2>/dev/null || echo "unknown")
    log_info "ollama 已安装: $OLLAMA_VER"
else
    log_info "正在安装 ollama..."
    if ! curl -fsSL "$OLLAMA_INSTALL_URL" | sh; then
        log_error "ollama 安装失败，请手动安装: curl -fsSL $OLLAMA_INSTALL_URL | sh"
        exit 1
    fi
    log_info "ollama 安装完成"
fi

# ═══════════════════════════════════════════════════════════════
# 3. 启动 ollama 服务
# ═══════════════════════════════════════════════════════════════
log_step "Step 3/5: 启动 ollama 服务"

# 检查服务是否已在运行
if curl -s http://localhost:11434/api/tags &>/dev/null; then
    log_info "ollama 服务已在运行"
else
    log_info "正在启动 ollama 服务..."
    # 尝试用 systemd
    if systemctl --version &>/dev/null 2>&1; then
        sudo systemctl start ollama 2>/dev/null || true
        sleep 2
    fi

    # 如果 systemd 没启动，手动后台启动
    if ! curl -s http://localhost:11434/api/tags &>/dev/null; then
        log_info "手动启动 ollama..."
        nohup ollama serve &>/tmp/ollama-serve.log &
        # 等待服务就绪
        for i in $(seq 1 30); do
            if curl -s http://localhost:11434/api/tags &>/dev/null; then
                break
            fi
            sleep 1
        done
    fi
fi

if ! curl -s http://localhost:11434/api/tags &>/dev/null; then
    log_error "ollama 服务启动失败，请检查 /tmp/ollama-serve.log"
    exit 1
fi
log_info "ollama 服务运行正常 (http://localhost:11434)"

# ═══════════════════════════════════════════════════════════════
# 4. 拉取模型
# ═══════════════════════════════════════════════════════════════
log_step "Step 4/5: 拉取模型 $OLLAMA_MODEL"

if ollama list 2>/dev/null | grep -q "$OLLAMA_MODEL"; then
    log_info "模型 $OLLAMA_MODEL 已存在"
else
    log_info "正在拉取 $OLLAMA_MODEL（约 4.5GB，首次下载需要几分钟）..."
    if ! ollama pull "$OLLAMA_MODEL"; then
        log_error "模型拉取失败"
        exit 1
    fi
    log_info "模型拉取完成"
fi

# ═══════════════════════════════════════════════════════════════
# 5. 安装 Python 依赖
# ═══════════════════════════════════════════════════════════════
log_step "Step 5/5: 安装 Python ollama 库"

if python -c "import ollama" 2>/dev/null; then
    log_info "Python ollama 库已安装"
else
    log_info "正在安装 ollama Python 库..."
    pip install ollama
    log_info "安装完成"
fi

# ═══════════════════════════════════════════════════════════════
# 完成
# ═══════════════════════════════════════════════════════════════
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         ✅ VLA LLM 环境安装完成                               ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  模型: $OLLAMA_MODEL"
printf "║  服务: http://localhost:11434                              ║\n"
echo "║                                                              ║"
echo "║  快速测试:                                                   ║"
echo "║    ollama run $OLLAMA_MODEL                                  "
printf "║                                                              ║\n"
echo "║  在代码中使用:                                               ║"
echo "║    import ollama                                             ║"
echo "║    resp = ollama.chat(model='$OLLAMA_MODEL',                 "
printf "║                        messages=[...])                       ║\n"
echo "║                                                              ║"
echo "║  停止服务: sudo systemctl stop ollama                        ║"
echo "║  查看日志: journalctl -u ollama -f                           ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
