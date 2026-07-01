#!/usr/bin/env bash
# ============================================================
# ElectronBot_SIM — Qwen2-VL 7B AWQ 一键部署脚本
# ============================================================
# Usage:
#   bash scripts/deploy_qwen_vl.sh download    # 下载模型权重
#   bash scripts/deploy_qwen_vl.sh serve       # 启动 vLLM 推理服务 (前台)
#   bash scripts/deploy_qwen_vl.sh daemon      # 启动 vLLM 推理服务 (后台守护)
#   bash scripts/deploy_qwen_vl.sh stop        # 停止后台服务
#   bash scripts/deploy_qwen_vl.sh test        # 测试推理
#   bash scripts/deploy_qwen_vl.sh status      # 查看服务状态
#   bash scripts/deploy_qwen_vl.sh all         # 一键下载 + 后台启动 + 测试
#
# 可选模型:
#   --model 7b-awq    Qwen2-VL-7B-Instruct-AWQ  (需 VRAM ~7GB)
#   --model 2b        Qwen2-VL-2B-Instruct      (需 VRAM ~4GB)
#   --port  PORT      指定 API 端口 (默认 8000)
#   --mirror          使用 ModelScope 镜像下载 (国内)
#   --no-mirror       直连 HuggingFace (默认)
# ============================================================

set -euo pipefail

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}============================================================${NC}"; echo -e "${BLUE}[STEP]${NC} $*"; echo -e "${BLUE}============================================================${NC}"; }

# --- Defaults ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$PROJECT_ROOT/models/qwen_vl"
PID_FILE="/tmp/electronbot_vllm.pid"
LOG_FILE="/tmp/electronbot_vllm.log"

MODEL_NAME="7b-awq"
PORT=8000
USE_MIRROR=false
ACTION=""

# --- Parse Args ---
while [[ $# -gt 0 ]]; do
    case $1 in
        download|serve|daemon|stop|test|status|all|health)
            ACTION="$1"
            shift
            ;;
        --model)
            MODEL_NAME="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --mirror)
            USE_MIRROR=true
            shift
            ;;
        --no-mirror)
            USE_MIRROR=false
            shift
            ;;
        --help|-h)
            echo "Usage: $0 <action> [--model 7b-awq|2b] [--port PORT] [--mirror]"
            echo ""
            echo "Actions:"
            echo "  download  下载模型权重"
            echo "  serve     启动 vLLM 服务 (前台)"
            echo "  daemon    启动 vLLM 服务 (后台)"
            echo "  stop      停止后台服务"
            echo "  test      测试推理"
            echo "  status    查看服务状态"
            echo "  health    健康检查"
            echo "  all       一键 download + daemon + test"
            echo ""
            echo "Options:"
            echo "  --model   7b-awq (默认) | 2b"
            echo "  --port    API 端口 (默认 8000)"
            echo "  --mirror  使用 ModelScope 镜像下载"
            exit 0
            ;;
        *)
            log_error "未知参数: $1"
            echo "Usage: $0 <action> [--model 7b-awq|2b] [--port PORT] [--mirror]"
            exit 1
            ;;
    esac
done

# --- Model Config ---
declare -A MODEL_HF_ID
declare -A MODEL_MS_ID
declare -A MODEL_LOCAL_DIR
declare -A MODEL_VRAM_GB

MODEL_HF_ID["7b-awq"]="Qwen/Qwen2-VL-7B-Instruct-AWQ"
MODEL_MS_ID["7b-awq"]="qwen/Qwen2-VL-7B-Instruct-AWQ"
MODEL_LOCAL_DIR["7b-awq"]="$MODELS_DIR/Qwen2-VL-7B-Instruct-AWQ"
MODEL_VRAM_GB["7b-awq"]=7

MODEL_HF_ID["2b"]="Qwen/Qwen2-VL-2B-Instruct"
MODEL_MS_ID["2b"]="qwen/Qwen2-VL-2B-Instruct"
MODEL_LOCAL_DIR["2b"]="$MODELS_DIR/Qwen2-VL-2B-Instruct"
MODEL_VRAM_GB["2b"]=4

if [[ -z "${MODEL_HF_ID[$MODEL_NAME]:-}" ]]; then
    log_error "未知模型: $MODEL_NAME (可选: 7b-awq, 2b)"
    exit 1
fi

HF_ID="${MODEL_HF_ID[$MODEL_NAME]}"
MS_ID="${MODEL_MS_ID[$MODEL_NAME]}"
LOCAL_DIR="${MODEL_LOCAL_DIR[$MODEL_NAME]}"
VRAM_GB="${MODEL_VRAM_GB[$MODEL_NAME]}"

# ============================================================
# Helper: CUDA / GPU check
# ============================================================
check_gpu() {
    if ! command -v nvidia-smi &>/dev/null; then
        log_error "未检测到 NVIDIA GPU / nvidia-smi。Qwen2-VL 需要 GPU。"
        exit 1
    fi

    local cuda_ver
    cuda_ver=$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[\d.]+' || echo "unknown")
    local vram_mb
    vram_mb=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 || echo "0")
    local vram_gb=$((vram_mb / 1024))

    log_info "GPU 驱动 CUDA: $cuda_ver, 显存: ${vram_gb}GB"

    if [ "$vram_gb" -lt "$VRAM_GB" ]; then
        log_error "显存不足: 需要 ≥${VRAM_GB}GB, 当前 ${vram_gb}GB"
        log_info "  请换用小模型: --model 2b (需 ~4GB)"
        exit 1
    fi
    log_info "GPU 检查: OK (${vram_gb}GB ≥ ${VRAM_GB}GB)"
}

check_vllm_installed() {
    if ! python -c "import vllm" 2>/dev/null; then
        log_error "vLLM 未安装。请先运行: bash setup_env.sh --gpu --full"
        exit 1
    fi
    log_info "vLLM: OK"
}

check_venv() {
    if [ -z "${VIRTUAL_ENV:-}" ]; then
        if [ -f "$PROJECT_ROOT/.venv/bin/activate" ]; then
            log_warn "未激活虚拟环境，自动激活..."
            source "$PROJECT_ROOT/.venv/bin/activate"
            log_info "Virtual env 已激活: $VIRTUAL_ENV"
        else
            log_error "未找到虚拟环境。请先运行: bash setup_env.sh"
            exit 1
        fi
    fi
}

# ============================================================
# Action: download
# ============================================================
do_download() {
    log_step "下载模型: $HF_ID"

    # Check disk
    local free_gb
    free_gb=$(df -BG "$PROJECT_ROOT" 2>/dev/null | tail -1 | awk '{print $4}' | sed 's/G//')
    local needed_gb=$((VRAM_GB * 3))
    if [ "${free_gb:-0}" -lt "$needed_gb" ]; then
        log_warn "磁盘可用: ${free_gb}GB, 建议 ≥${needed_gb}GB (模型 + 缓存)"
    fi

    # Already exists?
    if [ -d "$LOCAL_DIR" ] && [ -n "$(ls -A "$LOCAL_DIR" 2>/dev/null)" ] && [ -f "$LOCAL_DIR/config.json" ]; then
        log_info "模型已存在: $LOCAL_DIR"
        local size
        size=$(du -sh "$LOCAL_DIR" 2>/dev/null | cut -f1)
        log_info "  大小: $size"
        return 0
    fi

    mkdir -p "$(dirname "$LOCAL_DIR")"

    if $USE_MIRROR; then
        log_info "使用 ModelScope 镜像下载..."
        pip install modelscope -q 2>/dev/null || true

        python -c "
import os, sys
from modelscope import snapshot_download
os.makedirs('$LOCAL_DIR', exist_ok=True)
cache = snapshot_download('$MS_ID', cache_dir=os.path.join('$LOCAL_DIR', '..'))
# Symlink cache → local_dir
target = '$LOCAL_DIR'
if not os.path.exists(os.path.join(target, 'config.json')):
    for f in os.listdir(cache):
        src = os.path.join(cache, f)
        dst = os.path.join(target, f)
        if not os.path.exists(dst):
            os.symlink(src, dst)
print('Download OK')
" || {
            log_error "ModelScope 下载失败"
            log_info "手动下载: git lfs clone https://huggingface.co/$HF_ID $LOCAL_DIR"
            exit 1
        }
    else
        log_info "使用 HuggingFace Hub 下载..."

        # Check HF token
        local hf_token="${HF_TOKEN:-${HUGGINGFACE_HUB_TOKEN:-}}"
        local hf_extra_args=""
        if [ -n "$hf_token" ]; then
            hf_extra_args="--token $hf_token"
        else
            log_warn "未设置 HF_TOKEN 环境变量。公开模型仍可下载。"
        fi

        # Check huggingface-cli
        if ! command -v huggingface-cli &>/dev/null; then
            log_info "安装 huggingface_hub..."
            pip install huggingface_hub[cli] -q
        fi

        huggingface-cli download "$HF_ID" \
            --local-dir "$LOCAL_DIR" \
            --local-dir-use-symlinks False \
            --resume-download \
            $hf_extra_args || {
            log_error "HF 下载失败。重试建议:"
            log_error "  1. 设置 HF_TOKEN 环境变量"
            log_error "  2. 使用镜像: $0 download --mirror"
            log_error "  3. 手动: git lfs clone https://huggingface.co/$HF_ID $LOCAL_DIR"
            exit 1
        }
    fi

    # Verify
    if [ -f "$LOCAL_DIR/config.json" ]; then
        local size
        size=$(du -sh "$LOCAL_DIR" 2>/dev/null | cut -f1)
        log_info "模型下载完成: $LOCAL_DIR ($size)"
    else
        log_error "下载验证失败: config.json 未找到"
        log_info "请手动检查目录: $LOCAL_DIR"
        exit 1
    fi
}

# ============================================================
# Action: serve (前台)
# ============================================================
do_serve() {
    log_step "启动 vLLM 推理服务 (前台)"

    check_gpu
    check_vllm_installed
    check_venv

    if [ ! -d "$LOCAL_DIR" ]; then
        log_error "模型未下载: $LOCAL_DIR"
        log_info "先运行: $0 download"
        exit 1
    fi

    # Check port
    if lsof -i ":$PORT" &>/dev/null 2>&1; then
        log_error "端口 $PORT 已被占用"
        lsof -i ":$PORT" 2>/dev/null
        log_info "使用 --port 指定其他端口，或先 stop 旧实例"
        exit 1
    fi

    log_info "模型: $LOCAL_DIR"
    log_info "端口: $PORT"
    log_info "API:  http://localhost:$PORT/v1"
    log_info "日志: $LOG_FILE"
    log_info "按 Ctrl+C 停止服务"
    echo ""

    python -m vllm.entrypoints.openai.api_server \
        --model "$LOCAL_DIR" \
        --host 0.0.0.0 \
        --port "$PORT" \
        --max-model-len 4096 \
        --gpu-memory-utilization 0.85 \
        --trust-remote-code \
        --dtype auto \
        --enforce-eager \
        2>&1 | tee "$LOG_FILE"
}

# ============================================================
# Action: daemon (后台)
# ============================================================
do_daemon() {
    log_step "启动 vLLM 推理服务 (后台守护)"

    check_gpu
    check_vllm_installed
    check_venv

    if [ ! -d "$LOCAL_DIR" ]; then
        log_error "模型未下载: $LOCAL_DIR"
        log_info "先运行: $0 download"
        exit 1
    fi

    # Stop existing
    if [ -f "$PID_FILE" ]; then
        local old_pid
        old_pid=$(cat "$PID_FILE" 2>/dev/null || echo "")
        if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
            log_warn "已有运行中的实例 (PID $old_pid)，正在停止..."
            kill "$old_pid" 2>/dev/null || true
            sleep 3
            if kill -0 "$old_pid" 2>/dev/null; then
                kill -9 "$old_pid" 2>/dev/null || true
                sleep 1
            fi
        fi
        rm -f "$PID_FILE"
    fi

    # Check port
    if lsof -i ":$PORT" &>/dev/null 2>&1; then
        log_warn "端口 $PORT 被占用，尝试释放..."
        fuser -k "$PORT/tcp" 2>/dev/null || true
        sleep 2
    fi

    log_info "模型: $LOCAL_DIR"
    log_info "端口: $PORT"
    log_info "API:  http://localhost:$PORT/v1"
    log_info "PID:  写入 $PID_FILE"
    log_info "日志: $LOG_FILE"

    nohup python -m vllm.entrypoints.openai.api_server \
        --model "$LOCAL_DIR" \
        --host 0.0.0.0 \
        --port "$PORT" \
        --max-model-len 4096 \
        --gpu-memory-utilization 0.85 \
        --trust-remote-code \
        --dtype auto \
        --enforce-eager \
        >> "$LOG_FILE" 2>&1 &

    local pid=$!
    echo "$pid" > "$PID_FILE"

    log_info "服务启动中 (PID $pid)..."
    log_info "等待模型加载 (可能需要 30-90 秒)..."

    # Wait for ready state with timeout
    local max_wait=120
    local waited=0
    while [ $waited -lt $max_wait ]; do
        if do_health_check_silent; then
            log_info "服务就绪 (耗时 ${waited}s)!"
            log_info "API: http://localhost:$PORT/v1/chat/completions"
            return 0
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            log_error "服务异常退出! 查看日志: tail -50 $LOG_FILE"
            exit 1
        fi
        sleep 5
        waited=$((waited + 5))
        if [ $((waited % 15)) -eq 0 ]; then
            log_info "  等待中... (${waited}s / ${max_wait}s)"
        fi
    done

    log_error "服务启动超时 (${max_wait}s)"
    log_error "查看日志: tail -50 $LOG_FILE"
    kill "$pid" 2>/dev/null || true
    exit 1
}

do_health_check_silent() {
    curl -s -o /dev/null -w "%{http_code}" "http://localhost:$PORT/health" 2>/dev/null | grep -q "200"
}

# ============================================================
# Action: stop
# ============================================================
do_stop() {
    log_step "停止 vLLM 服务"

    if [ -f "$PID_FILE" ]; then
        local pid
        pid=$(cat "$PID_FILE" 2>/dev/null || echo "")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            log_info "发送 SIGTERM → PID $pid"
            kill "$pid" 2>/dev/null || true
            sleep 3
            if kill -0 "$pid" 2>/dev/null; then
                log_warn "强制终止 SIGKILL → PID $pid"
                kill -9 "$pid" 2>/dev/null || true
                sleep 1
            fi
            log_info "服务已停止 (PID $pid)"
        else
            log_warn "PID $pid 已不存在"
        fi
        rm -f "$PID_FILE"
    else
        log_warn "未找到 PID 文件 ($PID_FILE)"
    fi

    # Kill any process on our port
    if lsof -i ":$PORT" &>/dev/null 2>&1; then
        log_info "释放端口 $PORT..."
        fuser -k "$PORT/tcp" 2>/dev/null || true
    fi

    log_info "停止操作完成"
}

# ============================================================
# Action: status
# ============================================================
do_status() {
    log_step "vLLM 服务状态"

    if [ -f "$PID_FILE" ]; then
        local pid
        pid=$(cat "$PID_FILE" 2>/dev/null || echo "")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo -e "  状态: ${GREEN}运行中${NC}"
            echo "  PID:  $pid"
            echo "  端口: $PORT"
            local uptime_sec
            uptime_sec=$(ps -p "$pid" -o etimes= 2>/dev/null | tr -d ' ' || echo "?")
            echo "  运行: ${uptime_sec}s"
        else
            echo -e "  状态: ${YELLOW}PID 存在但进程已消失 (僵尸 PID 文件)${NC}"
        fi
    else
        echo -e "  状态: ${RED}未运行${NC} (无 PID 文件)"
    fi

    # Check port directly
    if lsof -i ":$PORT" &>/dev/null 2>&1; then
        echo -e "  端口 $PORT: ${GREEN}活跃${NC}"
        lsof -i ":$PORT" 2>/dev/null | tail -n +2 | head -3
    else
        echo -e "  端口 $PORT: ${RED}无监听${NC}"
    fi

    # Health check
    if do_health_check_silent; then
        echo -e "  健康检查: ${GREEN}OK${NC}"
    else
        echo -e "  健康检查: ${RED}FAIL${NC}"
    fi

    echo ""
    echo "  日志: tail -f $LOG_FILE"
}

# ============================================================
# Action: health
# ============================================================
do_health() {
    log_step "健康检查"

    local resp
    resp=$(curl -s "http://localhost:$PORT/health" 2>/dev/null || echo "")
    if echo "$resp" | grep -q "ok\|200"; then
        log_info "服务健康: OK"
    elif echo "$resp" | grep -q "."; then
        log_info "服务响应: $resp"
    else
        log_error "服务无响应 (端口 $PORT)"
        log_info "查看状态: $0 status"
        exit 1
    fi
}

# ============================================================
# Action: test
# ============================================================
do_test() {
    log_step "测试推理"

    # Ensure service is running
    if ! do_health_check_silent; then
        log_error "vLLM 服务未运行 (端口 $PORT)"
        log_info "先启动: $0 daemon"
        exit 1
    fi

    log_info "模型: $MODEL_NAME"
    log_info "端口: $PORT"

    python -c "
import base64, json, requests, sys
import numpy as np

# Generate test image (240x240 random)
img = np.random.randint(0, 255, (240, 240, 3), dtype=np.uint8)
import cv2
_, buf = cv2.imencode('.jpg', img)
img_b64 = base64.b64encode(buf).decode()

prompt = '请挥手打招呼，返回6个关节角度(度): [head, left_arm, left_roll, right_arm, right_roll, body]'

payload = {
    'model': '$(basename "$LOCAL_DIR")',
    'messages': [{
        'role': 'user',
        'content': [
            {'type': 'image_url', 'image_url': f\"data:image/jpeg;base64,{img_b64}\"},
            {'type': 'text', 'text': prompt},
        ]
    }],
    'max_tokens': 256,
    'temperature': 0.1,
}

print(f'[INFO] 发送推理请求...')
resp = requests.post(
    'http://localhost:$PORT/v1/chat/completions',
    json=payload,
    timeout=60,
)

if resp.status_code == 200:
    result = resp.json()
    output = result['choices'][0]['message']['content']
    print(f'[SUCCESS] 推理完成!')
    print(f'[OUTPUT] {output}')

    # Parse angles
    import re
    nums = re.findall(r'-?\d+\.?\d*', output)
    if len(nums) >= 6:
        angles = [float(n) for n in nums[:6]]
        print(f'[ANGLES] {angles}')
    else:
        print(f'[WARN] 未能解析 6 个角度值')

    # Performance
    usage = result.get('usage', {})
    if usage:
        prompt_tokens = usage.get('prompt_tokens', '?')
        completion_tokens = usage.get('completion_tokens', '?')
        elapsed = resp.elapsed.total_seconds()
        print(f'[STATS] prompt_tokens={prompt_tokens}, completion_tokens={completion_tokens}, time={elapsed:.2f}s')

    sys.exit(0)
else:
    print(f'[ERROR] HTTP {resp.status_code}: {resp.text[:500]}')
    sys.exit(1)
" 2>/dev/null && log_info "测试通过" || log_error "测试失败"
}

# ============================================================
# Action: all (download + daemon + test)
# ============================================================
do_all() {
    log_step "一键部署 Qwen2-VL (download → daemon → test)"

    do_download
    echo ""

    do_daemon
    echo ""

    do_test
    echo ""

    log_info "全部完成!"
    log_info "  API:   http://localhost:$PORT/v1"
    log_info "  状态:  $0 status"
    log_info "  停止:  $0 stop"
    log_info "  日志:  tail -f $LOG_FILE"
}

# ============================================================
# Main
# ============================================================
case "$ACTION" in
    download)  do_download ;;
    serve)     do_serve ;;
    daemon)    do_daemon ;;
    stop)      do_stop ;;
    test)      do_test ;;
    status)    do_status ;;
    health)    do_health ;;
    all)       do_all ;;
    *)
        log_error "未知操作: ${ACTION:-未指定}"
        echo ""
        echo "Usage: $0 <action> [--model 7b-awq|2b] [--port PORT] [--mirror]"
        echo ""
        echo "Actions: download | serve | daemon | stop | test | status | health | all"
        echo ""
        echo "示例:"
        echo "  $0 all                              # 一键部署 (7B-AWQ)"
        echo "  $0 all --model 2b                   # 一键部署 (2B, 轻量)"
        echo "  $0 download --mirror                # 镜像下载"
        echo "  $0 daemon --port 8080               # 后台启动，端口 8080"
        echo "  $0 test                             # 测试已有服务"
        exit 1
        ;;
esac
