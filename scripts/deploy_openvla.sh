#!/usr/bin/env bash
# ============================================================
# ElectronBot_SIM — OpenVLA 部署脚本
# ============================================================
# Usage:
#   bash scripts/deploy_openvla.sh install         # 安装 OpenVLA
#   bash scripts/deploy_openvla.sh download        # 下载 OpenVLA 模型
#   bash scripts/deploy_openvla.sh serve           # 启动推理 API (vLLM)
#   bash scripts/deploy_openvla.sh test            # 测试推理
#   bash scripts/deploy_openvla.sh all             # install + download + test
# ============================================================

set -euo pipefail

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}============================================================${NC}"; echo -e "${BLUE}[STEP]${NC} $*"; echo -e "${BLUE}============================================================${NC}"; }

# --- Defaults ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$PROJECT_ROOT/models/openvla"
PID_FILE="/tmp/electronbot_openvla.pid"
LOG_FILE="/tmp/electronbot_openvla.log"

PORT=8001
ACTION=""

# --- Model Configs ---
# OpenVLA 模型 (优先级从高到低):
#   openvla-7b: 官方 7B 版本 (基于 Prismatic VLMs, 对 RTX 2060 12G 较勉强)
#   openvla-v01-7b: 官方 v0.1 版本
OPENVLA_MODELS=(
    "openvla/openvla-7b"
    "openvla/openvla-7b-finetuned-libero-spatial"
)
DEFAULT_MODEL="${OPENVLA_MODELS[0]}"
MODEL_ID="$DEFAULT_MODEL"

# --- Parse Args ---
while [[ $# -gt 0 ]]; do
    case $1 in
        install|download|serve|test|all|status|stop)
            ACTION="$1"
            shift
            ;;
        --model)
            MODEL_ID="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 <action> [--model MODEL_ID] [--port PORT]"
            echo ""
            echo "Actions:"
            echo "  install   pip install openvla"
            echo "  download  下载模型权重"
            echo "  serve     启动推理服务"
            echo "  test      测试推理"
            echo "  status    查看状态"
            echo "  stop      停止服务"
            echo "  all       一键 install + download + serve + test"
            echo ""
            echo "Options:"
            echo "  --model   模型ID (默认: openvla/openvla-7b)"
            echo "  --port    API端口 (默认: 8001, 避免与 Qwen2-VL 冲突)"
            exit 0
            ;;
        *)
            log_error "未知参数: $1"
            exit 1
            ;;
    esac
done

# --- Helper ---
check_gpu() {
    if ! command -v nvidia-smi &>/dev/null; then
        log_error "OpenVLA 需要 NVIDIA GPU"
        exit 1
    fi
    local vram_mb
    vram_mb=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 || echo "0")
    local vram_gb=$((vram_mb / 1024))
    log_info "显存: ${vram_gb}GB"

    if [ "$vram_gb" -lt 10 ]; then
        log_warn "显存 ≤10GB, OpenVLA-7B 可能 OOM。建议:"
        log_warn "  1. 使用 4-bit 量化加载 (修改 serve 脚本)"
        log_warn "  2. 或者只用 Qwen2-VL 7B AWQ (已量化, ~7GB)"
    fi
}

check_venv() {
    if [ -z "${VIRTUAL_ENV:-}" ]; then
        if [ -f "$PROJECT_ROOT/.venv/bin/activate" ]; then
            log_warn "未激活虚拟环境，自动激活..."
            source "$PROJECT_ROOT/.venv/bin/activate"
            log_info "Virtual env 已激活: $VIRTUAL_ENV"
        else
            log_error "未找到虚拟环境。请先运行: bash setup_env.sh --gpu --full"
            exit 1
        fi
    fi
}

# ============================================================
# Action: install
# ============================================================
do_install() {
    log_step "安装 OpenVLA"

    check_venv
    check_gpu

    # Check if already installed
    if python -c "import prismatic" 2>/dev/null; then
        log_info "OpenVLA (prismatic) 已安装"
        python -c "import prismatic; print(f'  prismatic: {prismatic.__file__}')" 2>/dev/null || true
        return 0
    fi

    log_info "从 GitHub 安装 openvla..."
    pip install git+https://github.com/openvla/openvla.git 2>&1 | tail -5 || {
        log_error "OpenVLA 安装失败 (可能需要代理或镜像)"
        log_info "替代方案:"
        log_info "  1. git clone https://github.com/openvla/openvla.git /tmp/openvla"
        log_info "  2. cd /tmp/openvla && pip install -e ."
        log_info "  3. 或跳过 OpenVLA, 使用 Qwen2-VL (bash scripts/deploy_qwen_vl.sh)"
        return 1
    }

    # Verify
    if python -c "import prismatic" 2>/dev/null; then
        log_info "OpenVLA 安装成功"
    else
        log_error "OpenVLA 安装验证失败: 无法 import prismatic"
        log_info "OpenVLA 依赖较重 (~2-3GB), 如果网络/磁盘有问题, 可跳过此步骤仅用 Qwen2-VL。"
        return 1
    fi
}

# ============================================================
# Action: download
# ============================================================
do_download() {
    log_step "下载 OpenVLA 模型: $MODEL_ID"

    check_venv

    local model_safe
    model_safe=$(echo "$MODEL_ID" | tr '/' '_')
    local local_dir="$MODELS_DIR/$model_safe"

    # Already exists?
    if [ -d "$local_dir" ] && [ -f "$local_dir/config.json" ]; then
        local size
        size=$(du -sh "$local_dir" 2>/dev/null | cut -f1)
        log_info "模型已存在: $local_dir ($size)"
        return 0
    fi

    # Check disk (OpenVLA-7B ~15GB)
    local free_gb
    free_gb=$(df -BG "$PROJECT_ROOT" 2>/dev/null | tail -1 | awk '{print $4}' | sed 's/G//')
    if [ "${free_gb:-0}" -lt 30 ]; then
        log_warn "磁盘可用: ${free_gb}GB, OpenVLA 需要 ~15GB + 缓存"
    fi

    mkdir -p "$local_dir"

    # Use HF download
    local hf_token="${HF_TOKEN:-${HUGGINGFACE_HUB_TOKEN:-}}"
    local hf_extra_args=""
    if [ -n "$hf_token" ]; then
        hf_extra_args="--token $hf_token"
    fi

    if ! command -v huggingface-cli &>/dev/null; then
        pip install huggingface_hub[cli] -q
    fi

    log_info "下载中... (约 15GB, 取决于网络)"
    huggingface-cli download "$MODEL_ID" \
        --local-dir "$local_dir" \
        --local-dir-use-symlinks False \
        --resume-download \
        $hf_extra_args 2>&1 | tail -5

    # Verify
    if [ -f "$local_dir/config.json" ]; then
        local size
        size=$(du -sh "$local_dir" 2>/dev/null | cut -f1)
        log_info "模型下载完成: $local_dir ($size)"
    else
        log_error "下载验证失败。请手动:"
        log_error "  git lfs clone https://huggingface.co/$MODEL_ID $local_dir"
        return 1
    fi
}

# ============================================================
# Action: serve
# - 使用 transformers 直接加载 + 自定义 Flask API
# - 因为原始 OpenVLA 是 Python 库而非 OpenAI 兼容 API
# ============================================================
do_serve() {
    log_step "启动 OpenVLA 推理服务"

    check_venv
    check_gpu

    local model_safe
    model_safe=$(echo "$MODEL_ID" | tr '/' '_')
    local local_dir="$MODELS_DIR/$model_safe"

    # Find model path (downloaded or default HF id)
    local model_path
    if [ -d "$local_dir" ] && [ -f "$local_dir/config.json" ]; then
        model_path="$local_dir"
    else
        log_warn "本地模型未下载, 使用 HuggingFace ID: $MODEL_ID"
        model_path="$MODEL_ID"
    fi

    # Check port
    if lsof -i ":$PORT" &>/dev/null 2>&1; then
        log_error "端口 $PORT 已被占用"
        log_info "使用 --port 指定其他端口, 或先 stop"
        return 1
    fi

    log_info "模型: $model_path"
    log_info "端口: $PORT"

    # 写 Python 内联推理服务脚本
    local tmp_server
    tmp_server=$(mktemp /tmp/electronbot_openvla_server_XXXXXX.py)
    cat > "$tmp_server" << 'PYEOF'
import sys, os, json, base64, logging
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import io
from PIL import Image

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger(__name__)

MODEL_PATH = os.environ.get("OPENVLA_MODEL_PATH", "openvla/openvla-7b")
PORT = int(os.environ.get("OPENVLA_PORT", "8001"))

# Lazy load model
model = None
processor = None

def load_model():
    global model, processor
    import torch
    from transformers import AutoModelForVision2Seq, AutoProcessor

    log.info(f"Loading model: {MODEL_PATH}")
    processor = AutoProcessor.from_pretrained(MODEL_PATH, trust_remote_code=True)

    model = AutoModelForVision2Seq.from_pretrained(
        MODEL_PATH,
        torch_dtype=torch.bfloat16,
        device_map="auto",
        trust_remote_code=True,
        low_cpu_mem_usage=True,
    )
    model.eval()
    log.info("Model loaded OK")

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path != "/predict":
            self.send_response(404)
            self.end_headers()
            return

        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        data = json.loads(body)

        try:
            # Decode image (base64 → PIL)
            img_b64 = data.get("image", "")
            if img_b64.startswith("data:"):
                img_b64 = img_b64.split(",", 1)[1]
            img_bytes = base64.b64decode(img_b64)
            image = Image.open(io.BytesIO(img_bytes)).convert("RGB")
            image = image.resize((224, 224))

            # Prompt
            prompt = data.get("prompt", "")
            lang_prompt = f"In: What should the robot do to {prompt}?\nOut:"

            # Build inputs
            inputs = processor(prompt, image).to(model.device)

            # Generate
            with torch.no_grad():
                outputs = model.generate(**inputs, max_new_tokens=128, do_sample=False)

            raw_output = processor.decode(outputs[0], skip_special_tokens=True)

            # Parse 7 DoF actions (OpenVLA default)
            # 实际 OpenVLA 输出格式取决于训练数据, 此处做通用解析
            angles = self._parse_angles(raw_output)

            resp = {
                "success": True,
                "raw_output": raw_output,
                "joint_angles_deg": angles.tolist() if isinstance(angles, np.ndarray) else angles,
                "joint_angles_rad": (np.deg2rad(angles).tolist() if isinstance(angles, np.ndarray) else []),
            }

        except Exception as e:
            resp = {"success": False, "error": str(e)}
            log.error(f"Inference error: {e}")

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(resp, ensure_ascii=False).encode())

    @staticmethod
    def _parse_angles(text: str):
        """从文本中提取关节角度"""
        import re
        nums = re.findall(r'-?\d+\.?\d*', text)
        if len(nums) >= 6:
            return np.array([float(n) for n in nums[:6]])
        # 返回零位
        return np.zeros(6)

    def log_message(self, format, *args):
        log.info(f"{self.client_address[0]} - {format % args}")

if __name__ == "__main__":
    os.environ["OPENVLA_MODEL_PATH"] = MODEL_PATH
    os.environ["OPENVLA_PORT"] = str(PORT)

    load_model()

    server = HTTPServer(("0.0.0.0", PORT), Handler)
    log.info(f"OpenVLA serving on http://0.0.0.0:{PORT}")
    log.info(f"  Health: http://localhost:{PORT}/health")
    log.info(f"  Predict: POST http://localhost:{PORT}/predict")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Server stopped")
        server.shutdown()
PYEOF

    nohup python "$tmp_server" >> "$LOG_FILE" 2>&1 &
    local pid=$!
    echo "$pid" > "$PID_FILE"

    log_info "服务启动中 (PID $pid)..."
    log_info "等待模型加载 (OpenVLA-7B 约 60-120 秒)..."

    # Wait for ready
    local max_wait=180
    local waited=0
    while [ $waited -lt $max_wait ]; do
        if curl -s "http://localhost:$PORT/health" 2>/dev/null | grep -q "ok"; then
            log_info "服务就绪 (耗时 ${waited}s)!"
            return 0
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            log_error "服务异常退出! tail -30 $LOG_FILE"
            tail -30 "$LOG_FILE"
            exit 1
        fi
        sleep 5
        waited=$((waited + 5))
        if [ $((waited % 30)) -eq 0 ]; then
            log_info "  等待中... (${waited}s / ${max_wait}s)"
        fi
    done

    log_error "服务启动超时"
    kill "$pid" 2>/dev/null || true
    exit 1
}

# ============================================================
# Action: test
# ============================================================
do_test() {
    log_step "测试 OpenVLA 推理"

    if ! curl -s "http://localhost:$PORT/health" 2>/dev/null | grep -q "ok"; then
        log_error "服务未运行 (端口 $PORT)"
        log_info "先启动: $0 serve"
        return 1
    fi

    check_venv

    python -c "
import base64, json, requests, sys
import numpy as np

# Test image
img = np.random.randint(0, 255, (240, 240, 3), dtype=np.uint8)
import cv2
_, buf = cv2.imencode('.jpg', img)
img_b64 = base64.b64encode(buf).decode()

payload = {
    'image': f'data:image/jpeg;base64,{img_b64}',
    'prompt': 'wave the hand to say hello',
}

print(f'[INFO] 发送请求 (prompt={payload[\"prompt\"]})...')
resp = requests.post(
    'http://localhost:$PORT/predict',
    json=payload,
    timeout=120,
)

if resp.status_code == 200:
    data = resp.json()
    if data.get('success'):
        print(f'[SUCCESS] 推理完成!')
        print(f'  raw: {data.get(\"raw_output\", \"\")[:200]}')
        print(f'  angles_deg: {data.get(\"joint_angles_deg\")}')
        print(f'  angles_rad: {data.get(\"joint_angles_rad\")}')
    else:
        print(f'[ERROR] 推理返回错误: {data.get(\"error\")}')
        sys.exit(1)
else:
    print(f'[ERROR] HTTP {resp.status_code}: {resp.text[:300]}')
    sys.exit(1)
" && log_info "测试通过" || log_error "测试失败"
}

# ============================================================
# Action: status
# ============================================================
do_status() {
    log_step "OpenVLA 服务状态"

    if [ -f "$PID_FILE" ]; then
        local pid
        pid=$(cat "$PID_FILE" 2>/dev/null || echo "")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo -e "  状态: ${GREEN}运行中${NC}"
            echo "  PID:  $pid"
            echo "  端口: $PORT"
        else
            echo -e "  状态: ${YELLOW}PID 文件残留${NC}"
        fi
    else
        echo -e "  状态: ${RED}未运行${NC}"
    fi

    if do_health_check_silent; then
        echo -e "  健康检查: ${GREEN}OK${NC}"
    else
        echo -e "  健康检查: ${RED}FAIL${NC}"
    fi

    echo "  日志: tail -f $LOG_FILE"
}

do_health_check_silent() {
    curl -s "http://localhost:$PORT/health" 2>/dev/null | grep -q "ok"
}

# ============================================================
# Action: stop
# ============================================================
do_stop() {
    log_step "停止 OpenVLA 服务"

    if [ -f "$PID_FILE" ]; then
        local pid
        pid=$(cat "$PID_FILE" 2>/dev/null || echo "")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            sleep 3
            kill -9 "$pid" 2>/dev/null || true
            log_info "已停止 (PID $pid)"
        fi
        rm -f "$PID_FILE"
    fi

    fuser -k "$PORT/tcp" 2>/dev/null || true
    log_info "停止操作完成"
}

# ============================================================
# Action: all
# ============================================================
do_all() {
    log_step "一键部署 OpenVLA (install → download → serve → test)"

    do_install || {
        log_warn "安装未成功, 后续步骤跳过"
        return 1
    }
    echo ""
    do_download || log_warn "下载跳过"
    echo ""
    do_serve
    echo ""
    sleep 10
    do_test

    log_info "OpenVLA 部署完成!"
}

# ============================================================
# Main
# ============================================================
case "$ACTION" in
    install)   do_install ;;
    download)  do_download ;;
    serve)     do_serve ;;
    test)      do_test ;;
    status)    do_status ;;
    stop)      do_stop ;;
    all)       do_all ;;
    *)
        log_error "未知操作: ${ACTION:-未指定}"
        echo "Usage: $0 <action> [--model MODEL_ID] [--port PORT]"
        echo "Actions: install | download | serve | test | status | stop | all"
        exit 1
        ;;
esac
