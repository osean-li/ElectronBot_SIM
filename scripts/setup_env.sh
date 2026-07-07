#!/usr/bin/env bash
# ============================================================
# ElectronBot_SIM 环境部署脚本 v3.0
# 适配新目录结构 (src/electronbot_*)
# 支持 Ubuntu 22.04 / 24.04
# RTX 2060 12GB + CUDA 13.2
# ============================================================
# Usage:
#   bash setup_env.sh                  # Core (Phase 1-5: MuJoCo, Gym, MCP)
#   bash setup_env.sh --gpu            # Core + CUDA PyTorch
#   bash setup_env.sh --ai             # Core + Phase 6 AI 训练管线
#   bash setup_env.sh --deploy         # Core + Phase 8 Sim2Real 部署
#   bash setup_env.sh --full           # Everything (Phase 1-8 all deps)
#   bash setup_env.sh --dev            # Core + 开发工具 (black/pytest)
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

# --- Parse Args ---
GPU_MODE=false
AI_MODE=false
DEPLOY_MODE=false
FULL_MODE=false
DEV_MODE=false
SKIP_ROSDEP=false

for arg in "$@"; do
    case $arg in
        --gpu)       GPU_MODE=true ;;
        --ai)        AI_MODE=true ;;
        --deploy)    DEPLOY_MODE=true ;;
        --full)      FULL_MODE=true; AI_MODE=true; DEPLOY_MODE=true; DEV_MODE=true ;;
        --dev)       DEV_MODE=true ;;
        --skip-ros)  SKIP_ROSDEP=true ;;
        --help)
            echo "Usage: $0 [--gpu] [--ai] [--deploy] [--full] [--dev] [--skip-ros]"
            echo ""
            echo "  --gpu        Install CUDA-enabled PyTorch (Phase 6 RL/IL)"
            echo "  --ai         Install AI 训练管线 (Phase 6: SB3, PPO, VLA, IL)"
            echo "  --deploy     Install Sim2Real 部署依赖 (Phase 8: httpx, onnxruntime)"
            echo "  --full       Everything (Phase 1-8 all deps + dev tools)"
            echo "  --dev        Install dev tools (black, isort, pytest)"
            echo "  --skip-ros   Skip ROS2 apt installs"
            echo ""
            echo "Examples:"
            echo "  bash setup_env.sh                  # 仿真核心 (Phase 1-5)"
            echo "  bash setup_env.sh --gpu --ai       # 仿真 + RL/IL 训练"
            echo "  bash setup_env.sh --gpu --full     # 全部依赖"
            exit 0 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ============================================================
# System Detection
# ============================================================
log_step "1/7 Detecting System Environment"

OS_NAME=$(lsb_release -is 2>/dev/null || echo "Unknown")
OS_CODENAME=$(lsb_release -cs 2>/dev/null || echo "unknown")
OS_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
ARCH=$(uname -m)
PYTHON_VERSION=$(python3 --version 2>/dev/null || echo "NOT FOUND")
PYTHON_MINOR=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null || echo "0.0")

echo "  OS:       ${OS_NAME} ${OS_CODENAME} (${OS_VERSION})"
echo "  Arch:     ${ARCH}"
echo "  Python:   ${PYTHON_VERSION}"
echo "  Project:  ${SCRIPT_DIR}"

# Python version check
PYTHON_MAJOR_NUM=$(echo "$PYTHON_MINOR" | cut -d. -f1)
PYTHON_MINOR_NUM=$(echo "$PYTHON_MINOR" | cut -d. -f2)
if [ "$PYTHON_MAJOR_NUM" -lt 3 ] || { [ "$PYTHON_MAJOR_NUM" -eq 3 ] && [ "$PYTHON_MINOR_NUM" -lt 10 ]; }; then
    log_error "Python >= 3.10 是必需的。当前: ${PYTHON_VERSION}"
    log_info "  安装方法: sudo apt install python3.10 python3.10-venv"
    exit 1
fi
log_info "Python 版本检查: OK (>= 3.10)"

if [[ "$OS_NAME" != "Ubuntu" ]]; then
    log_warn "此脚本优化用于 Ubuntu 22.04/24.04。当前系统: ${OS_NAME}"
fi

if [[ "$ARCH" != "x86_64" ]]; then
    log_warn "非 x86_64 架构: ${ARCH}。部分包可能不可用。"
fi

# ============================================================
# CUDA / GPU Detection
# ============================================================
log_step "2/7 Checking CUDA / GPU"

resolve_torch_cuda_index() {
    local cuda_ver="$1"
    local major minor
    major=$(echo "$cuda_ver" | cut -d. -f1)
    minor=$(echo "$cuda_ver" | cut -d. -f2)

    # PyTorch 2.5+ indices: cpu, cu118, cu121, cu124, cu126, cu128
    if   [ "$major" -ge 13 ]; then echo "cu128"
    elif [ "$major" -eq 12 ] && [ "$minor" -ge 6 ]; then echo "cu126"
    elif [ "$major" -eq 12 ] && [ "$minor" -ge 4 ]; then echo "cu124"
    elif [ "$major" -eq 12 ]; then                    echo "cu121"
    elif [ "$major" -ge 11 ]; then                    echo "cu118"
    else
        echo ""
    fi
}

TORCH_INDEX="https://download.pytorch.org/whl/cpu"
CUDA_INDEX=""
CUDA_VER=""

if $GPU_MODE || $AI_MODE || $FULL_MODE; then
    if command -v nvidia-smi &> /dev/null; then
        CUDA_DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo "")
        CUDA_VER=$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[\d.]+' || echo "")

        log_info "NVIDIA Driver: ${CUDA_DRIVER_VER:-unknown}"
        log_info "CUDA Version:  ${CUDA_VER:-unknown}"

        GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")
        GPU_MEM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")
        log_info "GPU: ${GPU_NAME} (${GPU_MEM})"

        if [ -n "$CUDA_VER" ]; then
            CUDA_INDEX=$(resolve_torch_cuda_index "$CUDA_VER")
            if [ -z "$CUDA_INDEX" ]; then
                log_error "CUDA ${CUDA_VER} 太旧，PyTorch 不支持。降级为 CPU 模式。"
                GPU_MODE=false
                TORCH_INDEX="https://download.pytorch.org/whl/cpu"
            else
                TORCH_INDEX="https://download.pytorch.org/whl/${CUDA_INDEX}"
                log_info "CUDA ${CUDA_VER} → PyTorch index: ${CUDA_INDEX}"
            fi
        else
            log_error "无法检测 CUDA 版本。降级为 CPU 模式。"
            GPU_MODE=false
        fi
    else
        log_error "nvidia-smi 未找到。降级为 CPU 模式。"
        GPU_MODE=false
    fi
else
    log_info "CPU 模式 (使用 --gpu 或 --ai 启用 CUDA)"
fi

# ============================================================
# ROS2 Detection
# ============================================================
log_step "3/7 Checking ROS2"

ROS2_SOURCE=""
ROS2_DISTRO=""
if [ -f /opt/ros/humble/setup.bash ]; then
    ROS2_SOURCE="/opt/ros/humble/setup.bash"
    ROS2_DISTRO="humble"
    log_info "ROS2 Humble 已安装"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS2_SOURCE="/opt/ros/jazzy/setup.bash"
    ROS2_DISTRO="jazzy"
    log_info "ROS2 Jazzy 已安装"
else
    log_warn "未检测到 ROS2 安装 (可选, 不影响仿真核心)"
    log_info "  安装指南: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
fi

# ============================================================
# Step 4: System (apt) Dependencies
# ============================================================
log_step "4/7 System Dependencies (apt)"

# Resolve dpkg lock
resolve_dpkg_lock() {
    local lock_file="/var/lib/dpkg/lock-frontend"
    local max_wait=30
    local waited=0
    while sudo fuser "$lock_file" >/dev/null 2>&1; do
        local pid
        pid=$(sudo fuser "$lock_file" 2>/dev/null | tr -d ' ')
        if [ $waited -ge $max_wait ]; then
            log_warn "dpkg lock held by PID ${pid} > ${max_wait}s. 尝试强制释放..."
            sudo kill "$pid" 2>/dev/null || true
            sleep 2
            if sudo fuser "$lock_file" >/dev/null 2>&1; then
                sudo kill -9 "$pid" 2>/dev/null || true
                sleep 1
            fi
            break
        fi
        log_info "等待 dpkg lock (PID ${pid}, ${waited}s / ${max_wait}s)..."
        sleep 2
        waited=$((waited + 2))
    done
}
resolve_dpkg_lock

sudo apt update -qq -o Acquire::http::Timeout=10

# -----------------------------------------------------------
# Phase 1 (CAD→MJCF):    libassimp-dev (mesh loading), swig
# Phase 2 (MuJoCo Env):  libglfw3, libglew, libosmesa6, libegl1, libgles2
# Phase 3 (MCP Bridge):  (no extra apt deps)
# Phase 5 (Sensors):     libopencv-dev, libgl1-mesa-*
# Phase 8 (Sim2Real):    libusb-1.0 (USB CDC)
# General:               python3-pip, python3-venv, build-essential
# -----------------------------------------------------------
APT_PACKAGES=(
    # --- Python ---
    python3-pip
    python3-venv
    python3-dev

    # --- Build tools ---
    build-essential
    cmake
    wget
    curl
    git

    # --- MuJoCo rendering (Phase 2) ---
    libglfw3
    libglfw3-dev
    libglew-dev
    libegl1
    libgles2
    libosmesa6
    libgl1-mesa-glx
    libgl1-mesa-dri

    # --- Mesh processing (Phase 1: CAD→MJCF) ---
    libassimp-dev

    # --- OpenCV system libs (Phase 5: CameraSensor) ---
    libopencv-dev

    # --- Math libs ---
    libopenblas-dev
    libeigen3-dev

    # --- GLib (MuJoCo + OpenCV runtime) ---
    libglib2.0-0

    # --- SWIG (Phase 1: binding generation) ---
    swig

    # --- USB CDC (Phase 8: Sim2Real 串口) ---
    libusb-1.0-0-dev

    # --- HDF5 (Phase 6: IL 示范数据集) ---
    libhdf5-dev
)

log_info "安装 ${#APT_PACKAGES[@]} 个 apt 包..."
INSTALLED=0
SKIPPED=0
for pkg in "${APT_PACKAGES[@]}"; do
    if dpkg -l "$pkg" 2>/dev/null | grep -q '^ii'; then
        : # already installed
    else
        if sudo apt install -y -qq "$pkg" 2>/dev/null; then
            INSTALLED=$((INSTALLED + 1))
        else
            log_warn "跳过 ${pkg} (安装失败)"
            SKIPPED=$((SKIPPED + 1))
        fi
    fi
done
log_info "apt: ${INSTALLED} 新安装, ${SKIPPED} 跳过, $(( ${#APT_PACKAGES[@]} - INSTALLED - SKIPPED )) 已存在"

# ROS2 apt packages
if [ -n "$ROS2_SOURCE" ] && ! $SKIP_ROSDEP; then
    log_info "安装 ROS2 工具包..."
    ROS2_APT_PACKAGES=(
        python3-colcon-common-extensions
        "ros-${ROS2_DISTRO}-rviz2"
        "ros-${ROS2_DISTRO}-robot-state-publisher"
        "ros-${ROS2_DISTRO}-joint-state-publisher-gui"
        "ros-${ROS2_DISTRO}-cv-bridge"
        "ros-${ROS2_DISTRO}-tf2-ros"
    )
    for pkg in "${ROS2_APT_PACKAGES[@]}"; do
        sudo apt install -y -qq "$pkg" 2>/dev/null || log_warn "跳过 ${pkg}"
    done
fi

log_info "系统依赖完成"

# ============================================================
# Step 5: Python Virtual Environment
# ============================================================
log_step "5/7 Python Virtual Environment"

VENV_DIR="${SCRIPT_DIR}/.venv"

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    log_info "虚拟环境创建: ${VENV_DIR}"
else
    log_info "虚拟环境已存在: ${VENV_DIR}"
fi

source "$VENV_DIR/bin/activate"
log_info "虚拟环境已激活 (python=$(which python), v$(python --version 2>&1 | cut -d' ' -f2))"

# Pre-install pyyaml (prevents ROS2 launch-ros conflict during pip upgrade)
pip install pyyaml -q 2>/dev/null || true

pip install --upgrade pip setuptools wheel -q 2>/dev/null

pip check 2>/dev/null && log_info "无依赖冲突" || log_warn "存在依赖冲突（不影响后续安装）"

# ============================================================
# Step 6: Python Dependencies — by Phase
# ============================================================
log_step "6/7 Python Dependencies"

# -----------------------------------------------------------
# Phase 1: CAD → MJCF  (scripts/)
#   yourdfpy    — URDF/MJCF 转换
#   trimesh     — 网格简化 / 凸包分解
#   lxml        — XML 处理 (MJCF 生成)
#   numpy/scipy — 惯性矩阵计算
# -----------------------------------------------------------
log_info "[Phase 1] CAD→MJCF 依赖..."
pip install "yourdfpy>=0.0.56" "trimesh>=4.0.0" "lxml>=5.0.0" -q

# -----------------------------------------------------------
# Phase 2: MuJoCo 仿真核心  (src/electronbot_sim/env.py)
#   mujoco      — 物理引擎
#   gymnasium   — RL 环境接口
#   numpy       — 数值计算
# -----------------------------------------------------------
log_info "[Phase 2] MuJoCo 仿真核心依赖..."
pip install "mujoco>=3.2.0" "gymnasium>=0.29.0" -q

# -----------------------------------------------------------
# Phase 3: MCP Bridge  (src/electronbot_sim/mcp_bridge.py)
#   websockets  — WebSocket 服务器
# -----------------------------------------------------------
log_info "[Phase 3] MCP Bridge 依赖..."
pip install "websockets>=12.0" -q

# -----------------------------------------------------------
# Phase 4-5: 动作系统 + 传感器 (无需额外依赖, 用 Phase 1-3 已有)
#   opencv-python — CameraSensor RGB/D 图像处理
# -----------------------------------------------------------
log_info "[Phase 4-5] 动作 + 传感器依赖..."
pip install "opencv-python>=4.8.0" -q

# -----------------------------------------------------------
# 通用依赖 (跨 Phase)
# -----------------------------------------------------------
log_info "安装通用依赖..."
pip install numpy scipy matplotlib pyyaml h5py tqdm -q

# -----------------------------------------------------------
# Phase 6: AI 训练管线 (--ai)
#   stable-baselines3 — PPO/SAC RL 算法
#   torch             — IL (BC/ACT) + RL
#   einops            — ACT 模型张量操作
#   diffusers         — ACT 扩散策略
#   accelerate        — 模型加速
#   tensorboard       — 训练日志
#   py_trees          — 行为树 (AI→动作调度)
# -----------------------------------------------------------
if $AI_MODE; then
    log_info "[Phase 6] AI 训练管线依赖..."

    # PyTorch first (explicit, correct index)
    NEED_TORCH=false
    if python -c "import torch" 2>/dev/null; then
        TORCH_CUDA=$(python -c "import torch; print(torch.cuda.is_available())")
        if $GPU_MODE && [ "$TORCH_CUDA" != "True" ]; then
            log_info "当前为 CPU 版本，重装 GPU 版本..."
            NEED_TORCH=true
        else
            log_info "PyTorch 已安装: v$(python -c 'import torch; print(torch.__version__)')"
        fi
    else
        NEED_TORCH=true
    fi

    if $NEED_TORCH; then
        log_info "安装 PyTorch (index: ${TORCH_INDEX})..."
        pip install --force-reinstall torch --index-url "$TORCH_INDEX" -q
    fi

    pip install "stable-baselines3>=2.3.0" "py_trees>=2.0.0" tensorboard -q
    pip install einops -q

    if $GPU_MODE; then
        pip install diffusers accelerate -q --extra-index-url "$TORCH_INDEX"
    else
        pip install diffusers accelerate -q --extra-index-url "$TORCH_INDEX"
    fi

    # VLA 依赖 (transformers + vLLM)
    log_info "[Phase 6] VLA 依赖 (transformers)..."
    pip install "transformers>=4.40.0" peft accelerate sentencepiece -q
    pip install bitsandbytes -q 2>/dev/null || log_warn "bitsandbytes 安装失败 (CUDA 兼容性)"

    if $GPU_MODE; then
        log_info "[Phase 6] vLLM (Qwen2-VL 推理)..."
        pip install vllm -q 2>/dev/null || {
            log_warn "vLLM 安装失败。可尝试: pip install vllm --no-build-isolation"
        }
        log_info "[Phase 6] OpenVLA..."
        pip install git+https://github.com/openvla/openvla.git -q 2>/dev/null || {
            log_warn "OpenVLA 安装失败 (需 GitHub 访问)"
        }
    fi

    pip install datasets pyarrow -q  # LoRA finetuning
    log_info "[Phase 6] AI 依赖完成"
else
    log_info "跳过 AI 依赖 (使用 --ai 启用 Phase 6)"
fi

# -----------------------------------------------------------
# Phase 7: Benchmark (src/electronbot_benchmark/)
#   pandas     — 数据统计
#   tabulate   — 表格输出
# -----------------------------------------------------------
log_info "[Phase 7] Benchmark 依赖..."
pip install pandas tabulate -q

# -----------------------------------------------------------
# Phase 8: Sim2Real 部署 (--deploy)
#   httpx        — 异步 HTTP 客户端 (云端 API 透传)
#   onnxruntime  — ONNX 模型推理
# -----------------------------------------------------------
if $DEPLOY_MODE; then
    log_info "[Phase 8] Sim2Real 部署依赖..."
    pip install "httpx>=0.27.0" "onnxruntime>=1.17.0" -q
    pip install imageio[ffmpeg] -q 2>/dev/null || log_warn "imageio[ffmpeg] 跳过"
    log_info "[Phase 8] Sim2Real 依赖完成"
else
    log_info "跳过 Sim2Real 依赖 (使用 --deploy 启用 Phase 8)"
fi

# -----------------------------------------------------------
# Dev tools (--dev)
# -----------------------------------------------------------
if $DEV_MODE; then
    log_info "安装开发工具..."
    pip install black isort pytest pytest-asyncio -q
    log_info "开发工具完成"
fi

# Integrity check
echo ""
if pip check 2>/dev/null; then
    log_info "Python 依赖完整性: OK"
else
    log_warn "部分依赖冲突。运行 'pip check' 查看详情。"
fi

# ============================================================
# Step 7: Post-install Verification
# ============================================================
log_step "7/7 Verifying Installation"

check_python_pkg() {
    local pkg="$1"
    local import_name="${2:-$1}"
    python -c "import ${import_name}" 2>/dev/null && echo -e "  ${GREEN}✓${NC} ${pkg}" || echo -e "  ${RED}✗${NC} ${pkg}"
}

echo "Phase 1 (CAD→MJCF):"
check_python_pkg "yourdfpy"
check_python_pkg "trimesh"
check_python_pkg "lxml"

echo ""
echo "Phase 2 (MuJoCo 仿真):"
check_python_pkg "mujoco"
check_python_pkg "gymnasium"
check_python_pkg "numpy"
check_python_pkg "scipy"

echo ""
echo "Phase 3 (MCP Bridge):"
check_python_pkg "websockets"

echo ""
echo "Phase 4-5 (动作 + 传感器):"
check_python_pkg "opencv (cv2)" "cv2"

echo ""
echo "通用:"
check_python_pkg "h5py"
check_python_pkg "pyyaml" "yaml"
check_python_pkg "tqdm"
check_python_pkg "pandas"
check_python_pkg "tabulate"
check_python_pkg "matplotlib"

if $AI_MODE; then
    echo ""
    echo "Phase 6 (AI 训练):"
    check_python_pkg "torch"
    check_python_pkg "stable_baselines3"
    check_python_pkg "py_trees"
    check_python_pkg "einops"
    check_python_pkg "transformers"
    check_python_pkg "diffusers"
    check_python_pkg "accelerate"
    if $GPU_MODE; then
        check_python_pkg "vllm" 2>/dev/null || echo -e "  ${YELLOW}○${NC} vllm (可选)"
        check_python_pkg "peft" 2>/dev/null || echo -e "  ${YELLOW}○${NC} peft (可选)"
    fi
fi

if $DEPLOY_MODE; then
    echo ""
    echo "Phase 8 (Sim2Real 部署):"
    check_python_pkg "httpx"
    check_python_pkg "onnxruntime"
    check_python_pkg "imageio"
fi

# Project module check
echo ""
echo "项目模块:"
check_project_module() {
    local module_path="$1"
    local module_name="$2"
    if python -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}/src'); import ${module_name}" 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} ${module_name} (${module_path})"
    else
        echo -e "  ${YELLOW}○${NC} ${module_name} (src/ 待实现)"
    fi
}

check_project_module "src/electronbot_sim" "electronbot_sim"
check_project_module "src/electronbot_ai" "electronbot_ai"
check_project_module "src/electronbot_benchmark" "electronbot_benchmark"
check_project_module "src/electronbot_sim2real" "electronbot_sim2real"

# GPU status
if $GPU_MODE || $AI_MODE; then
    echo ""
    echo "GPU 状态:"
    python -c "
import torch
if torch.cuda.is_available():
    name = torch.cuda.get_device_name(0)
    mem  = torch.cuda.get_device_properties(0).total_memory / 1024**3
    cuda = torch.version.cuda
    print(f'  \033[0;32m✓\033[0m {name}')
    print(f'  \033[0;32m✓\033[0m VRAM: {mem:.1f} GB')
    print(f'  \033[0;32m✓\033[0m CUDA: {cuda} (PyTorch)')
else:
    print('  \033[0;31m✗\033[0m CUDA 不可用 (仅 CPU)')
" 2>&1 || echo -e "  ${RED}✗${NC} PyTorch 未安装"
fi

# ROS2 status
if [ -n "$ROS2_SOURCE" ]; then
    echo ""
    echo "ROS2 状态:"
    set +euo pipefail
    source "$ROS2_SOURCE" 2>/dev/null || true
    set -euo pipefail
    if command -v ros2 &> /dev/null; then
        echo -e "  ${GREEN}✓${NC} ROS2 CLI (${ROS2_DISTRO})"
        echo -e "  ${GREEN}✓${NC} colcon build"
    else
        echo -e "  ${RED}✗${NC} ROS2 CLI 不可用"
    fi
fi

# ============================================================
# Summary
# ============================================================
log_step "安装完成!"

echo ""
echo "  ┌──────────────────────────────────────────────────────────┐"
echo "  │  ElectronBot_SIM 环境信息                                 │"
echo "  ├──────────────────────────────────────────────────────────┤"
echo "  │  项目路径:   ${SCRIPT_DIR}"
echo "  │  虚拟环境:   ${VENV_DIR}"
echo "  │  GPU 模式:   ${GPU_MODE}"
echo "  │  AI 训练:    ${AI_MODE}"
echo "  │  Sim2Real:   ${DEPLOY_MODE}"
echo "  │  开发工具:   ${DEV_MODE}"
echo "  │  ROS2:       ${ROS2_DISTRO:-未安装}"
echo "  │  CUDA:       ${CUDA_VER:-N/A} → ${CUDA_INDEX:-cpu}"
echo "  └──────────────────────────────────────────────────────────┘"
echo ""
echo "  激活虚拟环境:"
echo "    source .venv/bin/activate"
echo ""

echo "  Phase 验证:"
echo "    # Phase 1: 模型验证"
echo "    python scripts/validate_model.py"
echo ""
echo "    # Phase 2: 环境测试"
echo "    python -m pytest tests/test_env.py -v"
echo ""
echo "    # Phase 3: MCP 端到端测试"
echo "    python -m pytest tests/test_websocket_e2e.py -v"
echo ""
echo "    # Phase 5: 传感器测试"
echo "    python -m pytest tests/test_sensors.py -v"
echo ""

if $AI_MODE; then
echo "  AI 训练:"
echo "    # Phase 6: RL 训练"
echo "    python -m electronbot_ai.rl.train_ppo --task reach"
echo ""
echo "    # Phase 6: IL 训练"
echo "    python -m electronbot_ai.il.train_bc --task reach"
echo ""
fi

if $DEPLOY_MODE; then
echo "  Sim2Real 部署:"
echo "    # Phase 8: 云端 API 连接测试"
echo "    python -m pytest tests/test_sim2real_cloud.py -v"
echo ""
fi

if [ -n "$ROS2_SOURCE" ]; then
echo "  ROS2 仿真启动:"
echo "    source ${ROS2_SOURCE}"
echo "    ros2 launch electronbot_mujoco_ros2 sim.launch.py"
echo ""
fi

echo "  Tips:"
echo "    setup_env.sh --gpu --ai --deploy  仿真+训练+部署"
echo "    setup_env.sh --full               全部依赖"
echo "    setup_env.sh --skip-ros           跳过 ROS2"
echo ""
