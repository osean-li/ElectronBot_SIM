#!/usr/bin/env bash
# ============================================================
# ElectronBot_SIM 环境部署脚本 v2.0
# 支持 Ubuntu 22.04 / 24.04
# RTX 2060 12GB + CUDA 13.2
# ============================================================
# Usage:
#   bash setup_env.sh                  # Core deps (MuJoCo, Gym, SB3, py_trees)
#   bash setup_env.sh --gpu            # Core + CUDA PyTorch
#   bash setup_env.sh --full           # Core + VLA/transformers/vLLM
#   bash setup_env.sh --gpu --full     # Everything
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
FULL_MODE=false
GPU_MODE=false
SKIP_ROSDEP=false

for arg in "$@"; do
    case $arg in
        --full)     FULL_MODE=true ;;
        --gpu)      GPU_MODE=true ;;
        --skip-ros) SKIP_ROSDEP=true ;;
        --help)
            echo "Usage: $0 [--gpu] [--full] [--skip-ros]"
            echo "  --gpu       Install CUDA-enabled PyTorch"
            echo "  --full      Install VLA dependencies (transformers, vLLM, OpenVLA)"
            echo "  --skip-ros  Skip ROS2 apt installs (if already installed)"
            exit 0 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ============================================================
# System Detection
# ============================================================
log_step "Detecting System Environment"

OS_NAME=$(lsb_release -is 2>/dev/null || echo "Unknown")
OS_CODENAME=$(lsb_release -cs 2>/dev/null || echo "unknown")
ARCH=$(uname -m)
PYTHON_VERSION=$(python3 --version 2>/dev/null || echo "NOT FOUND")

echo "  OS:       ${OS_NAME} ${OS_CODENAME}"
echo "  Arch:     ${ARCH}"
echo "  Python:   ${PYTHON_VERSION}"
echo "  Project:  ${SCRIPT_DIR}"

if [[ "$OS_NAME" != "Ubuntu" ]]; then
    log_warn "此脚本优化用于 Ubuntu 22.04/24.04。当前系统: ${OS_NAME}"
fi

# ============================================================
# CUDA / GPU Detection
# ============================================================
log_step "Checking CUDA / GPU"

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

if $GPU_MODE; then
    if command -v nvidia-smi &> /dev/null; then
        CUDA_DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo "")
        CUDA_VER=$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[\d.]+' || echo "")

        log_info "NVIDIA Driver: ${CUDA_DRIVER_VER:-unknown}"
        log_info "CUDA Version:  ${CUDA_VER:-unknown}"

        # Also show GPU info
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
    log_info "CPU 模式 (使用 --gpu 启用 CUDA)"
fi

# ============================================================
# ROS2 Detection
# ============================================================
log_step "Checking ROS2"

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
    log_warn "未检测到 ROS2 安装。Phase 3 之前需要安装。"
    log_info "  安装指南: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
fi

# ============================================================
# Step 1: System (apt) Dependencies
# ============================================================
log_step "Step 1/5: System Dependencies (apt)"

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
                log_warn "仍然被锁，强制 kill..."
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

APT_PACKAGES=(
    python3-pip python3-venv python3-dev
    build-essential cmake
    libopencv-dev
    libopenblas-dev
    libgl1-mesa-glx libgl1-mesa-dri
    libglib2.0-0
    libegl1 libgles2
    libosmesa6              # MuJoCo offscreen rendering
    libglfw3 libglfw3-dev   # MuJoCo viewer
    libglew-dev
    libassimp-dev            # MuJoCo mesh loading (STL/OBJ)
    wget curl git
    swig
    libeigen3-dev
    libusb-1.0-0-dev        # USB CDC (Sim2Real Phase 9)
    libhdf5-dev              # HDF5 for IL datasets
)

log_info "安装 ${#APT_PACKAGES[@]} 个 apt 包..."
for pkg in "${APT_PACKAGES[@]}"; do
    if dpkg -l "$pkg" 2>/dev/null | grep -q '^ii'; then
        :
    else
        sudo apt install -y -qq "$pkg" || log_warn "跳过 ${pkg} (安装失败)"
    fi
done

# ROS2 apt packages (if detected and not skipped)
if [ -n "$ROS2_SOURCE" ] && ! $SKIP_ROSDEP; then
    log_info "安装 ROS2 工具包..."
    for pkg in python3-colcon-common-extensions "ros-${ROS2_DISTRO}-rviz2" "ros-${ROS2_DISTRO}-robot-state-publisher" "ros-${ROS2_DISTRO}-joint-state-publisher-gui" "ros-${ROS2_DISTRO}-cv-bridge" "ros-${ROS2_DISTRO}-tf2-ros"; do
        sudo apt install -y -qq "$pkg" 2>/dev/null || log_warn "跳过 ${pkg}"
    done
fi

log_info "系统依赖安装完成。"

# ============================================================
# Step 2: Python Virtual Environment
# ============================================================
log_step "Step 2/5: Python Virtual Environment"

VENV_DIR="${SCRIPT_DIR}/.venv"

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    log_info "虚拟环境创建: ${VENV_DIR}"
else
    log_info "虚拟环境已存在: ${VENV_DIR}"
fi

# Activate
source "$VENV_DIR/bin/activate"
log_info "虚拟环境已激活 (python=$(which python))"

# Pre-install pyyaml before upgrading pip to prevent ROS2 launch-ros conflict
# (launch-ros is a system apt package that requires pyyaml; without it,
#  pip's dependency resolver prints noisy ERROR during setuptools upgrade)
pip install pyyaml -q 2>/dev/null || true

# Upgrade pip safely
pip install --upgrade pip setuptools wheel -q 2>/dev/null

# Verify no conflicts remain
pip check 2>/dev/null && log_info "无依赖冲突" || log_warn "存在依赖冲突（不影响后续安装）"

# ============================================================
# Step 3: Python Core Dependencies
# ============================================================
log_step "Step 3/5: Python Core Dependencies"

# --- PyTorch (installed first to avoid pulling wrong version) ---
log_info "检查 PyTorch..."
if python -c "import torch; print(torch.__version__)" 2>/dev/null; then
    log_info "PyTorch 已安装: $(python -c 'import torch; print(f\"v{torch.__version__} (CUDA={torch.cuda.is_available()})\")')"
    log_info "跳过 PyTorch 安装"
else
    log_info "安装 PyTorch (index: ${TORCH_INDEX})..."
    pip install torch>=2.1.0 --index-url "$TORCH_INDEX" -q
fi

if $GPU_MODE; then
    log_info "验证 CUDA 可用性..."
    if python -c "import torch; assert torch.cuda.is_available(), 'CUDA not available'" 2>/dev/null; then
        log_info "PyTorch CUDA: OK ($(python -c 'import torch; print(torch.cuda.get_device_name(0))'))"
    else
        log_error "PyTorch 报告 CUDA 不可用！检查 driver/CUDA 兼容性。"
        log_error "  Driver CUDA: ${CUDA_VER}, PyTorch index: ${CUDA_INDEX}"
    fi
fi

# --- MuJoCo (explicit install) ---
log_info "安装 MuJoCo..."
pip install mujoco>=3.1.0 -q

# --- Core RL/Simulation stack ---
log_info "安装仿真与 RL 依赖..."
pip install gymnasium>=0.29.0 stable-baselines3>=2.3.0 -q
pip install tensorboard -q

# --- Math & Data ---
log_info "安装数学与数据处理..."
pip install numpy scipy matplotlib opencv-python pyyaml -q
pip install h5py tqdm -q

# --- Behavior Tree ---
log_info "安装 behavior tree..."
pip install py_trees>=2.0.0 -q

# --- Imitation Learning ---
log_info "安装 IL 依赖 (PyTorch已安装)..."
pip install einops -q
# diffusers pulls additional torch deps, be careful
if $GPU_MODE; then
    pip install diffusers accelerate -q --extra-index-url "$TORCH_INDEX"
else
    pip install diffusers accelerate -q --extra-index-url "$TORCH_INDEX"
fi

# --- Image recording ---
log_info "安装视频录制..."
pip install imageio[ffmpeg] -q 2>/dev/null || log_warn "imageio[ffmpeg] 安装失败，跳过"

# --- Dev tools ---
log_info "安装开发工具..."
pip install black isort pytest -q

# --- Check integrity ---
if pip check 2>/dev/null; then
    log_info "核心依赖完整性: OK"
else
    log_warn "部分依赖冲突。运行 'pip check' 查看详情。"
fi

# ============================================================
# Step 4: Full/VLA Dependencies (optional)
# ============================================================
if $FULL_MODE; then
    log_step "Step 4/5: Full Dependencies (VLA/VLM/Transformers)"

    log_info "安装 transformers + PEFT + bitsandbytes..."
    pip install transformers>=4.40.0 peft accelerate sentencepiece -q
    pip install bitsandbytes -q 2>/dev/null || log_warn "bitsandbytes 安装失败 (可能不支持当前 CUDA)"

    # vLLM - only for GPU, needs > 7GB VRAM for Qwen2-VL 7B
    if $GPU_MODE; then
        log_info "安装 vLLM (用于 Qwen2-VL 7B 推理)..."
        pip install vllm -q 2>/dev/null || {
            log_warn "vLLM 安装失败。可尝试:"
            log_warn "  pip install vllm --no-build-isolation"
            log_warn "  或跳过 VLA Phase 6，先用 mock 模式开发。"
        }

        # OpenVLA
        log_info "安装 OpenVLA..."
        pip install git+https://github.com/openvla/openvla.git -q 2>/dev/null || {
            log_warn "OpenVLA 安装失败 (需要网络 + GitHub 访问)"
            log_info "  Phase 6 可先用 Qwen2-VL 模式，OpenVLA 为可选对比方案。"
        }
    else
        log_warn "VLA 依赖 (vLLM/OpenVLA) 需要 GPU。使用 --gpu 启用。"
    fi

    # LoRA finetuning
    log_info "安装 LoRA 微调依赖..."
    pip install datasets pyarrow -q

    # Plotting
    log_info "安装可视化..."
    pip install plotly -q

    # Re-check
    pip check 2>/dev/null || log_warn "完整依赖有冲突（不影响核心功能）"

    log_info "VLA/Full 依赖安装完成。"
else
    log_info "Skipping full dependencies (use --full to install VLA stack)"
fi

# ============================================================
# Step 5: Post-install Verification
# ============================================================
log_step "Step 5/5: Verifying Installation"

check_python_pkg() {
    python -c "import $1" 2>/dev/null && echo -e "  ${GREEN}✓${NC} $1" || echo -e "  ${RED}✗${NC} $1 (optional)"
}

echo "核心模块:"
check_python_pkg "torch"
check_python_pkg "numpy"
check_python_pkg "mujoco"
check_python_pkg "cv2"
check_python_pkg "gymnasium"
check_python_pkg "stable_baselines3"
check_python_pkg "py_trees"
check_python_pkg "h5py"
check_python_pkg "yaml"
check_python_pkg "einops"

if $FULL_MODE; then
    echo ""
    echo "VLA 模块:"
    check_python_pkg "transformers"
    check_python_pkg "peft"
    check_python_pkg "accelerate"
    check_python_pkg "vllm"
    check_python_pkg "datasets"
fi

echo ""
echo "ElectronBot 项目模块:"
# Check if electronbot_mujoco is importable
if python -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}/simulation/electronbot_mujoco'); \
    from electronbot_mujoco.robot import ElectronBotRobot" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} electronbot_mujoco (可导入)"
else
    echo -e "  ${RED}✗${NC} electronbot_mujoco (检查 PYTHONPATH)"
fi

if python -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}/simulation/electronbot_mujoco'); \
    from electronbot_mujoco.tasks import TASKS" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} Benchmark tasks (5 个任务可加载)"
fi

if python -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}/ai/rl'); \
    from electronbot_rl.emotional_reward import EmotionalRewardShaper" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} emotional_reward (3 种情绪模式)"
fi

if python -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}/behavior'); \
    from electronbot_behavior.behavior_tree import build_find_and_touch_tree" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} behavior_tree (py_trees 可用)"
fi

# GPU check
if $GPU_MODE; then
    echo ""
    echo "GPU 状态:"
    python -c "
import torch
if torch.cuda.is_available():
    name = torch.cuda.get_device_name(0)
    mem  = torch.cuda.get_device_properties(0).total_mem / 1024**3
    cuda = torch.version.cuda
    print(f'  ✓ {name}')
    print(f'  ✓ VRAM: {mem:.1f} GB')
    print(f'  ✓ CUDA: {cuda} (PyTorch)')
else:
    print('  ✗ CUDA 不可用')
" 2>/dev/null
fi

# ROS2 check
if [ -n "$ROS2_SOURCE" ]; then
    echo ""
    echo "ROS2 状态:"
    source "$ROS2_SOURCE" 2>/dev/null || true
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
echo "  ┌──────────────────────────────────────────────────────┐"
echo "  │  ElectronBot_SIM 环境信息                             │"
echo "  ├──────────────────────────────────────────────────────┤"
echo "  │  项目路径:   ${SCRIPT_DIR}"
echo "  │  虚拟环境:   ${VENV_DIR}"
echo "  │  GPU 模式:   $GPU_MODE"
echo "  │  Full 模式:  $FULL_MODE"
echo "  │  ROS2:       ${ROS2_DISTRO:-未安装}"
echo "  └──────────────────────────────────────────────────────┘"
echo ""
echo "  下一步:"
echo "    source .venv/bin/activate"
echo ""
echo "  验证仿真环境:"
echo "    python simulation/electronbot_mujoco/scripts/test_env.py --test all"
echo ""
echo "  开始 RL 训练:"
echo "    python ai/rl/electronbot_rl/train_ppo.py --task reach --arm right"
echo ""
echo "  行为树 Demo (需 MuJoCo):"
echo "    python behavior/electronbot_behavior/behavior_tree.py"
echo ""
if [ -n "$ROS2_SOURCE" ]; then
echo "  ROS2 仿真启动:"
echo "    source ${ROS2_SOURCE}"
echo "    ros2 launch electronbot_mujoco_ros2 sim.launch.py"
fi
echo ""
echo "  Tips:"
echo "    setup_env.sh --gpu --full    重装完整依赖"
echo "    setup_env.sh --skip-ros      跳过 ROS2 apt 安装"
echo ""