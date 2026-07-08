# ElectronBot_SIM 开发指南

> 面向 Ubuntu 22.04 / 24.04，RTX 2060 12GB + CUDA 13.2 环境
>
> **硬件文档**：关于 ElectronBot 真机硬件（ESP32-S3、舵机、PCB、焊接），参见 [概要设计 - 真机对接](概要设计/ElectronBot_SIM-概要设计文档.md#8-真机对接) 和 [原版 vs 小智版差异分析](概要设计/ElectronBot-原版vs小智版-差异分析.md)。

---

## 1. 环境部署

### 1.1 前置条件

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 / 24.04 |
| Python | >= 3.11 |
| 架构 | x86_64 |
| GPU (可选) | NVIDIA GPU + 驱动 + CUDA |

### 1.2 setup_env.sh 使用方式

脚本路径: `ElectronBot_SIM/setup_env.sh`

```bash
# 核心依赖 (Phase 1-5: MuJoCo, Gym, MCP Bridge, 传感器)
bash setup_env.sh

# 核心 + CUDA PyTorch
bash setup_env.sh --gpu

# 核心 + AI 训练管线 (Phase 6: SB3, PPO, VLA, IL)
bash setup_env.sh --gpu --ai

# 核心 + Sim2Real 部署 (Phase 8: httpx, onnxruntime)
bash setup_env.sh --deploy

# 全部依赖 (Phase 1-8 + 开发工具)
bash setup_env.sh --full

# 仅开发工具 (black, isort, pytest)
bash setup_env.sh --dev

# 跳过 ROS2 apt 安装 (如已安装)
bash setup_env.sh --skip-ros
```

#### 参数说明

| 参数 | 说明 |
|------|------|
| `--gpu` | 启用 CUDA PyTorch (自动检测 GPU/CUDA 版本) |
| `--ai` | 安装 Phase 6 AI 依赖 (SB3, transformers, VLA, vLLM) |
| `--deploy` | 安装 Phase 8 Sim2Real 依赖 (httpx, onnxruntime) |
| `--full` | 全部依赖 (等价于 --gpu + --ai + --deploy + --dev) |
| `--dev` | 开发工具 (black, isort, pytest, pytest-asyncio) |
| `--skip-ros` | 跳过 ROS2 apt 包安装 |

### 1.3 虚拟环境

脚本自动创建 `.venv/` 虚拟环境，激活方式:

```bash
source .venv/bin/activate
```