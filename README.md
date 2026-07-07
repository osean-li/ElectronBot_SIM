<p align="center">
  <h1 align="center">ElectronBot-SIM</h1>
  <p align="center">
    <strong>ElectronBot 桌面双臂机器人 — MuJoCo 物理仿真与 AI 训练平台</strong>
  </p>
  <p align="center">
    <a href="#-特性"><img src="https://img.shields.io/badge/version-0.2.0-blue" alt="Version"></a>
    <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-green" alt="License"></a>
    <a href="https://www.python.org/downloads/"><img src="https://img.shields.io/badge/python-3.10+-blue" alt="Python"></a>
    <a href="https://mujoco.org/"><img src="https://img.shields.io/badge/MuJoCo-3.0+-orange" alt="MuJoCo"></a>
  </p>
</p>

---

## 📖 项目简介

**ElectronBot-SIM** 是为 [ElectronBot](https://github.com/peng-zhihui/ElectronBot)（"小智"机器人，基于 ESP32 的 6 自由度桌面机器人）打造的完整仿真与 AI 训练平台。

项目提供从 **CAD 模型 → MuJoCo 物理仿真 → AI 训练 → 真机部署** 的全链路工具链，通过 **MCP 协议桥接** 实现仿真与真机的 1:1 对齐。

### 四层架构

```
┌────────────────────────────────────────────────┐
│  Layer 5   传感器系统 (Camera / Joint / Contact)  │
│  Layer 4   动作系统 (12 个预设动作, 固件 1:1 对齐)  │
│  Layer 3   MCP Bridge (12 工具, JSON-RPC 2.0)    │
│  Layer 2   MuJoCo 仿真环境 (Gymnasium RL Env)     │
│  Layer 1   CAD/MJCF 物理模型                      │
└────────────────────────────────────────────────┘
                           │
    ┌──────────────────────┼──────────────────────┐
    ▼                      ▼                      ▼
Phase 6  AI 训练        Phase 7  Benchmark     Phase 8  Sim2Real
(IL/RL/VLA)             (标准化评估)             (真机部署)
```

---

## ✨ 特性

- **🦾 双臂 6 关节仿真** — 左右手各 3 个舵机关节，完整运动学建模
- **🔌 MCP 协议桥接** — 12 个工具与 ESP32 固件 `release v2.2.6` 完全对齐
- **🧠 三类 AI 训练** — 模仿学习 (BC/ACT)、强化学习 (PPO)、VLA 大模型规划
- **🎯 7 个标准任务** — Reach / Push / PickPlace / Stack / Follow / Gesture / VoiceCmd
- **📊 标准化 Benchmark** — 成功率矩阵、完成时间、轨迹平滑度、Markdown/HTML 报告
- **🌐 仿真/真机无缝切换** — 统一 Backend，改一个 `mode` 参数即可
- **🔀 域随机化** — 7 维物理参数随机化 (摩擦/增益/质量/死区/电压/延迟/噪声)
- **🖥️ 64 并行环境** — SubprocVecEnv 高效并行训练
- **📸 多模态传感器** — RGB + Depth + Segmentation + 关节编码器 + 接触力

---

## 🚀 快速开始

### 环境要求

- Python 3.10+
- [MuJoCo](https://mujoco.org/) 3.0+

### 安装

```bash
# 克隆仓库
git clone https://github.com/electronbot/electronbot-sim.git
cd electronbot-sim

# 基础安装 (仿真核心)
pip install -e .

# 全量安装 (含 AI 训练 + Sim2Real)
pip install -e ".[full]"

# 按需安装
pip install -e ".[ai]"       # AI 训练管线
pip install -e ".[vla]"      # VLA 大模型规划
pip install -e ".[sensors]"  # 传感器系统
pip install -e ".[dev]"      # 开发工具
```

### 一分钟上手

```python
import gymnasium
import electronbot_sim  # 注册 ElectronBot-v0 环境

# 创建仿真环境
env = gymnasium.make("ElectronBot-v0", observation_mode="full")

obs, info = env.reset()
for _ in range(500):
    action = env.action_space.sample()  # 随机动作
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

### 运行 Demo

```bash
# Demo 1: 手动控制 (MuJoCo Viewer)
python demos/01-CAD-to-MJCF_Demo/01_manual_control.py

# Demo 2: 自动播放 12 个预设动作
python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py --interactive

# 键盘交互控制
python src/electronbot_sim/interactive.py
```

---

## 📁 项目结构

```
ElectronBot_SIM/
├── src/
│   ├── electronbot_sim/         # 仿真核心
│   │   ├── env.py               # Gymnasium RL 环境 (核心)
│   │   ├── backend.py           # 统一 Backend (sim/cloud)
│   │   ├── mcp_bridge.py        # MCP JSON-RPC 桥接器 (12 工具)
│   │   ├── mcp_server.py        # WebSocket 调试服务器
│   │   ├── interactive.py       # 键盘交互控制
│   │   ├── observation.py       # 观测构建器 (full/realistic)
│   │   ├── actions/             # 12 个预设动作
│   │   └── sensors/             # 传感器系统
│   │
│   ├── electronbot_ai/          # AI 训练管线
│   │   ├── tasks/               # 7 个标准任务
│   │   ├── il/                  # 模仿学习 (BC + ACT)
│   │   ├── rl/                  # 强化学习 (PPO + 域随机化)
│   │   └── vla/                 # VLA 规划器
│   │
│   ├── electronbot_benchmark/   # Benchmark 评估系统
│   │   ├── suite.py             # 核心评估引擎
│   │   ├── run.py               # CLI 运行入口
│   │   └── report.py            # Markdown/HTML 报告
│   │
│   └── electronbot_sim2real/    # Sim2Real 真机部署
│       ├── deploy_cloud.py      # 云端 API 透传
│       ├── deploy_onnx.py       # ONNX 推理部署
│       ├── calibrate.py         # 舵机校准
│       └── capability_downgrade.py  # 能力降级
│
├── tests/                       # pytest 测试套件
├── scripts/                     # 工具脚本
├── demos/                       # 演示与教程
├── assets/                      # 模型资源
│   ├── mjcf/                    # MuJoCo XML 模型
│   ├── cad/                     # CAD 图纸
│   └── meshes/                  # 网格文件
└── docs/                        # 详细文档
```

---

## 🎮 预设动作

12 个预设动作与 ESP32 固件 [movements.cc](https://github.com/peng-zhihui/ElectronBot) 1:1 对齐：

| 动作 | 函数 | 说明 |
|------|------|------|
| 抬手 | `hand_raise` | 右手缓慢抬起 |
| 放下 | `hand_lower` | 右手缓慢放下 |
| 挥手 | `hand_wave` | 右手正弦波摆动 |
| 拍手 | `hand_flap` | 右手快速拍击 |
| 左转 | `body_turn_left` | 身体向左旋转 |
| 右转 | `body_turn_right` | 身体向右旋转 |
| 回中 | `body_center` | 身体回到中位 |
| 抬头 | `head_look_up` | 头部向上看 |
| 低头 | `head_look_down` | 头部向下看 |
| 点头 | `head_nod` | 头部连续点头 |
| 头回中 | `head_center` | 头部回到中位 |
| —— | `head_continuous_nod` | 头部持续点头 (振荡器) |

---

## 🧪 AI 训练

### 7 个训练任务

| 任务 | 难度 | 类型 | 描述 |
|------|------|------|------|
| **EB-Reach** | ★☆☆☆☆ | 仿真 + 真机 | 控制末端触碰目标点 |
| **EB-Push** | ★★☆☆☆ | 仿真 + 真机 | 推动物体到目标位置 |
| **EB-PickPlace** | ★★★★☆ | 仅仿真 | 抓取并放置物体 |
| **EB-Stack** | ★★★★★ | 仅仿真 | 叠方块 (最高难度) |
| **EB-Follow** | ★★★☆☆ | 仿真 + 真机 | 追踪移动物体 |
| **EB-Gesture** | ★★☆☆☆ | 仿真 + 真机 | 手势模仿 |
| **EB-VoiceCmd** | —— | VLA 专属 | 语音指令理解 |

### 三条训练路径

```bash
# 1. 模仿学习 — Behavior Cloning
python src/electronbot_ai/il/collect_demos.py --task reach --episodes 50
python src/electronbot_ai/il/train_bc.py --task reach --epochs 100

# 2. 强化学习 — PPO
python src/electronbot_ai/rl/train_ppo.py --task push --timesteps 1_000_000

# 3. VLA 大模型规划
python src/electronbot_ai/vla/llm_planner.py --mode text --task "wave hand"
```

---

## 📊 Benchmark 评估

```bash
# 运行完整评估
python src/electronbot_benchmark/run.py --tasks all --episodes 10

# 评估特定任务
python src/electronbot_benchmark/run.py --tasks reach,push,pick_place

# 生成报告
python src/electronbot_benchmark/report.py --results results/benchmark.json
```

---

## 🔧 开发

```bash
# 安装开发依赖
pip install -e ".[dev]"

# 运行测试
pytest tests/ -v

# 代码检查
ruff check src/

# 运行 Benchmark
python scripts/benchmark.py
```

---

## 📚 文档

详细文档请见 [`docs/`](docs/) 目录：

- [开发指南](docs/dev_guide.md)
- [Phase 设计说明书](docs/tasks/)
- [视频教程](docs/bilibili/)

---

## 🏗️ 技术栈

| 领域 | 技术 |
|------|------|
| 物理引擎 | [MuJoCo 3.0+](https://mujoco.org/) |
| RL 接口 | [Gymnasium](https://gymnasium.farama.org/) |
| 深度学习 | [PyTorch](https://pytorch.org/), [Stable-Baselines3](https://stable-baselines3.readthedocs.io/) |
| 协议 | JSON-RPC 2.0, MCP, WebSocket |
| 测试 | pytest, pytest-asyncio, pytest-cov |
| 代码质量 | ruff |

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！请确保：

1. 新功能包含测试用例
2. 代码通过 `ruff check` 检查
3. 遵循项目中已有的代码规范

---

## 📄 开源协议

本项目继承原项目的 [MIT License](LICENSE)，允许任何人免费使用或用于商业用途。

---

## ⭐ Star History

如果这个项目对你有帮助，请给个 Star 支持一下！

[![Star History Chart](https://api.star-history.com/svg?repos=electronbot/electronbot-sim&type=Date)](https://star-history.com/#electronbot/electronbot-sim&Date)

---

## 🙏 致谢

- [ElectronBot](https://github.com/peng-zhihui/ElectronBot) — 稚晖君的桌面机器人项目
- [MuJoCo](https://mujoco.org/) — DeepMind 的物理仿真引擎
- [Gymnasium](https://gymnasium.farama.org/) — RL 环境标准接口
