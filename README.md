# ElectronBot_SIM

基于 [ElectronBot](https://github.com/peng-zhihui/ElectronBot) (稚晖君开源桌面双臂机器人) 的全栈 AI 机器人仿真与学习平台。

## 三层架构

```
┌──────────┐    ┌───────────────┐    ┌────────────────────┐
│  感知层   │───▶│    决策层      │───▶│      执行层         │
│          │    │               │    │                    │
│ D435相机  │    │ VLA 双模式    │    │ ACT Transformer   │
│ OpenCV   │    │ Qwen2-VL 7B  │    │ Diffusion Policy  │
│ 虚拟传感器│    │ OpenVLA      │    │ PPO 多风格+情绪    │
│          │    │ LoRA 微调     │    │ SAC RL            │
│          │    │ py_trees行为树│    │ 阻抗控制仿真       │
└──────────┘    └───────────────┘    └────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │  ROS2 统一通信    │
                    │  Sim2Real Bridge │
                    └────────┬────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
     ┌────────▼──────┐           ┌─────────▼────────┐
     │  MuJoCo 仿真   │           │  USB CDC Driver  │
     │  (模拟模式)    │           │  (实机模式)       │
     └───────────────┘           └──────────────────┘
```

## 项目结构

```
ElectronBot_SIM/
├── setup_env.sh                 # 一键环境部署脚本
├── simulation/                  # 仿真与 ROS2 工程
│   ├── electronbot_description/ # ROS2 URDF 模型描述
│   ├── electronbot_mujoco/      # MuJoCo 仿真环境 + 阻抗控制
│   └── electronbot_mujoco_ros2/ # MuJoCo↔ROS2 桥接
├── ai/                          # AI 训练与推理
│   ├── rl/                      # PPO/SAC 强化学习 + 情绪策略
│   ├── il/                      # ACT/Diffusion Policy 模仿学习
│   └── vla/                     # VLA 双模式 (自定义+OpenVLA) + LoRA
├── behavior/                    # py_trees 行为树编排
├── benchmark/                   # 统一 Benchmark 评估框架
├── sim2real/                    # Sim2Real 基础设施
└── docs/                        # 项目文档
```

## 快速开始

### 1. 环境部署
```bash
cd ElectronBot_SIM
bash setup_env.sh
source .venv/bin/activate
```

### 2. 各阶段入口

| 阶段 | 内容 | 关键脚本 |
|------|------|----------|
| Phase 1 | URDF/MJCF 建模 | `simulation/electronbot_mujoco/scripts/test_env.py` |
| Phase 2 | 仿真环境 + 阻抗控制 | `simulation/electronbot_mujoco/scripts/test_env.py` |
| Phase 3 | ROS2 通信 | `simulation/electronbot_mujoco_ros2/launch/sim.launch.py` |
| Phase 4 | RL 训练 (PPO/SAC/情绪) | `ai/rl/electronbot_rl/train_ppo.py` |
| Phase 5 | IL 训练 (ACT/Diffusion) | `ai/il/electronbot_il/act/train.py` |
| Phase 6 | VLA 部署 (双模式) | `ai/vla/electronbot_vla/vla_node.py` |
| Phase 7 | 行为树 | `behavior/electronbot_behavior/demos/` |
| Phase 8 | Benchmark | `benchmark/electronbot_benchmark/evaluator.py` |
| Phase 9 | Sim2Real | `sim2real/electronbot_real/` |

## 技术栈

| 组件 | 技术 | 版本 |
|------|------|------|
| 仿真引擎 | MuJoCo | ≥3.1 |
| RL 框架 | Stable-Baselines3 | ≥2.3 |
| 通信 | ROS2 Humble | Humble |
| 深度学习 | PyTorch | ≥2.1 |
| 行为树 | py_trees | ≥2.0 |
| VLM | Qwen2-VL 7B AWQ | 4-bit 量化 |
| VLA | OpenVLA | latest |

## 硬件需求

- **GPU**: NVIDIA RTX 2060 12GB (或以上)
- **RAM**: ≥16GB
- **OS**: Ubuntu 22.04/24.04
- **物理机器人**: 可选 (Phase 1-8 不需要)

## 预期产出

- 6 自由度 URDF/MJCF 机器人模型 (含 D435 仿真相机)
- 5 个标准化 Benchmark 任务 (Reach/Push/Wave/PointAt/Stack)
- PPO(含 3 种情绪风格) + SAC baseline
- ACT + Diffusion Policy 对比实验
- Qwen2-VL + OpenVLA 双模式 VLA 系统
- py_trees 行为树 Demo (找球触碰、情绪化序列)
- 阻抗控制仿真
- 完整 Sim2Real 基础设施

## License

MIT (基于 ElectronBot 开源项目)
