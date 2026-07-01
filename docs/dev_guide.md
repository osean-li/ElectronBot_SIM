# ElectronBot_SIM 编译·调试·使用手册

> 110 文件, ~9000 行代码, 8 个模块

---

## 1. 快速开始

### 1.1 首次安装

```bash
cd ElectronBot_SIM

bash setup_env.sh --gpu

source .venv/bin/activate
```

### 1.2 启动仿真

```bash
# 有桌面 — 交互式拖拽滑块
python3 -m mujoco.viewer --mjcf=simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot_inline.xml

# 无桌面 (SSH/服务器) — 加 EGL 前缀
MUJOCO_GL=egl python3 -m mujoco.viewer --mjcf=simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot_inline.xml
```

> 训练时用 `scene.xml`（几何基元，~2000 fps），比 inline 版快 4 倍。

### 1.4 一键验证

```bash
python3 simulation/electronbot_mujoco/scripts/test_env.py --test all
```

预期：7 项中 5 项 `[OK]`。

### 1.5 快速调试

```bash
source .venv/bin/activate && python3 -c "import mujoco,torch,gymnasium;print('OK')"

python3 -c "import mujoco;m=mujoco.MjModel.from_xml_path('simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot_inline.xml');print(m.ngeom)"

python3 simulation/electronbot_mujoco/scripts/test_env.py --test mapping

python3 simulation/electronbot_mujoco/scripts/test_env.py --test protocol
```

---

## 2. 环境部署

| 选项 | 作用 |
|------|------|
| (无) | 核心依赖: MuJoCo, Gym, SB3, py_trees, OpenCV, h5py |
| `--gpu` | 加装 CUDA PyTorch |
| `--full` | 加装 VLA: transformers, vLLM, OpenVLA |
| `--skip-ros` | 跳过 ROS2 apt 安装 |

```bash
bash setup_env.sh

bash setup_env.sh --gpu

bash setup_env.sh --gpu --full

bash setup_env.sh --gpu --full --skip-ros
```

安装 3-15 分钟，自动处理 dpkg 锁/CUDA 不匹配/pip 冲突。

---

## 3. 运行入口

### 3.1 仿真测试

```bash
python3 simulation/electronbot_mujoco/scripts/test_env.py --test all

python3 simulation/electronbot_mujoco/scripts/test_env.py --test mapping

python3 simulation/electronbot_mujoco/scripts/test_env.py --test firmware

python3 simulation/electronbot_mujoco/scripts/test_env.py --test i2c

python3 simulation/electronbot_mujoco/scripts/test_env.py --test protocol

python3 simulation/electronbot_mujoco/scripts/test_env.py --test servo

python3 simulation/electronbot_mujoco/scripts/test_env.py --test disturbance
```

### 3.2 AI 训练

```bash
python3 ai/rl/electronbot_rl/train_ppo.py --task reach --timesteps 100000

python3 ai/rl/electronbot_rl/train_ppo.py --task wave --emotion excited

python3 ai/rl/electronbot_rl/train_sac.py --task push --timesteps 100000

python3 ai/il/scripts/collect_demo.py --output demos.h5 --episodes 50

python3 ai/il/electronbot_il/act/train.py --data demos.h5 --epochs 500

python3 ai/il/electronbot_il/diffusion/train.py --data demos.h5 --epochs 1000

python3 ai/rl/electronbot_rl/inference.py --checkpoint checkpoints/ppo_reach.zip --record
```

### 3.3 VLA 部署

```bash
bash scripts/deploy_qwen_vl.sh all

bash scripts/deploy_qwen_vl.sh all --model 2b

bash scripts/deploy_qwen_vl.sh status

bash scripts/deploy_qwen_vl.sh stop

bash scripts/deploy_openvla.sh all
```

### 3.4 ROS2

```bash
source /opt/ros/humble/setup.bash

ros2 launch electronbot_mujoco_ros2 sim.launch.py

ros2 topic list
```

### 3.5 其他

```bash
python3 behavior/electronbot_behavior/behavior_tree.py

python3 benchmark/electronbot_benchmark/evaluator.py --task all

python3 sim2real/electronbot_sim2real/system_id.py

python3 sim2real/electronbot_real/calibration.py
```

---

## 4. 按阶段运行

| 阶段 | 命令 | 预期 |
|------|------|------|
| Phase 1-2 | `test_env.py --test mapping` | 往返误差 < 0.01° |
| Phase 2 | `test_env.py --test firmware` | 关节追踪移动 |
| Phase 3 | `ros2 launch ...` | topic 有数据 |
| Phase 4 | `train_ppo.py --task reach` | reward 上升 |
| Phase 5 | `act/train.py --data demos.h5` | loss 收敛 |
| Phase 6 | `deploy_qwen_vl.sh all` | 返回 6 角度 |
| Phase 7 | `behavior_tree.py` | 执行复合序列 |
| Phase 8 | `evaluator.py --task all` | 算法对比 |
| Phase 9 | `system_id.py` | 参数输出 |

---

## 5. 关键参数速查

| 参数 | 值 | 来源 |
|------|-----|------|
| 关节数 | 6 | `robot.h` |
| I2C 帧长 | 5 字节 | `main.cpp` |
| ExtraData | 32 字节 | `electron_low_level.cpp` |
| 舵机频率 | 200 Hz | `TIM14` |
| PWM 范围 | 0–1000 | `motor.cpp` |
| 图像 | 240×240×3 | `electron_low_level.cpp` |

完整参数见 `hardware_reference.md`

---

## 6. 常见问题

> *TODO: 实际开发中遇到的问题在此补充*
