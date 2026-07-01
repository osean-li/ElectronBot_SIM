# ElectronBot_SIM 架构设计文档

> 版本: 1.0 | 更新: 2026-07-01 | 57 文件 / 6833 行代码

---

## 目录

1. [系统概述](#1-系统概述)
2. [三层架构总览](#2-三层架构总览)
3. [感知层 (Perception)](#3-感知层-perception)
4. [决策层 (Decision)](#4-决策层-decision)
5. [执行层 (Execution)](#5-执行层-execution)
6. [通信层 (Communication)](#6-通信层-communication)
7. [仿真层 (Simulation)](#7-仿真层-simulation)
8. [数据流与时序](#8-数据流与时序)
9. [关键接口定义](#9-关键接口定义)
10. [RobotPose 协议规范](#10-robotpose-协议规范)
11. [物理参数与标定](#11-物理参数与标定)
12. [部署与运维](#12-部署与运维)

---

## 1. 系统概述

ElectronBot_SIM 是一个基于 [稚晖君 ElectronBot](https://github.com/peng-zhihui/ElectronBot) (6-DOF 桌面双臂机器人) 的全栈 AI 机器人仿真与学习平台。项目从零构建，覆盖 CAD→URDF→MJCF 建模、MuJoCo 物理仿真、ROS2 通信、强化学习/模仿学习训练、视觉语言动作决策、行为树编排、Benchmark 评估和 Sim2Real 迁移的全链路。

### 1.1 硬件目标

| 参数 | 值 |
|---|---|
| 自由度 | 6 (head×1, left_arm×2, right_arm×2, body×1) |
| 舵机协议 | I2C 5字节包, 6个 STM32F042 从设备 |
| 主控通信 | USB-CDC Bulk (VID:0x1001, PID:0x8023) |
| 显示 | 240×240 圆形 GC9A01 LCD |
| 仿真 GPU | NVIDIA RTX 2060 12GB, CUDA 13.2 |

### 1.2 技术栈

```
仿真引擎:  MuJoCo ≥3.1            │  VLM 推理:  vLLM + AWQ 量化
RL 框架:   Stable-Baselines3 ≥2.3 │  VLA 框架:  Qwen2-VL / OpenVLA
通信:      ROS2 Humble             │  LoRA 微调: PEFT
IL 框架:   PyTorch ≥2.1           │  行为树:    py_trees ≥2.0
建模:      URDF + xacro + MJCF    │  USB 驱动:  libusb
```

### 1.3 工程目录

```
ElectronBot_SIM/
├── setup_env.sh                          # 一键环境部署 (带异常处理)
├── simulation/
│   ├── electronbot_description/          # ROS2 URDF/xacro 模型描述
│   ├── electronbot_mujoco/              # MuJoCo 仿真环境 (核心)
│   └── electronbot_mujoco_ros2/         # ROS2 桥接 + Sim2Real 抽象
├── ai/
│   ├── rl/                              # PPO/SAC/情绪策略
│   ├── il/                              # ACT/Diffusion Policy
│   └── vla/                             # VLA 双模式 + LoRA
├── behavior/                            # py_trees 行为树
├── benchmark/                           # 统一评估框架
├── sim2real/                            # USB 驱动 + 标定 + 系统辨识
└── docs/                                # 架构文档
```

---

## 2. 三层架构总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        ElectronBot_SIM 系统架构                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐    │
│  │    感知层         │   │    决策层          │   │    执行层         │    │
│  │  (Perception)    │──▶│  (Decision)       │──▶│  (Execution)     │    │
│  │                  │   │                   │   │                  │    │
│  │ • D435 仿真相机   │   │ • 自定义VLA       │   │ • ACT Transformer│    │
│  │   RGB+Depth 240  │   │   Qwen2-VL 7B AWQ│   │   CVAE 架构      │    │
│  │ • OpenCV 预处理   │   │ • OpenVLA 对比    │   │ • Diffusion Policy│   │
│  │   目标检测/边缘   │   │ • LLM LoRA 微调   │   │   1D UNet+DDPM   │    │
│  │ • 虚拟 IMU        │   │ • py_trees 行为树 │   │ • PPO 3种情绪风格 │    │
│  │ • 力/触觉传感器   │   │   序列编排        │   │ • SAC RL 对比    │    │
│  │ • 点云生成(可选)  │   │                   │   │ • 阻抗控制仿真    │    │
│  └────────┬─────────┘   └────────┬──────────┘   └────────┬─────────┘    │
│           │                      │                         │              │
│           └──────────────────────┼─────────────────────────┘              │
│                                  │                                        │
│                   ┌──────────────▼──────────────┐                         │
│                   │     通信层 (ROS2 Humble)      │                         │
│                   │  /joint_states               │                         │
│                   │  /camera/image_raw, /depth    │                         │
│                   │  /joint_trajectory_commands   │                         │
│                   │  /tf                          │                         │
│                   └──────────────┬──────────────┘                         │
│                                  │                                        │
│                   ┌──────────────▼──────────────┐                         │
│                   │   Sim2Real Bridge (RobotInterface)│                    │
│                   │   仿真模式 ←→ 真实硬件模式     │                        │
│                   └──────┬───────────────┬───────┘                         │
│                          │               │                                 │
│            ┌─────────────▼───┐   ┌───────▼─────────────┐                   │
│            │  仿真层          │   │   实体层 (Phase 9)    │                   │
│            │  MuJoCo 物理引擎  │   │   USB CDC Driver    │                   │
│            │  + 5 Benchmark   │   │   libusb bulk       │                   │
│            │  + Domain Rand   │   │   ExtraData 协议    │                   │
│            └─────────────────┘   └─────────────────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.1 层间接口

| 流向 | 接口 | 数据类型 |
|---|---|---|
| 感知→决策 | `camera.get_rgb_from_robot()` | `np.ndarray (240,240,3) uint8` |
| 感知→决策 | `pipeline.preprocess(rgb)` | `np.ndarray (240,240,3) float32` |
| 决策→执行 | `robot.send_position_command(angles)` | `np.ndarray (6,) float64 rad` |
| 执行→通信 | `joint_state_pub.publish(msg)` | `sensor_msgs/JointState` |
| 通信→感知 | `camera_sub.callback(msg)` | `sensor_msgs/Image` |

### 2.2 模式切换

```
训练模式:  Python API 直连 MuJoCo (50Hz, 低延迟)
推理模式:  通过 ROS2 topics (仿真/实机统一接口)
实机模式:  USB CDC Bulk (libusb, VID:0x1001/PID:0x8023)
```

---

## 3. 感知层 (Perception)

### 3.1 D435 仿真相机 (`sensors.py` → `D435SimCamera`)

```
物理参数模拟:
  - 分辨率: 240×240 (对齐 ElectronBot LCD 规格)
  - FOV: 60°
  - 焦距: fx = fy = w/2 / tan(fov/2) ≈ 207.8
  - 输出: RGB 3通道 uint8 + Depth float32

实现方式:
  - RGB:  MuJoCo Renderer → camera sensor → 渲染帧
  - Depth: MuJoCo depth rendering → 归一化深度图
  - 颜色: cv2.COLOR_RGB2BGR 格式转换

关键方法:
  get_rgb_from_robot(robot) → np.ndarray (240,240,3)
  get_depth_from_robot(robot) → np.ndarray (240,240)
  depth_to_point_cloud(depth) → np.ndarray (N,3)  # PCL 扩展
```

### 3.2 OpenCV 预处理管线 (`sensors.py` → `PerceptionPipeline`)

```
处理步骤:
  1. cv2.resize(rgb, (240,240))           # 尺寸对齐
  2. img.astype(float32) / 255.0          # 归一化 [0,1]
  3. 按需: 颜色空间转换 / 目标检测 / 边缘提取

目标检测 (红色物体):
  - cv2.cvtColor(RGB → HSV)
  - cv2.inRange(red_lower, red_upper)    # 双阈值 (0-10, 160-180)
  - cv2.findContours → bbox + center
  - 过滤: cv2.contourArea > 50 px

边缘提取:
  - cv2.Canny(gray, 50, 150)
  - 用于视觉伺服和特征匹配
```

### 3.3 虚拟 IMU (`sensors.py` → `VirtualIMU`)

```
数据来源: MuJoCo sensor
  - accelerometer: 头部连杆加速度 (m/s²)
  - gyro:          头部连杆角速度 (rad/s)
  - quaternion:    头部连杆朝向 [w,x,y,z]

更新频率: 100Hz (MuJoCo sensor update_rate)
噪声模型: 高斯噪声 (accel std=1e-4, gyro std=1e-2)
```

### 3.4 虚拟力传感器 (`sensors.py` → `VirtualForceSensor`)

```
实现: 遍历 MuJoCo contact 数组 → 累加末端 site 的接触力
用途: 触碰检测、力控抓取、Sim2Real 力反馈标定
```

---

## 4. 决策层 (Decision)

### 4.1 VLA 双模式架构 (`vlm_backend.py`)

```
                    ┌─────────────────────────┐
                    │     VLAModel (ABC)        │
                    │  predict(image, text)     │
                    │  is_ready() → bool        │
                    └───────────┬───────────────┘
                                │
              ┌─────────────────┴─────────────────┐
              │                                   │
    ┌─────────▼──────────┐            ┌───────────▼──────────┐
    │  QwenVLAModel       │            │  OpenVLAModel         │
    │  (自定义 VLA)        │            │  (开源 VLA)            │
    │                     │            │                       │
    │  后端: vLLM          │            │  后端: transformers    │
    │  模型: Qwen2-VL 7B   │            │  模型: openvla-7b      │
    │  量化: AWQ 4-bit     │            │  预训练: OXE 数据集    │
    │  VRAM: ~7 GB         │            │  VRAM: ~4 GB          │
    │  Token: max 4096      │            │                       │
    └─────────────────────┘            └───────────────────────┘

运行时切换:
  vla_config.yaml → vla_mode: "qwen" | "openvla" | "keyword"
  create_vla_backend(mode) → VLAModel 实例
```

### 4.2 VLA 推理流程 (`vla_node.py`)

```
┌──────────────┐    ┌───────────────┐    ┌──────────────┐    ┌──────────────┐
│ ROS2 Camera  │───▶│ VLANode       │───▶│ VLM Backend  │───▶│ ActionParser │
│ Subscriber   │    │ _camera_cb()  │    │ predict()    │    │ parse()      │
│              │    │               │    │              │    │              │
│ 30Hz 240x240 │    │ 10Hz 控制循环 │    │ 关键词/VLM   │    │ →6维角度     │
└──────────────┘    └───────────────┘    └──────────────┘    └──────┬───────┘
                                                                    │
                                                    ┌───────────────▼───────────┐
                                                    │ /joint_trajectory_commands │
                                                    │ JointTrajectory Publisher   │
                                                    └───────────────────────────┘

解析策略链:
  1. VLA backend + 图像 → VLM 推理 (主)
  2. 关键词匹配 → 预设动作库 16种 (fallback)
  3. 安全默认 → 零位 (兜底)
```

### 4.3 提示词模板 (`prompt_templates.py`)

```
SYSTEM_PROMPT: 机器人能力描述 + 关节映射表 + JSON 输出格式约束

Few-shot 示例 (5个):
  "挥手" → {"joint_angles_deg": [0,0,0,0,80,10], "explanation": "..."}
  "点头" → {"joint_angles_deg": [0,10,0,0,0,0]}
  "比心" → {"joint_angles_deg": [0,0,60,20,60,20]}

输出解析 (parse_vlm_output):
  1. JSON regex → json.loads → joint_angles_deg
  2. 数组 regex → float 提取 → 6 角度
  3. None → fallback
```

### 4.4 预设动作库 (`action_parser.py`)

16 个预定义动作，覆盖常见交互场景:

| 动作 | 角度 (body,head,lp,lr,rp,rr)° | 描述 |
|---|---|---|
| wave | (0,0,0,0,80,15) | 右手挥手 |
| nod | (0,12,0,0,0,0) | 点头 |
| shake_head | (40,0,0,0,0,0) | 摇头 |
| point_right | (0,0,0,0,60,0) | 指向右方 |
| heart | (0,0,60,20,60,20) | 比心 |
| look_around | → 序列 | 四处张望 |
| excited | (30,15,80,20,80,20) | 兴奋 |
| tired | (0,-10,20,0,20,0) | 疲倦 |

### 4.5 LoRA 微调 (`lora_finetune.py`)

```
基础模型: Qwen2-VL-2B-Instruct
微调方式: PEFT LoRA (r=16, alpha=32, target=q/k/v/o_proj)
数据类型: fp16 + gradient_accumulation=2
数据集: ElectronBot 指令-动作 JSONL
  格式: {"image_path":"...", "instruction":"挥手", "joint_angles":[0,0,0,0,80,15]}
目的: 提升特定指令的遵循精度和输出稳定性
```

### 4.6 py_trees 行为树 (`behavior_tree.py`)

```
节点类型:
  叶子节点:
    PerceiveNode     → 调用感知管线 (detect_red_objects)
    VLANode          → 调用 VLA 推理 (image+prompt→angles)
    ExecuteNode      → 发送关节指令 (send_position_command)
  
  条件节点:
    HasTargetCondition  → target_found? → SUCCESS/FAILURE
    AtTargetCondition   → dist < threshold? → SUCCESS/FAILURE
  
  组合节点:
    Sequence  → 顺序执行所有子节点
    Selector  → 选择第一个成功的子节点 (优先级)

预定义行为树:

1. find_and_touch (找球触碰):
   Sequence[
     Perceive(detect red),
     VLANode("朝红色球移动"),
     Execute(duration=2s),
     VLANode("触碰红色球"),
     Execute(duration=1s)
   ]

2. emotion_sequence (情绪化行为):
   Selector[
     Sequence[VLA("开心"), Execute(3s)],    # 开心优先级最高
     Sequence[VLA("好奇"), Execute(3s)],
     Sequence[VLA("疲倦"), Execute(3s)]
   ]
```

---

## 5. 执行层 (Execution)

### 5.1 ACT (Action Chunking Transformer) (`act/model.py` + `act/train.py`)

```
架构: Conditional VAE

Encoder:
  输入: [obs_chunk (B,100,18) + action_chunk (B,100,6)] → flatten
  网络: Linear(2400→512→512→512) + mu/logvar(head) → z (B,32)

Decoder:
  输入: [obs_chunk + z] → flatten
  网络: Linear(1800+32→512→512→512) + output(6*100)
  输出: action_chunk (B,100,6)  ← 直接预测整个 action chunk

损失函数:
  L_total = L_recon + β * L_KL
  L_recon = MSE(action_pred, action_gt)
  L_KL = -0.5 * mean(1 + logvar - μ² - exp(logvar))

超参数:
  chunk_size=100, latent_dim=32, hidden_dim=512
  lr=1e-4, batch_size=32, β=1.0
  optimizer=Adam, grad_clip=1.0

推理:
  从 N(0,I) 采样 z → decoder(obs_chunk, z) → action_chunk
```

### 5.2 Diffusion Policy (`diffusion/model.py` + `diffusion/train.py`)

```
架构: 1D UNet + DDPM

UNet 组件:
  - SinusoidalPositionEmbedding(dim=256)     # 时间步编码
  - 观测编码: Linear(18→256→256)              # obs_cond
  - 输入投影: Linear(6+256+256→256)
  - 4x ResidualBlock(256, dropout=0.1)       # 1D Conv + GroupNorm + SiLU
  - 输出投影: GroupNorm + SiLU + Conv1d(256→6)

DDPM 调度器 (noise_scheduler.py):
  - Linear schedule: β ∈ [1e-4, 0.02], T=100
  - 前向: x_t = √ᾱ_t * x_0 + √(1-ᾱ_t) * ε
  - 反向: DDPM sampling (均值+方差)
  - 损失: L2(noise_pred, noise)

超参数:
  num_timesteps=100, hidden_dim=256
  lr=1e-4, batch_size=64
  optimizer=AdamW, weight_decay=1e-4
```

### 5.3 PPO 多风格 + SAC (`train_ppo.py` + `train_sac.py`)

```
PPO 配置:
  policy: MlpPolicy, net_arch=[256,256]
  n_steps=2048, batch_size=64, n_epochs=10
  γ=0.99, λ=0.95, clip=0.2
  lr=3e-4, max_grad_norm=0.5
  并行: DummyVecEnv/SubprocVecEnv (4-16 envs)

SAC 配置:
  policy: MlpPolicy, net_arch=pi[256,256] + qf[256,256]
  buffer=100000, batch=256, τ=0.005
  lr=3e-4, train_freq=1

情绪策略 (emotional_reward.py):
  ┌─────────┬──────────────────────────────────┐
  │ 模式     │ Reward Shaping                    │
  ├─────────┼──────────────────────────────────┤
  │ happy   │ base_reward + movement * weight   │
  │ curious │ base_reward + novelty * entropy   │
  │ tired   │ base_reward - energy_cost * weight│
  └─────────┴──────────────────────────────────┘

  movement = Σ|qd|            (动作活跃度)
  novelty  = mean(‖q - s_i‖)  (状态新颖性, 与历史 buffer 的距离)
  energy   = Σ(qd²)           (能耗)
```

### 5.4 阻抗控制 (`impedance_controller.py`)

```
控制律:
  τ = K_p * (q_d - q) + K_d * (qd_d - qd)

刚度模式:
  high:   K_p = diag[20,20,15,15,15,15]   → 精确位置跟踪
  medium: K_p = diag[10,10,5,5,5,5]       → 一般操作
  low:    K_p = diag[3,3,2,2,2,2]         → 柔顺交互

阻尼: K_d = 2 * √K_p (临界阻尼)

执行器:
  位置模式 → position actuator (kp=10, ctrlrange=joint_limits)
  阻抗模式 → motor actuator (torque mode)
  切换: switch_to_position_mode() / switch_to_impedance_mode()

测试 (test_disturbance_rejection):
  步骤:
    1. 设置目标 q_d = [0,0,30°,0,0,0]
    2. 150步时施加外力 -0.3 Nm @ left_shoulder
    3. 验证控制器恢复 (最终误差 < 5°)
```

---

## 6. 通信层 (Communication)

### 6.1 ROS2 Topic 接口

```
发布 (Publish):
  ┌─────────────────────────────┬──────────┬─────────────────────┐
  │ Topic                       │ 频率     │ 消息类型             │
  ├─────────────────────────────┼──────────┼─────────────────────┤
  │ /joint_states               │ 50 Hz    │ JointState           │
  │ /camera/image_raw           │ 30 Hz    │ Image (RGB8, 240x240)│
  │ /camera/depth               │ 30 Hz    │ Image (MONO16)       │
  │ /tf                         │ 50 Hz    │ TFMessage            │
  └─────────────────────────────┴──────────┴─────────────────────┘

订阅 (Subscribe):
  ┌─────────────────────────────────┬──────────┬──────────────────┐
  │ Topic                           │ 频率     │ 消息类型          │
  ├─────────────────────────────────┼──────────┼──────────────────┤
  │ /joint_trajectory_commands      │ 按需     │ JointTrajectory   │
  └─────────────────────────────────┴──────────┴──────────────────┘
```

### 6.2 MuJoCo ↔ ROS2 Bridge (`mujoco_ros2_bridge.py`)

```
节点名称: mujoco_ros2_bridge
启动: ros2 run electronbot_mujoco_ros2 mujoco_ros2_bridge

定时器:
  - joint_timer:   50Hz → _publish_joint_states()
  - camera_timer:  30Hz → _publish_camera()
  - step_timer:    50Hz → _step_simulation()

控制流:
  joint_trajectory_cmd → _target_angles → send_position_command → mujoco.mj_step

发布流:
  robot.get_joint_positions() → JointState message → /joint_states
  camera.get_rgb_from_robot() → CvBridge → Image message → /camera/image_raw
  camera.get_depth_from_robot() → CvBridge → Image message → /camera/depth
```

### 6.3 Launch 文件

```
sim.launch.py:
  ┌─────────────────────────────────┐
  │ Node: mujoco_ros2_bridge        │  MuJoCo + ROS2 桥接
  │ Node: rviz2 (可选)              │  3D 可视化 + TF
  │ Node: plotjuggler (可选)        │  数据监控
  └─────────────────────────────────┘

display.launch.py:
  ┌─────────────────────────────────┐
  │ Node: robot_state_publisher     │  发布 TF
  │ Node: joint_state_publisher_gui │  手动控制
  │ Node: rviz2                     │  URDF 可视化
  └─────────────────────────────────┘
```

---

## 7. 仿真层 (Simulation)

### 7.1 URDF/MJCF 运动学链

```
base_link (root, fixed)
  └── body_joint [revolute, Y轴, -90°~90°] ──► body
        ├── head_joint [revolute, X轴, -15°~15°] ──► head
        │     └── camera_link (D435 fixed offset)
        │
        ├── left_shoulder_joint [revolute, X轴, -20°~180°] ──► left_shoulder
        │     └── left_arm_roll_joint [revolute, Z轴, 0°~30°] ──► left_arm (ee)
        │
        └── right_shoulder_joint [revolute, X轴, -20°~180°] ──► right_shoulder
              └── right_arm_roll_joint [revolute, Z轴, 0°~30°] ──► right_arm (ee)
```

### 7.2 MuJoCo 模型配置 (`electronbot.xml`)

```
执行器层:
  位置控制:  6 个 position actuator (kp=10, ctrlrange=joint_limits)
  力矩控制:  6 个 motor actuator (ctrlrange=[-2.0,2.0] or [-1.0,1.0])
  
传感器层:
  IMU:       accelerometer + gyro @ imu_site (head 连杆)
  关节位置:  6 个 jointpos sensor
  关节速度:  6 个 jointvel sensor
  末端位置:  framepos @ left_ee_site + right_ee_site
  相机:      d435_camera (fovy=60°, res=240x240)
  
物理参数:
  timestep:  0.004s (50Hz control × 5 sub-steps)
  damping:   0.5 (joint default)
  friction:  [0.5, 0.1, 0.1] (geom default)
  gravity:   默认 (0,0,-9.81)
```

### 7.3 Gymnasium 环境 (`env.py`)

```
类: ElectronBotEnv(gym.Env)

状态空间 (18维):
  0-5:   joint_positions     (rad)    来自 robot.get_joint_positions()
  6-11:  joint_velocities    (rad/s)  来自 robot.get_joint_velocities()
  12-17: ee_positions        (m)      来自 robot.get_end_effector_positions()
         [left_ee_x, left_ee_y, left_ee_z, right_ee_x, right_ee_y, right_ee_z]

动作空间 (6维):
  关节角度增量 (rad), 范围 [-1.0, 1.0]
  实际应用: target_q = current_q + action * 0.1

奖励:  任务相关 (由子类 BaseTask._compute_reward 定义)
终止:  BaseTask._is_terminated() → success | max_steps

参数:
  control_freq=50Hz, episode_length=500 steps (10s)
  model.opt.timestep = 1/(50*5) = 0.004s
```

### 7.4 5 个 Benchmark 任务

```
┌───────────┬──────────────────┬────────────────────────────────────┐
│ 任务       │ 目标              │ 成功条件                           │
├───────────┼──────────────────┼────────────────────────────────────┤
│ Reach     │ 末端触碰目标球     │ dist(ee, target) < 0.02m          │
│ Push      │ 推方块到目标位置   │ dist(block, target) < 0.05m       │
│ Wave      │ 周期性挥手动作     │ 完成完整周期 (success≈完成)        │
│ PointAt   │ 指尖指向目标方向   │ angle(ee_dir, target) < 15°       │
│ Stack     │ 堆叠两个方块       │ dist(top_block, target) < 0.03m   │
└───────────┴──────────────────┴────────────────────────────────────┘

奖励函数示例 (Reach):
  reward = -dist(ee, target)        // 基础: 越近越好
  if dist < threshold: reward += 100 // 成功奖励

终止条件:
  - 达成成功率条件
  - step >= max_episode_steps (500)
```

### 7.5 Domain Randomization (`domain_randomizer.py`)

```
随机化参数:
  摩擦系数:    1.0 ± 30%   [0.7, 1.3]
  关节阻尼:    1.0 ± 20%   [0.8, 1.2]
  连杆质量:    1.0 ± 15%   [0.85, 1.15]
  执行器增益:  1.0 ± 25%   [0.75, 1.25]
  观测噪声:    高斯 σ=0.01

使用:
  randomizer = DomainRandomizer()
  randomizer.sample_params()         # 采样随机参数
  randomizer.apply_to_model(robot)   # 应用到 MuJoCo
  obs = randomizer.add_observation_noise(obs)  # 加噪声

预设配置:
  LIGHT_DR:  范围 * 0.5  (轻量)
  DEFAULT_DR: 如上       (标准)
  HEAVY_DR:  范围 * 2.0  (重随机, Sim2Real)
```

---

## 8. 数据流与时序

### 8.1 训练模式 (Python 直连 MuJoCo)

```
Timing: 50Hz control loop

for step in range(max_steps):
  ┌─ action = model.predict(obs)         # 推理 (RL policy)
  ├─ target_q = current_q + action*0.1   # 动作缩放
  ├─ robot.send_position_command(tq)     # 写入 MuJoCo actuator
  ├─ for i in range(5):                  # 5 sub-steps
  │    robot.step()                      #   mujoco.mj_step
  ├─ obs = robot.get_observation()       # 读取 18维状态
  ├─ reward = env._compute_reward()      # 计算奖励
  └─ [可选] 录制视频帧 / TensorBoard 日志
```

### 8.2 VLA 推理模式 (ROS2)

```
Timing: Camera 30Hz, Control 10Hz

Camera callback (30Hz):
  img_msg → CvBridge → rgb (240x240x3) → self._latest_image

Control loop (10Hz):
  ┌─ if self._latest_image:
  │    angles = parser.parse(text, image)
  ├─ JointTrajectory msg
  │    point.positions = angles.tolist()
  ├─ cmd_pub.publish(msg)
  └─ 机器人执行 → 新图像到达 → 再推理 (闭环)
```

### 8.3 行为树模式

```
60 FPS tick loop:

tick() →
  ├─ PerceiveNode.update()
  │    └─ camera.rgb → detect_red_objects → blackboard.objects
  ├─ HasTargetCondition.update()
  │    └─ objects非空? → SUCCESS → 进入 VLANode
  ├─ VLANode.update()
  │    └─ vla_model.predict(rgb, "触碰") → blackboard.target_angles
  ├─ ExecuteNode.update()
  │    └─ send_command(target_angles) → RUNNING (2秒)
  └─ AtTargetCondition → SUCCESS → 序列完成
```

---

## 9. 关键接口定义

### 9.1 RobotInterface (Sim2Real 抽象)

```python
class RobotInterface(ABC):
    def connect(self) -> bool
    def disconnect(self) -> None
    def get_joint_positions(self) -> np.ndarray          # → (6,) rad
    def get_joint_velocities(self) -> np.ndarray          # → (6,) rad/s
    def send_joint_command(self, angles: np.ndarray)      # angle (6,) rad
    def get_camera_image(self) -> np.ndarray              # → (240,240,3) uint8
    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]

实现:
  SimRobotInterface(robot)   → MuJoCo 后端
  RealRobotInterface(vid,pid) → USB CDC 后端 (Phase 9)
```

### 9.2 VLAModel (VLA 抽象)

```python
class VLAModel(ABC):
    def predict(self, image: np.ndarray, prompt: str) -> np.ndarray  # → (6,) rad
    def is_ready(self) -> bool

实现:
  QwenVLAModel(model_path)    → vLLM 本地推理
  OpenVLAModel(model_path)    → transformers 推理
```

### 9.3 BaseTask (RL 任务抽象)

```python
class BaseTask(ElectronBotEnv):
    def reset(seed, options) → obs, info
    def step(action) → obs, reward, terminated, truncated, info
    def _compute_reward() → float       # 子类实现
    def _is_terminated() → bool         # 子类实现
    def _get_success() → bool           # 子类实现
    def _get_initial_qpos() → np.ndarray # 可 override
```

---

## 10. RobotPose 协议规范

### 10.1 关节角度定义 (6维, 度)

```
索引 | 名称              | 模型角度范围  | 机械角度范围   | 反转 | 舵机ID | 轴
-----|-------------------|--------------|---------------|------|--------|----
 0   | body (腰部)       | [-90, 90]    | [0, 180]      | No   | 12     | Y
 1   | head (头部)       | [-15, 15]    | [70, 95]      | Yes  | 2      | X
 2   | left_arm_pitch    | [-20, 180]   | [-16, 117]    | No   | 6      | X
 3   | left_arm_roll     | [0, 30]      | [-9, 3]       | No   | 4      | Z
 4   | right_arm_pitch   | [-20, 180]   | [15, 150]     | Yes  | 10     | X
 5   | right_arm_roll    | [0, 30]      | [133, 141]    | Yes  | 8      | Z
```

### 10.2 角度转换 (`utils.py`)

```
模型角度 → 机械角度:
  mech = mech_min + (model - model_min)/(model_max - model_min) * (mech_max - mech_min)
  [如果 inverted: model = model_max - (model - model_min)]

机械角度 → 模型角度:
  同上公式反向计算

角度单位:
  仿真内部: 弧度 (rad), MuJoCo XML 中 compiler angle="radian"
  外部接口: 度 (deg), 用于日志和人类可读
```

### 10.3 USB ExtraData 格式 (32 字节)

```
Offset | Size | Type    | Field
-------|------|---------|--------------------------
 0     | 1    | uint8   | enable (0=disable, 1=enable)
 1-4   | 4    | float32 | joint_angle[0] (body, little-endian)
 5-8   | 4    | float32 | joint_angle[1] (head)
 9-12  | 4    | float32 | joint_angle[2] (left_arm_pitch)
13-16  | 4    | float32 | joint_angle[3] (left_arm_roll)
17-20  | 4    | float32 | joint_angle[4] (right_arm_pitch)
21-24  | 4    | float32 | joint_angle[5] (right_arm_roll)
25-31  | 7    | uint8[] | 保留
```

### 10.4 I2C 舵机协议 (5 字节包)

```
Byte | Field
-----|-----------------------------
 0   | I2C 地址 | R/W 标志
 1   | 寄存器地址
 2-4 | 数据 (小端)

寄存器映射:
  0x01: angle    (float, 读/写)
  0x02: speed    (float, 读/写)
  0x03: torque   (float, 读/写)
  0x10: kp       (float, 读/写)
  0x11: ki       (float, 读/写)
  0x12: kd       (float, 读/写)
  0x20: enable   (uint8, 写)
  0x30: id       (uint8, 读)

I2C 从设备地址 → 关节:
  0x02: head, 0x04: left_arm_roll, 0x06: left_arm_pitch
  0x08: right_arm_roll, 0x0A: right_arm_pitch, 0x0C: body
```

---

## 11. 物理参数与标定

### 11.1 连杆质量 (理论值, 单位 kg)

```
base_link:      0.15    (底座)
body:           0.12    (躯干, 含腰部电机)
head:           0.08    (头部, 含 LCD)
shoulder×2:     0.03×2  (肩部)
arm×2:          0.05×2  (臂 + 末端)
总计:           0.51 kg

惯性: 简化为均质长方体计算 (box_inertial 宏)
  Ixx = m/12 * (y² + z²)
  Iyy = m/12 * (x² + z²)
  Izz = m/12 * (x² + y²)
```

### 11.2 系统辨识 (`system_id.py`)

```
摩擦力估计:
  τ = sign(v) * f_c + f_v * v
  方法: 最小二乘拟合 (正/负方向分离)

质量偏差估计:
  Δm 使得 real ≈ sim / (1 + Δm)
  方法: 比较相同扭矩下的加速度比值 (中值滤波)

DR 参数校准:
  friction_range = 0.3 + 实际粘滞摩擦补偿
  mass_range = max(0.15, |1-Δm| * 1.5)
```

### 11.3 舵机标定 (`calibration.py`)

```
流程:
  1. generate_sweep_trajectory() → 正弦扫频激励
  2. 发送到真实机器人 → 采集角度数据
  3. estimate_gear_ratio() → 最小二乘
  4. estimate_backlash() → 反转点滞后分析
  5. estimate_zero_offset() → 均值偏差
  6. 输出 calib_params.json

输出:
  [{"joint": "body", "gear_ratio": 0.98, 
    "backlash_deg": 0.5, "zero_offset_deg": -1.2}, ...]
```

---

## 12. 部署与运维

### 12.1 环境部署

```bash
# 核心依赖 (MuJoCo, Gym, SB3, py_trees)
bash setup_env.sh

# 带 CUDA PyTorch (RTX 2060 12GB)
bash setup_env.sh --gpu

# 完整依赖 (VLA: transformers, vLLM, OpenVLA)
bash setup_env.sh --gpu --full

# 跳过 ROS2 apt 安装
bash setup_env.sh --gpu --skip-ros
```

### 12.2 各阶段入口

```bash
# Phase 1-2: 仿真测试
source .venv/bin/activate
python simulation/electronbot_mujoco/scripts/test_env.py --test all

# Phase 4: RL 训练
python ai/rl/electronbot_rl/train_ppo.py --task reach --arm right --timesteps 1000000
python ai/rl/electronbot_rl/train_sac.py --task reach --arm right --timesteps 1000000

# Phase 5: IL 数据采集 + 训练
python ai/il/scripts/collect_demo.py --output demos.h5
python ai/il/electronbot_il/act/train.py --data demos.h5 --epochs 200
python ai/il/electronbot_il/diffusion/train.py --data demos.h5 --epochs 500

# Phase 6: VLA 部署
python ai/vla/electronbot_vla/qwen_vl_server.py --action download
python ai/vla/electronbot_vla/qwen_vl_server.py --action serve

# Phase 6: VLA 推理 (ROS2)
ros2 run electronbot_vla vla_node --ros-args -p vla_mode:=qwen

# Phase 7: 行为树
python behavior/electronbot_behavior/behavior_tree.py

# Phase 8: Benchmark
python benchmark/electronbot_benchmark/evaluator.py

# Phase 9: 实机标定 (需硬件)
python sim2real/electronbot_real/calibration.py --joint all
```

### 12.3 资源消耗

```
┌───────────────────────┬────────┬──────────┬──────────┬──────────┐
│ 组件                   │ CPU    │ GPU VRAM │ RAM      │ 磁盘     │
├───────────────────────┼────────┼──────────┼──────────┼──────────┤
│ MuJoCo 仿真 (16env)    │ 8 核   │ 0        │ ~2 GB    │ <50 MB   │
│ PPO 训练               │ 8 核   │ 0        │ ~4 GB    │ <1 GB    │
│ ACT 训练               │ 2 核   │ ~4 GB    │ ~4 GB    │ ~500 MB  │
│ Diffusion Policy 训练   │ 2 核   │ ~4 GB    │ ~4 GB    │ ~300 MB  │
│ Qwen2-VL 7B AWQ        │ 2 核   │ ~7 GB    │ ~8 GB    │ ~15 GB   │
│ OpenVLA                │ 2 核   │ ~4 GB    │ ~8 GB    │ ~5 GB    │
│ LoRA 微调 (1.5B)       │ 2 核   │ ~6 GB    │ ~8 GB    │ ~3 GB    │
│ ROS2 + RViz2 + py_trees│ 2 核   │ 0        │ ~2 GB    │ 0        │
└───────────────────────┴────────┴──────────┴──────────┴──────────┘

注意: 峰值组件不会同时运行。典型工作流 (PPO+MuJoCo) 仅需 4-8 GB RAM。
```

---

## 附录 A: 文件清单

```
ElectronBot_SIM/ (57 文件, 6833 行)

simulation/
  electronbot_description/
    urdf/electronbot.urdf, electronbot.urdf.xacro, materials.xacro
    launch/display.launch.py
    package.xml, CMakeLists.txt

  electronbot_mujoco/
    electronbot_mujoco/
      assets/electronbot.xml, scene.xml
      robot.py, env.py, utils.py, sensors.py
      impedance_controller.py, domain_randomizer.py
      tasks/ (__init__, base, reach, push, wave, pointat, stack)
    scripts/test_env.py, convert_step_to_stl.py
    pyproject.toml

  electronbot_mujoco_ros2/
    electronbot_mujoco_ros2/
      mujoco_ros2_bridge.py, sim2real_bridge.py
    launch/sim.launch.py
    package.xml, CMakeLists.txt

ai/
  rl/electronbot_rl/
    train_ppo.py, train_sac.py, inference.py
    emotional_reward.py
    hyperparams/ (ppo_config.yaml, sac_config.yaml)
  il/electronbot_il/
    dataset.py
    act/model.py, act/train.py
    diffusion/model.py, diffusion/noise_scheduler.py, diffusion/train.py
    scripts/collect_demo.py
  vla/electronbot_vla/
    vlm_backend.py, qwen_vl_server.py
    vla_node.py, prompt_templates.py, action_parser.py
    lora_finetune.py

behavior/electronbot_behavior/
  behavior_tree.py

benchmark/electronbot_benchmark/
  evaluator.py, metrics.py, task_registry.py

sim2real/
  electronbot_real/
    usb_driver.py, protocol.py, real_robot.py, calibration.py
  electronbot_sim2real/
    system_id.py

setup_env.sh, README.md
```

---

## 附录 B: 依赖关系图

```
ElectronBot.step (CAD)
    │
    ├──► electronbot.urdf ──► electronbot.xml (MJCF)
    │                              │
    │    ┌─────────────────────────┘
    │    ▼
    ├──► robot.py (模型加载/控制)
    │       │
    │       ├──► env.py (Gymnasium)
    │       │       │
    │       │       ├──► train_ppo.py → PPO policy
    │       │       ├──► train_sac.py → SAC policy
    │       │       ├──► emotional_reward.py → 3 情绪风格
    │       │       └──► tasks/ (5 Benchmark)
    │       │
    │       ├──► sensors.py (D435相机, IMU, 力传感器)
    │       │       │
    │       │       └──► vlm_backend.py (VLA 双模式)
    │       │               │
    │       │               ├──► QwenVLAModel (vLLM)
    │       │               ├──► OpenVLAModel (transformers)
    │       │               └──► vla_node.py (ROS2 节点)
    │       │                       │
    │       │                       └──► behavior_tree.py (py_trees)
    │       │
    │       ├──► impedance_controller.py → torque/position 双模式
    │       │
    │       └──► mujoco_ros2_bridge.py → ROS2 topics
    │               │
    │               └──► sim2real_bridge.py (RobotInterface)
    │                       │
    │                       ├──► SimRobotInterface (仿真)
    │                       └──► RealRobotInterface (USB CDC)
    │
    └──► utils.py (角度转换, 前向运动学)
```

---

> **文档版本**: v1.0 | **作者**: ElectronBot_SIM Team | **许可证**: MIT
