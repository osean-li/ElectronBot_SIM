# 第四章 Gymnasium 环境与动作系统的实现

> **核心问题**：如何将 MJCF 模型封装为标准 RL 环境，并实现与真机固件行为一致的 6 舵机动作系统？

## 4.1 章节目标

本章从概要设计文档的 Layer 2（物理仿真引擎）及 Layer 3（动作系统层）出发，目标为：
1. 实现 `ElectronBotEnv`——基于 Gymnasium 标准的仿真环境
2. 实现 `ElectronBotActions`——包含预设动作与舵机级控制的 12 种工具
3. 确保插值算法、舵机映射比、安全限位与真机固件 1:1 对齐

## 4.2 Gymnasium 环境封装

### 4.2.1 环境基类

`ElectronBotEnv` 继承自 `gymnasium.Env`，是对 MuJoCo 的轻量封装：

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco

class ElectronBotEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(self, render_mode="human", obs_mode="full"):
        super().__init__()
        xml_path = "assets/mjcf/electronbot_scene.xml"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
        
        self.observation_space = spaces.Dict({
            "joint_pos": spaces.Box(-180, 180, shape=(6,)),
        })
```

### 4.2.2 核心方法

**`reset()`**：重置仿真状态至初始姿态，触发域随机化。

**`step(action)`**：接收 6 维归一化动作向量（范围 [-1, 1]），映射到物理角度后写入 `data.ctrl`，然后执行 10 步子步进（2ms × 10 = 20ms，对应 50Hz 控制频率）。

### 4.2.3 双观测模式

根据概要设计文档的要求，实现了两种观测模式：

**`obs_mode="full"`**（仿真专属，用于研究）：
```python
{
    "joint_pos":   Box(-180, 180, shape=(6,)),     # 关节角度
    "joint_vel":   Box(-inf, inf, shape=(6,)),      # 关节速度
    "ee_left_pos": Box(-inf, inf, shape=(3,)),      # 左末端位置
    "image":       Box(0, 255, shape=(240,240,3)),  # RGB 图像
}
```

**`obs_mode="realistic"`**（Sim2Real 用，仅含真机可获取数据）：
```python
{
    "commanded_joint_pos": Box(-180, 180, shape=(6,)),  # 最后发出的指令角度
    "is_moving":           Box(0, 1, shape=(1,)),       # 是否正在执行动作
    "battery_voltage":     Box(3.0, 4.2, shape=(1,)),   # 电池电压
}
```

**设计原则**：`realistic` 模式排除了真机无法获得的数据（关节速度、末端位置、摄像头图像）。任何计划 Sim2Real 的策略必须在 `realistic` 模式下验证通过。

## 4.3 动作系统

### 4.3.1 三层架构

`ElectronBotActions` 按抽象层级分为三层：

```
第一层：预设动作（用户级）
  hand_action() / body_turn() / head_move() / stop() / home()

第二层：舵机级控制（AI 级）
  servo_move() / servo_sequences()

第三层：内部执行（固件对齐）
  _move_servos() — 线性插值
  _oscillate() — 正弦振荡
```

### 4.3.2 固件行为对齐

真机固件 `movements.cc` 第 87 行的插值实现为：

```c
increment_[i] = (target[i] - pos[i]) / (time / 10.0);
```

对应 Python 实现：

```python
def _move_servos(self, targets, time_ms):
    n_steps = max(1, time_ms // 10)
    delta = (targets - current) / n_steps
    for _ in range(n_steps):
        current += delta
        self.env.data.ctrl[:] = current
        for _ in range(10):
            mujoco.mj_step(self.env.model, self.env.data)
```

对齐的要素包括：
- 每 10ms 步进的等量增量插值
- 6 组舵机硬限位裁剪 `ClampServoTarget()`
- 舵机→关节映射比（1.0/1.125/1.5/2.0）
- 50Hz 控制频率（20ms 周期）

### 4.3.3 舵机映射比与硬限位

**映射比**：舵机 PWM 控制范围与机械关节范围的换算关系。

| 关节 | 舵机安全范围 | 中心 | 机械范围 | 映射比 |
|------|:-----------:|:---:|:--------:|:-----:|
| BODY | 30°~150° | 90° | ±90° | 1.5 |
| HEAD | 75°~105° | 90° | ±30° | 2.0 |
| ARM_PITCH | 0°~180° | 90° | ±90° | 1.0 |
| LEFT_ROLL | 0°~80° | 40° | ±45° | 1.125 |
| RIGHT_ROLL | 100°~180° | 140° | ±45° | 1.125 |

**硬限位**：任何角度指令超过以下范围将被裁剪至边界：

```python
SERVO_LIMITS = {
    "rp": (0, 180),    # Right Pitch
    "rr": (100, 180),  # Right Roll
    "lp": (0, 180),    # Left Pitch
    "lr": (0, 80),     # Left Roll
    "b":  (30, 150),   # Body
    "h":  (75, 105),   # Head
}
```

### 4.3.4 预设动作实现

以 `hand_action` 为例，参数签名与内部逻辑如下：

```python
def hand_action(self, action, hand, steps, speed, amount=30):
    """
    action: 1=举手, 2=放手, 3=挥手, 4=拍打
    hand:   1=左手, 2=右手, 3=双手
    steps:  重复次数
    speed:  动作速度 (ms)
    amount: 举手幅度 (10-50)
    """
    servos = []
    if hand in [1, 3]: servos.extend(['lp', 'lr'])
    if hand in [2, 3]: servos.extend(['rp', 'rr'])
    
    if action == 3:  # 挥手
        for _ in range(steps):
            self._move_servos({s: 135 for s in servos}, speed // 2)
            self._move_servos({s: 90 for s in servos}, speed // 2)
    
    return {"status": "ok"}
```

### 4.3.5 振荡模式

对齐真机 `OscillateServos()` 的正弦振荡实现：

```python
def _oscillate(self, amplitudes, centers, period, cycles):
    sample_interval = 50  # ms，对齐固件 100Hz tick → 50ms
    total_samples = cycles * period // sample_interval
    
    for i in range(total_samples):
        phase = 2 * math.pi * i / (period / sample_interval)
        targets = centers + amplitudes * math.sin(phase)
        self._move_servos(targets, sample_interval)
```

## 4.4 本章小结

本章完成了 Gymnasium 环境封装与动作系统的实现。与概要设计文档的主要差异：

| 项目 | 设计文档 | 实际实现 |
|------|---------|---------|
| 关节命名 | `joint_body` / `joint_head` / `joint_lp` / `joint_lr` | `body_joint` / `head_joint` / `left_pitch_joint` / `left_roll_joint` |
| 插值方式 | 未明确指定 | 线性插值（对齐固件 movements.cc:87） |
| 手臂运动轴 | 未具体说明 | 左臂 Pitch(Y)、Roll(X) 已验证 |
| 工具总数 | 8 个预设 | 12 个（含 4 仿真专属） |
