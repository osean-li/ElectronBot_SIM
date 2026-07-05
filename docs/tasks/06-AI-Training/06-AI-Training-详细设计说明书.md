# Phase 6：AI 训练管线

> **目标**：在仿真中实现完整的模仿学习（IL）+ 强化学习（RL）+ 视觉语言动作（VLA）训练管线。训练出的策略网络输出的是 MCP 格式的动作指令，可直接部署到真机。
>
> **前置依赖**：Phase 5 完成（传感器+观测空间）
>
> **输出**：`src/electronbot_ai/`——训练脚本、数据集收集工具、策略评估脚本
>
> **文档版本**: v1.1  
> **最后更新**: 2026-07-04  
> **变更类型**: 补充软件工程规范章节

---

## 1. 预期效果

### 1.1 模仿学习——88 行训练一个策略

```python
# 1. 键盘遥控收集 50 条示范
python -m electronbot_ai.il.collect_demos --task reach --num_episodes 50

# 2. 训练 Behavior Cloning
python -m electronbot_ai.il.train_bc --task reach --epochs 100

# 3. 评估
python -m electronbot_ai.il.evaluate --task reach --checkpoint checkpoints/bc_reach.pt
# → 成功率: 87%  (35/40 次成功)
```

### 1.2 强化学习——并行训练

```python
python -m electronbot_ai.rl.train_ppo \
    --task pick_place \
    --num_envs 64 \
    --total_steps 1_000_000 \
    --output checkpoints/ppo_pick_place.zip

# 训练监控（TensorBoard）
# → 平均 reward 从 -10 收敛到 +50
# → 成功率从 0% 收敛到 92%
```

### 1.3 VLA 决策

```python
# 语音指令 → LLM 规划 → MCP 动作序列
result = vla_planner.execute("帮我拿那个红色的方块")
# LLM 输出：定位红色方块 → 生成 servo_sequence → 在仿真中执行 → 返回结果
```

---

## 2. 任务定义

### 2.1 统一任务接口

```python
# src/electronbot_ai/tasks/base.py

from abc import ABC, abstractmethod

class BaseTask(ABC):
    """所有训练任务的基类"""
    
    @abstractmethod
    def reset(self, env) -> dict:
        """重置任务——设置场景、放置物体"""
        pass
    
    @abstractmethod
    def get_observation(self) -> dict:
        """获取任务相关的观测"""
        pass
    
    @abstractmethod
    def compute_reward(self) -> float:
        """计算奖励——RL 训练用"""
        pass
    
    @abstractmethod
    def is_success(self) -> bool:
        """判断任务是否成功"""
        pass
    
    @abstractmethod
    def get_demo_action(self, keyboard_state) -> np.ndarray:
        """从键盘输入获取示范动作——IL 收集数据用"""
        pass
```

### 2.2 7 个标准任务（与 Phase 7 Benchmark 对齐）

> **任务命名统一**: 所有文档使用 EB-Stack（非 EB-Press）、EB-VoiceCmd（非 EB-VoiceCommand）、EB-Gesture。

| 任务 | 奖励设计 | 成功条件 | 真机对齐 |
|------|---------|---------|:---:|
| **EB-Reach** | d(ee, target) 的负值 | d < 2cm | ✅ |
| **EB-Push** | 物体离目标位置的距离 | d < 3cm | ✅ |
| **EB-PickPlace** | 抓取+放置分段奖励 | 物体离开桌面+放入目标区域 | ❌ 仿真专属（真机无手指） |
| **EB-Stack** | 物体到达目标高度的奖励 | 物体堆叠成功 | ❌ 仿真专属（真机无手指） |
| **EB-Follow** | 追踪移动物体的距离 | 连续 5s 内 d < 3cm | ✅ |
| **EB-Gesture** | 目标姿态与当前姿态距离 | 达到目标姿态 | ✅ |
| **EB-VoiceCmd** | LLM 评估动作序列是否完成指令 | 指令语义匹配 | ✅ |

> **⚠️ RL 部署限制**: PPO/SAC 策略在仿真中以 50Hz (20ms/步) 运行，但真机云端 API 路径延迟为 200-500ms RTT（有效闭环延迟 400-1000ms，对应 20-50 个仿真步）。RL 策略需要 ONNX 本地推理 + WebSocket 直连（路径 C）或本地推理（路径 D）才能部署。当前仅支持预设动作的云端调用（LLM/VLA 场景）。

### 2.3 Reach 任务示例

```python
# src/electronbot_ai/tasks/reach.py

class ReachTask(BaseTask):
    def __init__(self, target_pos=None):
        self.target_pos = target_pos or np.array([0.05, 0.02, -0.02])  # 桌面上的目标点
        self.ee_name = "right_hand"
    
    def reset(self, env):
        """重置：随机化目标位置 + 域随机化"""
        # 随机化目标位置（Gymnasium 风格的域随机化）
        self.target_pos = np.array([
            np.random.uniform(-0.08, 0.08),   # x: ±8cm
            np.random.uniform(-0.02, 0.05),   # y: -2~5cm
            np.random.uniform(-0.03, 0.0),    # z: -3~0cm（桌面附近）
        ])
        env.reset()
        return self.get_observation()
    
    def get_observation(self):
        ee_pos = self.env.get_ee_position(self.ee_name)
        return {
            "joint_pos": self.env.joint_positions(),
            "ee_pos": ee_pos,
            "target_pos": self.target_pos,
            "dist_to_target": np.linalg.norm(ee_pos - self.target_pos),
        }
    
    def compute_reward(self) -> float:
        dist = np.linalg.norm(
            self.env.get_ee_position(self.ee_name) - self.target_pos
        )
        # 稠密奖励：距离的负值 + 接近奖励 + 成功奖励
        reward = -dist
        if dist < 0.05:   # 5cm 内额外奖励
            reward += 0.5
        if dist < 0.02:   # 2cm 内大额奖励
            reward += 10.0
        return reward
    
    def is_success(self) -> bool:
        return np.linalg.norm(
            self.env.get_ee_position(self.ee_name) - self.target_pos
        ) < 0.02
    
    def get_demo_action(self, keyboard_state) -> np.ndarray:
        """键盘 → 关节增量"""
        action = np.zeros(6)
        # 右臂 Pitch (W/S)
        if keyboard_state.get("w"): action[0] = 2.0   # 上 → 增大
        if keyboard_state.get("s"): action[0] = -2.0  # 下 → 减小
        # 右臂 Roll (E/D)
        if keyboard_state.get("e"): action[1] = 2.0
        if keyboard_state.get("d"): action[1] = -2.0
        # ... 其他关节
        return action
```

---

## 3. 模仿学习管线

### 3.1 示范数据收集

```python
# src/electronbot_ai/il/collect_demos.py

import h5py
import numpy as np

class DemoCollector:
    """键盘遥控收集示范数据"""
    
    def __init__(self, env, task, save_path: str):
        self.env = env
        self.task = task
        self.save_path = save_path
        self.episodes = []
    
    def collect_episode(self) -> dict:
        """收集一条示范轨迹"""
        obs_list, action_list = [], []
        obs = self.task.reset(self.env)
        done = False
        
        while not done:
            # 从键盘读取控制
            action = self._get_keyboard_action()
            obs_list.append(obs)
            action_list.append(action)
            
            obs, _, done, _, _ = self.env.step(action)
            
            # 按空格键标记成功
            if self._is_space_pressed():
                done = True
        
        return {
            "observations": np.array(obs_list),
            "actions": np.array(action_list),
        }
    
    def save(self):
        """保存为 robomimic 兼容的 hdf5 格式"""
        with h5py.File(self.save_path, "w") as f:
            data_grp = f.create_group("data")
            for i, ep in enumerate(self.episodes):
                ep_grp = data_grp.create_group(f"demo_{i}")
                ep_grp.create_dataset("obs", data=ep["observations"])
                ep_grp.create_dataset("actions", data=ep["actions"])
                ep_grp.attrs["num_samples"] = len(ep["actions"])
            f.attrs["total"] = len(self.episodes)
```

### 3.2 Behavior Cloning 训练

```python
# src/electronbot_ai/il/train_bc.py

import torch
import torch.nn as nn

class BCPolicy(nn.Module):
    """Behavior Cloning 策略网络"""
    
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, act_dim),
        )
    
    def forward(self, obs):
        return self.net(obs)

def train_bc(demo_path: str, epochs: int = 100, batch_size: int = 64):
    # 加载数据
    with h5py.File(demo_path, "r") as f:
        all_obs = []
        all_act = []
        for key in f["data"]:
            all_obs.append(f[f"data/{key}/obs"][:])
            all_act.append(f[f"data/{key}/actions"][:])
        obs = torch.tensor(np.concatenate(all_obs), dtype=torch.float32)
        act = torch.tensor(np.concatenate(all_act), dtype=torch.float32)
    
    policy = BCPolicy(obs.shape[1], act.shape[1])
    optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)
    
    for epoch in range(epochs):
        idx = torch.randperm(len(obs))[:batch_size]
        pred = policy(obs[idx])
        loss = nn.MSELoss()(pred, act[idx])
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        if epoch % 10 == 0:
            print(f"Epoch {epoch}: loss={loss.item():.4f}")
    
    torch.save(policy.state_dict(), "checkpoints/bc_policy.pt")
    return policy

def evaluate_bc(policy, env, task, num_episodes: int = 40):
    """评估 BC 策略成功率"""
    successes = 0
    for _ in range(num_episodes):
        obs = task.reset(env)
        done = False
        while not done:
            with torch.no_grad():
                action = policy(torch.tensor(obs["obs_vector"]))
            obs, _, done, _, _ = env.step(action.numpy())
            if task.is_success():
                successes += 1
                break
    return successes / num_episodes
```

### 3.3 ACT（Action Chunking Transformer）

```python
# ACT 的核心思想：一次预测未来 K 个动作，减少累积误差
class ACTPolicy(nn.Module):
    """Action Chunking Transformer——比 BC 更强"""
    
    def __init__(self, obs_dim, act_dim, chunk_size=10, d_model=256):
        super().__init__()
        self.chunk_size = chunk_size
        self.obs_encoder = nn.Linear(obs_dim, d_model)
        
        # Transformer 编码器
        encoder_layer = nn.TransformerEncoderLayer(d_model, nhead=8, batch_first=True)
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=4)
        
        # 动作解码——输出 chunk_size × act_dim
        self.act_decoder = nn.Sequential(
            nn.Linear(d_model, d_model), nn.ReLU(),
            nn.Linear(d_model, chunk_size * act_dim),
        )
    
    def forward(self, obs_sequence):
        # obs_sequence: (B, T, obs_dim)
        x = self.obs_encoder(obs_sequence)
        x = self.transformer(x)
        # 取最后一个时间步的特征
        x = x[:, -1, :]
        act_chunk = self.act_decoder(x)
        return act_chunk.reshape(-1, self.chunk_size, 6)
```

---

## 4. 强化学习管线

### 4.1 并行训练环境

```python
# src/electronbot_ai/rl/parallel_env.py

from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env(rank: int, task_name: str, render: bool = False):
    def _init():
        env = ElectronBotEnv(render_mode="human" if render else None)
        task = create_task(task_name)
        env = TaskWrapper(env, task)
        return env
    return _init

# 64 并行环境
envs = SubprocVecEnv([make_env(i, "reach") for i in range(64)])
```

### 4.2 PPO 训练

```python
# src/electronbot_ai/rl/train_ppo.py

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

def train_ppo(task_name: str, num_envs: int = 64, total_steps: int = 1_000_000):
    env = SubprocVecEnv([make_env(i, task_name) for i in range(num_envs)])
    eval_env = SubprocVecEnv([make_env(0, task_name) for _ in range(4)])
    
    model = PPO(
        "MlpPolicy",
        env,
        n_steps=2048,
        batch_size=512,
        n_epochs=10,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        verbose=1,
        tensorboard_log="./logs/",
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./checkpoints/",
        eval_freq=10000,
        n_eval_episodes=20,
    )
    
    model.learn(total_timesteps=total_steps, callback=eval_callback)
    model.save(f"checkpoints/ppo_{task_name}")
    
    return model
```

### 4.3 域随机化 training wrapper

```python
# src/electronbot_ai/rl/domain_randomization.py

class DomainRandomizationWrapper(gym.Wrapper):
    """每次 reset 时随机化物理参数"""
    
    def reset(self, **kwargs):
        obs = super().reset(**kwargs)
        self._randomize_domain()
        return obs
    
    def _randomize_domain(self):
        env = self.unwrapped
        model = env.model
        rng = np.random.default_rng()
        
        # 关节阻尼随机化 ±30%
        for i in range(model.njnt):
            model.dof_damping[i] *= rng.uniform(0.7, 1.3)
        
        # 执行器增益随机化 ±15%
        for i in range(model.nu):
            model.actuator_gainprm[i, 0] *= rng.uniform(0.85, 1.15)
        
        # 物体质量随机化 ±20%
        for i in range(model.nbody):
            model.body_mass[i] *= rng.uniform(0.8, 1.2)
        
        # 摄像头延迟模拟
        self.delay_steps = rng.integers(0, 3)  # 0-2 帧延迟
        
        # 观测噪声
        self.joint_sensor.pos_noise_std = rng.uniform(0, 0.5)
        
        # === 以下对齐 Phase 2 MuJoCo Env 域随机化参数 ===
        
        # 伺服死区随机化 (对齐 firmware: SG90 deadband ~2-5°)
        # 仿真中表现为: 目标角度变化 < deadband 时舵机不响应
        self.servo_deadband = {
            "right_pitch": rng.uniform(2.0, 5.0),
            "right_roll":  rng.uniform(2.0, 5.0),
            "left_pitch":  rng.uniform(2.0, 5.0),
            "left_roll":   rng.uniform(2.0, 5.0),
            "body":        rng.uniform(2.0, 5.0),
            "head":        rng.uniform(2.0, 5.0),
        }
        
        # 电池电压效应 (3.5V-4.2V LiPo)
        # 电压低时舵机扭矩下降，响应变慢
        self.battery_voltage = rng.uniform(3.5, 4.2)
        # 电压→扭矩缩放因子: 3.5V=0.7x, 3.8V=0.85x, 4.2V=1.0x
        voltage_scale = max(0.5, min(1.0, (self.battery_voltage - 3.3) / 0.9))
        
        # 将电压效应叠加到执行器增益
        for i in range(model.nu):
            model.actuator_gainprm[i, 0] *= voltage_scale
        
        # 观测模式: "full"(仿真专属) 或 "realistic"(Sim2Real对齐)
        # realistic 模式下观测空间仅包含真机可获取的数据
        self.obs_mode = "full"  # 训练时默认 full，Sim2Real 测试时切换 realistic
```

---

## 5. VLA 决策管线

### 5.1 VLA 模式分类

> **⚠️ 关键区分**: 
> - **视觉 VLA**（需要摄像头图像）：仿真专属，真机 ElectronBot 无摄像头硬件
> - **纯文本 VLA**（仅语音指令）：真机可用，是当前最可行的 Sim2Real 路径

| 模式 | 输入 | 仿真 | 真机 | 推荐 |
|------|------|:---:|:---:|------|
| 视觉 VLA | 语音 + 摄像头图像 | ✅ | ❌ 无硬件 | 仿真研究用 |
| **纯文本 VLA** | 仅语音指令 | ✅ | ✅ | **推荐，Sim2Real 首选** |

### 5.2 纯文本 VLA 实现（推荐）

```python
# src/electronbot_ai/vla/llm_planner.py

class TextVLAPlanner:
    """纯文本 VLA 规划器——语音指令 → MCP 动作序列
    
    真机可用！这是当前最可行的 Sim2Real 路径。
    延迟: 200-500ms RTT (云端 API)，对预设动作序列可接受。
    """
    
    def __init__(self, llm_model: str = "qwen2.5-7b"):
        self.llm = self._load_llm(llm_model)
        self.action_interface = ElectronBotActionsInterface()
    
    def plan(self, instruction: str) -> dict:
        """
        输入：用户语音指令（纯文本）
        输出：MCP 动作序列（仅使用 8 个预设动作工具，真机可用）
        
        ⚠️ 仅输出 hand_action / body_turn / head_move 等预设动作，
           不使用 servo_move / servo_sequences（@sim_only）
        """
        prompt = self._build_prompt(instruction)
        response = self.llm.generate(prompt)
        sequence = self._parse_response(response)
        return sequence
    
    def _build_prompt(self, instruction: str) -> str:
        return f"""
你控制一个名为 ElectronBot 的桌面机器人。它有两个手臂、一个可旋转的头部和一个可旋转的身体。

可用 MCP 工具（真机可用）：
- self.electron.hand_action: action(1=举手,2=放手,3=挥手,4=拍打), hand(1=左,2=右,3=双), steps, speed, amount
- self.electron.body_turn: direction(1=左转,2=右转,3=回中), steps, speed, angle(0-90°)
- self.electron.head_move: action(1=抬头,2=低头,3=点头,4=回中,5=连续点头), steps, speed, angle(1-15°)
- self.electron.stop: 紧急停止

请根据用户指令"{instruction}"，生成一个 MCP 动作序列 JSON：
{{
  "actions": [
    {{"tool": "self.electron.hand_action", "args": {{"action": 3, "hand": 3, "steps": 2, "speed": 600}}}},
    ...
  ]
}}
只输出 JSON，不要其他文字。不要使用 servo_move 或 servo_sequences。
"""
    
    def _parse_response(self, response: str) -> dict:
        """解析 LLM 输出为可执行的序列"""
        import re, json
        json_match = re.search(r'\{[\s\S]*\}', response)
        return json.loads(json_match.group())
    
    def execute(self, instruction: str):
        """端到端：语音指令 → 生成动作 → 执行（云端 API，延迟 200-500ms）"""
        sequence = self.plan(instruction)
        for action in sequence.get("actions", []):
            self.action_interface.call(action["tool"], action["args"])
```

### 5.3 视觉 VLA 实现（仿真专属）

```python
# 仅用于仿真研究，真机不可用

class VisionVLAPlanner:
    """视觉 VLA 规划器——需要摄像头图像
    
    ⚠️ 仿真专属：真机 ElectronBot 无摄像头硬件
    用于仿真中的复杂场景理解和操作规划研究
    """
    
    def __init__(self, llm_model: str = "qwen2.5-vl"):
        self.llm = self._load_llm(llm_model)
        self.action_interface = ElectronBotActionsInterface()
    
    def plan(self, instruction: str, camera_image: np.ndarray) -> dict:
        """
        输入：用户语音指令 + 当前摄像头图像
        输出：MCP 动作序列
        
        ⚠️ 可以使用 servo_move / servo_sequences（仿真专属），
           因为仿真环境有这些工具
        """
        prompt = self._build_prompt(instruction, camera_image)
        response = self.llm.generate(prompt)
        sequence = self._parse_response(response)
        return sequence
    
    def _build_prompt(self, instruction: str, image: np.ndarray) -> str:
        return f"""
你控制一个名为 ElectronBot 的桌面机器人。它有两个手臂、一个可旋转的头部和一个可旋转的身体。

可用舵机（及其短键和安全范围）：
- right_pitch (rp): 0-180, 右臂上下摆动
- right_roll  (rr): 100-180, 右臂前后推拉
- left_pitch  (lp): 0-180, 左臂上下摆动
- left_roll   (lr): 0-80, 左臂前后推拉
- body (b): 30-150, 腰部旋转
- head (h): 75-105, 头部俯仰

请根据摄像头图像和用户指令"{instruction}"，
生成一个 servo_sequence JSON（可以使用 servo_move 精确控制）：
{{
  "a": [
    {{"s": {{"servo": angle, ...}}, "v": time_ms}},
    ...
  ]
}}
只输出 JSON，不要其他文字。
"""
    
    def _parse_response(self, response: str) -> dict:
        """解析 LLM 输出为可执行的序列"""
        import re, json
        json_match = re.search(r'\{[\s\S]*\}', response)
        return json.loads(json_match.group())
    
    def execute(self, instruction: str):
        """端到端：语音指令 → 生成动作 → 执行（仅仿真）"""
        cam = self.env.get_camera()
        sequence = self.plan(instruction, cam.capture()[0])
        self.action_interface.execute_sequence(sequence)
```

---

## 6. 验证方法

### 6.1 IL 验证

```
□ 收集 50 条 reach 任务示范轨迹 → demo_reach.hdf5
□ 训练 BC 策略 → checkpoints/bc_reach.pt
□ 评估 BC 策略：
    ${python -m electronbot_ai.il.evaluate --task reach}
    目标成功率: > 70%
□ 训练 ACT 策略（50 条示范数据）
    目标成功率: > 85%
```

### 6.2 RL 验证

```
□ PPO 训练 reach 任务 100万步
    目标: 平均 reward > 0, 成功率 > 90%
□ 域随机化训练后重新评估
    标准环境成功率 > 80%（域随机化会降低裸成功率，但提高泛化）
□ 64 并行环境：GPU 利用率 > 80%
```

### 6.3 Realistic 观测模式验证 (Sim2Real 对齐)

```
□ 切换 obs_mode="realistic" 后，观测空间仅包含真机可获取数据:
    - commanded_joint_pos (指令角度，非编码器反馈)
    - is_moving (动作执行中标记)
    - battery_voltage / battery_percent (电池状态)
    不再包含: joint_vel, ee_positions (仿真专属)

□ 在 realistic 模式下训练 BC 策略
    评估: realistic 模式成功率应接近 full 模式 (差距 < 10%)

□ 域随机化参数验证:
    - servo_deadband ∈ [2°, 5°] → 仿真中确认小角度指令被忽略
    - battery_voltage ∈ [3.5V, 4.2V] → 低电压时舵机响应变慢
    - 验证电压效应正确叠加到 actuator_gainprm
```

### 6.4 VLA 验证

```
□ 指令: "举起右手" → LLM输出: {"a":[{"s":{"rp":0},"v":1000}]}
    执行后右手举起 ✓
□ 指令: "挥手打招呼" → LLM输出包含振荡动作
    执行后机器人挥手 ✓
□ 指令: "转过来看我" → LLM输出 body 旋转序列
    执行后腰部旋转 ✓
```

---

## 7. 交付物清单

| 文件 | 描述 |
|------|------|
| `src/electronbot_ai/il/collect_demos.py` | 示范数据收集工具 |
| `src/electronbot_ai/il/train_bc.py` | BC 训练脚本 |
| `src/electronbot_ai/il/train_act.py` | ACT 训练脚本 |
| `src/electronbot_ai/rl/train_ppo.py` | PPO 训练脚本 |
| `src/electronbot_ai/rl/domain_randomization.py` | 域随机化 wrapper |
| `src/electronbot_ai/vla/llm_planner.py` | VLA 规划器 |
| `src/electronbot_ai/tasks/` | 6 个标准任务定义 |
| `demos/` | 示范数据存储目录 |
| `checkpoints/` | 模型权重存储目录 |

---

## 8. 接口设计

### 8.1 模块对外接口

#### 8.1.1 BaseTask 抽象接口

所有训练任务继承自 `BaseTask`，定义统一的任务生命周期接口：

```python
class BaseTask(ABC):
    @abstractmethod
    def reset(self, env) -> dict:
        """重置任务——设置场景、放置物体，返回初始观测"""
    
    @abstractmethod
    def get_observation(self) -> dict:
        """获取任务相关的观测字典"""
    
    @abstractmethod
    def compute_reward(self) -> float:
        """计算当前步的奖励值——RL 训练用"""
    
    @abstractmethod
    def is_success(self) -> bool:
        """判断任务是否成功——评估用"""
    
    @abstractmethod
    def get_demo_action(self, keyboard_state) -> np.ndarray:
        """从键盘输入获取示范动作——IL 收集数据用，返回 shape=(6,) 的关节增量"""
```

- **职责**：封装任务逻辑（场景初始化、观测构造、奖励计算、成功判定、示范采集）
- **入参 `env`**：`ElectronBotEnv` 实例，任务通过它驱动仿真
- **入参 `keyboard_state`**：键盘状态字典，如 `{"w": True, "s": False, ...}`
- **返回值约束**：
  - `reset` → `dict`：观测字典，至少包含 `joint_pos`、`ee_pos`、`target_pos`、`dist_to_target`
  - `get_observation` → `dict`：同上
  - `compute_reward` → `float`：标量奖励
  - `is_success` → `bool`：成功标志
  - `get_demo_action` → `np.ndarray`：shape=(6,)，dtype=float32，表示 6 个关节的增量

#### 8.1.2 7 个任务实现类

| 任务名 | 实现类 | 文件路径 | 真机对齐 | 算法适用性 |
|--------|--------|---------|:---:|------|
| EB-Reach | `ReachTask` | `src/electronbot_ai/tasks/reach.py` | ✅ | BC/ACT/PPO |
| EB-Push | `PushTask` | `src/electronbot_ai/tasks/push.py` | ✅ | BC/ACT/PPO |
| EB-PickPlace | `PickPlaceTask` | `src/electronbot_ai/tasks/pick_place.py` | ❌ 仿真专属 | BC/ACT/PPO |
| EB-Stack | `StackTask` | `src/electronbot_ai/tasks/stack.py` | ❌ 仿真专属 | BC/ACT/PPO |
| EB-Follow | `FollowTask` | `src/electronbot_ai/tasks/follow.py` | ✅ | BC/ACT/PPO |
| EB-Gesture | `GestureTask` | `src/electronbot_ai/tasks/gesture.py` | ✅ | BC/ACT/PPO |
| EB-VoiceCmd | `VoiceCmdTask` | `src/electronbot_ai/tasks/voice_cmd.py` | ✅ | VLA |

> **注**: `tasks/` 目录在交付物清单中标注为"6 个标准任务定义"，实际包含 7 个任务文件（EB-VoiceCmd 单独成文件）。文档统一以 7 任务为准。

#### 8.1.3 策略网络接口

**BCPolicy（Behavior Cloning）**：

```python
class BCPolicy(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int): ...
    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """入参 (B, obs_dim) → 出参 (B, act_dim)"""
```

- **输入**：观测向量，shape=(batch, obs_dim)
- **输出**：关节增量，shape=(batch, 6)，dtype=float32
- **网络结构**：4 层 MLP（256-256-256-act_dim），ReLU 激活

**ACTPolicy（Action Chunking Transformer）**：

```python
class ACTPolicy(nn.Module):
    def __init__(self, obs_dim, act_dim, chunk_size=10, d_model=256): ...
    def forward(self, obs_sequence: torch.Tensor) -> torch.Tensor:
        """入参 (B, T, obs_dim) → 出参 (B, chunk_size, act_dim)"""
```

- **输入**：观测序列，shape=(batch, T, obs_dim)
- **输出**：动作块，shape=(batch, chunk_size, 6)，一次预测未来 10 步动作
- **网络结构**：Linear 编码器 + TransformerEncoder（4 层，8 头）+ Linear 解码器

#### 8.1.4 VLA 规划器接口

**TextVLAPlanner（纯文本 VLA，推荐 Sim2Real 路径）**：

```python
class TextVLAPlanner:
    def __init__(self, llm_model: str = "qwen2.5-7b"): ...
    def plan(self, instruction: str) -> dict:
        """语音指令 → MCP 动作序列（仅使用 8 个真机可用工具）"""
    def execute(self, instruction: str):
        """端到端：语音指令 → 生成动作 → 执行（云端 API，延迟 200-500ms）"""
```

- **入参 `instruction`**：用户语音转写的纯文本指令
- **返回值**：`{"actions": [{"tool": str, "args": dict}, ...]}`
- **工具限制**：仅输出 `hand_action`/`body_turn`/`head_move`/`stop`，禁用 `sim_only` 工具

**VisionVLAPlanner（视觉 VLA，仿真专属）**：

```python
class VisionVLAPlanner:
    def __init__(self, llm_model: str = "qwen2.5-vl"): ...
    def plan(self, instruction: str, camera_image: np.ndarray) -> dict:
        """语音指令 + 摄像头图像 → servo_sequence（可使用 sim_only 工具）"""
    def execute(self, instruction: str):
        """端到端：仅仿真执行"""
```

- **入参 `camera_image`**：摄像头图像帧，np.ndarray
- **返回值**：`{"a": [{"s": {...}, "v": int}, ...]}`（servo_sequence 格式）
- **工具范围**：可使用 `servo_move`/`servo_sequences`（仿真专属）

#### 8.1.5 PPO 训练 CLI

```bash
python -m electronbot_ai.rl.train_ppo \
    --task <task_name> \
    --num_envs <int> \
    --total_steps <int> \
    --output <path>
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--task` | str | 必填 | 任务名：reach/push/pick_place/stack/follow/gesture |
| `--num_envs` | int | 64 | 并行环境数 |
| `--total_steps` | int | 1_000_000 | 总训练步数 |
| `--output` | str | `checkpoints/ppo_{task}.zip` | 模型权重输出路径 |
| `--eval_freq` | int | 10000 | 评估频率（步） |
| `--n_eval_episodes` | int | 20 | 每次评估回合数 |
| `--tensorboard_log` | str | `./logs/` | TensorBoard 日志目录 |

### 8.2 输入输出契约

#### 8.2.1 训练任务 × 算法适用性矩阵

| 任务 | BC | ACT | PPO | VLA（纯文本） | VLA（视觉） |
|------|:---:|:---:|:---:|:---:|:---:|
| EB-Reach | ✅ | ✅ | ✅ | ❌ | ✅ |
| EB-Push | ✅ | ✅ | ✅ | ❌ | ✅ |
| EB-PickPlace | ✅ | ✅ | ✅ | ❌ | ✅ |
| EB-Stack | ✅ | ✅ | ✅ | ❌ | ✅ |
| EB-Follow | ✅ | ✅ | ✅ | ❌ | ✅ |
| EB-Gesture | ✅ | ✅ | ⚠️ 奖励稀疏 | ✅ | ✅ |
| EB-VoiceCmd | ❌ | ❌ | ❌ | ✅ | ✅ |

> **说明**：
> - BC/ACT 需要示范数据，适用于所有有示范的任务
> - PPO 适用于奖励稠密的任务；EB-Gesture 奖励稀疏，PPO 训练难度高
> - EB-VoiceCmd 本质是语言理解任务，仅 VLA 路径适用
> - 视觉 VLA 在所有任务中仿真可用，但真机无摄像头硬件

#### 8.2.2 观测空间契约

| 观测字段 | 类型 | shape | 说明 | obs_mode |
|----------|------|-------|------|---------|
| `joint_pos` | float32 | (6,) | 6 个关节角度（度） | full / realistic |
| `ee_pos` | float32 | (3,) | 末端执行器位置 | full |
| `target_pos` | float32 | (3,) | 目标位置 | full / realistic |
| `dist_to_target` | float32 | () | 到目标距离 | full / realistic |
| `commanded_joint_pos` | float32 | (6,) | 指令角度（非编码器反馈） | realistic |
| `is_moving` | bool | () | 动作执行中标记 | realistic |
| `battery_voltage` | float32 | () | 电池电压 | realistic |
| `battery_percent` | float32 | () | 电池百分比 | realistic |
| `joint_vel` | float32 | (6,) | 关节速度 | full（仿真专属） |

#### 8.2.3 动作空间契约

- **类型**：`np.ndarray`，shape=(6,)，dtype=float32
- **语义**：6 个关节的增量（度），范围 [-2.0, 2.0]
- **顺序**：`[right_pitch, right_roll, left_pitch, left_roll, body, head]`
- **执行**：`env.step(action)` 将增量叠加到当前关节角度，驱动 MuJoCo 仿真

---

## 9. 数据模型

### 9.1 核心数据结构

#### 9.1.1 示范数据 HDF5 格式

```hdf5
demo_<task>.hdf5
├── attrs: {"total": <int>}              # 示范轨迹总数
└── data/
    ├── demo_0/
    │   ├── obs         (T, obs_dim)     # 观测序列
    │   ├── actions     (T, 6)           # 动作序列
    │   └── attrs: {"num_samples": T}
    ├── demo_1/
    │   ├── obs
    │   ├── actions
    │   └── attrs: {"num_samples": T}
    └── ... 
```

- **兼容性**：与 robomimic HDF5 格式对齐，便于复用现有工具链
- **`obs` 字段**：将 `get_observation()` 返回的字典展平为向量存储
- **`actions` 字段**：每步的 `get_demo_action()` 返回值
- **推荐规模**：每个任务 50 条示范轨迹，每条 100-300 步

#### 9.1.2 策略权重文件格式

| 算法 | 格式 | 后缀 | 加载方式 | 部署目标 |
|------|------|------|---------|---------|
| BC | PyTorch state_dict | `.pt` | `torch.load(path)` | 仿真/真机本地推理 |
| ACT | PyTorch state_dict | `.pt` | `torch.load(path)` | 仿真/真机本地推理 |
| PPO | Stable-Baselines3 | `.zip` | `PPO.load(path)` | 仿真/真机本地推理 |
| 任意 | ONNX | `.onnx` | `onnxruntime.InferenceSession` | 跨平台部署（推荐真机） |

**权重文件命名规范**：
```
checkpoints/
├── bc_<task>.pt                 # BC 最佳模型
├── act_<task>.pt                # ACT 最佳模型
├── ppo_<task>.zip               # PPO 最佳模型
├── ppo_<task>_latest.zip        # PPO 最新检查点
├── ppo_<task>.onnx              # PPO 导出的 ONNX 模型
└── ...
```

#### 9.1.3 训练配置结构

```python
# 训练配置（YAML/JSON）
{
    "task": "reach",                    # 任务名
    "algorithm": "ppo",                 # 算法：bc/act/ppo
    "hyperparams": {
        "n_steps": 2048,
        "batch_size": 512,
        "learning_rate": 3e-4,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip_range": 0.2
    },
    "env": {
        "num_envs": 64,
        "render_mode": null,
        "obs_mode": "full"              # full / realistic
    },
    "domain_randomization": {
        "enabled": true,
        "joint_damping_range": [0.7, 1.3],
        "actuator_gain_range": [0.85, 1.15],
        "body_mass_range": [0.8, 1.2],
        "camera_delay_steps": [0, 3],
        "joint_pos_noise_std": [0, 0.5],
        "servo_deadband_range": [2.0, 5.0],
        "battery_voltage_range": [3.5, 4.2]
    },
    "training": {
        "total_steps": 1000000,
        "eval_freq": 10000,
        "n_eval_episodes": 20,
        "checkpoint_dir": "checkpoints/",
        "tensorboard_log": "logs/"
    }
}
```

#### 9.1.4 VLA 动作序列结构

**TextVLAPlanner 输出**（预设动作序列）：
```json
{
    "actions": [
        {"tool": "self.electron.hand_action", "args": {"action": 3, "hand": 3, "steps": 2, "speed": 600}},
        {"tool": "self.electron.body_turn", "args": {"direction": 1, "angle": 30}}
    ]
}
```

**VisionVLAPlanner 输出**（servo_sequence 格式）：
```json
{
    "a": [
        {"s": {"rp": 90, "lp": 90}, "v": 500},
        {"osc": {"a": {"rp": 20}, "o": {"rp": 120}, "p": 300, "c": 2}}
    ]
}
```

### 9.2 数据流

#### 9.2.1 模仿学习数据流

```
[键盘输入] → [DemoCollector._get_keyboard_action]
   │
   │  action: np.ndarray, shape=(6,)
   ▼
[env.step(action)] → obs, reward, done, info
   │
   │  累积 obs_list, action_list
   ▼
[DemoCollector.save] → demo_<task>.hdf5
   │
   │  HDF5: data/demo_{i}/{obs, actions}
   ▼
[train_bc] → 加载 HDF5 → 拼接 obs/act 张量
   │
   │  BCPolicy(obs) → pred_act
   │  MSELoss(pred_act, act)
   ▼
[torch.save] → checkpoints/bc_<task>.pt
   │
   ▼
[evaluate_bc] → 加载权重 → env.step(policy(obs)) → 统计成功率
```

#### 9.2.2 强化学习数据流

```
[SubprocVecEnv(64 并行)] 
   │
   │  每个 env: DomainRandomizationWrapper(TaskWrapper(ElectronBotEnv))
   │  reset 时随机化物理参数
   ▼
[PPO policy] → 采样动作
   │
   │  n_steps=2048 步 rollout → buffer
   ▼
[PPO update] → 计算 GAE → clip surrogate loss → 更新网络
   │
   │  每 10000 步触发 EvalCallback
   ▼
[EvalCallback] → 评估 20 回合 → 保存 best_model + latest
   │
   ▼
[checkpoints/ppo_<task>.zip] + [logs/ TensorBoard]
```

#### 9.2.3 VLA 决策数据流

```
[用户语音] → [ASR] → instruction: str
   │
   ▼
[TextVLAPlanner._build_prompt(instruction)]
   │
   │  构造 LLM prompt（含可用工具说明）
   ▼
[LLM.generate(prompt)] → response: str
   │
   ▼
[_parse_response(response)] → sequence: dict
   │
   │  正则提取 JSON → json.loads
   ▼
[ElectronBotActionsInterface.call(tool, args)]
   │
   │  ElectronBotBackend.call(method, params)
   │  ├── sim: McpSimBridge 即时执行
   │  └── cloud: 云端 API 透传（200-500ms RTT）
   ▼
[机器人执行动作]
```

---

## 10. 错误处理与恢复

### 10.1 错误分类

| 错误类型 | 触发场景 | 影响范围 | 处理策略 |
|---------|---------|---------|---------|
| 训练 NaN loss | 学习率过大、奖励尺度异常、梯度爆炸 | 训练崩溃 | 早停 + 降低学习率至 1/10 + 恢复最近检查点 |
| GPU OOM | `batch_size`/`num_envs` 过大、模型过大、显存碎片 | 训练中断 | 自动减半 `batch_size`/`num_envs` 重试；清理 CUDA 缓存 |
| 示范数据质量差 | 示范轨迹不完整、动作抖动、成功率低 | BC/ACT 过拟合或欠拟合 | 数据过滤（剔除成功率 < 50% 的轨迹）+ 重新收集 |
| LLM 输出解析失败 | LLM 返回非 JSON、JSON 结构错误、工具名不存在 | VLA 决策中断 | 重试 3 次 + fallback 到预设安全动作（如 `stop`） |
| 域随机化过强 | 随机化范围超出物理合理区间 | 训练不收敛 | 监测 reward 曲线，连续 50k 步无提升则缩小随机化范围 |
| MuJoCo 步进失败 | 关节角度超限、物理状态异常 | 仿真崩溃 | `try/except` 捕获 + `env.reset()` 恢复 + 记录异常轨迹 |
| 检查点加载失败 | 文件损坏、版本不兼容、state_dict 键不匹配 | 评估/部署失败 | 校验文件完整性；提供 `strict=False` 加载选项 |
| 示范数据 HDF5 损坏 | 写入中断、磁盘空间不足 | 数据丢失 | 原子写入（临时文件 + rename）；定期备份 |
| 域随机化参数越界 | 配置错误、代码 bug | Sim2Real 性能下降 | 启动时校验参数范围；越界则拒绝启动 |
| PPO 奖励 hacking | 奖励设计漏洞导致策略钻空子 | 策略无效 | 监测异常 reward 峰值；人工审查奖励函数 |

### 10.2 异常恢复流程

#### 10.2.1 训练 NaN loss 恢复

```python
# train_bc / train_ppo 中的 NaN 检测
if torch.isnan(loss) or torch.isinf(loss):
    logger.warning(f"检测到 NaN/Inf loss: {loss.item()}")
    
    # 1. 早停当前 epoch
    if nan_count >= 3:
        logger.error("连续 3 次 NaN loss，触发早停")
        
        # 2. 降低学习率
        for pg in optimizer.param_groups:
            pg["lr"] *= 0.1
        logger.info(f"学习率降至 {optimizer.param_groups[0]['lr']}")
        
        # 3. 恢复最近检查点
        policy.load_state_dict(torch.load("checkpoints/latest.pt"))
        logger.info("已恢复最近检查点")
        
        # 4. 若降学习率后仍 NaN，终止训练
        if nan_count >= 6:
            raise RuntimeError("训练无法收敛，终止")
```

#### 10.2.2 GPU OOM 恢复

```python
try:
    loss.backward()
    optimizer.step()
except torch.cuda.OutOfMemoryError:
    torch.cuda.empty_cache()
    
    # 自动减半 batch_size
    batch_size = max(8, batch_size // 2)
    logger.warning(f"GPU OOM，batch_size 降至 {batch_size}")
    
    # PPO 场景：减少并行环境数
    if algorithm == "ppo":
        num_envs = max(8, num_envs // 2)
        logger.warning(f"num_envs 降至 {num_envs}")
```

#### 10.2.3 示范数据质量恢复

```
1. 加载 demo_<task>.hdf5
2. 对每条轨迹计算质量指标：
   - 轨迹长度（过短可能是误操作）
   - 末端抖动幅度（过大可能是随机按键）
   - 是否达到成功条件
3. 过滤规则：
   - 剔除长度 < 10 步的轨迹
   - 剔除末端抖动 > 90° 的轨迹
   - 剔除未达成功条件的轨迹（若 task.is_success 可判定）
4. 若过滤后剩余轨迹 < 20 条 → 提示重新收集
5. 记录过滤日志（剔除数量、原因分布）
```

#### 10.2.4 LLM 输出解析失败恢复

```python
def _parse_response_with_retry(self, response: str, max_retries: int = 3) -> dict:
    for attempt in range(max_retries):
        try:
            json_match = re.search(r'\{[\s\S]*\}', response)
            if not json_match:
                raise ValueError("未找到 JSON 块")
            sequence = json.loads(json_match.group())
            
            # 校验工具名合法性
            for action in sequence.get("actions", []):
                if action["tool"] not in ALLOWED_TOOLS:
                    raise ValueError(f"非法工具名: {action['tool']}")
            return sequence
        except (json.JSONDecodeError, ValueError) as e:
            logger.warning(f"LLM 输出解析失败（第 {attempt+1} 次）: {e}")
            if attempt < max_retries - 1:
                # 重新请求 LLM，附加纠错提示
                response = self.llm.generate(self._build_retry_prompt(response, str(e)))
            else:
                # fallback：返回 stop 动作
                logger.error("LLM 输出解析全部失败，fallback 到 stop")
                return {"actions": [{"tool": "self.electron.stop", "args": {}}]}
```

#### 10.2.5 域随机化过强恢复

```
1. 监测指标：滑动窗口 50000 步的平均 reward
2. 若连续 50000 步平均 reward 无提升（或下降）：
   a. 记录当前域随机化参数
   b. 缩小随机化范围至原来的 80%
      - joint_damping_range: [0.7, 1.3] → [0.76, 1.24]
      - actuator_gain_range: [0.85, 1.15] → [0.88, 1.12]
      - body_mass_range: [0.8, 1.2] → [0.84, 1.16]
   c. 从最近检查点恢复训练
   d. 若连续 2 次缩小后仍不收敛 → 停止训练，人工介入
3. 训练日志记录每次参数调整
```

---

## 11. 配置管理

### 11.1 配置参数表

#### 11.1.1 BC（Behavior Cloning）参数

| 参数 | 默认值 | 类型 | 说明 |
|------|--------|------|------|
| `lr` | `1e-3` | float | 学习率 |
| `epochs` | `100` | int | 训练轮数 |
| `batch_size` | `64` | int | 批大小 |
| `hidden_dim` | `256` | int | 隐藏层维度 |
| `num_layers` | `4` | int | MLP 层数 |
| `optimizer` | `Adam` | str | 优化器 |
| `loss_fn` | `MSE` | str | 损失函数 |

#### 11.1.2 ACT（Action Chunking Transformer）参数

| 参数 | 默认值 | 类型 | 说明 |
|------|--------|------|------|
| `chunk_size` | `10` | int | 动作块大小（一次预测未来 10 步） |
| `d_model` | `256` | int | Transformer 隐藏维度 |
| `nhead` | `8` | int | 多头注意力头数 |
| `num_layers` | `4` | int | Transformer 编码器层数 |
| `lr` | `1e-4` | float | 学习率（比 BC 小） |
| `epochs` | `200` | int | 训练轮数 |
| `batch_size` | `32` | int | 批大小 |

#### 11.1.3 PPO 参数

| 参数 | 默认值 | 类型 | 说明 |
|------|--------|------|------|
| `n_steps` | `2048` | int | 每次 rollout 步数 |
| `batch_size` | `512` | int | mini-batch 大小 |
| `n_epochs` | `10` | int | 每次更新的 epoch 数 |
| `lr` | `3e-4` | float | 学习率 |
| `gamma` | `0.99` | float | 折扣因子 |
| `gae_lambda` | `0.95` | float | GAE 参数 |
| `clip_range` | `0.2` | float | PPO clip 范围 |
| `ent_coef` | `0.01` | float | 熵正则系数 |
| `vf_coef` | `0.5` | float | 价值函数损失系数 |
| `max_grad_norm` | `0.5` | float | 梯度裁剪 |
| `policy` | `MlpPolicy` | str | 策略网络类型 |

#### 11.1.4 域随机化参数范围表

| 参数 | 范围 | 默认值 | 说明 |
|------|------|--------|------|
| `joint_damping_range` | [0.7, 1.3] | ±30% | 关节阻尼随机化倍数 |
| `actuator_gain_range` | [0.85, 1.15] | ±15% | 执行器增益随机化倍数 |
| `body_mass_range` | [0.8, 1.2] | ±20% | 物体质量随机化倍数 |
| `camera_delay_steps` | [0, 3) | 0-2 帧 | 摄像头延迟帧数 |
| `joint_pos_noise_std` | [0, 0.5] | 0-0.5° | 关节位置观测噪声标准差 |
| `servo_deadband_range` | [2.0, 5.0] | 2-5° | 舵机死区（对齐 SG90 固件） |
| `battery_voltage_range` | [3.5, 4.2] | 3.5-4.2V | 电池电压（LiPo） |

#### 11.1.5 训练硬件要求

| 资源 | 最低要求 | 推荐配置 | 说明 |
|------|---------|---------|------|
| GPU | RTX 2060 12GB | RTX 3090 24GB | 训练速度与 batch_size 上限 |
| CPU | 8 核 | 16 核 | SubprocVecEnv 并行环境 |
| 内存 | 32GB | 64GB | 64 并行环境 + 示范数据 |
| 磁盘 | 50GB | 200GB SSD | 检查点 + 日志 + 示范数据 |
| CUDA | 11.7+ | 12.1+ | PyTorch 兼容版本 |
| 并行环境数 | 16 | 64 | PPO 训练效率关键参数 |

### 11.2 环境变量

| 变量名 | 用途 | 默认值 | 示例 |
|--------|------|--------|------|
| `ELECTRONBOT_TASK` | 默认任务名 | 无 | `reach` |
| `ELECTRONBOT_ALGORITHM` | 默认算法 | 无 | `ppo` |
| `ELECTRONBOT_NUM_ENVS` | 默认并行环境数 | `64` | `32` |
| `ELECTRONBOT_TOTAL_STEPS` | 默认训练总步数 | `1000000` | `500000` |
| `ELECTRONBOT_CKPT_DIR` | 检查点目录 | `checkpoints/` | `/data/ckpt/` |
| `ELECTRONBOT_LOG_DIR` | 日志目录 | `logs/` | `/data/logs/` |
| `ELECTRONBOT_DEMO_DIR` | 示范数据目录 | `demos/` | `/data/demos/` |
| `ELECTRONBOT_OBS_MODE` | 观测模式 | `full` | `realistic` |
| `ELECTRONBOT_DEVICE` | 训练设备 | `cuda` | `cpu` |
| `ELECTRONBOT_LLM_MODEL` | VLA 使用的 LLM 模型 | `qwen2.5-7b` | `qwen2.5-vl` |
| `ELECTRONBOT_DR_ENABLED` | 是否启用域随机化 | `true` | `false` |
| `ELECTRONBOT_TENSORBOARD_LOG` | TensorBoard 日志目录 | `logs/` | `/data/tb/` |

**加载优先级**（从高到低）：
1. CLI 参数：`python -m electronbot_ai.rl.train_ppo --task reach --num_envs 32`
2. 训练配置文件：`config/ppo_reach.yaml`
3. 环境变量：`ELECTRONBOT_NUM_ENVS`
4. 代码内默认值

---

## 12. 日志与可观测性

### 12.1 日志规范

#### 12.1.1 日志格式

采用结构化日志（JSON Lines）：

```json
{"ts":"2026-07-04T10:23:45.123Z","level":"INFO","module":"train_ppo","task":"reach","step":50000,"reward_mean":12.3,"loss":0.045,"success_rate":0.85,"duration_ms":1200}
```

#### 12.1.2 日志级别

| 级别 | 使用场景 |
|------|---------|
| `DEBUG` | 单步 reward、梯度范数、中间张量统计 |
| `INFO` | 训练 epoch 开始/结束、检查点保存、评估结果 |
| `WARNING` | NaN loss 检测、GPU 显存 > 90%、域随机化参数调整 |
| `ERROR` | 训练崩溃、检查点加载失败、LLM 输出解析失败 |
| `CRITICAL` | GPU OOM、数据损坏、需要人工介入 |

#### 12.1.3 关键日志字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `task` | str | 任务名 |
| `algorithm` | str | 算法（bc/act/ppo/vla） |
| `step` | int | 训练步数 |
| `epoch` | int | 训练轮数（BC/ACT） |
| `reward_mean` | float | 平均奖励 |
| `reward_std` | float | 奖励标准差 |
| `loss` | float | 损失值 |
| `success_rate` | float | 成功率 |
| `lr` | float | 当前学习率 |
| `gpu_memory_used` | float | GPU 显存使用（GB） |
| `duration_ms` | float | 单步耗时 |

### 12.2 关键指标

#### 12.2.1 训练指标

| 指标名 | 类型 | 说明 | 目标值 | 告警阈值 |
|--------|------|------|--------|---------|
| `train/reward_mean` | gauge | 训练平均奖励 | 任务相关 | 连续 50k 步下降 |
| `train/loss` | gauge | 训练损失 | 持续下降 | NaN/Inf |
| `train/success_rate` | gauge | 训练成功率 | > 80% | < 50% 持续 100k 步 |
| `train/learning_rate` | gauge | 当前学习率 | 稳定 | 突变 > 10x |
| `train/epoch_duration_ms` | gauge | 单 epoch 耗时 | - | 突增 > 5x |
| `train/grad_norm` | gauge | 梯度范数 | < 10 | > 100（梯度爆炸） |
| `train/nan_count` | counter | NaN loss 次数 | 0 | > 0 即告警 |

#### 12.2.2 评估指标

| 指标名 | 类型 | 说明 | 目标值 |
|--------|------|------|--------|
| `eval/success_rate` | gauge | 评估成功率 | BC > 70%，ACT > 85%，PPO > 90% |
| `eval/episode_length_mean` | gauge | 平均回合长度 | 任务相关 |
| `eval/reward_mean` | gauge | 评估平均奖励 | 任务相关 |
| `eval/best_success_rate` | gauge | 历史最佳成功率 | 持续提升 |

#### 12.2.3 系统指标

| 指标名 | 类型 | 说明 | 告警阈值 |
|--------|------|------|---------|
| `system/gpu_utilization` | gauge | GPU 利用率 | < 80%（PPO 64 并行） |
| `system/gpu_memory_used` | gauge | GPU 显存使用 | > 95% |
| `system/cpu_utilization` | gauge | CPU 利用率 | - |
| `system/memory_used` | gauge | 内存使用 | > 90% |
| `system/disk_free` | gauge | 磁盘剩余空间 | < 10GB |

#### 12.2.4 TensorBoard 训练曲线

TensorBoard 自动记录以下曲线，位于 `--tensorboard_log` 指定目录：

| 曲线名 | 来源 | 说明 |
|--------|------|------|
| `rollout/ep_rew_mean` | SB3 | 回合平均奖励 |
| `rollout/success_rate` | 自定义 callback | 成功率 |
| `train/loss` | SB3 | 策略损失 |
| `train/policy_gradient_loss` | SB3 | 策略梯度损失 |
| `train/value_loss` | SB3 | 价值函数损失 |
| `train/entropy_loss` | SB3 | 熵损失 |
| `train/clip_fraction` | SB3 | PPO clip 比例 |
| `train/learning_rate` | SB3 | 学习率 |

**查看命令**：
```bash
tensorboard --logdir logs/ --port 6006
```

### 12.3 检查点与评估回调

#### 12.3.1 检查点保存策略

| 检查点类型 | 保存时机 | 文件名 | 用途 |
|-----------|---------|--------|------|
| best model | 评估成功率创新高时 | `checkpoints/{algo}_{task}.pt/zip` | 部署用 |
| latest | 每 10000 步 | `checkpoints/{algo}_{task}_latest.pt/zip` | 断点续训 |
| periodic | 每 50000 步（保留最近 5 个） | `checkpoints/{algo}_{task}_{step}.pt/zip` | 回溯分析 |

#### 12.3.2 评估回调

```python
# PPO 评估回调
eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./checkpoints/",
    eval_freq=10000,           # 每 10000 步评估
    n_eval_episodes=20,        # 每次 20 回合
    deterministic=True,        # 确定性策略评估
)
```

- **评估频率**：每 10000 训练步触发一次
- **评估回合数**：20 回合（统计成功率均值与标准差）
- **确定性评估**：`deterministic=True`，消除探索噪声
- **自动保存**：成功率创新高时自动保存 best model

### 12.4 GPU 利用率监控

```bash
# 实时监控 GPU 利用率
nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv -l 5

# 训练日志中记录 GPU 指标
{"ts":"...","level":"INFO","module":"system","gpu_util":85.2,"gpu_mem_used":9.6,"gpu_mem_total":12.0}
```

**目标**：PPO 64 并行环境下，GPU 利用率应 > 80%；若 < 80%，检查数据加载瓶颈。

---

## 13. 风险评估

### 13.1 技术风险

| 风险项 | 可能性 | 影响 | 风险等级 | 缓解措施 |
|--------|:---:|:---:|:---:|------|
| PPO@50Hz 无法通过云端部署 | 高 | 高 | **高** | 200-500ms 云端 RTT 对应 20-50 仿真步延迟，闭环控制不可行。缓解：ONNX 本地推理 + WebSocket 直连（路径 C）或本地推理（路径 D）；当前仅支持预设动作的云端调用 |
| 示范数据量不足导致 BC 过拟合 | 高 | 中 | **高** | 50 条示范可能不足以覆盖状态空间。缓解：数据增强（轨迹插值、加噪）；优先使用 ACT（chunk 预测减少误差累积）；收集更多示范 |
| 域随机化参数与真机偏差 | 中 | 高 | **高** | 随机化范围基于估计，可能与真机不符。缓解：定期真机标定；Sim2Real 评估；采用 realistic 观测模式对齐真机传感器 |
| VLA 的 LLM 幻觉导致不可控动作 | 中 | 高 | **高** | LLM 可能生成非法工具名、危险参数。缓解：输出 schema 校验；工具白名单过滤；参数安全范围裁剪；fallback 到 `stop` 动作 |
| 纯文本 VLA 能力有限 | 高 | 中 | **中** | 仅能调用 8 个预设动作工具，无法精细控制。缓解：明确能力边界；复杂任务需人工分解；视觉 VLA 用于仿真研究 |
| ACT 训练不稳定 | 中 | 中 | **中** | Transformer 对超参敏感。缓解：学习率预热；梯度裁剪；参考原论文超参 |
| PPO 奖励 hacking | 中 | 中 | **中** | 策略可能钻奖励函数漏洞。缓解：奖励函数审查；监测异常 reward 峰值；多指标评估 |
| Sim2Real 性能下降 | 高 | 高 | **高** | 仿真训练的策略在真机上性能下降。缓解：域随机化；realistic 观测模式；真机微调（如有条件） |
| 检查点文件损坏 | 低 | 中 | **中** | 训练中断导致检查点写入不完整。缓解：原子写入；保留多个 periodic 检查点；加载时校验 |

### 13.2 依赖风险

| 依赖项 | 版本要求 | 用途 | 风险 | 缓解措施 |
|--------|---------|------|------|---------|
| `torch` | >=2.0 | BC/ACT 训练 | CUDA 版本兼容性 | 锁定版本；CI 矩阵测试 |
| `stable-baselines3` | >=2.0 | PPO 训练 | API 变更、bug | 锁定版本；关注 release notes |
| `mujoco` | >=2.3 | 仿真引擎 | 升级改变物理行为 | 锁定版本；回归测试 |
| `h5py` | >=3.0 | 示范数据存储 | HDF5 格式兼容性 | 锁定版本；数据备份 |
| `onnx` / `onnxruntime` | >=1.14 | 模型导出/部署 | 算子支持差异 | 导出后验证数值一致性 |
| `transformers` | >=4.30 | VLA 的 LLM 推理 | 模型 API 变更 | 锁定版本；抽象 LLM 接口 |
| `tensorboard` | >=2.10 | 训练可视化 | 日志格式变更 | 锁定版本 |
| GPU 驱动 | CUDA 11.7+ | 训练加速 | 驱动版本与 PyTorch 不匹配 | CI 验证驱动兼容性 |
| 真机固件 | release v2.2.6 | Sim2Real 对齐 | OTA 升级改变工具集 | 跟踪固件版本；协议映射表 |

### 13.3 风险监控与告警

- **训练曲线监控**：TensorBoard 持续观察 reward/loss/success_rate，异常波动告警
- **GPU 资源监控**：显存 > 95% 或利用率 < 80% 持续 10 分钟告警
- **Sim2Real 评估**：定期在真机上验证策略，成功率下降 > 20% 告警
- **VLA 输出审计**：记录所有 LLM 生成的动作序列，定期抽检合法性与安全性
- **检查点完整性**：训练完成后自动验证最新检查点可加载

---

## 14. 变更记录

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|---------|------|
| v1.0 | 2026-07-04 | 初版：AI 训练管线详细设计，包含任务定义、IL/RL/VLA 管线、验证方法、交付物清单 | 架构组 |
| v1.1 | 2026-07-04 | 补充软件工程规范章节：接口设计、数据模型、错误处理、配置管理、日志与可观测性、风险评估 | 架构组 |
