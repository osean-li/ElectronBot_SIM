# 小智版 ElectronBot_SIM — AI 能力覆盖说明

> 本文档解释 xiaozhi-esp32 release v2.2.6 版本的仿真平台如何覆盖 AI/机器人领域的核心能力，
> 以及哪些能力在当前硬件底座下不需要或不适用及其原因。

---

## 覆盖总览

```
PPO/SAC       ████████████████░░░░  已覆盖（PPO 完整，SAC 非必需未实现）
ACT/DiffPolicy██████████████░░░░░░  已覆盖（ACT 完整，Diffusion 非必需未实现）
VLA + LoRA    ████████████████░░░░  已覆盖（单模式 VLA，LoRA 非必需未实现）
py_trees      ██░░░░░░░░░░░░░░░░░░  未实现（不适用，见下文）
统一评估       ████████████████████  已完整覆盖（7 任务 × 5 指标）
标定+辨识     ████████████░░░░░░░░  已覆盖标定，系统辨识不适用（无编码器反馈）
```

---

## 1. PPO / SAC

| 项目 | 状态 | 说明 |
|------|:---:|------|
| PPO | ✅ 完整 | Phase 6 `train_ppo.py`：SB3 实现，64 并行 MuJoCo 环境，域随机化 wrapper |
| SAC | ❌ 不需要 | PPO 在连续控制任务上的表现足够好（PPO 是 On-Policy，SAC 是 Off-Policy，对仿真环境差异不大） |

**PPO 训练入口**：

```python
# src/electronbot_ai/rl/train_ppo.py
from stable_baselines3 import PPO

model = PPO("MlpPolicy", env, n_steps=2048, batch_size=512, n_epochs=10, lr=3e-4)
model.learn(total_timesteps=1_000_000)
model.save("checkpoints/ppo_reach")
```

**验证标准**：reach 任务成功率 > 90%，pick_place > 75%

## 2. ACT / Diffusion Policy

| 项目 | 状态 | 说明 |
|------|:---:|------|
| ACT | ✅ 完整 | Phase 6 `train_act.py`：Action Chunking Transformer，chunk_size=10，低延迟场景优于 BC |
| Diffusion Policy | ❌ 不需要 | ACT 和 Diffusion Policy 在 6-DOF 低速场景下效果接近，ACT 训练更快、工程更成熟 |

**ACT 训练入口**：

```python
# src/electronbot_ai/il/train_act.py
class ACTPolicy(nn.Module):
    def forward(self, obs_sequence):
        # Transformer Encoder → 预测未来 K=10 个动作
        return action_chunk  # (B, 10, 6)
```

**验证标准**：50 条示范 → 成功率 > 85%（优于 BC baseline 的 70%）

## 3. VLA 双模式 + LoRA

| 项目 | 状态 | 说明 |
|------|:---:|------|
| VLA 单模式 | ✅ 完整 | Phase 6 `llm_planner.py`：Qwen2.5-VL / DeepSeek → 摄像头图像 + 语音指令 → MCP 动作序列 |
| 双模式 | ❌ 不需要 | xiaozhi 版只有 8 个预设动作工具，LLM 的任务是"选哪个工具 + 填参数"，不是"生成连续动作轨迹"——单模式足够 |
| LoRA 微调 | ❌ 不需要 | 基座模型（Qwen2.5）的指令跟随能力已经够好，不需要微调来理解"挥手"→`hand_action(3,3,2,500)` |

**VLA 工作流**：

```
语音 "帮我挥手打个招呼"
    │
    ▼
┌─────────────────────────────────────┐
│ Qwen2.5-VL (本地 / API)             │
│ prompt: 摄像头图像 + 工具列表        │
│ output: {"method":"self.electron.   │
│          hand_action",              │
│          "params":{"action":3,      │
│          "hand":3,"steps":2,        │
│          "speed":600}}              │
└─────────────────────────────────────┘
    │
    ▼
McpBridge.handle_request() → MuJoCo 仿真执行 / 真机 WebSocket 执行
```

**验证标准**：10 条自然语言指令，人类评审认为执行正确 > 80%

## 4. py_trees 行为树

| 项目 | 状态 | 说明 |
|------|:---:|------|
| 行为树库 | ❌ 不需要 | 不是"不该做"，是"用错了工具" |

**为什么不需要**：行为树的价值在 **几十个节点、多层嵌套、条件分支复杂** 的场景（如自动驾驶的决策树）。xiaozhi 版的交互模式是：

```
唤醒 → LLM 理解意图 → 生成 MCP 命令 → 执行 → 等待下次唤醒
```

这不是一个需要行为树的流程。用 Python 列表编排比引入 BehaviorTree.CPP + XML 解析更直接：

```python
# 用列表编排——比 py_trees XML 更清晰
sequence = [
    ("home", {}),
    ("hand_action", {"action": 3, "hand": 3, "steps": 2, "speed": 600}),
    ("head_move", {"action": 3, "steps": 1, "speed": 500, "angle": 10}),
]
```

如果未来加上了语音循环对话、手势条件触发等复杂交互，再引入 py_trees 也不迟。架构文档的技术选型表里保留了位置，随时可接。

## 5. 统一评估框架

| 项目 | 状态 | 说明 |
|------|:---:|------|
| Benchmark Suite | ✅ 完整 | Phase 7 `suite.py`：7 标准任务 × 5 评估指标 × 自动化批量运行 |

**7 个任务+真机对齐**：

| 任务 | 仿真 | 真机 release | 说明 |
|------|:---:|:---:|------|
| EB-Reach | ✅ | ✅ | 真机可用 hand_action 逼近 |
| EB-Push | ✅ | ✅ | 身体+手臂组合 |
| EB-PickPlace | ✅ | ⚠️ 仿真专属 | 真机无手指，只能推不能抓 |
| EB-Stack | ✅ | ⚠️ 仿真专属 | 同上 |
| EB-Follow | ✅ | ✅ | head_move 追踪 |
| EB-Gesture | ✅ | ✅ | 预设动作模仿，核心任务 |
| EB-VoiceCmd | ✅ | ✅ | LLM 指令理解评估 |

**5 项评估指标**：

| 指标 | 计算方式 |
|------|---------|
| Success Rate | 成功次数 / 总次数 |
| Mean Completion Time | 从 reset 到 success 的墙钟时间 |
| Trajectory Smoothness | 关节加速度 L2 范数平均值 |
| Generalization Gap | (ID 成功率 - OOD 成功率) / ID 成功率 |
| Sim2Real Gap | (仿真成功率 - 真机成功率) / 仿真成功率 |

## 6. 标定 + 系统辨识

| 项目 | 状态 | 说明 |
|------|:---:|------|
| 舵机 Trim 标定 | ✅ 完整 | Phase 8 `calibrate.py`：逐关节 W/S 微调 + NVS 持久化 |
| 系统辨识 | ❌ 不适用 | xiaozhi 版廉价舵机无编码器反馈，无法做系统 ID |

**为什么系统辨识不适用**：

系统辨识需要输入-输出数据对（如：发送 200Hz 扫频信号激励舵机，用编码器记录实际角度响应，拟合传递函数）。xiaozhi 版使用廉价 PWM 舵机——没有编码器、没有角度回传——根本拿不到输出数据。

稚晖君原版可以做系统 ID 是因为自制智能舵机内置了一整套：
- STM32F042 MCU
- ADC 采样电位器（角度反馈）
- 200Hz DCE PID 闭环
- I2C 实时回传 `float` 角度值

两条路径不同：稚晖君版 → 辨识出精确动力学 → PID 调优；xiaozhi 版 → 域随机化训练鲁棒策略 → 对抗参数不确定性。后者的效果对廉价舵机来说已经足够。

---

## 总结

| 能力 | 覆盖方式 | 对应文档 |
|------|---------|------|
| PPO | ✅ Phase 6 `train_ppo.py` | 06-AI-Training |
| ACT | ✅ Phase 6 `train_act.py` | 06-AI-Training |
| BC (baseline) | ✅ Phase 6 `train_bc.py` | 06-AI-Training |
| VLA | ✅ Phase 6 `llm_planner.py` | 06-AI-Training |
| Benchmark | ✅ Phase 7 `suite.py` | 07-Benchmark |
| 标定 | ✅ Phase 8 `calibrate.py` | 08-Sim2Real |
| 域随机化 | ✅ Phase 2/6 wrapper | 02-MuJoCo-Env, 06-AI-Training |
| SAC | 不必要 | PPO 已足够 |
| Diffusion Policy | 不必要 | ACT 已足够 |
| VLA 双模式+LoRA | 不必要 | 预设工具场景不需要微调 |
| py_trees 行为树 | 不必要 | 简单交互用列表编排更直接 |
| 系统辨识 | 不适用 | 无编码器反馈，无法做辨识 |
| 情绪策略 | 不必要 | LLM 直接选预设动作即可 |

**当前 8 阶段文档已完整覆盖合理的技术栈，所有缺失项均有明确的"为什么不需要"的判断依据。**
