"""ElectronBot AI 训练管线.

对齐 docs/tasks/06-AI-Training 详细设计说明书.

提供三条训练路径:
  1. 模仿学习 (IL): BC / ACT — 键盘遥控收集示范 → 训练策略网络
  2. 强化学习 (RL): PPO — 64 并行环境 + 域随机化
  3. 视觉语言动作 (VLA): 纯文本 VLA (真机可用) / 视觉 VLA (仿真专属)

7 个标准任务 (与 Phase 7 Benchmark 对齐):
  EB-Reach / EB-Push / EB-PickPlace / EB-Stack
  EB-Follow / EB-Gesture / EB-VoiceCmd

子模块:
  - electronbot_ai.tasks:  7 个标准任务定义 + BaseTask 抽象基类
  - electronbot_ai.il:     模仿学习 (collect_demos / train_bc / train_act)
  - electronbot_ai.rl:     强化学习 (train_ppo / parallel_env / domain_randomization)
  - electronbot_ai.vla:    VLA 规划器 (TextVLAPlanner / VisionVLAPlanner)
"""
from __future__ import annotations

__version__ = "0.2.0"
__all__ = ["tasks", "il", "rl", "vla"]
