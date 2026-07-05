"""强化学习 (Reinforcement Learning) 子模块.

对齐 docs/tasks/06-AI-Training §4.

提供:
  - parallel_env:           64 并行环境 (SubprocVecEnv)
  - train_ppo:              PPO 训练脚本
  - domain_randomization:   域随机化 wrapper
"""
from __future__ import annotations

__all__ = ["parallel_env", "train_ppo", "domain_randomization"]
