"""模仿学习 (Imitation Learning) 子模块.

对齐 docs/tasks/06-AI-Training §3.

提供:
  - collect_demos: 键盘遥控收集示范数据 (HDF5 格式)
  - train_bc:      Behavior Cloning 训练 (MLP 策略)
  - train_act:     Action Chunking Transformer 训练
"""
from __future__ import annotations

__all__ = ["collect_demos", "train_bc", "train_act"]
