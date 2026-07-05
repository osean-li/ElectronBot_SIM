"""Benchmark 标准任务适配器.

对齐 docs/tasks/07-Benchmark §2.

将 electronbot_ai.tasks 中的任务适配为 Benchmark 评估所需的接口.
7 个标准任务:
  EB-Reach / EB-Push / EB-PickPlace / EB-Stack
  EB-Follow / EB-Gesture / EB-VoiceCmd
"""
from __future__ import annotations

from electronbot_ai.tasks import (
    TASK_DISPLAY_NAMES,
    TASK_REGISTRY,
    create_task,
    list_tasks,
)

__all__ = ["TASK_DISPLAY_NAMES", "TASK_REGISTRY", "create_task", "list_tasks"]
