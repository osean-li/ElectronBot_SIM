"""任务注册表 — 7 个标准任务的统一入口.

对齐 docs/tasks/06-AI-Training §2.2 + docs/tasks/07-Benchmark §2.

任务清单:
  EB-Reach    — 触碰目标点 (★☆☆☆☆, BC/ACT/PPO/VLA)
  EB-Push     — 推物体到目标 (★★☆☆☆, BC/ACT/PPO/VLA)
  EB-PickPlace — 抓取放置 (★★★★☆, 仿真专属, BC/ACT/PPO/VLA)
  EB-Stack    — 叠方块 (★★★★★, 仿真专属, BC/ACT/PPO/VLA)
  EB-Follow   — 追踪移动物体 (★★★☆☆, BC/ACT/PPO/VLA)
  EB-Gesture  — 手势模仿 (★★☆☆☆, BC/ACT/PPO/VLA)
  EB-VoiceCmd — 语音指令理解 (仅 VLA)
"""
from __future__ import annotations

from typing import Dict, Type

from .base import BaseTask
from .reach import ReachTask
from .push import PushTask
from .pick_place import PickPlaceTask
from .stack import StackTask
from .follow import FollowTask
from .gesture import GestureTask, GESTURE_PRESETS
from .voice_cmd import VoiceCmdTask, VOICE_TEST_COMMANDS

# 任务注册表: 任务名 → 实现类
TASK_REGISTRY: Dict[str, Type[BaseTask]] = {
    "reach":     ReachTask,
    "push":      PushTask,
    "pick_place": PickPlaceTask,
    "stack":     StackTask,
    "follow":    FollowTask,
    "gesture":   GestureTask,
    "voice":     VoiceCmdTask,
    "voice_cmd": VoiceCmdTask,  # 别名
}

# 任务显示名映射
TASK_DISPLAY_NAMES = {
    "reach":      "EB-Reach",
    "push":       "EB-Push",
    "pick_place": "EB-PickPlace",
    "stack":      "EB-Stack",
    "follow":     "EB-Follow",
    "gesture":    "EB-Gesture",
    "voice":      "EB-VoiceCmd",
    "voice_cmd":  "EB-VoiceCmd",
}


def create_task(task_name: str, obs_mode: str = "full",
                seed: int | None = None, **kwargs) -> BaseTask:
    """工厂函数: 按名称创建任务实例.

    参数:
        task_name: 任务名 (如 "reach", "push", "voice_cmd")
        obs_mode:  观测模式 "full" / "realistic"
        seed:      随机种子
        **kwargs:  透传给任务构造函数

    返回:
        BaseTask 子类实例

    异常:
        ValueError: 未知任务名
    """
    key = task_name.lower().replace("-", "_").strip()
    if key not in TASK_REGISTRY:
        available = ", ".join(TASK_REGISTRY.keys())
        raise ValueError(f"未知任务名: {task_name}. 可用任务: {available}")
    cls = TASK_REGISTRY[key]
    return cls(obs_mode=obs_mode, seed=seed, **kwargs)


def list_tasks() -> list[str]:
    """列出所有可用任务名."""
    return list(TASK_REGISTRY.keys())


__all__ = [
    "BaseTask",
    "TASK_REGISTRY",
    "TASK_DISPLAY_NAMES",
    "create_task",
    "list_tasks",
    "ReachTask",
    "PushTask",
    "PickPlaceTask",
    "StackTask",
    "FollowTask",
    "GestureTask",
    "VoiceCmdTask",
    "GESTURE_PRESETS",
    "VOICE_TEST_COMMANDS",
]
