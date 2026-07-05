"""EB-Gesture 手势/动作模仿任务.

对齐 docs/tasks/07-Benchmark §2.6.

场景: 给定目标关节姿态
任务: 从当前姿态移动到目标姿态
成功条件: 所有关节误差 < 5°
难度: ★★☆☆☆

测试用目标姿态 (舵机角度):
  home:      [180,180,0,0,90,90]
  wave:      [150,180,0,0,90,90]
  t-pose:    [90,140,90,40,90,90]
  look_side: [180,180,0,0,60,90]
  nod:       [180,180,0,0,90,105]
"""
from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .base import BaseTask
from ...electronbot_sim.env import (
    SERVO_HOME,
    servo_array_to_joint_array,
)


# 预设目标姿态 (舵机角度 → 机械关节角度)
GESTURE_PRESETS = {
    "home":      np.array([180, 180, 0, 0, 90, 90], dtype=np.float32),
    "wave":      np.array([150, 180, 0, 0, 90, 90], dtype=np.float32),
    "t-pose":    np.array([90, 140, 90, 40, 90, 90], dtype=np.float32),
    "look_side": np.array([180, 180, 0, 0, 60, 90], dtype=np.float32),
    "nod":       np.array([180, 180, 0, 0, 90, 105], dtype=np.float32),
}


class GestureTask(BaseTask):
    """EB-Gesture: 移动到目标关节姿态."""

    name = "EB-Gesture"
    difficulty = 2

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 gesture_name: Optional[str] = None):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self._success_threshold = 5.0  # 5° 误差
        self._gesture_name = gesture_name
        self._target_joint_pos: Optional[np.ndarray] = None

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        # 随机选择目标姿态 (或使用指定姿态)
        if self._gesture_name and self._gesture_name in GESTURE_PRESETS:
            target_servo = GESTURE_PRESETS[self._gesture_name].copy()
        else:
            names = list(GESTURE_PRESETS.keys())
            chosen = names[self._rng.integers(0, len(names))]
            target_servo = GESTURE_PRESETS[chosen].copy()
        # 舵机角度 → 机械关节角度
        self._target_joint_pos = servo_array_to_joint_array(target_servo)
        # target_pos 用关节角度的前 3 维作为简化表示
        self._target_pos = self._target_joint_pos[:3].astype(np.float32)
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        if self._target_joint_pos is not None:
            obs["target_joint_pos"] = self._target_joint_pos.copy()
            current = self._get_joint_pos()
            obs["joint_error"] = np.array(
                float(np.linalg.norm(current - self._target_joint_pos)),
                dtype=np.float32,
            )
        return obs

    def compute_reward(self) -> float:
        if self._target_joint_pos is None:
            return 0.0
        current = self._get_joint_pos()
        error = float(np.linalg.norm(current - self._target_joint_pos))
        reward = -error
        if error < self._success_threshold:
            reward += 10.0
        return reward

    def is_success(self) -> bool:
        if self._target_joint_pos is None:
            return False
        current = self._get_joint_pos()
        error = float(np.linalg.norm(current - self._target_joint_pos))
        return error < self._success_threshold

    def get_demo_action(self, keyboard_state: dict) -> np.ndarray:
        """键盘 → 关节增量 (全关节控制)."""
        action = np.zeros(6, dtype=np.float32)
        if keyboard_state.get("w"):
            action[0] = 2.0
        if keyboard_state.get("s"):
            action[0] = -2.0
        if keyboard_state.get("e"):
            action[1] = 2.0
        if keyboard_state.get("d"):
            action[1] = -2.0
        if keyboard_state.get("r"):
            action[2] = 2.0
        if keyboard_state.get("f"):
            action[2] = -2.0
        if keyboard_state.get("t"):
            action[3] = 2.0
        if keyboard_state.get("g"):
            action[3] = -2.0
        if keyboard_state.get("a"):
            action[4] = 2.0
        if keyboard_state.get("h"):
            action[4] = -2.0
        if keyboard_state.get("q"):
            action[5] = 2.0
        if keyboard_state.get("z"):
            action[5] = -2.0
        return action
