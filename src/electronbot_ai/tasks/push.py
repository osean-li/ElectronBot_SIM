"""EB-Push 推物体任务.

对齐 docs/tasks/07-Benchmark §2.2.

场景: 桌面上有一个方块, 需推到目标区域
任务: 用左/右手将方块推到标记的位置
成功条件: 方块位置距目标 < 3cm
难度: ★★☆☆☆
"""
from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .base import BaseTask


class PushTask(BaseTask):
    """EB-Push: 推方块到目标位置."""

    name = "EB-Push"
    difficulty = 2

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 ee_name: str = "right_hand"):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self.ee_name = ee_name
        self._success_threshold = 0.03  # 3cm
        self._object_pos: Optional[np.ndarray] = None

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        # 随机化物体初始位置和目标位置
        self._object_pos = np.array([
            self._rng.uniform(-0.05, 0.05),
            self._rng.uniform(-0.03, 0.03),
            -0.02,
        ], dtype=np.float32)
        self._target_pos = np.array([
            self._rng.uniform(-0.08, 0.08),
            self._rng.uniform(-0.05, 0.05),
            -0.02,
        ], dtype=np.float32)
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        if self._object_pos is not None:
            obs["object_pos"] = self._object_pos.copy()
            obs["dist_object_to_target"] = np.array(
                float(np.linalg.norm(self._object_pos - self._target_pos)),
                dtype=np.float32,
            )
        return obs

    def compute_reward(self) -> float:
        if self._object_pos is None or self._target_pos is None:
            return 0.0
        dist = float(np.linalg.norm(self._object_pos - self._target_pos))
        reward = -dist
        if dist < self._success_threshold:
            reward += 10.0
        # 接近物体的额外奖励 (鼓励先靠近物体)
        ee_pos = self._get_ee_pos(self.ee_name)
        ee_to_obj = float(np.linalg.norm(ee_pos - self._object_pos))
        if ee_to_obj < 0.05:
            reward += 0.2
        return reward

    def is_success(self) -> bool:
        if self._object_pos is None or self._target_pos is None:
            return False
        return float(np.linalg.norm(self._object_pos - self._target_pos)) < self._success_threshold

    def get_demo_action(self, keyboard_state: dict) -> np.ndarray:
        action = np.zeros(6, dtype=np.float32)
        if keyboard_state.get("w"):
            action[0] = 2.0
        if keyboard_state.get("s"):
            action[0] = -2.0
        if keyboard_state.get("a"):
            action[4] = 2.0
        if keyboard_state.get("d"):
            action[4] = -2.0
        return action
