"""EB-Stack 叠方块任务 (仿真专属, 最高难度).

对齐 docs/tasks/07-Benchmark §2.4.

场景: 桌面上有两个方块
任务: 将一个方块叠在另一个上面
成功条件: 上方方块稳定叠放 > 2秒
难度: ★★★★★

⚠️ 仿真专属: 真机无手指, 不可部署
"""
from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np

from .base import BaseTask


class StackTask(BaseTask):
    """EB-Stack: 叠方块 (仿真专属, 最高难度)."""

    name = "EB-Stack"
    difficulty = 5

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 ee_name: str = "right_hand"):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self.ee_name = ee_name
        self._block_a_pos: Optional[np.ndarray] = None  # 底部方块
        self._block_b_pos: Optional[np.ndarray] = None  # 待叠起方块
        self._stable_start_time: Optional[float] = None
        self._stable_duration_threshold = 2.0  # 稳定 2 秒

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        self._stable_start_time = None
        # 两个方块在桌面不同位置
        self._block_a_pos = np.array([
            self._rng.uniform(-0.04, 0.04),
            self._rng.uniform(-0.04, 0.04),
            -0.025,
        ], dtype=np.float32)
        self._block_b_pos = np.array([
            self._rng.uniform(-0.06, 0.06),
            self._rng.uniform(-0.06, 0.06),
            -0.025,
        ], dtype=np.float32)
        # 确保两个方块初始不重叠
        while np.linalg.norm(self._block_a_pos[:2] - self._block_b_pos[:2]) < 0.04:
            self._block_b_pos[:2] = self._rng.uniform(-0.06, 0.06, 2)
        self._target_pos = self._block_a_pos.copy()
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        if self._block_a_pos is not None:
            obs["block_a_pos"] = self._block_a_pos.copy()
        if self._block_b_pos is not None:
            obs["block_b_pos"] = self._block_b_pos.copy()
        return obs

    def compute_reward(self) -> float:
        if self._block_a_pos is None or self._block_b_pos is None:
            return 0.0
        # 奖励: 方块 B 的高度 (越高越好, 鼓励抬起)
        height_reward = max(0, self._block_b_pos[2] - (-0.025)) * 10.0
        # 奖励: 方块 B 水平靠近方块 A
        horizontal_dist = float(np.linalg.norm(
            self._block_a_pos[:2] - self._block_b_pos[:2]
        ))
        align_reward = -horizontal_dist * 2.0
        # 大额奖励: 成功叠放
        if self._is_stacked():
            return 10.0 + height_reward + align_reward
        return height_reward + align_reward

    def _is_stacked(self) -> bool:
        """判断是否成功叠放 (B 在 A 上方且水平对齐)."""
        if self._block_a_pos is None or self._block_b_pos is None:
            return False
        horizontal_dist = float(np.linalg.norm(
            self._block_a_pos[:2] - self._block_b_pos[:2]
        ))
        height_diff = self._block_b_pos[2] - self._block_a_pos[2]
        return horizontal_dist < 0.02 and 0.01 < height_diff < 0.05

    def is_success(self) -> bool:
        if not self._is_stacked():
            self._stable_start_time = None
            return False
        # 需稳定 2 秒
        now = time.time()
        if self._stable_start_time is None:
            self._stable_start_time = now
            return False
        return (now - self._stable_start_time) >= self._stable_duration_threshold

    def get_demo_action(self, keyboard_state: dict) -> np.ndarray:
        action = np.zeros(6, dtype=np.float32)
        if keyboard_state.get("w"):
            action[0] = 2.0
        if keyboard_state.get("s"):
            action[0] = -2.0
        if keyboard_state.get("e"):
            action[1] = 2.0
        if keyboard_state.get("d"):
            action[1] = -2.0
        if keyboard_state.get("a"):
            action[4] = 2.0
        if keyboard_state.get("f"):
            action[4] = -2.0
        return action
