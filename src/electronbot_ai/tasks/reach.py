"""EB-Reach 触碰目标任务.

对齐 docs/tasks/06-AI-Training §2.3 + docs/tasks/07-Benchmark §2.1.

场景: 桌面上随机位置出现一个目标球
任务: 使用右手触碰目标球
成功条件: 手部与球的距离 < 2cm
难度: ★☆☆☆☆

随机化参数:
  - 目标位置: x~[-8,8]cm, y~[-2,5]cm, z~[-3,0]cm
  - 初始姿态: 关节随机偏移 ±10°
"""
from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .base import BaseTask


class ReachTask(BaseTask):
    """EB-Reach: 末端执行器触碰目标点."""

    name = "EB-Reach"
    difficulty = 1

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 ee_name: str = "right_hand"):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self.ee_name = ee_name
        self._success_threshold = 0.02  # 2cm

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        # 随机化目标位置 (对齐 §2.3)
        self._target_pos = np.array([
            self._rng.uniform(-0.08, 0.08),   # x: ±8cm
            self._rng.uniform(-0.02, 0.05),   # y: -2~5cm
            self._rng.uniform(-0.03, 0.0),    # z: -3~0cm
        ], dtype=np.float32)
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        # 额外任务信息
        obs["ee_name"] = self.ee_name
        return obs

    def compute_reward(self) -> float:
        if self._target_pos is None:
            return 0.0
        ee_pos = self._get_ee_pos(self.ee_name)
        dist = float(np.linalg.norm(ee_pos - self._target_pos))
        # 稠密奖励: 距离负值 + 接近奖励 + 成功奖励
        reward = -dist
        if dist < 0.05:   # 5cm 内额外奖励
            reward += 0.5
        if dist < self._success_threshold:  # 2cm 内大额奖励
            reward += 10.0
        return reward

    def is_success(self) -> bool:
        if self._target_pos is None:
            return False
        ee_pos = self._get_ee_pos(self.ee_name)
        return float(np.linalg.norm(ee_pos - self._target_pos)) < self._success_threshold

    def get_demo_action(self, keyboard_state: dict) -> np.ndarray:
        """键盘 → 关节增量 (右手控制).

        W/S: 右臂 Pitch (上/下)
        E/D: 右臂 Roll (前/后)
        A/D: 身体旋转
        Q/E: 头部
        """
        action = np.zeros(6, dtype=np.float32)
        if keyboard_state.get("w"):
            action[0] = 2.0   # RP 上
        if keyboard_state.get("s"):
            action[0] = -2.0  # RP 下
        if keyboard_state.get("e"):
            action[1] = 2.0   # RR
        if keyboard_state.get("d"):
            action[1] = -2.0  # RR
        if keyboard_state.get("a"):
            action[4] = 2.0   # BODY 左转
        if keyboard_state.get("f"):
            action[4] = -2.0  # BODY 右转
        return action
