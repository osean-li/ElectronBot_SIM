"""EB-Follow 追踪移动物体任务.

对齐 docs/tasks/07-Benchmark §2.5.

场景: 桌面上一个物体沿随机轨迹移动
任务: 头部/手臂持续追踪物体位置
成功条件: 连续 5 秒内手-物距离 < 3cm
难度: ★★★☆☆
"""
from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np

from .base import BaseTask


class FollowTask(BaseTask):
    """EB-Follow: 持续追踪移动目标."""

    name = "EB-Follow"
    difficulty = 3

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 ee_name: str = "right_hand"):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self.ee_name = ee_name
        self._success_threshold = 0.03  # 3cm
        self._tracking_duration = 5.0   # 需持续 5 秒
        self._tracking_start_time: Optional[float] = None
        self._object_velocity = np.zeros(3, dtype=np.float32)
        self._sim_time = 0.0

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        self._tracking_start_time = None
        self._sim_time = 0.0
        # 初始目标位置
        self._target_pos = np.array([
            self._rng.uniform(-0.05, 0.05),
            self._rng.uniform(-0.03, 0.03),
            -0.02,
        ], dtype=np.float32)
        # 随机运动速度 (m/s, 慢速移动)
        speed = self._rng.uniform(0.01, 0.03)
        angle = self._rng.uniform(0, 2 * np.pi)
        self._object_velocity = np.array([
            speed * np.cos(angle),
            speed * np.sin(angle),
            0.0,
        ], dtype=np.float32)
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        obs["object_velocity"] = self._object_velocity.copy()
        return obs

    def _update_target_position(self):
        """每步更新目标位置 (模拟物体移动)."""
        if self._target_pos is None:
            return
        dt = 0.02  # 50Hz
        self._target_pos = self._target_pos + self._object_velocity * dt
        # 边界反弹 (保持在桌面范围内)
        for i in range(2):  # x, y
            limit = 0.08 if i == 0 else 0.05
            if abs(self._target_pos[i]) > limit:
                self._object_velocity[i] *= -1
                self._target_pos[i] = np.clip(self._target_pos[i], -limit, limit)
        self._sim_time += dt

    def compute_reward(self) -> float:
        if self._target_pos is None:
            return 0.0
        self._update_target_position()
        ee_pos = self._get_ee_pos(self.ee_name)
        dist = float(np.linalg.norm(ee_pos - self._target_pos))
        reward = -dist
        if dist < self._success_threshold:
            reward += 1.0  # 持续追踪奖励
        return reward

    def is_success(self) -> bool:
        if self._target_pos is None:
            return False
        ee_pos = self._get_ee_pos(self.ee_name)
        dist = float(np.linalg.norm(ee_pos - self._target_pos))
        now = time.time()
        if dist < self._success_threshold:
            if self._tracking_start_time is None:
                self._tracking_start_time = now
                return False
            return (now - self._tracking_start_time) >= self._tracking_duration
        else:
            self._tracking_start_time = None
            return False

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
        if keyboard_state.get("q"):
            action[5] = 2.0  # head
        if keyboard_state.get("e"):
            action[5] = -2.0
        return action
