"""EB-PickPlace 抓取放置任务 (仿真专属).

对齐 docs/tasks/07-Benchmark §2.3.

场景: 桌面上有一个小物体, 需抓起放入指定容器
任务: 准确定位 → 抓取 → 抬臂 → 放入容器 → 释放
成功条件: 物体进入容器区域
难度: ★★★★☆

⚠️ 仿真专属: 真机 ElectronBot 无手指, 不可部署

奖励分解:
  - 靠近物体: +0.5 (d < 5cm)
  - 抓取物体: +2.0 (接触力 > 阈值, 物体离桌面 > 1cm)
  - 靠近容器: +1.0 (带物体距容器 < 5cm)
  - 放入容器: +10.0 (物体在容器内)
"""
from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .base import BaseTask


class PickPlaceTask(BaseTask):
    """EB-PickPlace: 抓取物体并放入容器 (仿真专属)."""

    name = "EB-PickPlace"
    difficulty = 4

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 ee_name: str = "right_hand"):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self.ee_name = ee_name
        self._object_pos: Optional[np.ndarray] = None
        self._container_pos: Optional[np.ndarray] = None
        self._object_height_threshold = 0.015  # 物体离桌面 > 1.5cm 视为被抓起
        self._container_radius = 0.03  # 容器半径 3cm

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        # 物体在桌面一侧, 容器在另一侧
        self._object_pos = np.array([
            self._rng.uniform(-0.06, -0.02),
            self._rng.uniform(-0.03, 0.03),
            -0.025,
        ], dtype=np.float32)
        self._container_pos = np.array([
            self._rng.uniform(0.02, 0.06),
            self._rng.uniform(-0.03, 0.03),
            -0.025,
        ], dtype=np.float32)
        self._target_pos = self._container_pos.copy()
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        if self._object_pos is not None:
            obs["object_pos"] = self._object_pos.copy()
        if self._container_pos is not None:
            obs["container_pos"] = self._container_pos.copy()
        return obs

    def compute_reward(self) -> float:
        if self._object_pos is None or self._container_pos is None:
            return 0.0
        ee_pos = self._get_ee_pos(self.ee_name)
        obj_to_ee = float(np.linalg.norm(ee_pos - self._object_pos))
        obj_to_container = float(np.linalg.norm(self._object_pos - self._container_pos))

        reward = 0.0
        # 阶段 1: 靠近物体
        if obj_to_ee < 0.05:
            reward += 0.5
        # 阶段 2: 抓取物体 (物体被抬起)
        if self._object_pos[2] > -self._object_height_threshold:
            reward += 2.0
        # 阶段 3: 带物体靠近容器
        if obj_to_container < 0.05 and self._object_pos[2] > -self._object_height_threshold:
            reward += 1.0
        # 阶段 4: 放入容器
        if obj_to_container < self._container_radius:
            reward += 10.0
        # 距离负奖励 (稠密引导)
        reward -= obj_to_container * 0.5
        return reward

    def is_success(self) -> bool:
        if self._object_pos is None or self._container_pos is None:
            return False
        dist = float(np.linalg.norm(self._object_pos - self._container_pos))
        return dist < self._container_radius

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
