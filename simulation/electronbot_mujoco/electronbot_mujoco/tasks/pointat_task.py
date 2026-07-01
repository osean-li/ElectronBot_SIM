"""
PointAt 任务: 指尖指向目标方向

将手臂末端对准空间中指定的方向向量

奖励函数:
  - 方向对齐奖励: cos(angle) 越接近 1 越好
  - 位置奖励: 末端越近越好

success criteria: 指向误差 < 15°
"""

import numpy as np
from .base_task import BaseTask


class PointAtTask(BaseTask):
    """指尖指向目标任务"""

    SUCCESS_ANGLE = np.deg2rad(15)

    def reset(self, seed=None, options=None):
        if self.np_random:
            rand = self.np_random.uniform
        else:
            rand = np.random.uniform

        # 随机目标方向
        theta = rand(0.3, 1.2)  # 极角
        phi = rand(-0.5, 0.5)   # 方位角
        r = 0.15  # 距离
        self._target_pos = np.array([
            r * np.sin(theta) * np.cos(phi),
            r * np.sin(theta) * np.sin(phi),
            r * np.cos(theta),
        ])

        return super().reset(seed=seed, options=options)

    def _compute_reward(self) -> float:
        left_ee, right_ee = self._get_ee_positions()

        # 使用右手
        ee = right_ee
        ee_dir = ee / (np.linalg.norm(ee) + 1e-8)
        target_dir = self._target_pos / (np.linalg.norm(self._target_pos) + 1e-8)

        # 方向对齐度
        alignment = np.dot(ee_dir, target_dir)  # [-1, 1]

        reward = alignment  # 基础奖励
        if alignment > np.cos(self.SUCCESS_ANGLE):
            reward += 50.0

        return float(reward)

    def _get_success(self) -> bool:
        left_ee, right_ee = self._get_ee_positions()
        ee = right_ee
        ee_dir = ee / (np.linalg.norm(ee) + 1e-8)
        target_dir = self._target_pos / (np.linalg.norm(self._target_pos) + 1e-8)
        angle = np.arccos(np.clip(np.dot(ee_dir, target_dir), -1, 1))
        return angle < self.SUCCESS_ANGLE
