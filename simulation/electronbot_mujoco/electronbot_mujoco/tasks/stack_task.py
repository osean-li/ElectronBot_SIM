"""
Stack 任务: 堆叠方块

用一只手拾取方块，放到另一个方块上方

奖励函数:
  - 靠近物体奖励
  - 抓取奖励
  - 放置奖励

success criteria: 方块堆叠成功 (顶部方块稳定在底部方块上方)

注意: 这是最复杂的任务，需要:
  1. 物体模型 (两个可抓取的方块)
  2. 简化的抓取语义 (末端接近 + 提升 = 抓取)
"""

import numpy as np
from .base_task import BaseTask


class StackTask(BaseTask):
    """堆叠方块任务"""

    SUCCESS_THRESHOLD = 0.03

    def reset(self, seed=None, options=None):
        if self.np_random:
            rand = self.np_random.uniform
        else:
            rand = np.random.uniform

        # 底部方块位置
        self._base_block_pos = np.array([
            rand(-0.03, 0.03),
            rand(-0.02, 0.02),
            0.015,
        ])

        # 顶部方块初始位置 (随机)
        self._top_block_pos = np.array([
            rand(-0.08, 0.08),
            rand(-0.05, 0.05),
            0.015,
        ])

        # 目标位置 (底部方块上方)
        self._target_pos = self._base_block_pos + np.array([0, 0, 0.03])

        self._grasped = False
        return super().reset(seed=seed, options=options)

    def _compute_reward(self) -> float:
        left_ee, right_ee = self._get_ee_positions()
        ee = right_ee  # 用右手

        reward = 0.0

        if not self._grasped:
            # 阶段1: 移动到顶部方块上方
            dist_to_block = np.linalg.norm(ee - self._top_block_pos)
            reward = -dist_to_block

            # 检查抓取: 足够近
            if dist_to_block < 0.02:
                self._grasped = True
                reward += 50.0
        else:
            # 阶段2: 移动到目标位置
            dist_to_target = np.linalg.norm(ee - self._target_pos)
            reward = -dist_to_target + 30.0  # 基础抓取奖励

            if dist_to_target < self.SUCCESS_THRESHOLD:
                reward += 100.0

        return float(reward)

    def _get_success(self) -> bool:
        if not self._grasped:
            return False
        left_ee, right_ee = self._get_ee_positions()
        return np.linalg.norm(right_ee - self._target_pos) < self.SUCCESS_THRESHOLD
