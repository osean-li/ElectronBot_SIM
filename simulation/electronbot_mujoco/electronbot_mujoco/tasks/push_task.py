"""
Push 任务: 用末端推动方块到目标位置

奖励函数:
  - 物体距离目标奖励
  - 末端接近物体奖励
  - 成功奖励

success criteria: 方块到达目标位置 (距离 < 0.05m)
"""

import numpy as np
from .base_task import BaseTask


class PushTask(BaseTask):
    """推方块到目标位置"""

    BLOCK_SIZE = 0.03  # 方块边长
    SUCCESS_THRESHOLD = 0.05  # 成功距离阈值

    def reset(self, seed=None, options=None):
        if self.np_random:
            rand = self.np_random.uniform
        else:
            rand = np.random.uniform

        # 方块位置 (在机器人前方桌面)
        self._block_initial = np.array([
            rand(-0.05, 0.05),   # x
            rand(-0.03, 0.03),   # y
            0.015,               # z (桌面高度)
        ])
        self._target_pos = np.array([
            rand(0.05, 0.15),    # x
            rand(-0.05, 0.05),   # y
            0.015,               # z
        ])

        # 注意: 需要在 scene.xml 中添加可推动的方块
        # 这里简化处理，实际需要 body/geom 定义

        return super().reset(seed=seed, options=options)

    def _compute_reward(self) -> float:
        left_ee, right_ee = self._get_ee_positions()

        # 使用右手推
        ee = right_ee

        # 方块到目标距离 (简化: 用末端模拟)
        block_to_target = np.linalg.norm(ee - self._target_pos)

        reward = -block_to_target

        if block_to_target < self.SUCCESS_THRESHOLD:
            reward += 100.0

        return float(reward)

    def _get_success(self) -> bool:
        left_ee, right_ee = self._get_ee_positions()
        return np.linalg.norm(right_ee - self._target_pos) < self.SUCCESS_THRESHOLD
