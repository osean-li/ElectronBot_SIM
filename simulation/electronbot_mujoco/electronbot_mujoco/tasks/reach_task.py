"""
Reach 任务: 用指定手臂的末端触碰目标球

状态空间: 标准 18 维
动作空间: 6 维关节角度增量

奖励函数:
  - 距离奖励: -dist(ee, target)
  - 成功奖励: +100 if dist < threshold
  - 动作平滑惩罚: -0.01 * |a|

终止条件:
  - 末端触碰目标球 (距离 < 0.02m)
  - 达到最大步数

success criteria: dist(ee, target) < 0.02m
"""

import numpy as np
from .base_task import BaseTask


class ReachTask(BaseTask):
    """触碰目标球任务"""

    TARGET_RADIUS = 0.015  # 目标球半径 (米)
    SUCCESS_THRESHOLD = 0.02  # 成功距离阈值 (米)

    def __init__(self, arm: str = "right", **kwargs):
        """
        参数:
          arm: "left" / "right" / "both" - 使用的手臂
        """
        super().__init__(**kwargs)
        self.arm = arm
        self._target_pos = np.array([0.08, 0, 0.12])  # 默认目标位置

    def reset(self, seed=None, options=None):
        # 随机目标位置
        if self.np_random:
            rand = self.np_random.uniform
        else:
            rand = np.random.uniform

        self._target_pos = np.array([
            rand(-0.1, 0.1),          # x
            rand(-0.05, 0.05),        # y
            rand(0.08, 0.18),         # z
        ])
        return super().reset(seed=seed, options=options)

    def _compute_reward(self) -> float:
        left_ee, right_ee = self._get_ee_positions()

        if self.arm == "left":
            ee = left_ee
        elif self.arm == "right":
            ee = right_ee
        else:
            ee = (left_ee + right_ee) / 2.0

        # 距离
        dist = np.linalg.norm(ee - self._target_pos)

        # 奖励
        reward = -dist  # 基础: 越近越好

        if dist < self.SUCCESS_THRESHOLD:
            reward += 100.0  # 成功奖励

        # 动作平滑惩罚
        reward -= 0.01 * np.sum(np.abs(
            self.robot.get_joint_velocities()
        ))

        return float(reward)

    def _get_success(self) -> bool:
        left_ee, right_ee = self._get_ee_positions()
        if self.arm == "left":
            ee = left_ee
        elif self.arm == "right":
            ee = right_ee
        else:
            ee = (left_ee + right_ee) / 2.0
        return np.linalg.norm(ee - self._target_pos) < self.SUCCESS_THRESHOLD

    def _get_info(self):
        info = super()._get_info()
        info["target_pos"] = self._target_pos.tolist()
        info["arm"] = self.arm
        return info
