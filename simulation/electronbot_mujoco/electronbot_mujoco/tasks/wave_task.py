"""
Wave 任务: 周期性挥手动作

奖励函数:
  - 轨迹跟踪奖励: 关节角度跟随正弦波参考轨迹
  - 平滑度奖励: 动作连续性

success criteria: 完成完整挥手周期
"""

import numpy as np
from .base_task import BaseTask


class WaveTask(BaseTask):
    """挥手任务"""

    def __init__(self, arm: str = "right", **kwargs):
        super().__init__(**kwargs)
        self.arm = arm
        self._phase = 0.0
        self._max_phase = 4 * np.pi  # 2 个完整挥手周期

    def reset(self, seed=None, options=None):
        self._phase = 0.0
        return super().reset(seed=seed, options=options)

    def _get_reference_trajectory(self) -> float:
        """正弦波参考轨迹 (用于肩俯仰关节)"""
        amplitude = np.deg2rad(40)  # 40° 振幅
        frequency = 2 * np.pi / self.max_episode_steps * 4  # 4Hz 等效
        return amplitude * np.sin(frequency * self._step_count * 0.02)

    def _compute_reward(self) -> float:
        self._phase += 0.02  # 相位累加

        # 参考轨迹
        ref_pitch = self._get_reference_trajectory()
        ref_roll = np.deg2rad(10) * np.sin(self._phase * 0.5)

        # 当前关节角度
        q = self.robot.get_joint_positions()
        qd = self.robot.get_joint_velocities()

        if self.arm == "right":
            actual_pitch = q[4]  # right_arm_pitch
            actual_roll = q[5]  # right_arm_roll
        else:
            actual_pitch = q[2]  # left_arm_pitch
            actual_roll = q[3]  # left_arm_roll

        # 轨迹跟踪误差
        pitch_error = abs(actual_pitch - ref_pitch)
        roll_error = abs(actual_roll - ref_roll)

        reward = -pitch_error - 0.5 * roll_error

        # 平滑度奖励
        reward -= 0.001 * np.sum(qd ** 2)

        return float(reward)

    def _get_success(self) -> bool:
        return self._step_count >= self.max_episode_steps
