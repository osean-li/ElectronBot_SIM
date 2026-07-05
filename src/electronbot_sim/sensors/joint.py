"""JointSensor — 关节编码器传感器.

对齐 docs/tasks/05-Sensors-Observation 详细设计说明书 §4.

═══════════════════════════════════════════════════════════════════
  关键约束 (Sim2Real)
═══════════════════════════════════════════════════════════════════
  ⚠️ 真机 SG90/2g/4.3g 舵机无编码器, 以下数据真机不可得:
     - joint_vel (角速度)
     - ee_positions (末端执行器位置)
  obs_mode="realistic" 会移除这些字段, 仅保留 commanded_joint_pos.

  仿真中可加观测噪声 (对齐 §3.1 pos_noise_std / vel_noise_std),
  域随机化时用于训练鲁棒策略.
"""
from __future__ import annotations

import logging
import os
from typing import Dict

import numpy as np

logger = logging.getLogger("electronbot_sim.sensors.joint")


class JointSensor:
    """关节编码器传感器.

    参数:
        env: ElectronBotEnv 实例
    """

    def __init__(self, env):
        self.env = env
        # 观测噪声 (对齐设计文档 §6.1)
        # pos_noise_std: 0-5.0 度, 默认 0 (可在域随机化时设置)
        # vel_noise_std: 0-20.0 度/秒, 默认 0
        self.pos_noise_std = float(os.environ.get("ELECTRONBOT_POS_NOISE", 0.0))
        self.vel_noise_std = float(os.environ.get("ELECTRONBOT_VEL_NOISE", 0.0))
        # 噪声 RNG (与 env 共享, 保证可复现)
        self._rng = env.np_random

    def get_positions(self, add_noise: bool = True) -> np.ndarray:
        """获取 6 关节当前角度 (度).

        参数:
            add_noise: 是否添加观测噪声 (域随机化时启用)

        返回: (6,) float64, 顺序 [RP, RR, LP, LR, BODY, HEAD]
        """
        pos = self.env._get_joint_angles_deg().astype(np.float64)
        if add_noise and self.pos_noise_std > 0:
            pos = pos + self._rng.normal(0, self.pos_noise_std, size=6)
        return pos

    def get_velocities(self, add_noise: bool = True) -> np.ndarray:
        """获取 6 关节角速度 (度/秒).

        ⚠️ 真机无编码器, 此数据仅仿真可用.

        返回: (6,) float64
        """
        vel = self.env._get_joint_velocities_deg().astype(np.float64)
        if add_noise and self.vel_noise_std > 0:
            vel = vel + self._rng.normal(0, self.vel_noise_std, size=6)
        return vel

    def get_servo_angles(self) -> np.ndarray:
        """获取 6 舵机角度 (度, 由机械关节角度反推).

        使用 env.py 的 joint_array_to_servo_array() (单一数据源).
        供 get_trims / get_status 等工具使用.

        返回: (6,) float32, 舵机坐标系角度
        """
        from ..env import joint_array_to_servo_array
        joint_angles = self.env._get_joint_angles_deg()
        return joint_array_to_servo_array(joint_angles)

    def get_end_effector_positions(self) -> Dict[str, np.ndarray]:
        """获取左右末端执行器世界坐标 (米).

        ⚠️ 真机无编码器, 此数据仅仿真可用.

        返回:
            {"left": (3,) float32 m, "right": (3,) float32 m}
        """
        return {
            "left": self.env._get_ee_position("left_hand"),
            "right": self.env._get_ee_position("right_hand"),
        }

    def get_commanded_positions(self) -> np.ndarray:
        """获取最后指令的关节角度 (度, 真机可得).

        对应真机舵机指令值 (开环, 无编码器反馈).
        obs_mode="realistic" 使用此字段替代真实关节角度.

        返回: (6,) float32
        """
        return self.env.get_commanded_joint_pos()
