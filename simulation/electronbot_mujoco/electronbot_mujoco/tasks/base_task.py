"""
Benchmark 任务基类

所有任务继承此类，需实现:
- _compute_reward(): 奖励函数
- _is_terminated(): 终止条件
- _get_success(): 成功判断
"""

from abc import ABC, abstractmethod
import numpy as np
from ..env import ElectronBotEnv


class BaseTask(ElectronBotEnv):
    """任务基类"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._success = False
        self._target_pos = None

    def reset(self, seed=None, options=None):
        self._success = False
        if options is None:
            options = {}
        options["qpos"] = self._get_initial_qpos()
        return super().reset(seed=seed, options=options)

    def _get_initial_qpos(self) -> np.ndarray:
        """默认初始位置 (零位)"""
        return np.zeros(self.NUM_JOINTS)

    @abstractmethod
    def _compute_reward(self) -> float:
        """计算当前步的奖励"""
        pass

    def _is_terminated(self) -> bool:
        """判断 episode 是否结束 (默认: 达到成功条件)"""
        return self._get_success()

    @abstractmethod
    def _get_success(self) -> bool:
        """判断是否完成任务"""
        pass

    def _get_info(self):
        info = super()._get_info()
        info["success"] = float(self._get_success())
        return info

    # ---- 辅助 ----

    def _get_ee_positions(self):
        """获取左右末端执行器位置"""
        return self.robot.get_end_effector_positions()

    def _get_joint_positions_deg(self):
        """获取关节角度 (度)"""
        return self.robot.get_joint_positions_deg()
