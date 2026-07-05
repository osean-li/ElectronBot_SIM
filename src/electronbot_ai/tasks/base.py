"""任务抽象基类 — 所有训练任务的统一接口.

对齐 docs/tasks/06-AI-Training §2.1 BaseTask 接口设计.

设计原则:
  1. 任务逻辑与仿真环境解耦 — Task 通过 env 驱动仿真, 不直接持有 MuJoCo
  2. 统一观测字典 — 所有任务返回相同 schema 的 obs, 便于策略网络复用
  3. 奖励/成功判定分离 — compute_reward 供 RL, is_success 供评估
  4. 键盘示范接口 — get_demo_action 供 IL 数据收集

关节顺序 (全项目统一): [RP, RR, LP, LR, BODY, HEAD]
动作语义: 6 维关节角度增量 (度), 范围 [-2.0, 2.0] (训练用, 比 env 的 [-5,5] 更保守)
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.tasks.base")


class BaseTask(ABC):
    """所有训练任务的抽象基类.

    子类必须实现 5 个抽象方法:
      reset(env)          — 重置任务场景, 返回初始观测
      get_observation()   — 获取当前观测字典
      compute_reward()    — 计算当前步奖励 (RL 用)
      is_success()        — 判断任务是否成功 (评估用)
      get_demo_action()   — 键盘 → 关节增量 (IL 数据收集用)

    属性:
      name:        任务名 (如 "EB-Reach")
      difficulty:  难度等级 1-5
      obs_mode:    "full" (仿真研究) / "realistic" (Sim2Real 对齐)
    """

    name: str = "BaseTask"
    difficulty: int = 1

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None):
        self.obs_mode = obs_mode
        self.env: Optional[Any] = None
        self._rng = np.random.default_rng(seed)
        self._target_pos: Optional[np.ndarray] = None
        self._step_count = 0
        self._max_steps = 1000

    def bind(self, env: Any) -> "BaseTask":
        """绑定仿真环境 (ElectronBotEnv 实例)."""
        self.env = env
        return self

    # ================================================================
    #  抽象方法 (子类必须实现)
    # ================================================================

    @abstractmethod
    def reset(self, env: Any) -> dict:
        """重置任务 — 设置场景、放置物体、随机化参数.

        参数:
            env: ElectronBotEnv 实例

        返回:
            dict: 初始观测字典, 至少包含:
              - joint_pos: (6,) 关节角度 (度)
              - ee_pos: (3,) 末端执行器位置 (m)
              - target_pos: (3,) 目标位置 (m)
              - dist_to_target: () 到目标距离 (m)
        """

    @abstractmethod
    def get_observation(self) -> dict:
        """获取当前观测字典."""

    @abstractmethod
    def compute_reward(self) -> float:
        """计算当前步的奖励值 (RL 训练用)."""

    @abstractmethod
    def is_success(self) -> bool:
        """判断任务是否成功 (评估用)."""

    @abstractmethod
    def get_demo_action(self, keyboard_state: dict) -> np.ndarray:
        """从键盘状态获取示范动作 (IL 数据收集用).

        参数:
            keyboard_state: 按键状态字典, 如 {"w": True, "s": False, ...}

        返回:
            np.ndarray: (6,) 关节角度增量 (度)
        """

    # ================================================================
    #  公共工具方法 (子类可复用)
    # ================================================================

    def step(self, action: np.ndarray):
        """推进一个仿真步 + 任务逻辑更新.

        返回: (obs, reward, terminated, truncated, info)
        """
        if self.env is None:
            raise RuntimeError("Task 未绑定 env, 请先调用 task.bind(env)")
        obs_env, _, _, truncated, info = self.env.step(action)
        self._step_count += 1
        obs = self.get_observation()
        reward = self.compute_reward()
        terminated = self.is_success()
        info["task_name"] = self.name
        info["step_count"] = self._step_count
        info["dist_to_target"] = float(obs.get("dist_to_target", 0.0))
        if self._step_count >= self._max_steps:
            truncated = True
        return obs, reward, terminated, truncated, info

    @property
    def target_pos(self) -> Optional[np.ndarray]:
        return self._target_pos

    def _get_ee_pos(self, body_name: str = "right_hand") -> np.ndarray:
        """获取末端执行器世界坐标 (m)."""
        if self.env is None:
            return np.zeros(3, dtype=np.float32)
        return self.env.get_ee_position(body_name)

    def _get_joint_pos(self) -> np.ndarray:
        """获取 6 关节当前角度 (度)."""
        if self.env is None:
            return np.zeros(6, dtype=np.float32)
        return self.env.get_joint_positions()

    def _build_base_obs(self) -> dict:
        """构建基础观测字典 (子类可扩展)."""
        joint_pos = self._get_joint_pos()
        ee_pos = self._get_ee_pos("right_hand")
        target = self._target_pos if self._target_pos is not None else np.zeros(3, dtype=np.float32)
        dist = float(np.linalg.norm(ee_pos - target))
        obs = {
            "joint_pos": joint_pos.astype(np.float32),
            "ee_pos": ee_pos.astype(np.float32),
            "target_pos": target.astype(np.float32),
            "dist_to_target": np.array(dist, dtype=np.float32),
        }
        # realistic 模式: 移除真机不可获取的字段
        if self.obs_mode == "realistic":
            obs.pop("ee_pos", None)
            obs["commanded_joint_pos"] = self.env.get_commanded_joint_pos().astype(np.float32)
            battery = self.env.get_battery_info()
            obs["battery_voltage"] = np.array(battery["voltage"], dtype=np.float32)
            obs["battery_percent"] = np.array(battery["percent"], dtype=np.float32)
        return obs
