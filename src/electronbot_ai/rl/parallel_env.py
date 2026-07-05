"""并行环境构建 — SubprocVecEnv 64 并行.

对齐 docs/tasks/06-AI-Training §4.1.

将 ElectronBotEnv + Task 包装为 Gymnasium 兼容环境,
再用 Stable-Baselines3 的 SubprocVecEnv 实现多进程并行.
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.rl.parallel_env")


class TaskWrapper:
    """将 BaseTask 包装为 Gymnasium 兼容环境.

    提供:
      - reset() → obs (展平向量, 供 SB3 使用)
      - step(action) → (obs, reward, terminated, truncated, info)
      - observation_space / action_space 属性
    """

    def __init__(self, env, task, obs_mode: str = "full"):
        self.env = env
        self.task = task
        self.obs_mode = obs_mode
        task.bind(env)

        # 从 env 继承空间定义
        self.action_space = env.action_space

        # 观测展平后的维度 (通过一次 reset 探测)
        obs, _ = self.reset()
        import gymnasium as gym
        self._obs_dim = len(obs)
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._obs_dim,), dtype=np.float32
        )
        self.metadata = env.metadata

    def _flatten_obs(self, obs_dict: dict) -> np.ndarray:
        """将观测字典展平为 1D 向量 (供 SB3 使用)."""
        from ..il.collect_demos import _flatten_obs
        return _flatten_obs(obs_dict)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        """重置环境 + 任务, 返回 (obs_vector, info)."""
        obs_dict = self.task.reset(self.env)
        obs_vec = self._flatten_obs(obs_dict).astype(np.float32)
        info = {"task_name": self.task.name}
        return obs_vec, info

    def step(self, action: np.ndarray):
        """执行一步, 返回 Gymnasium 5-tuple."""
        obs_dict, reward, terminated, truncated, info = self.task.step(action)
        obs_vec = self._flatten_obs(obs_dict).astype(np.float32)
        return obs_vec, float(reward), bool(terminated), bool(truncated), info

    def render(self):
        return self.env.render()

    def close(self):
        self.env.close()

    @property
    def spec(self):
        return None


def make_env(task_name: str, rank: int, seed: int = 0,
             render: bool = False, obs_mode: str = "full") -> Callable:
    """创建环境工厂函数 (供 SubprocVecEnv 使用).

    参数:
        task_name: 任务名
        rank:      环境序号 (用于种子偏移)
        seed:      基础种子
        render:    是否渲染
        obs_mode:  观测模式

    返回:
        Callable: 无参函数, 返回 TaskWrapper 实例
    """
    def _init():
        from electronbot_sim.env import ElectronBotEnv
        from electronbot_ai.tasks import create_task

        env = ElectronBotEnv(
            render_mode="human" if render else None,
            obs_mode=obs_mode,
        )
        task = create_task(task_name, obs_mode=obs_mode, seed=seed + rank)
        wrapped = TaskWrapper(env, task, obs_mode=obs_mode)
        return wrapped

    return _init


def make_vec_envs(task_name: str, num_envs: int = 64, seed: int = 0,
                  obs_mode: str = "full", use_subproc: bool = True):
    """创建并行向量化环境.

    参数:
        task_name:    任务名
        num_envs:     并行环境数
        seed:         基础种子
        obs_mode:     观测模式
        use_subproc:  True=SubprocVecEnv (多进程), False=DummyVecEnv (单进程, 调试用)

    返回:
        VecEnv: 向量化环境
    """
    try:
        from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
    except ImportError as e:
        raise ImportError(
            "Stable-Baselines3 未安装, 请: pip install stable-baselines3. "
            f"原始错误: {e}"
        ) from e

    env_fns = [make_env(task_name, i, seed, render=False, obs_mode=obs_mode)
               for i in range(num_envs)]

    if use_subproc and num_envs > 1:
        vec_env = SubprocVecEnv(env_fns)
        logger.info("SubprocVecEnv: %d 并行环境", num_envs)
    else:
        vec_env = DummyVecEnv(env_fns)
        logger.info("DummyVecEnv: %d 环境 (单进程)", num_envs)

    return vec_env
