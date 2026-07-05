"""示范数据收集工具 — 键盘遥控收集 IL 训练数据.

对齐 docs/tasks/06-AI-Training §3.1.

数据格式: robomimic 兼容的 HDF5
  demo_<task>.hdf5
  ├── attrs: {"total": N}
  └── data/
      ├── demo_0/
      │   ├── obs         (T, obs_dim)
      │   ├── actions     (T, 6)
      │   └── attrs: {"num_samples": T}
      └── ...

使用方式:
  # 交互式收集
  python -m electronbot_ai.il.collect_demos --task reach --num_episodes 50

  # 编程式收集 (随机策略生成基线数据)
  from electronbot_ai.il.collect_demos import DemoCollector
"""
from __future__ import annotations

import logging
import os
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.il.collect_demos")


def _flatten_obs(obs: dict) -> np.ndarray:
    """将观测字典展平为 1D 向量 (供 BC/ACT 训练用).

    顺序: joint_pos(6) + ee_pos(3) + target_pos(3) + dist(1) = 13 维
    (realistic 模式: commanded_joint_pos(6) + battery(2) + ... = 自适应)
    """
    parts = []
    for key in ["joint_pos", "commanded_joint_pos", "ee_pos",
                "target_pos", "target_joint_pos", "object_pos",
                "container_pos", "block_a_pos", "block_b_pos"]:
        if key in obs:
            val = obs[key]
            if isinstance(val, np.ndarray):
                parts.append(val.flatten().astype(np.float32))
    if "dist_to_target" in obs:
        parts.append(np.array([obs["dist_to_target"]], dtype=np.float32))
    if "dist_object_to_target" in obs:
        parts.append(np.array([obs["dist_object_to_target"]], dtype=np.float32))
    if "joint_error" in obs:
        parts.append(np.array([obs["joint_error"]], dtype=np.float32))
    if not parts:
        return np.array([], dtype=np.float32)
    return np.concatenate(parts)


class DemoCollector:
    """键盘遥控示范数据收集器.

    参数:
        env:      ElectronBotEnv 实例
        task:     BaseTask 实例 (已 bind env)
        save_path: HDF5 保存路径
    """

    def __init__(self, env: Any, task: Any, save_path: str):
        self.env = env
        self.task = task
        self.save_path = Path(save_path)
        self.episodes: list[dict] = []

    def collect_episode_random(self, max_steps: int = 200) -> dict:
        """使用随机策略收集一条轨迹 (基线数据/测试用).

        生产环境应使用 collect_episode_keyboard 进行人工示范.
        """
        obs_list, action_list = [], []
        obs = self.task.reset(self.env)
        done = False

        for _ in range(max_steps):
            action = self.env.action_space.sample().astype(np.float32) * 0.5  # 缩小随机范围
            obs_flat = _flatten_obs(obs)
            obs_list.append(obs_flat)
            action_list.append(action)
            obs, _, done, truncated, _ = self.task.step(action)
            if done or truncated:
                break

        return {
            "observations": np.array(obs_list, dtype=np.float32),
            "actions": np.array(action_list, dtype=np.float32),
        }

    def collect_episode_keyboard(self, max_steps: int = 300) -> dict:
        """键盘遥控收集一条示范轨迹.

        ⚠️ 需要 GUI 环境 (pygame/pynput), 无头环境请用 collect_episode_random.
        """
        try:
            import pygame
        except ImportError as e:
            logger.warning("pygame 未安装, 回退到随机策略: %s", e)
            return self.collect_episode_random(max_steps)

        pygame.init()
        screen = pygame.display.set_mode((400, 200))
        pygame.display.set_caption(f"Demo Collector: {self.task.name}")
        clock = pygame.time.Clock()

        obs_list, action_list = [], []
        obs = self.task.reset(self.env)
        done = False
        running = True

        while running and not done and len(obs_list) < max_steps:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        done = True  # 标记成功

            keys = pygame.key.get_pressed()
            keyboard_state = {
                "w": keys[pygame.K_w], "s": keys[pygame.K_s],
                "a": keys[pygame.K_a], "d": keys[pygame.K_d],
                "e": keys[pygame.K_e], "f": keys[pygame.K_f],
                "q": keys[pygame.K_q], "z": keys[pygame.K_z],
                "r": keys[pygame.K_r], "t": keys[pygame.K_t],
                "g": keys[pygame.K_g], "h": keys[pygame.K_h],
            }
            action = self.task.get_demo_action(keyboard_state)
            obs_flat = _flatten_obs(obs)
            obs_list.append(obs_flat)
            action_list.append(action)
            obs, _, done, truncated, _ = self.task.step(action)
            if truncated:
                break

            self.env.render()
            clock.tick(50)  # 50Hz

        pygame.quit()
        return {
            "observations": np.array(obs_list, dtype=np.float32),
            "actions": np.array(action_list, dtype=np.float32),
        }

    def save(self) -> Path:
        """保存所有已收集的轨迹为 HDF5 格式 (robomimic 兼容)."""
        try:
            import h5py
        except ImportError:
            logger.error("h5py 未安装, 无法保存 HDF5. 请: pip install h5py")
            # 回退到 numpy 格式
            return self._save_numpy()

        self.save_path.parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(self.save_path, "w") as f:
            data_grp = f.create_group("data")
            for i, ep in enumerate(self.episodes):
                ep_grp = data_grp.create_group(f"demo_{i}")
                ep_grp.create_dataset("obs", data=ep["observations"])
                ep_grp.create_dataset("actions", data=ep["actions"])
                ep_grp.attrs["num_samples"] = len(ep["actions"])
            f.attrs["total"] = len(self.episodes)
            f.attrs["task_name"] = self.task.name
            f.attrs["created_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")

        logger.info("示范数据已保存: %s (%d 条轨迹)", self.save_path, len(self.episodes))
        return self.save_path

    def _save_numpy(self) -> Path:
        """回退: 保存为 .npz 格式."""
        path = self.save_path.with_suffix(".npz")
        np.savez_compressed(path,
                            episodes=self.episodes,
                            task_name=self.task.name)
        logger.info("示范数据已保存 (npz): %s", path)
        return path

    def collect_batch(self, num_episodes: int, use_keyboard: bool = False,
                      max_steps: int = 200) -> Path:
        """批量收集示范数据.

        参数:
            num_episodes:  收集的轨迹数
            use_keyboard:  True=键盘遥控, False=随机策略
            max_steps:     每条轨迹最大步数

        返回:
            Path: 保存的文件路径
        """
        collect_fn = self.collect_episode_keyboard if use_keyboard else self.collect_episode_random
        method = "键盘" if use_keyboard else "随机"
        logger.info("开始收集 %d 条 %s示范 (任务=%s)", num_episodes, method, self.task.name)

        for i in range(num_episodes):
            ep = collect_fn(max_steps=max_steps)
            if len(ep["actions"]) > 0:
                self.episodes.append(ep)
                logger.info("  轨迹 %d: %d 步", i + 1, len(ep["actions"]))
            else:
                logger.warning("  轨迹 %d 为空, 跳过", i + 1)

        return self.save()


def main():
    """CLI 入口: python -m electronbot_ai.il.collect_demos --task reach --num_episodes 50"""
    import argparse
    from electronbot_sim.env import ElectronBotEnv
    from electronbot_ai.tasks import create_task

    parser = argparse.ArgumentParser(description="收集 IL 示范数据")
    parser.add_argument("--task", type=str, required=True, help="任务名 (reach/push/...)")
    parser.add_argument("--num_episodes", type=int, default=50, help="轨迹数")
    parser.add_argument("--output", type=str, default=None, help="输出路径")
    parser.add_argument("--keyboard", action="store_true", help="使用键盘遥控 (默认随机)")
    parser.add_argument("--max_steps", type=int, default=200, help="每条轨迹最大步数")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    args = parser.parse_args()

    env = ElectronBotEnv(render_mode="human" if args.keyboard else None)
    task = create_task(args.task, seed=args.seed)
    output = args.output or f"demos/demo_{args.task}.hdf5"
    collector = DemoCollector(env, task, output)
    collector.collect_batch(
        num_episodes=args.num_episodes,
        use_keyboard=args.keyboard,
        max_steps=args.max_steps,
    )


if __name__ == "__main__":
    main()
