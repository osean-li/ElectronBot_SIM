"""PPO 训练脚本 — Stable-Baselines3.

对齐 docs/tasks/06-AI-Training §4.2.

使用 SubprocVecEnv 实现 64 并行环境 PPO 训练.
训练完成后导出 ONNX 模型供真机部署 (路径 B).

使用方式:
  python -m electronbot_ai.rl.train_ppo \
      --task pick_place \
      --num_envs 64 \
      --total_steps 1000000 \
      --output checkpoints/ppo_pick_place.zip
"""
from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.rl.train_ppo")


def train_ppo(task_name: str, num_envs: int = 64, total_steps: int = 1_000_000,
              output: str = "checkpoints/ppo_model.zip",
              eval_freq: int = 10000, n_eval_episodes: int = 20,
              tensorboard_log: str = "logs/",
              use_domain_randomization: bool = True,
              obs_mode: str = "full") -> str:
    """PPO 训练入口.

    参数:
        task_name:                  任务名
        num_envs:                   并行环境数
        total_steps:                总训练步数
        output:                     模型输出路径
        eval_freq:                  评估频率 (步)
        n_eval_episodes:            每次评估回合数
        tensorboard_log:            TensorBoard 日志目录
        use_domain_randomization:   是否启用域随机化
        obs_mode:                   观测模式

    返回:
        str: 模型保存路径
    """
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
        from stable_baselines3.common.vec_env import VecMonitor
    except ImportError as e:
        raise ImportError(
            "Stable-Baselines3 未安装, 请: pip install stable-baselines3. "
            f"原始错误: {e}"
        ) from e

    from .parallel_env import make_vec_envs
    from .domain_randomization import DomainRandomizationWrapper

    logger.info("PPO 训练启动: task=%s, num_envs=%d, total_steps=%d",
                task_name, num_envs, total_steps)

    # 创建训练环境
    train_env = make_vec_envs(task_name, num_envs=num_envs, obs_mode=obs_mode)
    if use_domain_randomization:
        # 域随机化在 ElectronBotEnv.reset() 内部已实现,
        # 此处可额外添加 DomainRandomizationWrapper (如需更激进随机化)
        pass
    train_env = VecMonitor(train_env)

    # 创建评估环境 (4 个, 无域随机化)
    eval_env = make_vec_envs(task_name, num_envs=4, obs_mode=obs_mode, use_subproc=False)
    eval_env = VecMonitor(eval_env)

    # PPO 超参 (对齐 §4.2)
    model = PPO(
        "MlpPolicy",
        train_env,
        policy_kwargs=dict(
            net_arch=dict(pi=[256, 256], vf=[256, 256]),
        ),
        n_steps=2048,
        batch_size=512,
        n_epochs=10,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=tensorboard_log,
    )

    # 回调
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(Path(output).parent),
        eval_freq=eval_freq,
        n_eval_episodes=n_eval_episodes,
        deterministic=True,
    )
    checkpoint_callback = CheckpointCallback(
        save_freq=50000,
        save_path=str(Path(output).parent / "periodic"),
        name_prefix=f"ppo_{task_name}",
    )

    # 训练
    logger.info("开始训练...")
    model.learn(
        total_timesteps=total_steps,
        callback=[eval_callback, checkpoint_callback],
    )

    # 保存最终模型
    model.save(output)
    logger.info("PPO 模型已保存: %s", output)

    # 导出 ONNX (可选, 供路径 B 部署)
    try:
        onnx_path = str(Path(output).with_suffix(".onnx"))
        _export_onnx(model, onnx_path, train_env)
        logger.info("ONNX 模型已导出: %s", onnx_path)
    except Exception as e:
        logger.warning("ONNX 导出失败 (非致命): %s", e)

    train_env.close()
    eval_env.close()
    return output


def _export_onnx(model, onnx_path: str, vec_env) -> None:
    """导出 PPO 策略为 ONNX 格式."""
    import torch

    class PolicyWrapper(torch.nn.Module):
        def __init__(self, policy):
            super().__init__()
            self.policy = policy

        def forward(self, obs):
            return self.policy(obs)

    policy_net = model.policy
    policy_net.eval()
    dummy_input = torch.zeros(1, vec_env.observation_space.shape[0])

    torch.onnx.export(
        PolicyWrapper(policy_net),
        dummy_input,
        onnx_path,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
    )


def main():
    """CLI 入口."""
    import argparse

    parser = argparse.ArgumentParser(description="PPO 训练")
    parser.add_argument("--task", type=str, required=True, help="任务名")
    parser.add_argument("--num_envs", type=int, default=64, help="并行环境数")
    parser.add_argument("--total_steps", type=int, default=1_000_000)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--eval_freq", type=int, default=10000)
    parser.add_argument("--n_eval_episodes", type=int, default=20)
    parser.add_argument("--tensorboard_log", type=str, default="logs/")
    parser.add_argument("--no_dr", action="store_true", help="禁用域随机化")
    parser.add_argument("--obs_mode", type=str, default="full")
    args = parser.parse_args()

    output = args.output or f"checkpoints/ppo_{args.task}.zip"
    train_ppo(
        task_name=args.task,
        num_envs=args.num_envs,
        total_steps=args.total_steps,
        output=output,
        eval_freq=args.eval_freq,
        n_eval_episodes=args.n_eval_episodes,
        tensorboard_log=args.tensorboard_log,
        use_domain_randomization=not args.no_dr,
        obs_mode=args.obs_mode,
    )


if __name__ == "__main__":
    main()
