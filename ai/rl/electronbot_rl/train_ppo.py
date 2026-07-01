#!/usr/bin/env python3
"""
ElectronBot PPO 训练脚本

基于 Stable-Baselines3 的 PPO 算法，支持:
- VecEnv 并行 (DummyVecEnv / SubprocVecEnv)
- TensorBoard 日志
- Checkpoint 自动保存
- Domain Randomization

使用方法:
  python train_ppo.py --task reach --arm right --timesteps 1000000
"""

import os
import sys
import argparse
import yaml
from pathlib import Path

# 添加项目路径
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation" / "electronbot_mujoco"))

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.monitor import Monitor

from electronbot_mujoco.tasks import TASKS


def make_env(task_name: str, rank: int = 0, seed: int = 0, **kwargs):
    """创建环境工厂函数 (用于 VecEnv)"""

    def _init():
        task_cls = TASKS[task_name]
        env = task_cls(**kwargs)
        env = Monitor(env)
        env.reset(seed=seed + rank)
        return env

    return _init


def build_hyperparams(config_path: str = None) -> dict:
    """构建 PPO 超参数"""
    default = {
        "policy": "MlpPolicy",
        "learning_rate": 3e-4,
        "n_steps": 2048,
        "batch_size": 64,
        "n_epochs": 10,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip_range": 0.2,
        "ent_coef": 0.0,
        "vf_coef": 0.5,
        "max_grad_norm": 0.5,
        "policy_kwargs": dict(
            net_arch=dict(pi=[256, 256], vf=[256, 256]),
            activation_fn=torch.nn.ReLU,
        ),
    }

    if config_path and os.path.exists(config_path):
        with open(config_path, "r") as f:
            custom = yaml.safe_load(f)
            default.update(custom)

    return default


def main():
    parser = argparse.ArgumentParser(description="ElectronBot PPO 训练")
    parser.add_argument("--task", type=str, default="reach",
                        choices=list(TASKS.keys()),
                        help="训练任务")
    parser.add_argument("--arm", type=str, default="right",
                        choices=["left", "right", "both"],
                        help="使用的手臂")
    parser.add_argument("--timesteps", type=int, default=1_000_000,
                        help="总训练步数")
    parser.add_argument("--n-envs", type=int, default=4,
                        help="并行环境数")
    parser.add_argument("--seed", type=int, default=42,
                        help="随机种子")
    parser.add_argument("--log-dir", type=str, default="./logs/ppo",
                        help="日志目录")
    parser.add_argument("--model-dir", type=str, default="./models/ppo",
                        help="模型保存目录")
    parser.add_argument("--load", type=str, default=None,
                        help="加载已有模型继续训练")
    parser.add_argument("--config", type=str, default=None,
                        help="超参数 YAML 文件")
    args = parser.parse_args()

    # 创建目录
    log_dir = Path(args.log_dir) / f"{args.task}_{args.arm}_{args.seed}"
    model_dir = Path(args.model_dir) / f"{args.task}_{args.arm}_{args.seed}"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)

    print(f"={'>'*60}")
    print(f"ElectronBot PPO 训练")
    print(f"  任务: {args.task}")
    print(f"  手臂: {args.arm}")
    print(f"  环境数: {args.n_envs}")
    print(f"  总步数: {args.timesteps}")
    print(f"  日志: {log_dir}")
    print(f"{'='*60}")

    # 创建并行环境
    env_fns = [
        make_env(args.task, rank=i, seed=args.seed, arm=args.arm)
        for i in range(args.n_envs)
    ]

    if args.n_envs > 1 and False:  # TODO: SubprocVecEnv 需要 pickle 支持
        env = SubprocVecEnv(env_fns)
    else:
        env = DummyVecEnv(env_fns)

    # 可选: 归一化
    # env = VecNormalize(env, norm_obs=True, norm_reward=True)

    # 超参数
    hyperparams = build_hyperparams(args.config)
    hyperparams["tensorboard_log"] = str(log_dir)
    hyperparams["verbose"] = 1

    # 创建或加载模型
    if args.load:
        print(f"\n[INFO] 加载已有模型: {args.load}")
        model = PPO.load(args.load, env=env, **hyperparams)
    else:
        print(f"\n[INFO] 创建新模型")
        model = PPO(env=env, **hyperparams)

    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=max(args.timesteps // 10, 10000),
        save_path=str(model_dir),
        name_prefix="ppo_electronbot",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    callbacks = [checkpoint_cb]

    # 训练
    print(f"\n[INFO] 开始训练...")
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            tb_log_name=f"PPO_{args.task}_{args.arm}",
            progress_bar=True,
        )
    except KeyboardInterrupt:
        print(f"\n[INFO] 用户中断训练")

    # 保存最终模型
    final_path = os.path.join(model_dir, "final_model")
    model.save(final_path)
    print(f"\n[SUCCESS] 模型已保存: {final_path}.zip")

    env.close()


if __name__ == "__main__":
    main()
