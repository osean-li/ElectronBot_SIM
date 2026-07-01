#!/usr/bin/env python3
"""
ElectronBot SAC 训练脚本

与 PPO 对比: SAC 是 Off-Policy 算法，样本效率更高，
适合连续动作空间，但超参数调优更复杂。
"""
import os
import sys
import argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation" / "electronbot_mujoco"))

import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from electronbot_mujoco.tasks import TASKS


def main():
    parser = argparse.ArgumentParser(description="ElectronBot SAC 训练")
    parser.add_argument("--task", type=str, default="reach",
                        choices=list(TASKS.keys()))
    parser.add_argument("--arm", type=str, default="right")
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--log-dir", type=str, default="./logs/sac")
    parser.add_argument("--model-dir", type=str, default="./models/sac")
    args = parser.parse_args()

    log_dir = Path(args.log_dir) / f"{args.task}_{args.arm}_{args.seed}"
    model_dir = Path(args.model_dir) / f"{args.task}_{args.arm}_{args.seed}"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)

    print(f"={'='*60}")
    print(f"ElectronBot SAC 训练")
    print(f"  任务: {args.task} | 手臂: {args.arm}")
    print(f"  步数: {args.timesteps}")
    print(f"{'='*60}")

    # 环境
    task_cls = TASKS[args.task]
    env = task_cls(arm=args.arm)
    env = Monitor(env)
    env = DummyVecEnv([lambda: env])

    # SAC 超参数
    model = SAC(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        buffer_size=100_000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        policy_kwargs=dict(
            net_arch=dict(pi=[256, 256], qf=[256, 256]),
            activation_fn=torch.nn.ReLU,
        ),
        tensorboard_log=str(log_dir),
        verbose=1,
    )

    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=max(args.timesteps // 10, 10000),
        save_path=str(model_dir),
        name_prefix="sac_electronbot",
    )

    print(f"\n[INFO] 开始 SAC 训练...")
    model.learn(
        total_timesteps=args.timesteps,
        callback=[checkpoint_cb],
        tb_log_name=f"SAC_{args.task}_{args.arm}",
        progress_bar=True,
    )

    final_path = os.path.join(model_dir, "final_model")
    model.save(final_path)
    print(f"\n[SUCCESS] SAC 模型已保存: {final_path}.zip")
    env.close()


if __name__ == "__main__":
    main()
