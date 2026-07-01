#!/usr/bin/env python3
"""
ElectronBot 策略推理 & 录制脚本

功能:
- 加载训练好的 PPO/SAC/ACT/Diffusion 模型
- 在 MuJoCo 环境中 rollout
- 录制视频 (mp4)
- 可选: 通过 ROS2 接口控制 (验证 Sim2Real Bridge)

用法:
  python inference.py --model ./models/ppo/reach_right_42/final_model.zip --task reach --record
"""

import os
import sys
import argparse
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation" / "electronbot_mujoco"))

from stable_baselines3 import PPO, SAC
from electronbot_mujoco.tasks import TASKS


def record_rollout(env, model, max_steps: int = 500, video_path: str = None):
    """执行 rollout 并录制"""
    frames = []
    obs, _ = env.reset()
    total_reward = 0.0

    for step in range(max_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward

        if video_path:
            frame = env.render()
            if frame is not None:
                frames.append(frame)

        if terminated or truncated:
            break

    # 保存视频
    if video_path and frames:
        import imageio
        os.makedirs(os.path.dirname(video_path), exist_ok=True)
        imageio.mimsave(video_path, frames, fps=30)
        print(f"[INFO] 视频已保存: {video_path}")

    print(f"[INFO] Rollout 完成: {step+1} 步, 总奖励={total_reward:.2f}")
    return total_reward, step + 1


def load_model(model_path: str, env):
    """自动识别模型类型并加载"""
    abs_path = os.path.abspath(model_path)

    for cls, name in [(PPO, "PPO"), (SAC, "SAC")]:
        try:
            model = cls.load(abs_path, env=env)
            print(f"[INFO] 加载 {name} 模型: {abs_path}")
            return model
        except Exception:
            continue

    # 尝试加载 ACT/Diffusion checkpoint
    try:
        import torch
        checkpoint = torch.load(abs_path, map_location="cpu")
        print(f"[INFO] 加载 PyTorch checkpoint: {abs_path}")
        return SimplePolicy(checkpoint)
    except Exception:
        raise ValueError(f"无法加载模型: {abs_path}")


class SimplePolicy:
    """ACT/Diffusion 等 PyTorch checkpoint 的简单包装"""
    def __init__(self, checkpoint):
        self.ckpt = checkpoint
        self.model = checkpoint.get("model", None)

    def predict(self, obs, deterministic=True):
        if self.model is not None:
            with torch.no_grad():
                obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
                action = self.model.predict(obs_tensor)
                return action.squeeze(0).numpy(), None
        # Fallback: zero action
        return np.zeros(6), None


def main():
    parser = argparse.ArgumentParser(description="ElectronBot 策略推理")
    parser.add_argument("--model", type=str, required=True, help="模型路径 (.zip / .pt)")
    parser.add_argument("--task", type=str, default="reach", choices=list(TASKS.keys()))
    parser.add_argument("--arm", type=str, default="right")
    parser.add_argument("--record", action="store_true", help="录制视频")
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--video-dir", type=str, default="./videos",
                        help="视频保存目录")
    parser.add_argument("--ros2", action="store_true",
                        help="通过 ROS2 接口控制 (需先启动 bridge)")
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"[ERROR] 模型文件不存在: {args.model}")
        return 1

    # 创建环境
    task_cls = TASKS[args.task]
    env = task_cls(arm=args.arm, render_mode="rgb_array" if args.record else None)

    # 加载模型
    model = load_model(args.model, env)

    # 视频路径
    video_path = None
    if args.record:
        model_name = os.path.splitext(os.path.basename(args.model))[0]
        video_path = os.path.join(args.video_dir, f"{model_name}_{args.task}.mp4")

    # ROS2 模式
    if args.ros2:
        print("[INFO] ROS2 模式 (通过 /joint_trajectory_commands 控制)")
        # 这里不做实际 ROS2 发布，由 mujoco_ros2_bridge 节点处理
        # 推理节点只需产出动作序列

    # Run rollout
    total_reward, steps = record_rollout(
        env, model, args.max_steps, video_path
    )

    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
