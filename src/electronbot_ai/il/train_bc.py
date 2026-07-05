"""Behavior Cloning (BC) 训练脚本.

对齐 docs/tasks/06-AI-Training §3.2.

策略网络: 4 层 MLP (256-256-256-act_dim), ReLU 激活, MSE 损失
输入: 观测向量 (展平)
输出: 6 维关节角度增量

使用方式:
  python -m electronbot_ai.il.train_bc --task reach --epochs 100 --demo demos/demo_reach.hdf5
"""
from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.il.train_bc")

# 延迟导入 torch (仅在训练时需要)
_torch = None


def _get_torch():
    global _torch
    if _torch is None:
        try:
            import torch
            import torch.nn as nn
            _torch = (torch, nn)
        except ImportError as e:
            raise ImportError(
                "PyTorch 未安装, 请: pip install torch. "
                f"原始错误: {e}"
            ) from e
    return _torch


class BCPolicy:
    """Behavior Cloning 策略网络 (4 层 MLP).

    对齐 §3.2: nn.Linear(obs_dim, 256) → 256 → 256 → act_dim
    """

    def __init__(self, obs_dim: int, act_dim: int = 6,
                 hidden_dim: int = 256, num_layers: int = 4):
        torch, nn = _get_torch()
        self.torch = torch
        self.obs_dim = obs_dim
        self.act_dim = act_dim

        layers = []
        in_dim = obs_dim
        for _ in range(num_layers - 1):
            layers.extend([nn.Linear(in_dim, hidden_dim), nn.ReLU()])
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, act_dim))
        self.net = nn.Sequential(*layers)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net = self.net.to(self.device)

    def predict(self, obs: np.ndarray) -> np.ndarray:
        """推理 (评估用)."""
        self.net.eval()
        with self.torch.no_grad():
            obs_t = self.torch.as_tensor(obs, dtype=self.torch.float32, device=self.device)
            if obs_t.dim() == 1:
                obs_t = obs_t.unsqueeze(0)
            action = self.net(obs_t)
        return action.cpu().numpy().flatten()

    def save(self, path: str):
        """保存权重."""
        self.torch.save(self.net.state_dict(), path)
        logger.info("BC 策略权重已保存: %s", path)

    def load(self, path: str):
        """加载权重."""
        self.net.load_state_dict(self.torch.load(path, map_location=self.device))
        logger.info("BC 策略权重已加载: %s", path)

    @property
    def name(self) -> str:
        return "bc"

    def reset(self):
        """episode 开始时重置 (BC 无状态, 空操作)."""
        pass


def load_demos(demo_path: str) -> tuple[np.ndarray, np.ndarray]:
    """加载 HDF5 示范数据, 返回 (obs, actions) 拼接张量.

    支持 HDF5 和 npz 两种格式.
    """
    path = Path(demo_path)
    if not path.exists():
        raise FileNotFoundError(f"示范数据文件不存在: {demo_path}")

    all_obs, all_act = [], []

    if path.suffix == ".hdf5":
        try:
            import h5py
        except ImportError as e:
            raise ImportError("h5py 未安装, 无法读取 HDF5") from e
        with h5py.File(path, "r") as f:
            for key in f["data"]:
                all_obs.append(f[f"data/{key}/obs"][:])
                all_act.append(f[f"data/{key}/actions"][:])
    elif path.suffix == ".npz":
        data = np.load(path, allow_pickle=True)
        episodes = data["episodes"]
        for ep in episodes:
            all_obs.append(ep["observations"])
            all_act.append(ep["actions"])
    else:
        raise ValueError(f"不支持的文件格式: {path.suffix}")

    if not all_obs:
        raise ValueError(f"示范数据为空: {demo_path}")

    return np.concatenate(all_obs), np.concatenate(all_act)


def train_bc(demo_path: str, epochs: int = 100, batch_size: int = 64,
             lr: float = 1e-3, output_path: Optional[str] = None) -> BCPolicy:
    """训练 BC 策略.

    参数:
        demo_path:   示范数据路径 (HDF5/npz)
        epochs:      训练轮数
        batch_size:  批大小
        lr:          学习率
        output_path: 权重保存路径 (None 则不保存)

    返回:
        BCPolicy: 训练好的策略
    """
    torch, _ = _get_torch()

    # 加载数据
    obs, actions = load_demos(demo_path)
    obs_t = torch.as_tensor(obs, dtype=torch.float32, device="cuda" if torch.cuda.is_available() else "cpu")
    act_t = torch.as_tensor(actions, dtype=torch.float32, device="cuda" if torch.cuda.is_available() else "cpu")
    logger.info("示范数据加载: %d 样本, obs_dim=%d, act_dim=%d",
                len(obs), obs.shape[1], actions.shape[1])

    # 构建策略
    policy = BCPolicy(obs_dim=obs.shape[1], act_dim=actions.shape[1])
    optimizer = torch.optim.Adam(policy.net.parameters(), lr=lr)
    loss_fn = torch.nn.MSELoss()

    # 训练循环
    n_samples = len(obs)
    nan_count = 0

    for epoch in range(epochs):
        # 随机采样 batch
        idx = torch.randperm(n_samples, device=policy.device)[:batch_size]
        batch_obs = obs_t[idx]
        batch_act = act_t[idx]

        # 前向 + 反向
        pred = policy.net(batch_obs)
        loss = loss_fn(pred, batch_act)

        # NaN 检测
        if torch.isnan(loss) or torch.isinf(loss):
            nan_count += 1
            logger.warning("Epoch %d: NaN/Inf loss 检测 (count=%d)", epoch, nan_count)
            if nan_count >= 3:
                # 降低学习率
                for pg in optimizer.param_groups:
                    pg["lr"] *= 0.1
                logger.warning("学习率降至 %e", optimizer.param_groups[0]["lr"])
                nan_count = 0
            continue

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if epoch % 10 == 0:
            logger.info("Epoch %d: loss=%.6f", epoch, loss.item())

    # 保存
    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        policy.save(output_path)

    return policy


def evaluate_bc(policy: BCPolicy, task, env, num_episodes: int = 40) -> float:
    """评估 BC 策略成功率.

    参数:
        policy:      BCPolicy 实例
        task:        BaseTask 实例
        env:         ElectronBotEnv 实例
        num_episodes: 评估回合数

    返回:
        float: 成功率 [0, 1]
    """
    successes = 0
    task.bind(env)

    for ep in range(num_episodes):
        obs = task.reset(env)
        done = False
        steps = 0
        max_steps = 1000

        while not done and steps < max_steps:
            # 构建观测向量 (需要与训练时展平方式一致)
            from .collect_demos import _flatten_obs
            obs_vec = _flatten_obs(obs)
            if len(obs_vec) == 0:
                break
            action = policy.predict(obs_vec)
            obs, _, done, truncated, _ = task.step(action)
            steps += 1
            if truncated:
                break

        if task.is_success():
            successes += 1

    success_rate = successes / num_episodes
    logger.info("BC 评估完成: %d/%d 成功 (%.1%%%)",
                successes, num_episodes, success_rate * 100)
    return success_rate


def main():
    """CLI 入口."""
    import argparse
    from electronbot_sim.env import ElectronBotEnv
    from electronbot_ai.tasks import create_task

    parser = argparse.ArgumentParser(description="训练 BC 策略")
    parser.add_argument("--task", type=str, required=True)
    parser.add_argument("--demo", type=str, required=True, help="示范数据路径")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--eval_episodes", type=int, default=40)
    args = parser.parse_args()

    output = args.output or f"checkpoints/bc_{args.task}.pt"
    policy = train_bc(args.demo, args.epochs, args.batch_size, args.lr, output)

    # 评估
    env = ElectronBotEnv(render_mode=None)
    task = create_task(args.task)
    sr = evaluate_bc(policy, task, env, args.eval_episodes)
    print(f"BC @ {task.name}: 成功率 {sr:.1%}")


if __name__ == "__main__":
    main()
