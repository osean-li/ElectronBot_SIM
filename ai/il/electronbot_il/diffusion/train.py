#!/usr/bin/env python3
"""
Diffusion Policy 训练脚本

用法:
  python train.py --data ./demos.h5 --epochs 500 --device cuda
"""

import os, sys, argparse
from pathlib import Path
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation" / "electronbot_mujoco"))

from electronbot_il.dataset import DemoDataset
from electronbot_il.diffusion.model import ActionUNet
from electronbot_il.diffusion.noise_scheduler import DDPMScheduler


def train_epoch(model, scheduler, dataloader, optimizer, device):
    model.train()
    total_loss = 0.0

    for obs, action in dataloader:
        obs = obs.to(device)         # (B, chunk, obs_dim)
        action = action.to(device)   # (B, chunk, act_dim)
        B, T, _ = action.shape

        # 取第一个观测作为条件
        obs_cond = obs[:, 0, :]      # (B, obs_dim)

        # 动作转换成 (B, act_dim, T) UNet 格式
        x_0 = action.permute(0, 2, 1)  # (B, act_dim, T)

        # 随机采样时间步
        timesteps = torch.randint(0, scheduler.num_train_timesteps, (B,), device=device)

        # 加噪
        noise = torch.randn_like(x_0)
        x_t, noise = scheduler.add_noise(x_0, timesteps, noise)

        # 模型预测噪声
        noise_pred = model(x_t, timesteps, obs_cond)

        # L2 损失
        loss = F.mse_loss(noise_pred, noise)

        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        total_loss += loss.item()

    return total_loss / len(dataloader)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, required=True, help="HDF5 演示数据")
    parser.add_argument("--epochs", type=int, default=500)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--chunk-size", type=int, default=100)
    parser.add_argument("--num-timesteps", type=int, default=100, help="DDPM 噪声步数")
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--output", type=str, default="./diffusion_checkpoint.pt")
    args = parser.parse_args()

    device = torch.device(args.device)
    print(f"[INFO] 设备: {device}")

    # 数据集
    dataset = DemoDataset(args.data, chunk_size=args.chunk_size)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True)
    print(f"[INFO] 数据集: {len(dataset)} chunks")

    # 模型 + 调度器
    scheduler = DDPMScheduler(num_train_timesteps=args.num_timesteps)
    model = ActionUNet(
        action_dim=6, obs_dim=18,
        hidden_dim=args.hidden_dim,
    ).to(device)
    print(f"[INFO] 参数量: {sum(p.numel() for p in model.parameters()):,}")

    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    best_loss = float("inf")

    for epoch in range(1, args.epochs + 1):
        train_loss = train_epoch(model, scheduler, loader, optimizer, device)

        if train_loss < best_loss:
            best_loss = train_loss
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "scheduler_config": {
                    "num_train_timesteps": args.num_timesteps,
                },
                "loss": best_loss,
            }, args.output)

        if epoch % 50 == 0:
            print(f"  Epoch {epoch:3d} | loss={train_loss:.6f} | best={best_loss:.6f}")

    print(f"[SUCCESS] 模型已保存: {args.output}")


if __name__ == "__main__":
    main()
