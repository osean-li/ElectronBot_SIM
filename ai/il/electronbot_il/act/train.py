#!/usr/bin/env python3
"""
ACT (Action Chunking Transformer) 训练脚本

用法:
  python train.py --data ./demos.h5 --epochs 200 --device cuda
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
from electronbot_il.act.model import ACTModel


def train_epoch(model, dataloader, optimizer, device, beta: float = 1.0):
    model.train()
    total_loss = 0.0
    for obs, action in dataloader:
        obs = obs.to(device)        # (B, chunk, obs_dim)
        action = action.to(device)  # (B, chunk, act_dim)

        optimizer.zero_grad()
        action_pred, mu, logvar = model(obs, action)
        loss = model.loss_fn(action_pred, action, mu, logvar, beta)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        total_loss += loss.item()

    return total_loss / len(dataloader)


def validate(model, dataloader, device):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for obs, action in dataloader:
            obs, action = obs.to(device), action.to(device)
            action_pred, mu, logvar = model(obs, action)
            loss = model.loss_fn(action_pred, action, mu, logvar, beta=1.0)
            total_loss += loss.item()
    return total_loss / len(dataloader)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, required=True, help="HDF5 演示数据")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--chunk-size", type=int, default=100)
    parser.add_argument("--latent-dim", type=int, default=32)
    parser.add_argument("--hidden-dim", type=int, default=512)
    parser.add_argument("--beta", type=float, default=1.0, help="KL 权重")
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--output", type=str, default="./act_checkpoint.pt")
    args = parser.parse_args()

    device = torch.device(args.device)
    print(f"[INFO] 设备: {device}")

    # 数据集
    dataset = DemoDataset(args.data, chunk_size=args.chunk_size)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True)
    print(f"[INFO] 数据集: {len(dataset)} chunks, batch={args.batch_size}")

    # 模型
    model = ACTModel(
        obs_dim=18, action_dim=6,
        chunk_size=args.chunk_size,
        latent_dim=args.latent_dim,
        hidden_dim=args.hidden_dim,
    ).to(device)
    print(f"[INFO] 参数量: {sum(p.numel() for p in model.parameters()):,}")

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    best_loss = float("inf")

    for epoch in range(1, args.epochs + 1):
        train_loss = train_epoch(model, loader, optimizer, device, args.beta)

        if train_loss < best_loss:
            best_loss = train_loss
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "loss": best_loss,
            }, args.output)

        if epoch % 20 == 0:
            print(f"  Epoch {epoch:3d} | loss={train_loss:.6f} | best={best_loss:.6f}")

    print(f"[SUCCESS] 模型已保存: {args.output} (best_loss={best_loss:.6f})")


if __name__ == "__main__":
    main()
