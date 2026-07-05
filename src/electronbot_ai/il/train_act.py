"""Action Chunking Transformer (ACT) 训练脚本.

对齐 docs/tasks/06-AI-Training §3.3.

核心思想: 一次预测未来 K 个动作, 减少累积误差
网络结构: Linear 编码器 + TransformerEncoder(4层,8头) + Linear 解码器
输入: 观测序列 (B, T, obs_dim)
输出: 动作块 (B, chunk_size, 6)

使用方式:
  python -m electronbot_ai.il.train_act --task reach --epochs 200 --demo demos/demo_reach.hdf5
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import numpy as np

from .train_bc import load_demos

logger = logging.getLogger("electronbot_ai.il.train_act")

_torch = None


def _get_torch():
    global _torch
    if _torch is None:
        try:
            import torch
            import torch.nn as nn
            _torch = (torch, nn)
        except ImportError as e:
            raise ImportError(f"PyTorch 未安装: {e}") from e
    return _torch


class ACTPolicy:
    """Action Chunking Transformer 策略.

    对齐 §3.3: 一次预测未来 chunk_size 个动作
    """

    def __init__(self, obs_dim: int, act_dim: int = 6, chunk_size: int = 10,
                 d_model: int = 256, nhead: int = 8, num_layers: int = 4):
        torch, nn = _get_torch()
        self.torch = torch
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.chunk_size = chunk_size

        # 观测编码器
        self.obs_encoder = nn.Linear(obs_dim, d_model)

        # Transformer 编码器
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead, batch_first=True,
            dim_feedforward=d_model * 4, dropout=0.1,
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        # 动作解码器
        self.act_decoder = nn.Sequential(
            nn.Linear(d_model, d_model), nn.ReLU(),
            nn.Linear(d_model, chunk_size * act_dim),
        )

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        for module in [self.obs_encoder, self.transformer, self.act_decoder]:
            module.to(self.device)

    def forward(self, obs_sequence):
        """前向推理.

        参数:
            obs_sequence: (B, T, obs_dim) 观测序列

        返回:
            (B, chunk_size, act_dim) 动作块
        """
        x = self.obs_encoder(obs_sequence)
        x = self.transformer(x)
        x = x[:, -1, :]  # 取最后一个时间步
        act_chunk = self.act_decoder(x)
        return act_chunk.reshape(-1, self.chunk_size, self.act_dim)

    def predict(self, obs_sequence: np.ndarray) -> np.ndarray:
        """推理 (评估用). 返回第一个动作."""
        self.act_decoder.eval()
        self.transformer.eval()
        self.obs_encoder.eval()
        with self.torch.no_grad():
            obs_t = self.torch.as_tensor(obs_sequence, dtype=self.torch.float32,
                                         device=self.device)
            if obs_t.dim() == 2:
                obs_t = obs_t.unsqueeze(0)
            act_chunk = self.forward(obs_t)
        return act_chunk[0, 0].cpu().numpy()  # 返回第一个动作

    def save(self, path: str):
        """保存权重."""
        state = {
            "obs_encoder": self.obs_encoder.state_dict(),
            "transformer": self.transformer.state_dict(),
            "act_decoder": self.act_decoder.state_dict(),
        }
        self.torch.save(state, path)
        logger.info("ACT 策略权重已保存: %s", path)

    def load(self, path: str):
        """加载权重."""
        state = self.torch.load(path, map_location=self.device)
        self.obs_encoder.load_state_dict(state["obs_encoder"])
        self.transformer.load_state_dict(state["transformer"])
        self.act_decoder.load_state_dict(state["act_decoder"])

    @property
    def name(self) -> str:
        return "act"

    def reset(self):
        """episode 开始时重置内部状态."""
        self._obs_buffer = []


def build_sequences(obs: np.ndarray, actions: np.ndarray,
                    seq_len: int, chunk_size: int) -> tuple:
    """将轨迹切分为 (观测序列, 动作块) 训练样本.

    参数:
        obs:        (N, obs_dim)
        actions:    (N, act_dim)
        seq_len:    输入观测序列长度 T
        chunk_size: 预测动作块长度 K

    返回:
        obs_seqs:   (M, T, obs_dim)
        act_chunks: (M, K, act_dim)
    """
    N = len(obs)
    M = max(0, N - seq_len - chunk_size + 1)
    if M == 0:
        return np.array([]), np.array([])

    obs_seqs = np.array([obs[i:i + seq_len] for i in range(M)], dtype=np.float32)
    act_chunks = np.array([actions[i + seq_len:i + seq_len + chunk_size]
                           for i in range(M)], dtype=np.float32)
    return obs_seqs, act_chunks


def train_act(demo_path: str, epochs: int = 200, batch_size: int = 32,
              lr: float = 1e-4, chunk_size: int = 10, seq_len: int = 10,
              output_path: Optional[str] = None) -> ACTPolicy:
    """训练 ACT 策略.

    参数:
        demo_path:   示范数据路径
        epochs:      训练轮数
        batch_size:  批大小
        lr:          学习率 (比 BC 小)
        chunk_size:  动作块大小 K
        seq_len:     输入序列长度 T
        output_path: 权重保存路径

    返回:
        ACTPolicy: 训练好的策略
    """
    torch, _ = _get_torch()

    # 加载数据
    obs, actions = load_demos(demo_path)
    obs_seqs, act_chunks = build_sequences(obs, actions, seq_len, chunk_size)
    if len(obs_seqs) == 0:
        raise ValueError(f"示范数据不足以构建训练序列: 需要至少 {seq_len + chunk_size} 步")

    obs_seqs_t = torch.as_tensor(obs_seqs, dtype=torch.float32,
                                  device="cuda" if torch.cuda.is_available() else "cpu")
    act_chunks_t = torch.as_tensor(act_chunks, dtype=torch.float32,
                                    device="cuda" if torch.cuda.is_available() else "cpu")
    logger.info("ACT 训练数据: %d 序列, seq_len=%d, chunk_size=%d",
                len(obs_seqs), seq_len, chunk_size)

    # 构建策略
    policy = ACTPolicy(obs_dim=obs.shape[1], act_dim=actions.shape[1],
                       chunk_size=chunk_size)
    params = (list(policy.obs_encoder.parameters()) +
              list(policy.transformer.parameters()) +
              list(policy.act_decoder.parameters()))
    optimizer = torch.optim.Adam(params, lr=lr)
    loss_fn = torch.nn.MSELoss()

    # 训练循环
    n_samples = len(obs_seqs)
    nan_count = 0

    for epoch in range(epochs):
        idx = torch.randperm(n_samples, device=policy.device)[:batch_size]
        batch_obs = obs_seqs_t[idx]
        batch_act = act_chunks_t[idx]

        pred = policy.forward(batch_obs)
        loss = loss_fn(pred, batch_act)

        if torch.isnan(loss) or torch.isinf(loss):
            nan_count += 1
            logger.warning("Epoch %d: NaN loss (count=%d)", epoch, nan_count)
            if nan_count >= 3:
                for pg in optimizer.param_groups:
                    pg["lr"] *= 0.1
                nan_count = 0
            continue

        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(params, max_norm=0.5)  # 梯度裁剪
        optimizer.step()

        if epoch % 20 == 0:
            logger.info("Epoch %d: loss=%.6f", epoch, loss.item())

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        policy.save(output_path)

    return policy


def main():
    """CLI 入口."""
    import argparse

    parser = argparse.ArgumentParser(description="训练 ACT 策略")
    parser.add_argument("--task", type=str, required=True)
    parser.add_argument("--demo", type=str, required=True)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch_size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--chunk_size", type=int, default=10)
    parser.add_argument("--seq_len", type=int, default=10)
    parser.add_argument("--output", type=str, default=None)
    args = parser.parse_args()

    output = args.output or f"checkpoints/act_{args.task}.pt"
    train_act(args.demo, args.epochs, args.batch_size, args.lr,
              args.chunk_size, args.seq_len, output)


if __name__ == "__main__":
    main()
