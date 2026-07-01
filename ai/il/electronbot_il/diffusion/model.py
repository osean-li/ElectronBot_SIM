#!/usr/bin/env python3
"""
Diffusion Policy: 1D UNet + DDPM

基于去噪扩散概率模型 (DDPM) 的动作生成
参考: Diffusion Policy: Visuomotor Policy Learning via Action Diffusion (Chi et al., 2023)
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
import math
from typing import Optional


class SinusoidalPositionEmbedding(nn.Module):
    """正弦位置编码"""
    def __init__(self, dim: int):
        super().__init__()
        self.dim = dim

    def forward(self, timesteps: torch.Tensor):
        half_dim = self.dim // 2
        embeddings = math.log(10000) / (half_dim - 1)
        embeddings = torch.exp(torch.arange(half_dim, device=timesteps.device) * -embeddings)
        embeddings = timesteps[:, None] * embeddings[None, :]
        embeddings = torch.cat([torch.sin(embeddings), torch.cos(embeddings)], dim=-1)
        return embeddings


class ResidualBlock(nn.Module):
    """1D 残差块"""
    def __init__(self, dim: int, dropout: float = 0.1):
        super().__init__()
        self.conv1 = nn.Conv1d(dim, dim, 3, padding=1)
        self.conv2 = nn.Conv1d(dim, dim, 3, padding=1)
        self.norm1 = nn.GroupNorm(8, dim)
        self.norm2 = nn.GroupNorm(8, dim)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x):
        residual = x
        x = self.norm1(x)
        x = F.silu(x)
        x = self.conv1(x)
        x = self.norm2(x)
        x = F.silu(x)
        x = self.dropout(x)
        x = self.conv2(x)
        return x + residual


class ActionUNet(nn.Module):
    """1D UNet 用于动作去噪"""

    def __init__(
        self,
        action_dim: int = 6,
        obs_dim: int = 18,
        hidden_dim: int = 256,
        num_res_blocks: int = 4,
        dropout: float = 0.1,
    ):
        super().__init__()

        self.action_dim = action_dim
        self.obs_dim = obs_dim

        # 时间编码
        self.time_embed = nn.Sequential(
            SinusoidalPositionEmbedding(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
        )

        # 观测编码
        self.obs_encoder = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )

        # 输入投影
        self.input_proj = nn.Linear(action_dim + hidden_dim + hidden_dim, hidden_dim)

        # 残差块
        self.res_blocks = nn.ModuleList([
            ResidualBlock(hidden_dim, dropout) for _ in range(num_res_blocks)
        ])

        # 输出投影
        self.output_proj = nn.Sequential(
            nn.GroupNorm(8, hidden_dim),
            nn.SiLU(),
            nn.Conv1d(hidden_dim, action_dim, 1),
        )

    def forward(
        self,
        x: torch.Tensor,
        timestep: torch.Tensor,
        obs_cond: Optional[torch.Tensor] = None,
    ):
        """
        参数:
          x: 噪声动作 (B, action_dim, T)
          timestep: 时间步 (B,)
          obs_cond: 观测条件 (B, obs_dim) 或 (B, obs_dim, T)
        """
        B, _, T = x.shape

        # 时间嵌入
        t_emb = self.time_embed(timestep)  # (B, hidden_dim)

        # 观测嵌入
        if obs_cond is not None:
            if obs_cond.dim() == 2:
                obs_cond = obs_cond.unsqueeze(-1).repeat(1, 1, T)
            o_emb = self.obs_encoder(obs_cond.permute(0, 2, 1))
            o_emb = o_emb.mean(dim=1)  # (B, hidden_dim)
        else:
            o_emb = torch.zeros(B, hidden_dim, device=x.device)

        # 拼接条件
        cond = torch.cat([t_emb, o_emb], dim=-1)
        cond = cond.unsqueeze(-1).repeat(1, 1, T)

        # 组合输入
        h = torch.cat([x, cond], dim=1)
        h = self.input_proj(h.permute(0, 2, 1)).permute(0, 2, 1)

        # 残差块
        for res_block in self.res_blocks:
            h = res_block(h)

        # 输出
        return self.output_proj(h)
