#!/usr/bin/env python3
"""
ACT (Action Chunking Transformer)

Conditional VAE: Encoder-Decoder 架构
  - Encoder: 将 (示范obs, action chunk) 编码为 latent z
  - Decoder: 从 (当前obs, z) 解码出 action chunk

参考: Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (Zhao et al., 2023)
"""
import torch
import torch.nn as nn
import torch.nn.functional as F


class CVAEEncoder(nn.Module):
    """CVAE Encoder: (obs_chunk, action_chunk) → latent z"""

    def __init__(
        self,
        obs_dim: int = 18,
        action_dim: int = 6,
        chunk_size: int = 100,
        latent_dim: int = 32,
        hidden_dim: int = 512,
    ):
        super().__init__()
        self.chunk_size = chunk_size
        input_dim = (obs_dim + action_dim) * chunk_size

        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.mu = nn.Linear(hidden_dim, latent_dim)
        self.logvar = nn.Linear(hidden_dim, latent_dim)

    def forward(self, obs: torch.Tensor, action: torch.Tensor):
        # obs: (B, chunk, obs_dim), action: (B, chunk, action_dim)
        B = obs.shape[0]
        x = torch.cat([
            obs.reshape(B, -1),
            action.reshape(B, -1),
        ], dim=-1)
        h = self.net(x)
        mu = self.mu(h)
        logvar = self.logvar(h)
        return mu, logvar


class CVAEDecoder(nn.Module):
    """CVAE Decoder: (obs_chunk, z) → action_chunk"""

    def __init__(
        self,
        obs_dim: int = 18,
        action_dim: int = 6,
        chunk_size: int = 100,
        latent_dim: int = 32,
        hidden_dim: int = 512,
    ):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        input_dim = obs_dim * chunk_size + latent_dim

        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.out = nn.Linear(hidden_dim, action_dim * chunk_size)

    def forward(self, obs: torch.Tensor, z: torch.Tensor):
        B = obs.shape[0]
        x = torch.cat([obs.reshape(B, -1), z], dim=-1)
        h = self.net(x)
        action = self.out(h)
        return action.reshape(B, self.chunk_size, self.action_dim)


class ACTModel(nn.Module):
    """ACT: Action Chunking Transformer with CVAE"""

    def __init__(
        self,
        obs_dim: int = 18,
        action_dim: int = 6,
        chunk_size: int = 100,
        latent_dim: int = 32,
        hidden_dim: int = 512,
    ):
        super().__init__()
        self.latent_dim = latent_dim
        self.encoder = CVAEEncoder(obs_dim, action_dim, chunk_size, latent_dim, hidden_dim)
        self.decoder = CVAEDecoder(obs_dim, action_dim, chunk_size, latent_dim, hidden_dim)

    def encode(self, obs, action):
        return self.encoder(obs, action)

    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def decode(self, obs, z):
        return self.decoder(obs, z)

    def forward(self, obs, action):
        mu, logvar = self.encode(obs, action)
        z = self.reparameterize(mu, logvar)
        action_pred = self.decode(obs, z)
        return action_pred, mu, logvar

    def predict(self, obs):
        """推理: 给定当前观测，预测 action chunk"""
        z = torch.randn(obs.shape[0], self.latent_dim, device=obs.device)
        return self.decode(obs, z)

    def loss_fn(self, action_pred, action_gt, mu, logvar, beta: float = 1.0):
        """VAE 损失: 重构误差 + KL 散度"""
        recon_loss = F.mse_loss(action_pred, action_gt)
        kl_loss = -0.5 * torch.mean(1 + logvar - mu.pow(2) - logvar.exp())
        return recon_loss + beta * kl_loss
