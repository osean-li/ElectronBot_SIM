#!/usr/bin/env python3
"""
DDPM 噪声调度器

前向过程: x_t = sqrt(alpha_bar_t) * x_0 + sqrt(1 - alpha_bar_t) * epsilon
反向过程: 模型预测噪声 epsilon_theta(x_t, t)，然后去噪

参考: Denoising Diffusion Probabilistic Models (Ho et al., 2020)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Tuple


class DDPMScheduler:
    """DDPM 噪声调度器 (linear schedule)"""

    def __init__(
        self,
        num_train_timesteps: int = 100,
        beta_start: float = 1e-4,
        beta_end: float = 0.02,
    ):
        self.num_train_timesteps = num_train_timesteps

        # Linear schedule
        self.betas = torch.linspace(beta_start, beta_end, num_train_timesteps)
        self.alphas = 1.0 - self.betas
        self.alpha_bars = torch.cumprod(self.alphas, dim=0)

        # 预计算 (用于快速采样)
        self.sqrt_alpha_bars = torch.sqrt(self.alpha_bars)
        self.sqrt_one_minus_alpha_bars = torch.sqrt(1.0 - self.alpha_bars)

    def add_noise(
        self,
        x_0: torch.Tensor,
        timesteps: torch.Tensor,
        noise: torch.Tensor = None,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        前向加噪

        参数:
          x_0: 原始动作 (B, act_dim, T)
          timesteps: 时间步 (B,)
          noise: 预生成的噪声 (可选)

        返回:
          x_t: 加噪后的动作
          noise: 添加的噪声
        """
        if noise is None:
            noise = torch.randn_like(x_0)

        # 提取对应时间步的系数并扩展维度
        sqrt_alpha_bar = self.sqrt_alpha_bars.to(x_0.device)[timesteps]
        sqrt_one_minus = self.sqrt_one_minus_alpha_bars.to(x_0.device)[timesteps]

        # 扩展为 (B, 1, 1) 以广播
        sqrt_alpha_bar = sqrt_alpha_bar[:, None, None]
        sqrt_one_minus = sqrt_one_minus[:, None, None]

        x_t = sqrt_alpha_bar * x_0 + sqrt_one_minus * noise
        return x_t, noise

    def step(
        self,
        model_output: torch.Tensor,
        timestep: int,
        x_t: torch.Tensor,
    ) -> torch.Tensor:
        """
        单步去噪 (DDPM sampling)

        参数:
          model_output: 模型预测的噪声
          timestep: 当前时间步
          x_t: 当前带噪动作

        返回:
          x_{t-1}: 去噪一步后的动作
        """
        beta = self.betas.to(x_t.device)[timestep]
        alpha = self.alphas.to(x_t.device)[timestep]
        alpha_bar = self.alpha_bars.to(x_t.device)[timestep]

        # 均值
        coeff = beta / torch.sqrt(1.0 - alpha_bar)
        mean = (1.0 / torch.sqrt(alpha)) * (x_t - coeff * model_output)

        # 方差
        if timestep > 0:
            variance = beta
            noise = torch.randn_like(x_t)
        else:
            variance = 0.0
            noise = 0.0

        return mean + torch.sqrt(variance) * noise

    def sample(
        self,
        model: nn.Module,
        shape: Tuple[int, ...],
        obs_cond: torch.Tensor = None,
        device: str = "cpu",
    ) -> torch.Tensor:
        """
        完整去噪采样 (DDPM reverse process)

        参数:
          model: 去噪模型 (ActionUNet)
          shape: 输出形状 (B, T, act_dim)
          obs_cond: 观测条件 (可选)
          device: 设备

        返回:
          去噪后的动作 (B, T, act_dim)
        """
        model.eval()
        B, T, act_dim = shape

        # 从纯噪声开始 (UNet 格式: B, C, T)
        x = torch.randn(B, act_dim, T, device=device)

        with torch.no_grad():
            for t in reversed(range(self.num_train_timesteps)):
                timesteps = torch.full((B,), t, device=device, dtype=torch.long)
                noise_pred = model(x, timesteps, obs_cond)
                x = self.step(noise_pred, t, x)

        return x.permute(0, 2, 1)  # → (B, T, act_dim)
