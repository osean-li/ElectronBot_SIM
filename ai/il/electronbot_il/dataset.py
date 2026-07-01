#!/usr/bin/env python3
"""
HDF5 示范数据集加载与预处理
"""
import h5py
import numpy as np
import torch
from torch.utils.data import Dataset
from typing import Tuple, List, Optional


class DemoDataset(Dataset):
    """演示数据集 (PyTorch Dataset)"""

    def __init__(
        self,
        hdf5_path: str,
        obs_dim: int = 18,
        action_dim: int = 6,
        chunk_size: int = 100,
        normalize: bool = True,
    ):
        self.chunk_size = chunk_size
        self.obs_dim = obs_dim
        self.action_dim = action_dim

        with h5py.File(hdf5_path, "r") as f:
            episodes = list(f.keys())
            self.episodes = []
            for ep in episodes:
                obs = np.array(f[f"{ep}/observations"])
                act = np.array(f[f"{ep}/actions"])
                if len(obs) >= chunk_size:
                    self.episodes.append((obs, act))

        if normalize:
            self._compute_stats()

        self._build_chunks()

    def _compute_stats(self):
        """计算归一化统计量"""
        all_obs = np.concatenate([e[0] for e in self.episodes], axis=0)
        all_act = np.concatenate([e[1] for e in self.episodes], axis=0)

        self.obs_mean = all_obs.mean(axis=0)
        self.obs_std = all_obs.std(axis=0) + 1e-6
        self.act_mean = all_act.mean(axis=0)
        self.act_std = all_act.std(axis=0) + 1e-6

    def _build_chunks(self):
        """构建 chunked 样本"""
        self.samples = []
        for obs, act in self.episodes:
            n = len(obs) - self.chunk_size + 1
            for i in range(0, n, max(1, self.chunk_size // 2)):
                obs_chunk = obs[i:i + self.chunk_size]
                act_chunk = act[i:i + self.chunk_size]
                self.samples.append((obs_chunk, act_chunk))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        obs, act = self.samples[idx]
        if hasattr(self, 'obs_mean'):
            obs = (obs - self.obs_mean) / self.obs_std
            act = (act - self.act_mean) / self.act_std
        return (
            torch.FloatTensor(obs),
            torch.FloatTensor(act),
        )
