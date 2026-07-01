"""
Domain Randomization 管理器

在训练时动态随机化物理参数，增强策略鲁棒性，为 Sim2Real 迁移铺路。

随机化参数:
  - 摩擦系数: ±30%
  - 关节阻尼: ±20%
  - 连杆质量: ±15%
  - 执行器增益 (Kp): ±25%
  - 观测噪声: 高斯 σ=0.01
"""

import numpy as np
from typing import Dict, Any, Optional


class DomainRandomizer:
    """Domain Randomization 管理器"""

    def __init__(
        self,
        friction_range: float = 0.3,      # ±30%
        damping_range: float = 0.2,       # ±20%
        mass_range: float = 0.15,         # ±15%
        kp_range: float = 0.25,           # ±25%
        obs_noise_std: float = 0.01,      # 观测噪声标准差
        seed: Optional[int] = None,
    ):
        self.friction_range = friction_range
        self.damping_range = damping_range
        self.mass_range = mass_range
        self.kp_range = kp_range
        self.obs_noise_std = obs_noise_std

        self.rng = np.random.RandomState(seed)
        self._current_params = {}

    def sample_params(self) -> Dict[str, Any]:
        """采样一组随机的物理参数"""
        params = {
            "friction_scale": 1.0 + self.rng.uniform(
                -self.friction_range, self.friction_range
            ),
            "damping_scale": 1.0 + self.rng.uniform(
                -self.damping_range, self.damping_range
            ),
            "mass_scale": 1.0 + self.rng.uniform(
                -self.mass_range, self.mass_range
            ),
            "kp_scale": 1.0 + self.rng.uniform(
                -self.kp_range, self.kp_range
            ),
        }
        self._current_params = params
        return params

    def apply_to_model(self, robot):
        """将随机化参数应用到 MuJoCo 模型"""
        p = self._current_params
        if not p:
            p = self.sample_params()

        model = robot.model

        # 修改关节阻尼
        for jid in robot._joint_ids:
            model.dof_damping[jid] *= p.get("damping_scale", 1.0)

        # 修改执行器增益
        for aid in robot._actuator_ids:
            model.actuator_gainprm[aid, 0] *= p.get("kp_scale", 1.0)

        # 修改几何体摩擦
        for i in range(model.ngeom):
            for j in range(3):
                model.geom_friction[i, j] *= p.get("friction_scale", 1.0)

    def add_observation_noise(self, obs: np.ndarray) -> np.ndarray:
        """向观测添加高斯噪声"""
        noise = self.rng.normal(0, self.obs_noise_std, size=obs.shape)
        return obs + noise

    def reset_env(self, robot):
        """在新 episode 开始时调用: 采样 + 应用"""
        self.sample_params()
        self.apply_to_model(robot)

    def get_current_params(self) -> Dict[str, Any]:
        """获取当前随机化参数 (用于日志)"""
        return self._current_params.copy()


# ---- 预设的 DR 配置 ----
DEFAULT_DR = {
    "friction_range": 0.3,
    "damping_range": 0.2,
    "mass_range": 0.15,
    "kp_range": 0.25,
    "obs_noise_std": 0.01,
}

LIGHT_DR = {
    "friction_range": 0.1,
    "damping_range": 0.1,
    "mass_range": 0.05,
    "kp_range": 0.1,
    "obs_noise_std": 0.005,
}

HEAVY_DR = {
    "friction_range": 0.5,
    "damping_range": 0.4,
    "mass_range": 0.3,
    "kp_range": 0.4,
    "obs_noise_std": 0.02,
}
