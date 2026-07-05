"""域随机化 Wrapper — 每次 reset 随机化物理参数.

对齐 docs/tasks/06-AI-Training §4.3 + Phase 2 §3 Step 4.

随机化维度:
  - 关节阻尼 ±30% (dof_damping)
  - 执行器增益 ±15% (actuator_gainprm)
  - 物体质量 ±20% (body_mass)
  - 舵机死区 2-5° (servo_deadband)
  - 电池电压 3.5-4.2V (battery_voltage → 增益缩放)
  - 摄像头延迟 0-2 帧 (camera_delay_steps)
  - 观测噪声 0-0.5° (joint_pos_noise_std)
"""
from __future__ import annotations

import logging
from typing import Any, Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.rl.domain_randomization")


class DomainRandomizationWrapper:
    """域随机化包装器.

    包装 TaskWrapper, 在每次 reset 时随机化物理参数.
    目标: 提高策略的 Sim2Real 泛化能力.

    参数:
        wrapped_env: 被包装的环境 (TaskWrapper 或 ElectronBotEnv)
        joint_damping_range: 关节阻尼缩放范围, 默认 (0.7, 1.3)
        actuator_gain_range: 执行器增益缩放范围, 默认 (0.85, 1.15)
        body_mass_range:     物体质量缩放范围, 默认 (0.8, 1.2)
        camera_delay_steps:  摄像头延迟帧数范围, 默认 (0, 3)
        joint_pos_noise_std: 关节位置噪声标准差范围, 默认 (0, 0.5)
        servo_deadband_range: 舵机死区范围 (度), 默认 (2.0, 5.0)
        battery_voltage_range: 电池电压范围 (V), 默认 (3.5, 4.2)
    """

    def __init__(self, wrapped_env: Any,
                 joint_damping_range: tuple = (0.7, 1.3),
                 actuator_gain_range: tuple = (0.85, 1.15),
                 body_mass_range: tuple = (0.8, 1.2),
                 camera_delay_steps: tuple = (0, 3),
                 joint_pos_noise_std: tuple = (0.0, 0.5),
                 servo_deadband_range: tuple = (2.0, 5.0),
                 battery_voltage_range: tuple = (3.5, 4.2)):
        self.wrapped = wrapped_env
        self.env = wrapped_env.env if hasattr(wrapped_env, "env") else wrapped_env
        self.model = self.env.model

        self.joint_damping_range = joint_damping_range
        self.actuator_gain_range = actuator_gain_range
        self.body_mass_range = body_mass_range
        self.camera_delay_range = camera_delay_steps
        self.joint_pos_noise_range = joint_pos_noise_std
        self.servo_deadband_range = servo_deadband_range
        self.battery_voltage_range = battery_voltage_range

        # 保存原始参数 (用于 reset 恢复)
        self._original_damping = self.model.dof_damping.copy()
        self._original_gainprm = self.model.actuator_gainprm.copy()
        self._original_mass = self.model.body_mass.copy()

        self._current_delay_steps = 0
        self._obs_buffer: list = []
        self._joint_pos_noise_std = 0.0
        self._servo_deadband_deg = 0.0

    def _randomize_domain(self):
        """执行域随机化 (每次 reset 调用)."""
        rng = np.random.default_rng()

        # 1. 关节阻尼 ±30%
        damping_scale = rng.uniform(*self.joint_damping_range)
        self.model.dof_damping[:] = self._original_damping * damping_scale

        # 2. 执行器增益 ±15%
        gain_scale = rng.uniform(*self.actuator_gain_range)
        self.model.actuator_gainprm[:, 0] = self._original_gainprm[:, 0] * gain_scale

        # 3. 物体质量 ±20%
        for i in range(self.model.nbody):
            mass_scale = rng.uniform(*self.body_mass_range)
            self.model.body_mass[i] = self._original_mass[i] * mass_scale

        # 4. 摄像头延迟 0-2 帧
        self._current_delay_steps = int(rng.integers(*self.camera_delay_range))

        # 5. 观测噪声
        self._joint_pos_noise_std = rng.uniform(*self.joint_pos_noise_range)

        # 6. 舵机死区 2-5°
        self._servo_deadband_deg = rng.uniform(*self.servo_deadband_range)

        # 7. 电池电压 3.5-4.2V → 增益缩放
        voltage = rng.uniform(*self.battery_voltage_range)
        voltage_scale = max(0.5, min(1.0, (voltage - 3.3) / 0.9))
        self.model.actuator_gainprm[:, 0] *= voltage_scale

        logger.debug(
            "域随机化: damping=%.2f, gain=%.2f, voltage=%.2fV, deadband=%.1f°, delay=%d, noise=%.2f",
            damping_scale, gain_scale * voltage_scale, voltage,
            self._servo_deadband_deg, self._current_delay_steps,
            self._joint_pos_noise_std,
        )

    def _apply_obs_noise(self, obs: np.ndarray) -> np.ndarray:
        """给观测添加噪声."""
        if self._joint_pos_noise_std > 0 and len(obs) >= 6:
            obs = obs.copy()
            obs[:6] += np.random.randn(6) * self._joint_pos_noise_std
        return obs

    def _apply_camera_delay(self, obs: np.ndarray) -> np.ndarray:
        """模拟摄像头延迟."""
        if self._current_delay_steps == 0:
            return obs
        self._obs_buffer.append(obs)
        if len(self._obs_buffer) > self._current_delay_steps:
            return self._obs_buffer.pop(0)
        return obs  # 缓冲区未满时返回当前 obs

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        """重置 + 域随机化."""
        self._randomize_domain()
        self._obs_buffer = []
        obs, info = self.wrapped.reset(seed=seed, options=options)
        obs = self._apply_obs_noise(obs)
        obs = self._apply_camera_delay(obs)
        return obs, info

    def step(self, action: np.ndarray):
        """步进 + 观测噪声 + 延迟."""
        # 应用舵机死区
        if self._servo_deadband_deg > 0:
            action = action.copy()
            small = np.abs(action) < self._servo_deadband_deg
            action = np.where(small, 0, action)

        obs, reward, terminated, truncated, info = self.wrapped.step(action)
        obs = self._apply_obs_noise(obs)
        obs = self._apply_camera_delay(obs)
        return obs, reward, terminated, truncated, info

    def render(self):
        return self.wrapped.render()

    def close(self):
        self.wrapped.close()

    # 透传属性
    @property
    def action_space(self):
        return self.wrapped.action_space

    @property
    def observation_space(self):
        return self.wrapped.observation_space

    @property
    def metadata(self):
        return self.wrapped.metadata

    @property
    def spec(self):
        return getattr(self.wrapped, "spec", None)


def wrap_with_dr(env: Any, **kwargs) -> DomainRandomizationWrapper:
    """便捷函数: 用域随机化包装器包装环境.

    参数:
        env:    被包装的环境
        **kwargs: 透传给 DomainRandomizationWrapper

    返回:
        DomainRandomizationWrapper 实例
    """
    return DomainRandomizationWrapper(env, **kwargs)
