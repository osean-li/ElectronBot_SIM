"""环境单元测试 — 对齐 Phase 2 §4.1.

测试:
  - reset 返回正确观测
  - action bounds (超大动作不崩溃)
  - physics stable (1000 步无 NaN)
  - render rgb_array
"""
from __future__ import annotations

import os

import numpy as np
import pytest

# 跳过需要 MuJoCo 渲染的测试 (无头 CI 环境)
os.environ.setdefault("MUJOCO_GL", "osmesa")


def _make_env(render_mode=None, obs_mode="realistic"):
    """创建测试环境 (延迟导入, 避免 MuJoCo 未安装时 import 失败)."""
    from electronbot_sim.env import ElectronBotEnv, HOME_QPOS
    return ElectronBotEnv(render_mode=render_mode, obs_mode=obs_mode), HOME_QPOS


@pytest.fixture
def env():
    """测试用环境 fixture."""
    env, _ = _make_env(render_mode=None)
    yield env
    env.close()


class TestEnvReset:
    """环境 reset 测试."""

    def test_reset_returns_obs_info(self, env):
        """reset 返回 (obs, info) 二元组."""
        obs, info = env.reset()
        assert isinstance(obs, dict)
        assert isinstance(info, dict)

    def test_reset_home_pose(self, env):
        """reset 后关节角度应为 home 姿态 [0,-45,0,-45,0,0]."""
        obs, _ = env.reset()
        if "joint_pos" in obs:
            joint_pos = obs["joint_pos"]
            expected = np.array([0, -45, 0, -45, 0, 0], dtype=np.float32)
            assert np.allclose(joint_pos, expected, atol=2.0), \
                f"home 姿态不符: {joint_pos} vs {expected}"
        # realistic 模式检查 commanded_joint_pos
        if "commanded_joint_pos" in obs:
            commanded = obs["commanded_joint_pos"]
            expected = np.array([0, -45, 0, -45, 0, 0], dtype=np.float32)
            assert np.allclose(commanded, expected, atol=2.0)

    def test_reset_info_keys(self, env):
        """reset info 含必要字段."""
        obs, info = env.reset()
        assert "domain_randomization" in info
        assert "object_positions" in info
        assert "seed" in info


class TestEnvActionBounds:
    """动作边界测试."""

    def test_oversized_action_no_crash(self, env):
        """超大动作不应崩溃, 应裁剪到限位."""
        env.reset()
        action = np.array([999, 999, 999, 999, 999, 999], dtype=np.float32)
        obs, _, _, _, _ = env.step(action)
        # 不应崩溃
        assert obs is not None

    def test_zero_action_stays_home(self, env):
        """零动作应保持 home 姿态."""
        env.reset()
        for _ in range(10):
            obs, _, _, _, _ = env.step(np.zeros(6, dtype=np.float32))
        if "joint_pos" in obs:
            expected = np.array([0, -45, 0, -45, 0, 0], dtype=np.float32)
            assert np.allclose(obs["joint_pos"], expected, atol=5.0)


class TestEnvPhysicsStable:
    """物理稳定性测试."""

    def test_1000_steps_no_nan(self, env):
        """连续 1000 步随机动作, 不应出现 NaN."""
        env.reset()
        for _ in range(1000):
            action = env.action_space.sample()
            obs, _, _, _, _ = env.step(action)
        # 验证无 NaN
        for key, val in obs.items():
            if isinstance(val, np.ndarray):
                assert not np.any(np.isnan(val)), f"观测 {key} 含 NaN"

    def test_joint_within_limits(self, env):
        """关节角度应始终在限位范围内."""
        from electronbot_sim.env import JOINT_MIN, JOINT_MAX
        env.reset()
        for _ in range(500):
            action = env.action_space.sample()
            obs, _, _, _, _ = env.step(action)
            if "joint_pos" in obs:
                for i in range(6):
                    # 允许小的数值误差
                    assert JOINT_MIN[i] - 5 <= obs["joint_pos"][i] <= JOINT_MAX[i] + 5, \
                        f"关节 {i} 超限: {obs['joint_pos'][i]}"


class TestEnvObservationModes:
    """观测模式测试."""

    def test_full_mode_keys(self):
        """full 模式应包含 joint_vel/ee_pos 等仿真专属字段."""
        env, _ = _make_env(obs_mode="full")
        obs, _ = env.reset()
        assert "joint_pos" in obs
        assert "joint_vel" in obs
        assert "ee_left_pos" in obs
        assert "ee_right_pos" in obs
        env.close()

    def test_realistic_mode_keys(self):
        """realistic 模式应仅含真机可获取数据."""
        env, _ = _make_env(obs_mode="realistic")
        obs, _ = env.reset()
        assert "commanded_joint_pos" in obs
        assert "is_moving" in obs
        assert "battery_voltage" in obs
        # 不应含仿真专属字段
        assert "joint_vel" not in obs
        assert "ee_left_pos" not in obs
        env.close()


class TestEnvState:
    """状态保存/恢复测试."""

    def test_get_set_state(self, env):
        """get_state/set_state 应正确保存恢复."""
        env.reset()
        env.step(np.array([2.0, 0, 0, 0, 0, 0], dtype=np.float32))
        state = env.get_state()
        assert "qpos" in state
        assert "qvel" in state

        env.reset()
        env.set_state(state)
        # 验证状态恢复
        new_state = env.get_state()
        assert np.allclose(state["qpos"], new_state["qpos"], atol=1e-5)
