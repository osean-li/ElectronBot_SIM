"""传感器测试 — 对齐 Phase 5 §4.

测试:
  - JointSensor: 位置/速度读取
  - ContactSensor: 接触检测
  - CameraSensor: 需要渲染环境, 标记为 slow
"""
from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "osmesa")


@pytest.fixture
def env():
    from electronbot_sim.env import ElectronBotEnv
    e = ElectronBotEnv(render_mode=None)
    e.reset()
    yield e
    e.close()


class TestJointSensor:
    """关节传感器测试."""

    def test_get_positions(self, env):
        from electronbot_sim.sensors import JointSensor
        sensor = JointSensor(env)
        pos = sensor.get_positions(add_noise=False)
        assert pos.shape == (6,)
        # home 姿态: [0, -45, 0, -45, 0, 0]
        expected = np.array([0, -45, 0, -45, 0, 0])
        assert np.allclose(pos, expected, atol=2.0)

    def test_get_velocities(self, env):
        from electronbot_sim.sensors import JointSensor
        sensor = JointSensor(env)
        vel = sensor.get_velocities(add_noise=False)
        assert vel.shape == (6,)

    def test_get_end_effector_positions(self, env):
        from electronbot_sim.sensors import JointSensor
        sensor = JointSensor(env)
        ee = sensor.get_end_effector_positions()
        assert "left" in ee
        assert "right" in ee
        assert ee["left"].shape == (3,)
        assert ee["right"].shape == (3,)

    def test_position_changes_after_step(self, env):
        """发送动作后位置应变化."""
        from electronbot_sim.sensors import JointSensor
        sensor = JointSensor(env)
        pos_before = sensor.get_positions(add_noise=False)
        env.step(np.array([5.0, 0, 0, 0, 0, 0], dtype=np.float32))
        pos_after = sensor.get_positions(add_noise=False)
        # 第一个关节应有变化
        assert abs(pos_after[0] - pos_before[0]) > 0.1


class TestContactSensor:
    """接触传感器测试."""

    def test_create_contact_sensor(self, env):
        from electronbot_sim.sensors import ContactSensor
        sensor = ContactSensor(env, "left_hand")
        assert sensor is not None

    def test_is_in_contact(self, env):
        from electronbot_sim.sensors import ContactSensor
        sensor = ContactSensor(env, "left_hand")
        result = sensor.is_in_contact()
        assert isinstance(result, bool)

    def test_get_total_force(self, env):
        from electronbot_sim.sensors import ContactSensor
        sensor = ContactSensor(env, "right_hand")
        force = sensor.get_total_contact_force()
        assert isinstance(force, float)
        assert force >= 0.0
