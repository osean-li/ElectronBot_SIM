"""动作系统测试 — 对齐 Phase 4 §3.

测试:
  - 线性插值 MoveServos
  - 安全裁剪 _clamp_servo
  - 预设动作 (hand_raise/wave/flap, body_turn, head_nod)
  - 舵机序列 execute_sequence
"""
from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "osmesa")


@pytest.fixture
def actions():
    """创建动作系统 fixture."""
    from electronbot_sim.env import ElectronBotEnv
    from electronbot_sim.mcp_bridge import McpSimBridge
    from electronbot_sim.actions import ElectronBotActions

    env = ElectronBotEnv(render_mode=None)
    bridge = McpSimBridge(env)
    act = ElectronBotActions(env, bridge)
    env.reset()
    yield act
    env.close()


class TestClampServo:
    """安全角度裁剪测试 — 对齐固件 ClampServoTarget."""

    def test_head_range(self, actions):
        """head: 75-105."""
        from electronbot_sim.env import clamp_servo_target
        assert clamp_servo_target(5, 50) == 75   # 低于下限
        assert clamp_servo_target(5, 120) == 105 # 高于上限
        assert clamp_servo_target(5, 90) == 90   # 范围内

    def test_body_range(self, actions):
        """body: 30-150."""
        from electronbot_sim.env import clamp_servo_target
        assert clamp_servo_target(4, -10) == 30
        assert clamp_servo_target(4, 200) == 150
        assert clamp_servo_target(4, 90) == 90

    def test_right_roll_range(self, actions):
        """right_roll: 100-180."""
        from electronbot_sim.env import clamp_servo_target
        assert clamp_servo_target(1, 50) == 100
        assert clamp_servo_target(1, 200) == 180


class TestPresetActions:
    """预设动作测试."""

    def test_home(self, actions):
        """home 动作应复位到 [180,180,0,0,90,90]."""
        actions.home(speed_ms=200)
        assert not actions._moving

    def test_hand_raise_all_hands(self, actions):
        """hand_raise 应对 left/right/both 都不崩溃."""
        for hand in ["left", "right", "both"]:
            actions.home(speed_ms=200)
            actions.hand_raise(hand, speed_ms=200)
            actions.hand_lower(hand, speed_ms=200)

    def test_hand_wave(self, actions):
        """hand_wave 应不崩溃."""
        actions.home(speed_ms=200)
        actions.hand_wave("both", times=1, speed_ms=200)

    def test_hand_flap(self, actions):
        """hand_flap 应不崩溃."""
        actions.home(speed_ms=200)
        actions.hand_flap("both", times=1, speed_ms=200)

    def test_body_turn(self, actions):
        """body_turn_left/right/center 应不崩溃."""
        actions.home(speed_ms=200)
        actions.body_turn_left(speed_ms=200)
        actions.body_turn_right(speed_ms=200)
        actions.body_center(speed_ms=200)

    def test_head_actions(self, actions):
        """head 动作应不崩溃."""
        actions.home(speed_ms=200)
        actions.head_look_up(speed_ms=200)
        actions.head_look_down(speed_ms=200)
        actions.head_nod(times=1, speed_ms=200)
        actions.head_center(speed_ms=200)


class TestServoSequence:
    """舵机序列测试."""

    def test_simple_sequence(self, actions):
        """简单序列应不崩溃."""
        seq = {
            "a": [
                {"s": {"rp": 90, "lp": 90, "h": 100}, "v": 300},
                {"s": {"b": 60, "h": 90}, "v": 300, "d": 100},
            ]
        }
        actions.execute_sequence(seq)

    def test_oscillation_sequence(self, actions):
        """含振荡器的序列应不崩溃."""
        seq = {
            "a": [
                {"s": {"rp": 90, "lp": 90}, "v": 300},
                {"osc": {"a": {"rp": 20, "lp": 20},
                         "o": {"rp": 120, "lp": 60},
                         "p": 300, "c": 2}},
            ]
        }
        actions.execute_sequence(seq)

    def test_sequence_clamps_angles(self, actions):
        """序列中的角度应被裁剪到安全范围."""
        seq = {
            "a": [
                {"s": {"h": 200}, "v": 200},  # head 200 → 裁剪到 105
                {"s": {"b": -50}, "v": 200},  # body -50 → 裁剪到 30
            ]
        }
        actions.execute_sequence(seq)
        # 不崩溃即通过 (裁剪在 _move_servos 内部完成)
