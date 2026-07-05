"""AI 训练任务测试 — 对齐 Phase 6.

测试:
  - 7 个标准任务创建与 reset
  - 任务 step 不崩溃
  - 奖励/成功判定逻辑
  - 任务工厂函数
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


class TestTaskRegistry:
    """任务注册表测试."""

    def test_list_tasks(self):
        from electronbot_ai.tasks import list_tasks
        tasks = list_tasks()
        assert "reach" in tasks
        assert "push" in tasks
        assert "voice" in tasks or "voice_cmd" in tasks

    def test_create_task_reach(self):
        from electronbot_ai.tasks import create_task, ReachTask
        task = create_task("reach")
        assert isinstance(task, ReachTask)
        assert task.name == "EB-Reach"

    def test_create_task_unknown(self):
        from electronbot_ai.tasks import create_task
        with pytest.raises(ValueError):
            create_task("unknown_task_name")

    def test_create_all_7_tasks(self):
        """7 个标准任务都应可创建."""
        from electronbot_ai.tasks import create_task
        for name in ["reach", "push", "pick_place", "stack", "follow", "gesture", "voice"]:
            task = create_task(name)
            assert task is not None, f"任务 {name} 创建失败"


class TestTaskReset:
    """任务 reset 测试."""

    def test_reach_reset(self, env):
        from electronbot_ai.tasks import create_task
        task = create_task("reach")
        obs = task.reset(env)
        assert "joint_pos" in obs
        assert "target_pos" in obs
        assert "dist_to_target" in obs

    def test_gesture_reset(self, env):
        from electronbot_ai.tasks import create_task
        task = create_task("gesture")
        obs = task.reset(env)
        assert "target_joint_pos" in obs

    def test_voice_cmd_reset(self, env):
        from electronbot_ai.tasks import create_task
        task = create_task("voice")
        obs = task.reset(env)
        assert "command" in obs
        assert obs["command"] is not None


class TestTaskStep:
    """任务 step 测试."""

    def test_reach_step(self, env):
        from electronbot_ai.tasks import create_task
        task = create_task("reach")
        task.reset(env)
        action = np.zeros(6, dtype=np.float32)
        obs, reward, terminated, truncated, info = task.step(action)
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
        assert "dist_to_target" in info

    def test_all_tasks_step(self, env):
        """所有任务都应能执行 step 不崩溃."""
        from electronbot_ai.tasks import create_task
        for name in ["reach", "push", "pick_place", "stack", "follow", "gesture"]:
            task = create_task(name)
            task.reset(env)
            action = np.zeros(6, dtype=np.float32)
            obs, reward, terminated, truncated, info = task.step(action)
            assert isinstance(reward, (int, float))


class TestTaskReward:
    """任务奖励测试."""

    def test_reach_reward_negative(self, env):
        """Reach 任务初始奖励应为负 (距离目标)."""
        from electronbot_ai.tasks import create_task
        task = create_task("reach")
        task.reset(env)
        reward = task.compute_reward()
        assert reward <= 0  # 距离负奖励

    def test_gesture_reward(self, env):
        """Gesture 任务奖励应有限."""
        from electronbot_ai.tasks import create_task
        task = create_task("gesture")
        task.reset(env)
        reward = task.compute_reward()
        assert np.isfinite(reward)


class TestTaskSuccess:
    """任务成功判定测试."""

    def test_reach_not_success_initially(self, env):
        """Reach 任务初始不应成功."""
        from electronbot_ai.tasks import create_task
        task = create_task("reach")
        task.reset(env)
        # 初始状态末端执行器不在目标点
        assert not task.is_success()

    def test_voice_cmd_success(self, env):
        """VoiceCmd 任务设置正确动作后应成功."""
        from electronbot_ai.tasks import create_task, VoiceCmdTask
        task = create_task("voice", command="举起右手")
        task.reset(env)
        # 设置正确的动作序列
        task.set_generated_actions([
            {"tool": "self.electron.hand_action", "args": {"action": 1, "hand": 2}}
        ])
        assert task.is_success()
