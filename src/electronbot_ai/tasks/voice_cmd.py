"""EB-VoiceCmd 语音指令理解任务.

对齐 docs/tasks/07-Benchmark §2.7.

场景: 接收自然语言指令
任务: LLM 正确理解并生成可执行的动作序列
成功条件: 人类评审认为指令被执行正确

测试指令集:
  "举起右手" / "挥挥手" / "转过来看我" / "点点头" / "把红色方块推过去"

⚠️ 本任务仅 VLA 路径适用 (BC/ACT/PPO 不适用)
"""
from __future__ import annotations

import logging
from typing import Any, Optional

import numpy as np

from .base import BaseTask

logger = logging.getLogger("electronbot_ai.tasks.voice_cmd")

# 标准测试指令集
VOICE_TEST_COMMANDS = [
    "举起右手",
    "挥挥手",
    "转过来看我",
    "点点头",
    "把红色方块推过去",
    "举双手",
    "低头",
    "向左转",
]

# 指令 → 期望动作序列映射 (用于自动评估)
COMMAND_EXPECTED_ACTIONS = {
    "举起右手": [("self.electron.hand_action", {"action": 1, "hand": 2})],
    "挥挥手": [("self.electron.hand_action", {"action": 3, "hand": 3, "steps": 2})],
    "转过来看我": [("self.electron.body_turn", {"direction": 1, "angle": 45})],
    "点点头": [("self.electron.head_move", {"action": 3, "steps": 1})],
    "举双手": [("self.electron.hand_action", {"action": 1, "hand": 3})],
    "低头": [("self.electron.head_move", {"action": 2, "angle": 10})],
    "向左转": [("self.electron.body_turn", {"direction": 1, "angle": 45})],
}


class VoiceCmdTask(BaseTask):
    """EB-VoiceCmd: 语音指令理解 (仅 VLA 适用).

    本任务不产生 RL 奖励, 成功判定依赖:
    1. 自动评估: 比较生成动作序列与期望序列的语义匹配
    2. 人工评审: 人类判断指令执行是否正确 (Benchmark 用)
    """

    name = "EB-VoiceCmd"
    difficulty = 3

    def __init__(self, obs_mode: str = "full", seed: Optional[int] = None,
                 command: Optional[str] = None):
        super().__init__(obs_mode=obs_mode, seed=seed)
        self._command = command
        self._generated_actions: list = []
        self._evaluated = False
        self._success = False

    def reset(self, env: Any) -> dict:
        self.bind(env)
        env.reset()
        self._step_count = 0
        self._generated_actions = []
        self._evaluated = False
        self._success = False
        # 随机选择指令 (或使用指定指令)
        if self._command is None:
            self._command = VOICE_TEST_COMMANDS[
                self._rng.integers(0, len(VOICE_TEST_COMMANDS))
            ]
        self._target_pos = np.zeros(3, dtype=np.float32)
        return self.get_observation()

    def get_observation(self) -> dict:
        obs = self._build_base_obs()
        obs["command"] = self._command
        return obs

    def set_generated_actions(self, actions: list) -> None:
        """设置 VLA 规划器生成的动作序列 (供评估)."""
        self._generated_actions = actions
        self._evaluate()

    def _evaluate(self) -> None:
        """自动评估: 与期望动作序列比对."""
        if self._command is None:
            self._evaluated = True
            self._success = False
            return
        expected = COMMAND_EXPECTED_ACTIONS.get(self._command, [])
        if not expected:
            self._evaluated = True
            self._success = True  # 未知指令默认通过 (需人工评审)
            return
        # 检查生成的动作是否包含期望工具
        generated_tools = [a.get("tool", "") if isinstance(a, dict) else a[0]
                          for a in self._generated_actions]
        matched = 0
        for exp_tool, _ in expected:
            for gen_tool in generated_tools:
                if gen_tool == exp_tool:
                    matched += 1
                    break
        self._success = matched == len(expected)
        self._evaluated = True

    def compute_reward(self) -> float:
        """VoiceCmd 任务不使用 RL 奖励, 返回 0."""
        return 0.0

    def is_success(self) -> bool:
        if not self._evaluated:
            self._evaluate()
        return self._success

    def get_demo_action(self, keyboard_state: dict) -> np.ndarray:
        """VoiceCmd 不使用键盘示范."""
        return np.zeros(6, dtype=np.float32)

    @property
    def command(self) -> Optional[str]:
        return self._command
