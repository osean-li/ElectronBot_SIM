#!/usr/bin/env python3
"""
PPO 情绪策略: 通过不同 reward shaping 训练情绪化动作风格

三种情绪:
1. 开心 (happy):   高频小幅度动作，高 reward → 活泼、兴奋
2. 好奇 (curious): 探索性动作，信息增益 reward → 四处张望、试探
3. 疲倦 (tired):   低能耗，缓慢动作 → 慵懒、节能

使用方式:
  from emotional_reward import EmotionalRewardShaper
  shaper = EmotionalRewardShaper(mode="happy")
  reward = shaper.shape(q, qd, base_reward)
"""

import numpy as np
from typing import Dict, Any


class EmotionalRewardShaper:
    """
    情绪奖励塑型器

    设计原理:
    - 开心: 鼓励快速动作 + 积极反馈
    - 好奇: 鼓励探索新状态 + 减少重复
    - 疲倦: 惩罚能量消耗 + 平滑动作
    """

    MODES = ["happy", "curious", "tired", "disdain", "angry", "fear", "sad", "excited"]

    def __init__(
        self,
        mode: str = "happy",
        movement_weight: float = 1.0,
        entropy_weight: float = 0.1,
        energy_weight: float = 0.01,
    ):
        self.mode = mode
        self.movement_weight = movement_weight
        self.entropy_weight = entropy_weight
        self.energy_weight = energy_weight

        # 历史状态 (用于好奇模式)
        self._state_buffer = []
        self._buffer_size = 100

    def _novelty_bonus(self, q: np.ndarray) -> float:
        """计算状态新颖性 (好奇模式)"""
        if not self._state_buffer:
            self._state_buffer.append(q.copy())
            return 1.0

        # 计算与最近状态的距离
        distances = [np.linalg.norm(q - s) for s in self._state_buffer[-10:]]
        novelty = np.mean(distances) if distances else 0.0

        self._state_buffer.append(q.copy())
        if len(self._state_buffer) > self._buffer_size:
            self._state_buffer.pop(0)

        return float(novelty)

    def _movement_reward(self, qd: np.ndarray) -> float:
        """动作活跃度奖励"""
        return float(np.sum(np.abs(qd)))

    def _energy_cost(self, qd: np.ndarray) -> float:
        """能量消耗 = 速度的平方和"""
        return float(np.sum(qd ** 2))

    def _smoothness_reward(self, qd: np.ndarray) -> float:
        """动作平滑度奖励 (速度变化量)"""
        pass  # 需要历史速度

    def _load_emoji_target(self, emotion: str) -> np.ndarray:
        """加载 Emoji 视频提取的目标姿态 (°, 6维)"""
        try:
            from .emoji_pose_extractor import get_target_pose
        except ImportError:
            from emoji_pose_extractor import get_target_pose
        return get_target_pose(emotion, "loop")

    def shape(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        base_reward: float,
    ) -> float:
        """
        根据情绪模式塑型奖励

        参数:
          q: 当前关节位置 (rad, 6维)
          qd: 当前关节速度 (rad/s, 6维)
          base_reward: 基础任务奖励

        返回:
          塑型后的总奖励
        """
        movement = self._movement_reward(qd)
        energy = self._energy_cost(qd)

        if self.mode == "happy":
            # 开心: 鼓励活跃动作 + 高奖励
            bonus = movement * self.movement_weight
            reward = base_reward + bonus

        elif self.mode == "curious":
            # 好奇: 探索奖励 + 新颖性
            novelty = self._novelty_bonus(q)
            bonus = novelty * self.entropy_weight * 10.0
            reward = base_reward + bonus

        elif self.mode == "tired":
            # 疲倦: 惩罚能耗 + 缓慢动作
            penalty = energy * self.energy_weight
            reward = base_reward - penalty

        elif self.mode in ("disdain", "angry", "fear", "sad", "excited"):
            # 模仿 Emoji 视频中的真实情绪姿势
            # 加载目标姿态 (首次调用时)
            if not hasattr(self, "_target_pose"):
                self._target_pose = self._load_emoji_target(self.mode)
            target_deg = self._target_pose
            current_deg = np.degrees(q)

            # 角度误差越小, 奖励越高
            angle_error = np.linalg.norm(current_deg - target_deg)
            imitation_reward = np.exp(-angle_error / 30.0) * 5.0  # σ=30° 高斯

            # 同时鼓励到达目标后的微调动作 (情绪表达不是完全静止)
            micro_movement = movement * 0.01
            reward = base_reward + imitation_reward + micro_movement

        else:
            reward = base_reward

        return float(reward)

    def get_style_parameters(self) -> Dict[str, Any]:
        """获取当前风格参数"""
        return {
            "mode": self.mode,
            "movement_weight": self.movement_weight,
            "entropy_weight": self.entropy_weight,
            "energy_weight": self.energy_weight,
        }


# ---- 情绪风格可视化 ----
def visualize_emotional_styles():
    """可视化三种情绪风格的动作轨迹差异"""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    t = np.linspace(0, 2, 100)

    for ax, (mode, color, label) in zip(
        axes,
        [
            ("happy", "orange", "开心: 高频小幅度"),
            ("curious", "green", "好奇: 探索随机"),
            ("tired", "blue", "疲倦: 缓慢渐变"),
        ]
    ):
        np.random.seed(hash(mode) % 10000)

        if mode == "happy":
            # 高频正弦波
            traj = 0.3 * np.sin(2 * np.pi * 3 * t) + 0.05 * np.random.randn(len(t))
        elif mode == "curious":
            # 随机游走
            traj = np.cumsum(0.1 * np.random.randn(len(t)))
            traj = np.clip(traj, -0.5, 0.5)
        else:
            # 缓慢斜坡
            traj = 0.2 * (t / t.max()) + 0.02 * np.random.randn(len(t))

        ax.plot(t, traj, color=color, label=label)
        ax.set_ylabel("关节角度 (rad)")
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("时间 (s)")
    plt.tight_layout()
    plt.savefig("emotional_styles.png", dpi=100)
    print("[INFO] 情绪风格对比图已保存: emotional_styles.png")


if __name__ == "__main__":
    # 演示三种风格
    for mode in EmotionalRewardShaper.MODES:
        shaper = EmotionalRewardShaper(mode=mode)
        q = np.zeros(6)
        qd = np.ones(6) * 0.1
        reward = shaper.shape(q, qd, 1.0)
        print(f"  模式={mode:<10} 奖励={reward:+.3f}")

    visualize_emotional_styles()
