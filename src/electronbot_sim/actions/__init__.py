"""动作系统 — 语义化高层动作接口.

对齐 docs/tasks/04-Action-System 详细设计说明书.
对齐真机固件: xiaozhi-esp32/main/boards/electron-bot/movements.cc

═══════════════════════════════════════════════════════════════════
  设计原则 (嵌入式固件工程师强制规范)
═══════════════════════════════════════════════════════════════════
  1. 线性插值: 对齐 movements.cc:87 MoveServos(), 禁用 EaseOutCubic
     increment_[i] = (target[i] - pos[i]) / (time / 10.0)
     每 10ms 步进等量增加 → 纯线性等分
  2. 安全裁剪: 对齐 ClampServoTarget(), 6 组硬限位
  3. 振荡器: 对齐 OscillateServos(), 50ms 采样 (vTaskDelay(5) @ 100Hz)
  4. 舵机→关节转换: 委托给 bridge (单一数据源, 不在本模块重复定义)
  5. hand_action times 限制: 2 * max(3, min(100, steps)) (movements.cc:225)

  12 个预设动作 (对齐固件 HandAction/BodyAction/HeadAction):
  ── 手部 (4 种 × 3 手 = 12 子动作) ──
    hand_raise / hand_lower / hand_wave / hand_flap
  ── 身体 (3 种) ──
    body_turn_left / body_turn_right / body_center
  ── 头部 (5 种) ──
    head_look_up / head_look_down / head_nod / head_center / head_continuous_nod
"""
from __future__ import annotations

import json
import logging
import time
from typing import Any, Dict, Optional

import numpy as np

from ..env import (
    SERVO_HOME,
    SERVO_LIMITS,
    SERVO_NAME_TO_INDEX,
    clamp_servo_target,
    joint_array_to_servo_array,
    servo_array_to_joint_array,
)

logger = logging.getLogger("electronbot_sim.actions")


class ElectronBotActions:
    """动作系统 — 1:1 对齐真机固件 movements.cc.

    提供两层接口:
    1. 语义化高层方法 (hand_raise / body_turn_left / head_nod 等)
       供 AI 训练 / 行为树 / VLA 规划器使用
    2. 底层固件 API (_move_servos / _oscillate / _clamp_servo)
       对齐固件函数名, 供精确控制与调试

    所有舵机↔关节转换委托给 bridge, 确保单一数据源.

    参数:
        env:    ElectronBotEnv 实例
        bridge: McpSimBridge 实例 (提供底层运动控制)
    """

    def __init__(self, env, bridge):
        self.env = env
        self.bridge = bridge

    # ================================================================
    #  底层固件 API (对齐 movements.cc 函数名)
    # ================================================================
    def _move_servos(self, targets: np.ndarray, time_ms: int) -> None:
        """6 舵机联动缓动 (线性插值, 对齐 movements.cc:87 MoveServos).

        固件实现:
            increment_[i] = (target[i] - pos[i]) / (time / 10.0)
            每 10ms 步进等量增加 → 纯线性插值

        参数:
            targets:  (6,) 舵机目标角度 (度)
            time_ms:  总时长 (毫秒), 10ms 步进
        """
        # 委托给 bridge 的 _move_all_servos (复用同一份插值逻辑)
        self.bridge._move_all_servos(targets, time_ms)

    def _oscillate(self, amplitudes: np.ndarray, centers: np.ndarray,
                   period: int, cycles: int) -> None:
        """正弦振荡器 (对齐 movements.cc:167 OscillateServos).

        固件实现:
            vTaskDelay(5)  // 5 ticks @ 100Hz = 50ms 采样
            targets[i] = centers[i] + amplitudes[i] * sin(phase)

        参数:
            amplitudes: (6,) 振幅 (度)
            centers:    (6,) 中心角度 (度)
            period:     周期 (毫秒)
            cycles:     周期数
        """
        # 构造 bridge._oscillate 需要的字典格式
        amp_dict = {}
        cen_dict = {}
        names = ["rp", "rr", "lp", "lr", "body", "head"]
        for i, name in enumerate(names):
            if amplitudes[i] != 0:
                amp_dict[name] = float(amplitudes[i])
            cen_dict[name] = float(centers[i])
        self.bridge._oscillate(amp_dict, cen_dict, period, cycles)

    def _clamp_servo(self, idx: int, angle: int) -> int:
        """安全角度裁剪 (对齐 ClampServoTarget).

        参数:
            idx:   舵机索引 0-5
            angle: 待裁剪角度 (度)

        返回: 裁剪后的整数角度
        """
        return clamp_servo_target(idx, angle)

    def _apply_ctrl(self, joint_angles_deg: np.ndarray) -> None:
        """【统一角度单位入口】设置机械关节目标 (度) 并推进仿真.

        严禁直接写度数到 data.ctrl (会因未转弧度导致 57.3 倍偏差).
        本方法通过 env.apply_joint_targets_deg() 统一转换.

        参数:
            joint_angles_deg: (6,) 机械关节角度 (度)
        """
        self.env.apply_joint_targets_deg(joint_angles_deg)
        self.env.step_simulation()

    # ================================================================
    #  12 个预设动作 (语义化高层接口)
    # ================================================================

    # ── 手部动作 (4 种, 对齐 HandAction case 1-4) ──

    def hand_raise(self, hand: str = "both", speed_ms: int = 1000) -> Dict:
        """举手 (对齐 HandAction action=1).

        参数:
            hand:     "left" / "right" / "both"
            speed_ms: 动作时长 (毫秒), 越小越快
        """
        hand_id = self._parse_hand(hand)
        return self.bridge._hand_action(
            action=1, hand=hand_id, steps=1, speed=speed_ms
        )

    def hand_lower(self, hand: str = "both", speed_ms: int = 1000) -> Dict:
        """放手 (对齐 HandAction action=2).

        参数:
            hand:     "left" / "right" / "both"
            speed_ms: 动作时长 (毫秒)
        """
        hand_id = self._parse_hand(hand)
        return self.bridge._hand_action(
            action=2, hand=hand_id, steps=1, speed=speed_ms
        )

    def hand_wave(self, hand: str = "both", times: int = 3,
                  speed_ms: int = 600) -> Dict:
        """挥手 (对齐 HandAction action=3).

        参数:
            hand:     "left" / "right" / "both"
            times:    挥手次数 (固件: 实际执行 2*max(3,min(100,times)) 次)
            speed_ms: 单次动作时长 (毫秒)
        """
        hand_id = self._parse_hand(hand)
        return self.bridge._hand_action(
            action=3, hand=hand_id, steps=times, speed=speed_ms
        )

    def hand_flap(self, hand: str = "both", times: int = 2,
                  amount: int = 30, speed_ms: int = 500) -> Dict:
        """拍打 (对齐 HandAction action=4).

        参数:
            hand:     "left" / "right" / "both"
            times:    拍打次数
            amount:   幅度 (10-50, 固件上限 40)
            speed_ms: 单次动作时长 (毫秒)
        """
        hand_id = self._parse_hand(hand)
        return self.bridge._hand_action(
            action=4, hand=hand_id, steps=times,
            speed=speed_ms, amount=amount
        )

    # ── 身体动作 (3 种, 对齐 BodyAction 1-3) ──

    def body_turn_left(self, angle: int = 45, speed_ms: int = 1000) -> Dict:
        """身体左转 (对齐 BodyAction direction=1).

        参数:
            angle:    转动角度 (0-90°)
            speed_ms: 动作时长 (毫秒)
        """
        return self.bridge._body_turn(
            direction=1, angle=angle, speed=speed_ms
        )

    def body_turn_right(self, angle: int = 45, speed_ms: int = 1000) -> Dict:
        """身体右转 (对齐 BodyAction direction=2).

        参数:
            angle:    转动角度 (0-90°)
            speed_ms: 动作时长 (毫秒)
        """
        return self.bridge._body_turn(
            direction=2, angle=angle, speed=speed_ms
        )

    def body_center(self, speed_ms: int = 1000) -> Dict:
        """身体回中心 (对齐 BodyAction direction=3).

        参数:
            speed_ms: 动作时长 (毫秒)
        """
        return self.bridge._body_turn(
            direction=3, angle=0, speed=speed_ms
        )

    # ── 头部动作 (5 种, 对齐 HeadAction 1-5) ──

    def head_look_up(self, angle: int = 10, speed_ms: int = 500) -> Dict:
        """抬头 (对齐 HeadAction action=1).

        参数:
            angle:    抬头角度 (1-15°)
            speed_ms: 动作时长 (毫秒)
        """
        return self.bridge._head_move(
            action=1, angle=angle, speed=speed_ms
        )

    def head_look_down(self, angle: int = 10, speed_ms: int = 500) -> Dict:
        """低头 (对齐 HeadAction action=2).

        参数:
            angle:    低头角度 (1-15°)
            speed_ms: 动作时长 (毫秒)
        """
        return self.bridge._head_move(
            action=2, angle=angle, speed=speed_ms
        )

    def head_nod(self, times: int = 1, angle: int = 10,
                 speed_ms: int = 500) -> Dict:
        """点头 (对齐 HeadAction action=3).

        参数:
            times:    点头次数
            angle:    幅度 (1-15°)
            speed_ms: 单次动作时长 (毫秒)
        """
        return self.bridge._head_move(
            action=3, angle=angle, speed=speed_ms, steps=times
        )

    def head_center(self, speed_ms: int = 500) -> Dict:
        """头部回中心 (对齐 HeadAction action=4).

        参数:
            speed_ms: 动作时长 (毫秒)
        """
        return self.bridge._head_move(
            action=4, angle=0, speed=speed_ms
        )

    def head_continuous_nod(self, times: int = 2, angle: int = 10,
                            speed_ms: int = 500) -> Dict:
        """连续点头 (对齐 HeadAction action=5).

        参数:
            times:    连续点头次数
            angle:    幅度 (1-15°)
            speed_ms: 单次动作时长 (毫秒)
        """
        return self.bridge._head_move(
            action=5, angle=angle, speed=speed_ms, steps=times
        )

    # ================================================================
    #  组合动作
    # ================================================================

    def home(self, speed_ms: int = 1000) -> Dict:
        """复位到初始姿态 (对齐固件 home).

        舵机 home: [180, 180, 0, 0, 90, 90]
        机械关节: [0, -45, 0, -45, 0, 0]
        """
        return self.bridge._home(speed=speed_ms)

    def stop(self) -> Dict:
        """紧急停止并复位 (对齐固件 Stop)."""
        return self.bridge._stop()

    def execute_sequence(self, sequence: Dict) -> Dict:
        """执行 AI 生成的动作序列 (对齐 ExecuteServoSequence).

        参数:
            sequence: dict 或 JSON 字符串, 格式:
                {
                  "a": [
                    {"s": {"rp": 90, "lp": 90, "h": 100}, "v": 800, "d": 0},
                    {"osc": {"a": {"rp": 20}, "o": {"rp": 120}, "p": 400, "c": 4}}
                  ],
                  "d": 0
                }
        """
        if isinstance(sequence, dict):
            sequence = json.dumps(sequence)
        return self.bridge._servo_sequences(sequence)

    # ================================================================
    #  单舵机控制 (仿真专属, 对齐 servo_move)
    # ================================================================

    def servo_move(self, servo_type: str, position: float,
                   speed_ms: int = 800) -> Dict:
        """单舵机精确定位 (线性插值).

        参数:
            servo_type: 舵机类型, "rp"/"rr"/"lp"/"lr"/"body"/"head" 或全名
            position:   目标角度 (度, 舵机坐标系)
            speed_ms:   动作时长 (毫秒)
        """
        return self.bridge._servo_move(
            servo_type=servo_type, position=position, speed=speed_ms
        )

    # ================================================================
    #  内部辅助
    # ================================================================

    @staticmethod
    def _parse_hand(hand: str) -> int:
        """手部参数解析: 'left'/'right'/'both' → 1/2/3 (对齐固件 hand 参数)."""
        hand_lower = hand.lower()
        if hand_lower in ("left", "l", "1"):
            return 1
        elif hand_lower in ("right", "r", "2"):
            return 2
        elif hand_lower in ("both", "b", "all", "3"):
            return 3
        else:
            raise ValueError(f"无效 hand 参数: {hand}, 可选: left/right/both")


# ═══════════════════════════════════════════════════════════════════
#  便捷工厂函数
# ═══════════════════════════════════════════════════════════════════
def create_actions(env, bridge=None) -> ElectronBotActions:
    """便捷工厂: 创建 ElectronBotActions 实例.

    若 bridge 为 None, 自动创建 McpSimBridge.
    """
    if bridge is None:
        from ..mcp_bridge import McpSimBridge
        bridge = McpSimBridge(env)
    return ElectronBotActions(env, bridge)


__all__ = ["ElectronBotActions", "create_actions"]
