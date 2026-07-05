"""CapabilityDowngrader — 能力降级转换器.

对齐 docs/tasks/08-Sim2Real 详细设计说明书 §4.

═══════════════════════════════════════════════════════════════════
  设计目标
═══════════════════════════════════════════════════════════════════
  仿真中可使用 12 个 MCP 工具 (含 servo_move / servo_sequences 等 @sim_only),
  但真机 release v2.2.6 仅支持 8 个预设动作工具.
  本模块负责将仿真独有命令降级为预设动作组合, 实现 Sim2Real.

  降级映射表:
  ┌─────────────────────┬───────────────────────────────┬──────────┐
  │ 仿真独有命令         │ 降级目标 (真机预设动作)         │ 精度损失 │
  ├─────────────────────┼───────────────────────────────┼──────────┤
  │ servo_move          │ hand_action (按角度方向判断)    │ 高       │
  │ servo_sequences     │ hand_action + body_turn + ...  │ 中       │
  │ home                │ stop (功能等价)                │ 无       │
  │ osc (振荡器)        │ 多次 hand_action               │ 高       │
  │ ws://IP:8080        │ 云端 API 调用                  │ 无       │
  └─────────────────────┴───────────────────────────────┴──────────┘
"""
from __future__ import annotations

import json
import logging
from typing import Dict, List, Optional

import numpy as np

logger = logging.getLogger("electronbot_sim2real.capability_downgrade")

# ONNX 舵机中心 (基准位置, 用于降级判断)
SERVO_CENTER_ONNX = np.array([180.0, 140.0, 0.0, 40.0, 90.0, 90.0], dtype=np.float32)


class CapabilityDowngrader:
    """能力降级转换器 — 仿真命令 → 真机预设动作.

    参数:
        target: 降级目标版本, 默认 "release_v2.2.6"
    """

    def __init__(self, target: str = "release_v2.2.6"):
        self.target = target
        self.downgrade_count = 0
        self.untranslatable: List[Dict] = []

    def downgrade_sequence(self, sequence_json: str) -> Optional[Dict]:
        """将 servo_sequences JSON 降级为预设动作组合.

        参数:
            sequence_json: servo_sequences 的 JSON 字符串

        返回:
            成功: {"actions": [{"tool": ..., "args": {...}}, ...],
                   "warnings": [...]}
            失败: None (不可转换的动作已记录到 untranslatable)
        """
        try:
            seq = json.loads(sequence_json) if isinstance(sequence_json, str) else sequence_json
        except json.JSONDecodeError as e:
            logger.error("序列 JSON 解析失败: %s", e)
            self.untranslatable.append({"reason": "json_parse_error", "raw": sequence_json})
            return None

        actions: List[Dict] = []
        warnings: List[str] = []

        for i, act in enumerate(seq.get("a", [])):
            if "s" in act:
                # 普通动作: {"s": {"rp": 90, "lp": 90}, "v": 800}
                servo_dict = act["s"]
                downgraded = self._downgrade_servo_dict(servo_dict, act.get("v", 800))
                if downgraded:
                    actions.extend(downgraded)
                else:
                    warnings.append(f"动作 {i} 无法降级: {servo_dict}")
                    self.untranslatable.append({
                        "reason": "servo_dict_untranslatable",
                        "action_index": i,
                        "raw": act,
                    })
            elif "osc" in act:
                # 振荡器: {"osc": {"a": {...}, "o": {...}, "p": 400, "c": 4}}
                osc = act["osc"]
                downgraded = self._downgrade_oscillation(osc)
                if downgraded:
                    actions.extend(downgraded)
                else:
                    warnings.append(f"振荡器 {i} 无法降级: {osc}")
                    self.untranslatable.append({
                        "reason": "oscillation_untranslatable",
                        "action_index": i,
                        "raw": act,
                    })

        self.downgrade_count += len(actions)
        return {"actions": actions, "warnings": warnings}

    def _downgrade_servo_dict(self, servo_dict: Dict, velocity: int) -> List[Dict]:
        """将单步舵机目标字典降级为预设动作.

        降级逻辑:
        - rp_diff = servo[0] - 180, lp_diff = servo[2] - 0
        - 若 |rp_diff| > 15 或 |lp_diff| > 15 → hand_action
        - b_diff = servo[4] - 90, 若 |b_diff| > 5 → body_turn
        - h_diff = servo[5] - 90, 若 |h_diff| > 2 → head_move
        """
        actions: List[Dict] = []
        speed = max(500, min(1500, int(velocity)))

        # 构造 6 维舵机数组 (缺失的用中心值填充)
        servo_arr = SERVO_CENTER_ONNX.copy()
        name_to_idx = {"rp": 0, "rr": 1, "lp": 2, "lr": 3, "body": 4, "head": 5}
        for name, val in servo_dict.items():
            if name in name_to_idx:
                servo_arr[name_to_idx[name]] = val

        # ── 手部动作判断 ──
        rp_diff = servo_arr[0] - 180.0  # RP 中心 180
        lp_diff = servo_arr[2] - 0.0    # LP 中心 0
        hand_action_triggered = False

        if rp_diff < -30:
            # 右臂上举 (RP 角度小 = 高举)
            actions.append({
                "tool": "self.electron.hand_action",
                "args": {"action": 1, "hand": 2, "steps": 1, "speed": speed},
            })
            hand_action_triggered = True
        elif rp_diff > 30:
            # 右臂下放
            actions.append({
                "tool": "self.electron.hand_action",
                "args": {"action": 2, "hand": 2, "steps": 1, "speed": speed},
            })
            hand_action_triggered = True

        if lp_diff > 30:
            # 左臂上举 (LP 角度大 = 高举)
            actions.append({
                "tool": "self.electron.hand_action",
                "args": {"action": 1, "hand": 1, "steps": 1, "speed": speed},
            })
            hand_action_triggered = True
        elif lp_diff < -30:
            # 左臂下放
            actions.append({
                "tool": "self.electron.hand_action",
                "args": {"action": 2, "hand": 1, "steps": 1, "speed": speed},
            })
            hand_action_triggered = True

        # ── 身体动作判断 ──
        b_diff = servo_arr[4] - 90.0  # BODY 中心 90
        if abs(b_diff) > 5:
            if b_diff > 0:
                # 左转
                actions.append({
                    "tool": "self.electron.body_turn",
                    "args": {"direction": 1, "angle": min(90, int(abs(b_diff))),
                             "speed": 800},
                })
            else:
                # 右转
                actions.append({
                    "tool": "self.electron.body_turn",
                    "args": {"direction": 2, "angle": min(90, int(abs(b_diff))),
                             "speed": 800},
                })

        # ── 头部动作判断 ──
        h_diff = servo_arr[5] - 90.0  # HEAD 中心 90
        if abs(h_diff) > 2:
            if h_diff > 0:
                # 抬头
                actions.append({
                    "tool": "self.electron.head_move",
                    "args": {"action": 1, "angle": min(15, int(abs(h_diff))),
                             "speed": 600},
                })
            else:
                # 低头
                actions.append({
                    "tool": "self.electron.head_move",
                    "args": {"action": 2, "angle": min(15, int(abs(h_diff))),
                             "speed": 600},
                })

        if not actions:
            # 无显著动作, 降级为 stop (避免无操作)
            logger.debug("舵机字典无显著动作, 降级为 stop: %s", servo_dict)

        return actions

    def _downgrade_oscillation(self, osc: Dict) -> List[Dict]:
        """将振荡器降级为多次预设动作 (高精度损失).

        降级策略: 按周期数 c 展开为离散挥手动作.
        """
        cycles = osc.get("c", 1)
        actions: List[Dict] = []
        for _ in range(max(1, int(cycles))):
            actions.append({
                "tool": "self.electron.hand_action",
                "args": {"action": 3, "hand": 3, "steps": 1, "speed": 600},
            })
        return actions

    def downgrade_action_6d(self, action: np.ndarray) -> List[Dict]:
        """将 6D 舵机角度动作降级为预设动作列表.

        参数:
            action: (6,) 舵机角度数组, 顺序 [RP, RR, LP, LR, BODY, HEAD]

        返回: 预设动作列表 [{"tool": ..., "args": {...}}, ...]
        """
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] != 6:
            logger.error("降级失败: 动作维度 %s != 6", action.shape)
            return []

        servo_dict = {
            "rp": float(action[0]),
            "rr": float(action[1]),
            "lp": float(action[2]),
            "lr": float(action[3]),
            "body": float(action[4]),
            "head": float(action[5]),
        }
        return self._downgrade_servo_dict(servo_dict, velocity=800)

    def get_report(self) -> str:
        """生成降级统计报告."""
        return (
            f"CapabilityDowngrader 报告 (目标: {self.target})\n"
            f"  降级动作总数: {self.downgrade_count}\n"
            f"  不可转换动作数: {len(self.untranslatable)}\n"
            f"  不可转换详情: {self.untranslatable[:10]}"
        )
