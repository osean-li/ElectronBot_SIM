#!/usr/bin/env python3
"""
动作解析器: 自然语言 → 6 维关节角度

支持多种解析策略:
1. VLM 推理 (主模式)
2. 关键词匹配 (fallback 兜底)
3. 动作库查找 (预定义动作)

确保每一步产出的关节角度都在安全范围内。
"""

import numpy as np
from typing import List, Tuple, Optional
from enum import Enum


class ActionParser:
    """动作指令解析器"""

    # 预定义动作库
    PRESET_ACTIONS = {
        "wave":          [0, 0, 0, 0, 80, 15],
        "wave_left":     [0, 0, 80, 15, 0, 0],
        "point_right":   [0, 0, 0, 0, 60, 0],
        "point_left":    [0, 0, 60, 0, 0, 0],
        "point_up":      [0, 0, 0, 0, 120, 0],
        "nod":           [0, 12, 0, 0, 0, 0],
        "shake_head":    [40, 0, 0, 0, 0, 0],
        "heart":         [0, 0, 60, 20, 60, 20],
        "look_left":     [45, 0, 0, 0, 0, 0],
        "look_right":    [-45, 0, 0, 0, 0, 0],
        "look_up":       [0, 10, 0, 0, 0, 0],
        "look_down":     [0, -10, 0, 0, 0, 0],
        "tired":         [0, -10, 20, 0, 20, 0],
        "excited":       [30, 15, 80, 20, 80, 20],
        "greet":         [0, 10, 0, 0, 80, 15],
        "bye":           [0, 0, 0, 0, 80, 15],
        "rest":          [0, 0, 0, 0, 0, 0],
        "zero":          [0, 0, 0, 0, 0, 0],
    }

    # 关键词匹配 (汉字 → 预设动作)
    KEYWORD_MAP = {
        ("挥", "招呼", "hi", "hello", "wave"): "wave",
        ("点", "nod", "点头", "同意"): "nod",
        ("摇", "shake", "摇头", "不同意"): "shake_head",
        ("指", "point", "指向"): "point_right",
        ("心", "heart", "比心", "比个心"): "heart",
        ("累", "tired", "疲倦", "困"): "tired",
        ("开心", "兴奋", "excited", "激动"): "excited",
        ("再见", "bye", "拜拜"): "bye",
        ("左看", "look left"): "look_left",
        ("右看", "look right"): "look_right",
        ("休息", "rest", "零位", "归零"): "rest",
    }

    def __init__(self, vla_backend=None):
        self.vla_backend = vla_backend

    def parse_by_keyword(self, text: str) -> Optional[np.ndarray]:
        """关键词匹配 (fallback)"""
        text_lower = text.lower()

        for keywords, action_name in self.KEYWORD_MAP.items():
            for kw in keywords:
                if kw.lower() in text_lower:
                    if action_name in self.PRESET_ACTIONS:
                        return np.radians(np.array(
                            self.PRESET_ACTIONS[action_name],
                            dtype=np.float64
                        ))
        return None

    def parse_by_vla(self, text: str, image: np.ndarray) -> Optional[np.ndarray]:
        """VLM 推理 (主模式)"""
        if self.vla_backend is None:
            return None

        from .prompt_templates import parse_vlm_output
        # 实际 VLM 调用
        output_raw = self.vla_backend.predict(image, text)
        # output_raw 在 mock 模式下直接返回角度数组
        if isinstance(output_raw, np.ndarray) and len(output_raw) == 6:
            return output_raw

        return parse_vlm_output(str(output_raw))

    def parse(
        self,
        text: str,
        image: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        解析指令 → 关节角度 (rad)

        策略链:
        1. 如果有 VLA backend + 图像 → VLM 推理
        2. 否则 → 关键词匹配
        3. 都失败 → 零位 (安全默认)
        """
        # 策略 1: VLA
        if image is not None and self.vla_backend is not None:
            result = self.parse_by_vla(text, image)
            if result is not None:
                return self._clip_angles(result)

        # 策略 2: 关键词
        result = self.parse_by_keyword(text)
        if result is not None:
            return result

        # 策略 3: 安全默认
        print(f"[WARN] 无法解析指令: '{text}', 返回零位")
        return np.zeros(6)

    def _clip_angles(self, angles: np.ndarray) -> np.ndarray:
        """裁切到安全范围"""
        from electronbot_mujoco.utils import JOINT_MODEL_MIN, JOINT_MODEL_MAX
        low = np.radians(JOINT_MODEL_MIN)
        high = np.radians(JOINT_MODEL_MAX)
        return np.clip(angles, low, high)

    def get_preset_list(self) -> List[str]:
        """获取所有可用预设动作名称"""
        return list(self.PRESET_ACTIONS.keys())

    def get_preset_angles(self, name: str) -> Optional[np.ndarray]:
        """获取预设动作角度 (rad)"""
        if name in self.PRESET_ACTIONS:
            return np.radians(np.array(self.PRESET_ACTIONS[name], dtype=np.float64))
        return None
