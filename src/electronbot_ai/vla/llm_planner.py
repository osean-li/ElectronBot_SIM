"""VLA 规划器 — 语音指令 → MCP 动作序列.

对齐 docs/tasks/06-AI-Training §5.

═══════════════════════════════════════════════════════════════════
  关键区分 (Sim2Real 设计决策)
═══════════════════════════════════════════════════════════════════
  1. 纯文本 VLA (TextVLAPlanner) — 真机可用, 推荐 Sim2Real 首选
     输入: 仅语音指令 (纯文本)
     输出: 8 个真机可用预设动作的 MCP 序列
     延迟: 200-500ms RTT (云端 API), 对预设动作可接受

  2. 视觉 VLA (VisionVLAPlanner) — 仿真专属, 真机无摄像头
     输入: 语音指令 + 摄像头图像
     输出: servo_sequence (可使用 sim_only 工具)
     用途: 仿真中的复杂场景理解和操作规划研究
═══════════════════════════════════════════════════════════════════
"""
from __future__ import annotations

import json
import logging
import re
from typing import Any, Optional

import numpy as np

logger = logging.getLogger("electronbot_ai.vla.llm_planner")

# 真机可用的 8 个预设动作工具 (release v2.2.6)
REAL_MACHINE_TOOLS = {
    "self.electron.hand_action",
    "self.electron.body_turn",
    "self.electron.head_move",
    "self.electron.stop",
    "self.electron.get_status",
    "self.electron.set_trim",
    "self.electron.get_trims",
    "self.battery.get_level",
}

# sim_only 工具 (真机 release v2.2.6 不可用)
SIM_ONLY_TOOLS = {
    "self.electron.servo_move",
    "self.electron.servo_sequences",
    "self.electron.home",
    "self.electron.get_ip",
}

# LLM 模型默认配置
DEFAULT_LLM_MODELS = {
    "text": "qwen2.5-7b",   # 纯文本 LLM
    "vision": "qwen2.5-vl", # 视觉语言模型
}


class TextVLAPlanner:
    """纯文本 VLA 规划器 — 真机可用的 Sim2Real 首选路径.

    输入: 用户语音转写的纯文本指令
    输出: MCP 动作序列 (仅使用 8 个真机可用预设动作)

    优势:
      - 真机可用 (无需摄像头硬件)
      - 延迟可接受 (200-500ms RTT)
      - 输出格式固定, 易于校验

    使用方式:
      planner = TextVLAPlanner()
      sequence = planner.plan("举起右手")
      planner.execute("挥手打招呼")
    """

    def __init__(self, llm_model: str = DEFAULT_LLM_MODELS["text"],
                 backend: Optional[Any] = None):
        """
        参数:
            llm_model: LLM 模型名
            backend:   ElectronBotBackend 实例 (sim/cloud), None 则不执行
        """
        self.llm_model = llm_model
        self.backend = backend
        self._llm = None  # 延迟加载 LLM

    def _load_llm(self):
        """延迟加载 LLM 模型."""
        if self._llm is not None:
            return self._llm
        try:
            from transformers import pipeline
            self._llm = pipeline("text-generation", model=self.llm_model)
            logger.info("LLM 已加载: %s", self.llm_model)
        except ImportError:
            logger.warning("transformers 未安装, 使用规则匹配 fallback")
            self._llm = None
        except Exception as e:
            logger.warning("LLM 加载失败 (%s), 使用规则匹配 fallback", e)
            self._llm = None
        return self._llm

    def _build_prompt(self, instruction: str) -> str:
        """构造 LLM prompt (含可用工具说明)."""
        return f"""你控制一个名为 ElectronBot 的桌面机器人。它有两个手臂、一个可旋转的头部和一个可旋转的身体。

可用 MCP 工具（真机可用）：
- self.electron.hand_action: action(1=举手,2=放手,3=挥手,4=拍打), hand(1=左,2=右,3=双), steps, speed, amount
- self.electron.body_turn: direction(1=左转,2=右转,3=回中), steps, speed, angle(0-90°)
- self.electron.head_move: action(1=抬头,2=低头,3=点头,4=回中,5=连续点头), steps, speed, angle(1-15°)
- self.electron.stop: 紧急停止

请根据用户指令"{instruction}"，生成一个 MCP 动作序列 JSON：
{{
  "actions": [
    {{"tool": "self.electron.hand_action", "args": {{"action": 3, "hand": 3, "steps": 2, "speed": 600}}}},
    ...
  ]
}}
只输出 JSON，不要其他文字。不要使用 servo_move 或 servo_sequences。"""

    def _rule_based_plan(self, instruction: str) -> dict:
        """规则匹配 fallback (LLM 不可用时使用)."""
        instruction = instruction.lower().strip()
        actions = []

        if "举手" in instruction or "举起" in instruction:
            if "右" in instruction:
                actions.append({"tool": "self.electron.hand_action",
                               "args": {"action": 1, "hand": 2, "steps": 1, "speed": 800}})
            elif "左" in instruction:
                actions.append({"tool": "self.electron.hand_action",
                               "args": {"action": 1, "hand": 1, "steps": 1, "speed": 800}})
            else:
                actions.append({"tool": "self.electron.hand_action",
                               "args": {"action": 1, "hand": 3, "steps": 1, "speed": 800}})

        if "挥手" in instruction or "打招呼" in instruction:
            actions.append({"tool": "self.electron.hand_action",
                           "args": {"action": 3, "hand": 3, "steps": 2, "speed": 600}})

        if "转身" in instruction or "转过来" in instruction or "看" in instruction:
            if "左" in instruction:
                actions.append({"tool": "self.electron.body_turn",
                               "args": {"direction": 1, "angle": 45, "speed": 800}})
            elif "右" in instruction:
                actions.append({"tool": "self.electron.body_turn",
                               "args": {"direction": 2, "angle": 45, "speed": 800}})
            else:
                actions.append({"tool": "self.electron.body_turn",
                               "args": {"direction": 1, "angle": 45, "speed": 800}})

        if "点头" in instruction:
            actions.append({"tool": "self.electron.head_move",
                           "args": {"action": 3, "steps": 2, "speed": 500, "angle": 10}})

        if "抬头" in instruction:
            actions.append({"tool": "self.electron.head_move",
                           "args": {"action": 1, "steps": 1, "speed": 500, "angle": 10}})

        if "低头" in instruction:
            actions.append({"tool": "self.electron.head_move",
                           "args": {"action": 2, "steps": 1, "speed": 500, "angle": 10}})

        if "拍" in instruction or "拍打" in instruction:
            actions.append({"tool": "self.electron.hand_action",
                           "args": {"action": 4, "hand": 3, "steps": 2, "speed": 500}})

        if not actions:
            actions.append({"tool": "self.electron.stop", "args": {}})

        return {"actions": actions}

    def _parse_response(self, response: str) -> dict:
        """解析 LLM 输出为可执行的序列 (带校验)."""
        # 尝试提取 JSON 块
        json_match = re.search(r'\{[\s\S]*\}', response)
        if not json_match:
            logger.warning("LLM 输出未找到 JSON 块, fallback 到规则匹配")
            return None
        try:
            sequence = json.loads(json_match.group())
        except json.JSONDecodeError as e:
            logger.warning("LLM 输出 JSON 解析失败: %s", e)
            return None

        # 校验工具名合法性 (仅允许真机可用工具)
        for action in sequence.get("actions", []):
            tool = action.get("tool", "")
            if tool not in REAL_MACHINE_TOOLS and tool not in SIM_ONLY_TOOLS:
                logger.warning("非法工具名: %s, 跳过该动作", tool)
                action["tool"] = "self.electron.stop"
                action["args"] = {}

        return sequence

    def plan(self, instruction: str, max_retries: int = 3) -> dict:
        """语音指令 → MCP 动作序列.

        参数:
            instruction: 用户语音转写的纯文本指令
            max_retries: LLM 输出解析失败时的重试次数

        返回:
            dict: {"actions": [{"tool": str, "args": dict}, ...]}
        """
        llm = self._load_llm()

        if llm is None:
            # 规则匹配 fallback
            result = self._rule_based_plan(instruction)
            logger.info("规则匹配规划: %s → %d 个动作", instruction, len(result["actions"]))
            return result

        # LLM 推理
        for attempt in range(max_retries):
            try:
                prompt = self._build_prompt(instruction)
                response = llm(prompt, max_new_tokens=256, return_full_text=False)[0]["generated_text"]
                sequence = self._parse_response(response)
                if sequence is not None:
                    logger.info("LLM 规划: %s → %d 个动作", instruction, len(sequence.get("actions", [])))
                    return sequence
            except Exception as e:
                logger.warning("LLM 推理失败 (第 %d 次): %s", attempt + 1, e)

        # 全部失败, fallback 到规则匹配
        logger.warning("LLM 推理全部失败, fallback 到规则匹配")
        return self._rule_based_plan(instruction)

    def execute(self, instruction: str) -> dict:
        """端到端: 语音指令 → 生成动作 → 执行.

        需要 self.backend 已设置 (sim/cloud 模式).
        """
        if self.backend is None:
            logger.warning("backend 未设置, 仅规划不执行")
            return self.plan(instruction)

        sequence = self.plan(instruction)
        results = []
        for action in sequence.get("actions", []):
            tool = action["tool"]
            args = action.get("args", {})
            try:
                result = self.backend.call(tool, args)
                results.append({"tool": tool, "result": result})
                logger.info("执行 %s: %s", tool, result)
            except Exception as e:
                logger.error("执行 %s 失败: %s", tool, e)
                results.append({"tool": tool, "error": str(e)})
        return {"actions": sequence["actions"], "results": results}


class VisionVLAPlanner:
    """视觉 VLA 规划器 — 仿真专属 (真机无摄像头).

    输入: 语音指令 + 摄像头图像
    输出: servo_sequence (可使用 sim_only 工具)

    ⚠️ 仿真专属: 真机 ElectronBot 无摄像头硬件
    """

    def __init__(self, llm_model: str = DEFAULT_LLM_MODELS["vision"],
                 env: Optional[Any] = None, backend: Optional[Any] = None):
        self.llm_model = llm_model
        self.env = env
        self.backend = backend
        self._llm = None

    def _load_llm(self):
        """延迟加载视觉语言模型."""
        if self._llm is not None:
            return self._llm
        try:
            from transformers import pipeline
            self._llm = pipeline("image-text-to-text", model=self.llm_model)
            logger.info("VLM 已加载: %s", self.llm_model)
        except Exception as e:
            logger.warning("VLM 加载失败 (%s), 使用规则匹配 fallback", e)
            self._llm = None
        return self._llm

    def _build_prompt(self, instruction: str) -> str:
        return f"""你控制一个名为 ElectronBot 的桌面机器人。它有两个手臂、一个可旋转的头部和一个可旋转的身体。

可用舵机（及其短键和安全范围）：
- right_pitch (rp): 0-180, 右臂上下摆动
- right_roll  (rr): 100-180, 右臂前后推拉
- left_pitch  (lp): 0-180, 左臂上下摆动
- left_roll   (lr): 0-80, 左臂前后推拉
- body (b): 30-150, 腰部旋转
- head (h): 75-105, 头部俯仰

请根据摄像头图像和用户指令"{instruction}"，
生成一个 servo_sequence JSON（可以使用 servo_move 精确控制）：
{{
  "a": [
    {{"s": {{"rp": 90, "lp": 90}}, "v": 500}},
    ...
  ]
}}
只输出 JSON，不要其他文字。"""

    def plan(self, instruction: str, camera_image: Optional[np.ndarray] = None) -> dict:
        """语音指令 + 摄像头图像 → servo_sequence."""
        llm = self._load_llm()

        if llm is None or camera_image is None:
            # 规则匹配 fallback
            return self._rule_based_plan(instruction)

        try:
            prompt = self._build_prompt(instruction)
            response = llm(camera_image, prompt, max_new_tokens=256)
            return self._parse_response(response)
        except Exception as e:
            logger.warning("VLM 推理失败: %s, fallback 到规则匹配", e)
            return self._rule_based_plan(instruction)

    def _rule_based_plan(self, instruction: str) -> dict:
        """规则匹配 fallback."""
        instruction = instruction.lower().strip()
        actions = []

        if "举手" in instruction or "举起" in instruction:
            actions.append({"s": {"rp": 0, "lp": 180}, "v": 800})

        if "挥手" in instruction:
            actions.append({"s": {"rp": 30, "lp": 150}, "v": 600})
            actions.append({"osc": {"a": {"rp": 30, "lp": 30},
                                     "o": {"rp": 30, "lp": 150}, "p": 400, "c": 3}})

        if "转身" in instruction or "转过来" in instruction:
            actions.append({"s": {"b": 135}, "v": 800})

        if "点头" in instruction:
            actions.append({"s": {"h": 105}, "v": 300})
            actions.append({"s": {"h": 75}, "v": 300})
            actions.append({"s": {"h": 90}, "v": 300})

        if not actions:
            actions.append({"s": {"rp": 180, "lp": 0, "b": 90, "h": 90}, "v": 1000})

        return {"a": actions}

    def _parse_response(self, response: str) -> dict:
        """解析 LLM 输出."""
        json_match = re.search(r'\{[\s\S]*\}', response)
        if not json_match:
            return self._rule_based_plan("unknown")
        try:
            return json.loads(json_match.group())
        except json.JSONDecodeError:
            return self._rule_based_plan("unknown")

    def execute(self, instruction: str) -> dict:
        """端到端执行 (仅仿真)."""
        cam_image = None
        if self.env is not None:
            try:
                from electronbot_sim.sensors import CameraSensor
                cam = CameraSensor(self.env)
                cam_image = cam.capture()[0]
            except Exception as e:
                logger.warning("摄像头采集失败: %s", e)

        sequence = self.plan(instruction, cam_image)

        if self.backend is not None:
            # 通过 servo_sequences 工具执行
            return self.backend.call("self.electron.servo_sequences",
                                     {"sequence": json.dumps(sequence)})
        return sequence


# 便捷工厂函数
def create_vla_planner(mode: str = "text", **kwargs) -> Any:
    """创建 VLA 规划器.

    参数:
        mode: "text" (纯文本, 真机可用) / "vision" (视觉, 仿真专属)
        **kwargs: 透传给规划器构造函数

    返回:
        TextVLAPlanner 或 VisionVLAPlanner
    """
    if mode == "text":
        return TextVLAPlanner(**kwargs)
    elif mode == "vision":
        return VisionVLAPlanner(**kwargs)
    else:
        raise ValueError(f"未知 VLA 模式: {mode}. 可选: text / vision")
