#!/usr/bin/env python3
"""
VLA 提示词模板库

定义系统提示词、任务描述、输出格式约束，
确保 VLM 输出可解析的结构化动作指令。
"""

from typing import Dict, List, Optional
import numpy as np


# ============================================================
# 系统提示词
# ============================================================

SYSTEM_PROMPT = """你是一个桌面双臂机器人 ElectronBot 的动作控制器。

机器人参数:
- 6 个关节自由度: [body_yaw, head_pitch, left_arm_pitch, left_arm_roll, right_arm_pitch, right_arm_roll]
- 关节角度范围 (度):
  0. body_yaw (腰部旋转):       [-90, 90]
  1. head_pitch (头部俯仰):     [-15, 15]
  2. left_arm_pitch (左臂俯仰): [-20, 180]
  3. left_arm_roll (左臂roll):  [0, 30]
  4. right_arm_pitch (右臂俯仰):[-20, 180]
  5. right_arm_roll (右臂roll): [0, 30]

你的任务:
1. 观察摄像头图像
2. 理解用户指令
3. 输出 6 维关节角度数组

输出格式 (严格遵守):
```json
{"joint_angles_deg": [body, head, left_p, left_r, right_p, right_r], "explanation": "简短说明"}
```
"""

# ============================================================
# 任务模板
# ============================================================

TASK_TEMPLATES: Dict[str, str] = {
    "wave": "请向我挥手打招呼。抬起右手臂，做 2-3 次挥手动作。",
    "wave_left": "请用左手向我挥手打招呼。",
    "point_left": "请用右手往左边指。",
    "point_right": "请用右手往右边指。",
    "point_up": "请用右手往上指。",
    "nod": "请点头表示同意。",
    "shake_head": "请摇头表示不同意。",
    "heart": "请摆出比心姿势。",
    "look_around": "请好奇地四处张望，先左边再右边。",
    "tired": "请表现得很疲倦，缓慢地低下头。",
    "excited": "请表现得很兴奋，快速摇头和挥手。",
    "greet": "看到人时，请点头并挥手打招呼。",
    "bye": "请挥手告别。",
    "touch_object": "请用右手去触碰前方的物体。",
    "follow_hand": "请用眼睛追踪移动的手。",
}

# ============================================================
# Few-shot 示例 (帮助 VLM 输出正确格式)
# ============================================================

FEW_SHOT_EXAMPLES = [
    {
        "user": "挥手",
        "assistant": json.dumps({
            "joint_angles_deg": [0, 0, 0, 0, 80, 10],
            "explanation": "抬起右臂80°做挥手姿势，roll 10°"
        })
    },
    {
        "user": "点头",
        "assistant": json.dumps({
            "joint_angles_deg": [0, 10, 0, 0, 0, 0],
            "explanation": "头部前倾10°表示点头"
        })
    },
    {
        "user": "摇头",
        "assistant": json.dumps({
            "joint_angles_deg": [30, 0, 0, 0, 0, 0],
            "explanation": "腰部旋转30°表示摇头"
        })
    },
    {
        "user": "指左边",
        "assistant": json.dumps({
            "joint_angles_deg": [0, 0, 0, 0, 45, 0],
            "explanation": "右臂抬起45°指向左方"
        })
    },
    {
        "user": "比心",
        "assistant": json.dumps({
            "joint_angles_deg": [0, 0, 60, 20, 60, 20],
            "explanation": "双臂抬起60°并roll 20°，形成心形"
        })
    },
]

# ============================================================
# 输出解析
# ============================================================

def parse_vlm_output(text: str) -> Optional[np.ndarray]:
    """
    从 VLM 输出中提取 6 维关节角度

    支持格式:
    1. JSON: {"joint_angles_deg": [...]}
    2. 纯数组: [0, 10, 30, 0, 30, 0]
    3. 逗号分隔: 0,10,30,0,30,0
    """
    import json
    import re

    # 尝试 JSON
    try:
        # 提取 JSON 块
        match = re.search(r'\{[^}]*"joint_angles_deg"[^}]*\}', text)
        if match:
            data = json.loads(match.group())
            angles = data.get("joint_angles_deg", [])
            if len(angles) == 6:
                return np.radians(np.array(angles, dtype=np.float64))
    except (json.JSONDecodeError, KeyError):
        pass

    # 尝试提取纯数组
    arr_match = re.findall(r'-?\d+\.?\d*', text)
    if len(arr_match) >= 6:
        angles = [float(x) for x in arr_match[:6]]
        return np.radians(np.array(angles, dtype=np.float64))

    return None


def build_vla_messages(
    task_name: str,
    custom_prompt: Optional[str] = None,
    include_few_shot: bool = True,
) -> List[Dict]:
    """
    构建发送给 VLM 的完整消息

    返回:
      [{"role": "system", "content": "..."},
       {"role": "user", "content": [{"type": "image", ...}, {"type": "text", ...}]}]
    """
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    # Few-shot examples
    if include_few_shot:
        for ex in FEW_SHOT_EXAMPLES[:2]:  # 只用前 2 个示例节省 token
            messages.append({"role": "user", "content": ex["user"]})
            messages.append({"role": "assistant", "content": ex["assistant"]})

    # 任务提示词
    task_prompt = custom_prompt or TASK_TEMPLATES.get(
        task_name, f"请执行: {task_name}"
    )

    # VLM 消息格式 (多模态)
    user_content = [
        {"type": "image", "image": "{IMAGE_PLACEHOLDER}"},
        {"type": "text", "text": task_prompt},
    ]
    messages.append({"role": "user", "content": user_content})

    return messages
