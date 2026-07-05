"""视觉语言动作 (VLA) 子模块.

对齐 docs/tasks/06-AI-Training §5.

两种 VLA 模式:
  - 纯文本 VLA (TextVLAPlanner): 仅语音指令 → MCP 动作序列 [真机可用, 推荐]
  - 视觉 VLA (VisionVLAPlanner): 语音 + 摄像头图像 → servo_sequence [仿真专属]
"""
from __future__ import annotations

__all__ = ["llm_planner"]
