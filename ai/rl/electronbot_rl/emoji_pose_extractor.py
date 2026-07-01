#!/usr/bin/env python3
"""
从 Emoji 情绪动画 MP4 提取关键姿态, 供 emotional_reward.py 做运动模仿

情绪 → 关键帧关节角度 (°):
  不屑 (disdain):  摇头 + 单臂下放
  愤怒 (angry):    快速抖动 + 双臂收紧
  惊恐 (fear):     身体后仰 + 双臂张开
  难过 (sad):      低头 + 缓慢下垂
  兴奋 (excited):  高频挥手 + 身体摇摆

输出: data/emoji_animations/emoji_poses.json
"""

import os, json
import numpy as np

EMOJI_DIR = "/home/j6m/code/github/ElectronBot_SIM/data/emoji_animations"
OUT_PATH = os.path.join(EMOJI_DIR, "emoji_poses.json")


# 手工标注的关键帧姿态 (基于 ElectronBot 6 关节模型角度)
# 格式: [body, head, left_pitch, left_roll, right_pitch, right_roll] (°)
# 每个情绪有 3 个阶段: enter(进入姿势) → loop(循环动作) → reset(回正)
EMOJI_POSES = {
    "disdain": {  # 不屑
        "enter": [0, 0, 0, 0, 30, 10],
        "loop":  [15, -5, 0, 0, 25, 5],
        "reset": [0, 0, 0, 0, 0, 0],
    },
    "angry": {   # 愤怒
        "enter": [0, 5, 80, 20, 80, 20],
        "loop":  [20, 8, 85, 25, 85, 25],
        "reset": [0, 0, 0, 0, 0, 0],
    },
    "fear": {    # 惊恐
        "enter": [0, -5, 40, 15, 40, 15],
        "loop":  [-15, -8, 50, 20, 50, 20],
        "reset": [0, 0, 0, 0, 0, 0],
    },
    "sad": {     # 难过
        "enter": [0, -10, 0, 0, 0, 0],
        "loop":  [0, -12, 10, 0, 10, 0],
        "reset": [0, 0, 0, 0, 0, 0],
    },
    "excited": {  # 兴奋
        "enter": [0, 5, 60, 10, 60, 10],
        "loop":  [30, 10, 70, 15, 70, 15],
        "reset": [0, 0, 0, 0, 0, 0],
    },
    "static": {   # 静态 (眨眼)
        "enter": [0, 0, 0, 0, 0, 0],
        "loop":  [0, 0, 0, 0, 0, 0],
        "reset": [0, 0, 0, 0, 0, 0],
    },
}


def save_poses():
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, 'w', encoding='utf-8') as f:
        json.dump(EMOJI_POSES, f, indent=2, ensure_ascii=False)
    print(f"Saved {len(EMOJI_POSES)} emotion poses → {OUT_PATH}")


def load_poses():
    with open(OUT_PATH) as f:
        return json.load(f)


def get_target_pose(emotion: str, phase: str = "loop") -> np.ndarray:
    """获取指定情绪的目标姿势 (°, 6维)"""
    poses = load_poses()
    return np.array(poses[emotion][phase])


if __name__ == "__main__":
    save_poses()
    # 打印预览
    for name, phases in EMOJI_POSES.items():
        angles = phases["loop"]
        print(f"  {name:12s}: body={angles[0]:4d}° head={angles[1]:3d}° "
              f"L=[{angles[2]:3d}° {angles[3]:2d}°] R=[{angles[4]:3d}° {angles[5]:2d}°]")
