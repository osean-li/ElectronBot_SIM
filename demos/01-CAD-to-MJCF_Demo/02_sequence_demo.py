#!/usr/bin/env python3
"""
Demo 2: 程序控制 — 自动执行预设动作序列

用法:
  # 有桌面 - 交互窗口实时演示
  python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py --interactive

  # 无桌面 (SSH/服务器) - 生成帧序列和 GIF
  MUJOCO_GL=egl python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py

  # 指定输出目录
  python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py --output /tmp/my_demo

关节顺序 (对应 electronbot_full_arm.xml):
  [body_joint, head_joint, left_pitch_joint, left_roll_joint, right_pitch_joint, right_roll_joint]
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import numpy as np

# ── 项目路径 ──
PROJECT = Path(__file__).resolve().parent.parent.parent
XML_PATH = PROJECT / "assets" / "mjcf" / "electronbot_full_arm.xml"


# ── 预设动作 (角度制) ──
# 关节顺序: [body(腰部Z), head(头部Y), L_pitch(左臂Y), L_roll(左臂X), R_pitch(右臂Y), R_roll(右臂X)]
BUILTIN_POSES = {
    "zero":         [  0,  0,   0,   0,   0,  0],
    "wave":         [  0,  0,   0,   0,  60, 40],
    "wave_left":    [  0,  0,  60,  40,   0,  0],
    "nod":          [  0, 15,   0,   0,   0,  0],
    "heart":        [  0,  0, 160,  40, 160, 40],
    "point_left":   [  0,  0, 140,   0,   0,  0],
    "point_right":  [  0,  0,   0,   0, 140,  0],
    "excited":      [ 30, 15, 170,  40, 170, 40],
    "look_left":    [-50,  0,   0,   0,   0,  0],
    "look_right":   [ 50,  0,   0,   0,   0,  0],
    "bye_left":     [-40,  0,  90,   0,  90, 40],
    "bye_right":    [ 40,  0,  90,  40,  90,  0],
}

SEQUENCE = [
    ("zero",        3.0),
    ("wave",        5.0),
    ("wave_left",   5.0),
    ("zero",        2.5),
    ("nod",         5.0),
    ("zero",        2.5),
    ("heart",       6.0),
    ("zero",        2.5),
    ("point_left",  4.0),
    ("point_right", 4.0),
    ("zero",        2.5),
    ("bye_left",    5.0),
    ("bye_right",   5.0),
    ("zero",        2.5),
    ("look_left",   4.0),
    ("look_right",  4.0),
    ("zero",        2.5),
    ("excited",     6.0),
]


def describe_action(name: str) -> str:
    """返回动作的中文描述"""
    descriptions = {
        "zero":        "回到初始姿态",
        "wave":        "挥右手",
        "wave_left":   "挥左手",
        "nod":         "点头",
        "heart":       "比心（双臂举高）",
        "point_left":  "手指向左",
        "point_right": "手指向右",
        "excited":     "兴奋（转身+举手）",
        "look_left":   "向左看（腰部左转）",
        "look_right":  "向右看（腰部右转）",
        "bye_left":    "左手再见",
        "bye_right":   "右手再见",
    }
    return descriptions.get(name, name)


def interpolate(prev: np.ndarray, current: np.ndarray, alpha: float) -> np.ndarray:
    """ease-in-out 插值"""
    alpha = alpha * alpha * (3 - 2 * alpha)
    return prev + (current - prev) * alpha


def run_headless(xml_path: str, output_dir: str = "/tmp/electronbot_demo"):
    """无头模式: 渲染帧序列 → 图片 / GIF"""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 480, 640)
    # 使用默认相机，MuJoCo 自动适配机器人模型

    frame_dir = Path(output_dir)
    frame_dir.mkdir(parents=True, exist_ok=True)

    # 初始化
    data.qpos[:6] = np.zeros(6)
    mujoco.mj_forward(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)

    print("=" * 60)
    print("  ElectronBot 动作序列演示")
    print(f"  输出目录: {frame_dir}/")
    print("=" * 60)

    current_target = np.zeros(6)
    prev_target = np.zeros(6)
    pose_idx = 0
    frame_count = 0
    current_target = np.array(BUILTIN_POSES[SEQUENCE[0][0]], dtype=float)
    pose_start_frame = 0
    total_frames = int(sum(d for _, d in SEQUENCE) * 10)

    print(f"  ▶ {describe_action(SEQUENCE[0][0])}")

    for frame in range(total_frames):
        elapsed = frame - pose_start_frame
        pose_name, pose_duration = SEQUENCE[pose_idx]

        if elapsed >= pose_duration * 10:
            prev_target = current_target.copy()
            pose_idx = (pose_idx + 1) % len(SEQUENCE)
            pose_name, pose_duration = SEQUENCE[pose_idx]
            current_target = np.array(BUILTIN_POSES[pose_name], dtype=float)
            pose_start_frame = frame
            elapsed = 0
            print(f"  ▶ {describe_action(pose_name)}")

        target = interpolate(prev_target, current_target,
                             min(elapsed / (pose_duration * 10), 1.0))

        data.ctrl[:6] = np.radians(target)
        for _ in range(50):
            mujoco.mj_step(model, data)

        if frame % 5 == 0:
            renderer.update_scene(data)
            pixels = renderer.render()
            if pixels is not None:
                try:
                    import cv2
                    cv2.imwrite(str(frame_dir / f"frame_{frame_count:04d}.png"),
                                cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
                except ImportError:
                    from PIL import Image
                    Image.fromarray(pixels).save(
                        frame_dir / f"frame_{frame_count:04d}.png")
                frame_count += 1

    renderer.close()
    print(f"\n  完成! 共渲染 {frame_count} 帧 → {frame_dir}/")

    try:
        from PIL import Image
        frames = []
        for i in range(frame_count):
            fp = frame_dir / f"frame_{i:04d}.png"
            if fp.exists():
                frames.append(Image.open(fp))
        if len(frames) > 1:
            gif_path = str(frame_dir / "demo.gif")
            frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                           duration=50, loop=0)
            print(f"  GIF: {gif_path}")
            for i in range(frame_count):
                (frame_dir / f"frame_{i:04d}.png").unlink(missing_ok=True)
    except Exception as e:
        print(f"  (GIF: {e})")


def run_interactive(xml_path: str):
    """交互模式: mujoco.viewer 实时窗口 + 自动播序列"""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    data.qpos[:6] = np.zeros(6)
    mujoco.mj_forward(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)

    print("=" * 60)
    print("  ElectronBot 动作序列演示")
    print("  空格键 暂停/恢复 | 右键+拖拽 旋转 | 滚轮 缩放")
    print("=" * 60)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        pose_idx = 0
        prev_target = np.zeros(6)
        current_target = np.array(BUILTIN_POSES[SEQUENCE[0][0]], dtype=float)
        pose_start = time.time()
        paused = False
        prev_pose_name = ""

        while viewer.is_running():
            if not paused:
                elapsed = time.time() - pose_start
                pose_name, pose_duration = SEQUENCE[pose_idx]

                if elapsed > pose_duration:
                    prev_target = current_target.copy()
                    pose_idx = (pose_idx + 1) % len(SEQUENCE)
                    pose_name, pose_duration = SEQUENCE[pose_idx]
                    current_target = np.array(BUILTIN_POSES[pose_name], dtype=float)
                    pose_start = time.time()
                    prev_pose_name = ""

                if pose_name != prev_pose_name:
                    print(f"  ▶ {describe_action(pose_name)}")
                    prev_pose_name = pose_name

                target = interpolate(prev_target, current_target,
                                     min(elapsed / pose_duration, 1.0))
                data.ctrl[:6] = np.radians(target)
                for _ in range(50):
                    mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(0.001)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="ElectronBot 动作序列演示")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互模式 (需要 display)")
    parser.add_argument("--output", default=str(PROJECT / "demos/01-CAD-to-MJCF_Demo/02_sequence_demo_gif"),
                        help="无头模式输出目录")
    args = parser.parse_args()

    xml = str(XML_PATH)
    if not Path(xml).exists():
        print(f"错误: 模型文件不存在: {xml}")
        sys.exit(1)

    if args.interactive:
        run_interactive(xml)
    else:
        run_headless(xml, args.output)


if __name__ == "__main__":
    main()
