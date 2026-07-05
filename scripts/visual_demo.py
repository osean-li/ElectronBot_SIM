#!/usr/bin/env python3
"""
ElectronBot 可视化运动演示 (适配 xiaozhi/ElectronBot_SIM 当前结构)

用法:
  # 无桌面 (SSH/服务器) - 生成帧序列
  MUJOCO_GL=egl python scripts/visual_demo.py

  # 有桌面 - 交互窗口
  python scripts/visual_demo.py --interactive

关节顺序: [body, head, L_pitch, L_roll, R_pitch, R_roll]
"""
from __future__ import annotations

import sys
import time
import os
import argparse
from pathlib import Path

import mujoco
import numpy as np

# ── 项目路径 ──
PROJECT = Path(__file__).parent.parent

# 预设动作 (度数)
BUILTIN_POSES = {
    "zero":        [  0,  0,   0,   0,   0,  0],
    "wave":        [  0,  0,   0,   0,  60, 40],
    "wave_left":   [  0,  0,  60,  40,   0,  0],
    "nod":         [  0, 15,   0,   0,   0,  0],
    "heart":       [  0,  0, 160,  40, 160, 40],
    "point_left":  [  0,  0, 140,   0,   0,  0],
    "point_right": [  0,  0,   0,   0, 140,  0],
    "excited":     [ 30, 15, 170,  40, 170, 40],
    "look_left":   [-50,  0,   0,   0,   0,  0],
    "look_right":  [ 50,  0,   0,   0,   0,  0],
    "bye_left":    [-40,  0,  90,   0,  90, 40],
    "bye_right":   [ 40,  0,  90,  40,  90,  0],
}

SEQUENCE = [
    ("zero",        1.0),
    ("wave",        2.0),
    ("wave_left",   2.0),
    ("zero",        0.5),
    ("nod",         1.5),
    ("zero",        0.5),
    ("heart",       2.0),
    ("zero",        0.5),
    ("point_left",  1.0),
    ("point_right", 1.0),
    ("zero",        0.5),
    ("bye_left",    1.5),
    ("bye_right",   1.5),
    ("zero",        0.5),
    ("look_left",   1.0),
    ("look_right",  1.0),
    ("zero",        0.5),
    ("excited",     2.0),
]


def run_headless(xml_path: str, output_dir: str = "/tmp/electronbot_demo"):
    """无头模式: 渲染帧序列, 可选生成 GIF"""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 480, 640)

    frame_dir = Path(output_dir)
    frame_dir.mkdir(parents=True, exist_ok=True)

    # 初始化 - 让模型稳定
    data.qpos[:6] = np.zeros(6)
    mujoco.mj_forward(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)

    print("=" * 60)
    print("  ElectronBot 可视化演示 (无头模式)")
    print(f"  模型: {xml_path}")
    print(f"  帧保存到: {frame_dir}/")
    print(f"  动作序列: {' → '.join([s[0] for s in SEQUENCE])}")
    print("=" * 60)

    current_target = np.zeros(6)
    prev_target = np.zeros(6)
    pose_idx = 0
    frame_count = 0
    pose_name, pose_duration = SEQUENCE[0]
    current_target = np.array(BUILTIN_POSES[pose_name], dtype=float)
    pose_start_frame = 0
    total_frames = int(sum(d for _, d in SEQUENCE) * 10)  # 10 ctrl-frame/s

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
            print(f"  [{pose_name:12s}] {pose_duration:.1f}s  target={current_target.astype(int)}")

        # ease-in-out 插值
        alpha = min(elapsed / (pose_duration * 10), 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)
        target = prev_target + (current_target - prev_target) * alpha

        # 设置控制并仿真
        data.ctrl[:6] = np.radians(target)
        for _ in range(50):  # 每控制帧 50 个物理子步
            mujoco.mj_step(model, data)

        # 渲染 (每 5 帧一次, ~2Hz, 节省 IO)
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

    # 生成 GIF
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
        print(f"  (GIF: {e}, 保留单帧图片)")


def run_interactive(xml_path: str):
    """交互模式: mujoco.viewer 原生窗口"""
    print("=" * 60)
    print("  ElectronBot 可视化演示 (交互模式)")
    print("  空格键暂停/恢复 | 右键拖拽旋转 | 滚轮缩放")
    print("=" * 60)

    # 直接使用 mujoco viewer 的被动模式
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 初始化
    data.qpos[:6] = np.zeros(6)
    mujoco.mj_forward(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 重置相机位置
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        pose_idx = 0
        prev_target = np.zeros(6)
        current_target = np.array(BUILTIN_POSES[SEQUENCE[0][0]], dtype=float)
        pose_start = time.time()
        paused = False

        print(f"  开始演示! 动作: {' → '.join([s[0] for s in SEQUENCE])}")

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
                    print(f"  [{pose_name:12s}] {pose_duration:.1f}s")

                alpha = min(elapsed / pose_duration, 1.0)
                alpha = alpha * alpha * (3 - 2 * alpha)
                target = prev_target + (current_target - prev_target) * alpha

                data.ctrl[:6] = np.radians(target)
                for _ in range(50):
                    mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(0.001)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ElectronBot 可视化运动演示")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互模式 (需要 display)")
    parser.add_argument("--model", default=str(PROJECT / "assets/mjcf/scene_mesh.xml"),
                        help="MJCF 模型路径")
    parser.add_argument("--output", default="/tmp/electronbot_demo",
                        help="无头模式输出目录")
    args = parser.parse_args()

    xml = args.model
    if not Path(xml).exists():
        print(f"错误: 模型文件不存在: {xml}")
        sys.exit(1)

    if args.interactive:
        run_interactive(xml)
    else:
        run_headless(xml, args.output)
