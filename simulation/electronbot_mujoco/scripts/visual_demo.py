#!/usr/bin/env python3
"""
ElectronBot 可视化运动演示 (兼容无桌面环境)

用法:
  # 有桌面 (原生窗口):
  python simulation/electronbot_mujoco/scripts/visual_demo.py

  # 无桌面 (SSH/服务器):
  MUJOCO_GL=egl python simulation/electronbot_mujoco/scripts/visual_demo.py --headless

演示内容:
  1. 挥手  (右手左右摆动)
  2. 点头  (头部俯仰)
  3. 比心  (双臂抬起内收)
  4. 左右指点 / 转身 / 兴奋
"""

import sys
import time
import os
import numpy as np
from pathlib import Path

_project = Path(__file__).parent.parent
sys.path.insert(0, str(_project))

import mujoco
from electronbot_mujoco.robot import ElectronBotRobot


# ── 预设动作 (模型角度: body, head, left_pitch, left_roll, right_pitch, right_roll) 度 ──
BUILTIN_POSES = {
    "zero":        np.array([ 0,  0,  0,  0,  0,  0]),
    "wave":        np.array([ 0,  0,  0,  0, 80, 15]),
    "nod":         np.array([ 0, 12,  0,  0,  0,  0]),
    "heart":       np.array([ 0,  0, 60, 20, 60, 20]),
    "point_left":  np.array([ 0,  0, 60,  0,  0,  0]),
    "point_right": np.array([ 0,  0,  0,  0, 60,  0]),
    "excited":     np.array([30, 15, 80, 20, 80, 20]),
    "look_left":   np.array([-40, 0,  0,  0,  0,  0]),
    "look_right":  np.array([ 40, 0,  0,  0,  0,  0]),
}

SEQUENCE = [
    ("zero",        1.0),
    ("wave",        2.0),
    ("zero",        0.5),
    ("nod",         1.5),
    ("zero",        0.5),
    ("heart",       2.0),
    ("zero",        0.5),
    ("point_left",  1.0),
    ("point_right", 1.0),
    ("zero",        0.5),
    ("look_left",   1.0),
    ("look_right",  1.0),
    ("zero",        0.5),
    ("excited",     2.0),
    ("zero",        1.0),
]


def run_headless():
    """无头模式: 用 offscreen renderer 渲染帧并输出进度"""
    xml_path = str(_project / "electronbot_mujoco" / "assets" / "electronbot_inline.xml")
    robot = ElectronBotRobot(xml_path=xml_path)
    model = robot.model
    data = robot.data

    renderer = mujoco.Renderer(model, 480, 480)
    frame_dir = Path("/tmp/electronbot_demo_frames")
    frame_dir.mkdir(exist_ok=True)

    print("=" * 60)
    print("  ElectronBot 可视化演示 (无头模式)")
    print(f"  帧保存到: {frame_dir}/")
    print("=" * 60)

    sim_timestep = 0.002    # 500Hz physics
    control_interval = 10   # 每 10 步 = 50Hz control
    render_interval = 30    # 每 30 步 = ~16Hz render

    current_target = np.zeros(6)
    prev_target = np.zeros(6)
    pose_idx = 0
    frame_count = 0

    # 初始化第一个动作
    pose_name, duration = SEQUENCE[0]
    current_target = BUILTIN_POSES[pose_name].copy()
    pose_start_step = 0
    total_steps = int(sum(d for _, d in SEQUENCE) * 500)  # 500 steps/s

    for step in range(total_steps):
        elapsed = step - pose_start_step
        pose_name, pose_duration = SEQUENCE[pose_idx]

        # 切换动作
        if elapsed >= pose_duration * 500:
            prev_target = current_target.copy()
            pose_idx = (pose_idx + 1) % len(SEQUENCE)
            pose_name, pose_duration = SEQUENCE[pose_idx]
            current_target = BUILTIN_POSES[pose_name].copy()
            pose_start_step = step
            elapsed = 0
            print(f"  [{pose_name:12s}] {pose_duration:.1f}s  target={current_target}")

        # 插值
        alpha = min(elapsed / (pose_duration * 500), 1.0)
        alpha = alpha * alpha * (3 - 2 * alpha)  # ease-in-out
        target = prev_target + (current_target - prev_target) * alpha

        if step % control_interval == 0:
            robot.send_position_command(np.radians(target))

        mujoco.mj_step(model, data)

        # 渲染
        if step % render_interval == 0:
            renderer.update_scene(data)
            pixels = renderer.render()
            if pixels is not None:
                from PIL import Image
                img = Image.fromarray(pixels)
                img.save(frame_dir / f"frame_{frame_count:04d}.png")
                frame_count += 1

    renderer.close()
    print(f"\n  完成! 共渲染 {frame_count} 帧")
    print(f"  查看: ls {frame_dir}/")

    # 生成 GIF (如果有 Pillow)
    try:
        from PIL import Image
        frames = []
        for i in range(frame_count):
            fp = frame_dir / f"frame_{i:04d}.png"
            if fp.exists():
                frames.append(Image.open(fp))
        if frames:
            gif_path = str(frame_dir / "demo.gif")
            frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                           duration=50, loop=0)
            print(f"  GIF: {gif_path}")
            # 清理单帧
            for i in range(frame_count):
                (frame_dir / f"frame_{i:04d}.png").unlink(missing_ok=True)
    except Exception as e:
        print(f"  (GIF 生成跳过: {e})")


def run_interactive():
    """交互模式: mujoco.viewer 原生窗口 (需要 display)"""
    xml_path = str(_project / "electronbot_mujoco" / "assets" / "electronbot_inline.xml")
    robot = ElectronBotRobot(xml_path=xml_path)

    print("=" * 60)
    print("  ElectronBot 可视化演示 (交互模式)")
    print("  空格键暂停/恢复")
    print("=" * 60)

    import glfw
    if not glfw.init():
        raise RuntimeError("GLFW 初始化失败")

    glfw.window_hint(glfw.VISIBLE, 1)
    window = glfw.create_window(800, 600, "ElectronBot Demo", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("无法创建 GLFW 窗口")

    glfw.make_context_current(window)
    scene = mujoco.MjvScene(robot.model, maxgeom=1000)
    context = mujoco.MjrContext(robot.model, mujoco.mjtFontScale.mjFONTSCALE_150)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0, 0, 0.01]
    cam.distance = 0.35
    cam.azimuth = 180
    cam.elevation = -15
    opt = mujoco.MjvOption()

    paused = False
    pose_idx = 0
    prev_target = np.zeros(6)
    current_target = BUILTIN_POSES[SEQUENCE[0][0]].copy()
    pose_start = time.time()

    print(f"  [{SEQUENCE[0][0]:12s}] {SEQUENCE[0][1]:.1f}s  target={current_target}")

    while not glfw.window_should_close(window):
        if not paused:
            elapsed = time.time() - pose_start
            pose_name, pose_duration = SEQUENCE[pose_idx]

            if elapsed > pose_duration:
                prev_target = current_target.copy()
                pose_idx = (pose_idx + 1) % len(SEQUENCE)
                pose_name, pose_duration = SEQUENCE[pose_idx]
                current_target = BUILTIN_POSES[pose_name].copy()
                pose_start = time.time()
                print(f"  [{pose_name:12s}] {pose_duration:.1f}s  target={current_target}")
                elapsed = 0

            alpha = min(elapsed / pose_duration, 1.0)
            alpha = alpha * alpha * (3 - 2 * alpha)
            target = prev_target + (current_target - prev_target) * alpha
            robot.send_position_command(np.radians(target))
            robot.step()

        # 渲染
        viewport = mujoco.MjrRect(0, 0, 800, 600)
        mujoco.mjv_updateScene(robot.model, robot.data, opt, None, cam,
                                mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
        glfw.poll_events()

        # 空格键
        if glfw.get_key(window, glfw.KEY_SPACE) == glfw.PRESS:
            paused = not paused
            print(f"  {'暂停' if paused else '恢复'}")
            time.sleep(0.2)

        time.sleep(0.001)

    glfw.terminate()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--headless", action="store_true", help="无头模式 (off-screen 渲染)")
    args = p.parse_args()

    if args.headless or ("DISPLAY" not in os.environ and "WAYLAND_DISPLAY" not in os.environ):
        run_headless()
    else:
        run_interactive()
