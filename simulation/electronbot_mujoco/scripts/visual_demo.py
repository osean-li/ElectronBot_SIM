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
# 增大角度幅度，使动作更明显
# 注意: roll关节限位为0-30°(0-0.5236rad)，shoulder限位为-20°~180°(-0.3491~3.1416rad)
# 避免使用边界值，留1-2°余量
BUILTIN_POSES = {
    "zero":        np.array([ 0,  0,   0,  0,   0,  0]),
    "wave":        np.array([ 0,  0,   0,  0, 120, 22]),      # 大幅挥手 (roll留余量)
    "nod":         np.array([ 0, 15,   0,  0,   0,  0]),       # 点头 (head限位±15°)
    "heart":       np.array([ 0,  0,  85, 28,  85, 28]),      # 比心 (避免边界)
    "point_left":  np.array([ 0,  0,  85,  0,   0,  0]),       # 左手指
    "point_right": np.array([ 0,  0,   0,  0,  85,  0]),       # 右手指
    "excited":     np.array([40, 15, 110, 28, 110, 28]),       # 兴奋 (各轴留余量)
    "look_left":   np.array([-50, 0,   0,  0,   0,  0]),       # 转身 (body限位±90°)
    "look_right":  np.array([ 50, 0,   0,  0,   0,  0]),       # 转身
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
    cam.lookat[:] = [0, 0, 0.12]
    cam.distance = 1.0
    cam.azimuth = 135
    cam.elevation = -20
    
    # 初始化相机到默认视角
    mujoco.mjv_defaultFreeCamera(robot.model, cam)
    opt = mujoco.MjvOption()

    paused = False
    pose_idx = 0
    prev_target = np.zeros(6)
    current_target = BUILTIN_POSES[SEQUENCE[0][0]].copy()
    pose_start = time.time()

    print(f"  [{SEQUENCE[0][0]:12s}] {SEQUENCE[0][1]:.1f}s  target={current_target}")

    # 完全重置物理状态到零位
    robot.data.qpos[:6] = np.zeros(6)
    robot.data.qvel[:6] = np.zeros(6)
    robot.data.qacc[:6] = np.zeros(6)
    robot.send_position_command(np.zeros(6))
    mujoco.mj_forward(robot.model, robot.data)
    
    # 稳定化
    for _ in range(500):
        robot.step()
    
    # 打印初始状态
    init_angles = np.degrees(robot.get_joint_positions())
    init_error = np.abs(init_angles - 0)
    print(f"\n  [INIT] 初始角度: {init_angles.astype(int)}°")
    print(f"  [INIT] 初始误差: {init_error.astype(int)}° (应该接近0)")
    if np.any(init_error > 5):
        print(f"  ⚠️  警告: 初始误差过大! 某些关节可能未正确归零")
    
    print("\n" + "="*60)
    print("  🎬 开始演示！按空格键可暂停/恢复")
    print("  📋 动作序列:", " → ".join([f"{s[0]}({s[1]}s)" for s in SEQUENCE]))
    print("="*60 + "\n")

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
                
                # 动作切换提示（带emoji和进度）
                progress = f"[{pose_idx+1}/{len(SEQUENCE)}]"
                emoji_map = {
                    "zero": "⚪", "wave": "👋", "nod": "😊",
                    "heart": "❤️", "point_left": "👈", "point_right": "👉"
                }
                emoji = emoji_map.get(pose_name, "🎯")
                
                # 检测是否完成一个完整循环
                if pose_idx == 0:
                    print(f"\n{'🔄'*30}")
                    print(f"  🎊 完成一个演示周期！开始新一轮循环...")
                    print(f"{'🔄'*30}\n")
                
                print(f"\n{'─'*60}")
                print(f"  🔄 切换动作 {progress}: {emoji} **{pose_name.upper()}** (持续{pose_duration}秒)")
                print(f"     目标角度: {current_target.astype(int)}°")
                print(f"     [body, head, L_pitch, L_roll, R_pitch, R_roll]")
                print(f"{'─'*60}\n")
                
                elapsed = 0

            alpha = min(elapsed / pose_duration, 1.0)
            alpha = alpha * alpha * (3 - 2 * alpha)
            target = prev_target + (current_target - prev_target) * alpha
            robot.send_position_command(np.radians(target))
            
            # 多步物理仿真以确保到达目标位置
            for _ in range(50):
                robot.step()
            
            # 关键调试输出: 每10帧打印一次（约0.3秒一次）
            if int(elapsed * 10) % 3 == 0:
                current_angles = np.degrees(robot.get_joint_positions())
                error = np.abs(target - current_angles)
                max_error_idx = np.argmax(error)
                joint_names = ['body', 'head', 'L_pitch', 'L_roll', 'R_pitch', 'R_roll']
                print(f"  [DEBUG] {pose_name:12s} t={elapsed:5.2f}/{pose_duration:.1f}s | "
                      f"MAX_ERR={error[max_error_idx]:5.1f}° @ {joint_names[max_error_idx]} | "
                      f"target[{max_error_idx}]={target[max_error_idx]:5.1f}° vs now={current_angles[max_error_idx]:5.1f}°")
            
            # 动作即将结束时提示
            if pose_duration - elapsed < 0.2 and elapsed > 0 and int(elapsed * 10) == int((elapsed - 0.01) * 10):
                print(f"  [{pose_name}] 完成")

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
            if paused:
                print(f"\n{'='*60}")
                print(f"  ⏸️  **已暂停** - 当前动作: {pose_name}")
                print(f"     进度: {elapsed:.1f}s / {pose_duration:.1f}s ({100*elapsed/pose_duration:.0f}%)")
                current_angles = np.degrees(robot.get_joint_positions())
                print(f"     当前角度: {current_angles.astype(int)}°")
                print(f"  💡 再次按空格键继续...")
                print(f"{'='*60}\n")
            else:
                print(f"\n  ▶️  **已恢复** - 继续执行 {pose_name} 动作\n")
            time.sleep(0.2)

        time.sleep(0.001)

    print(f"\n{'='*60}")
    print(f"  👋 演示结束！感谢使用 ElectronBot 可视化演示")
    print(f"{'='*60}\n")
    
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