#!/usr/bin/env python3
"""
测试关闭重力后的运动情况
"""

import sys
import numpy as np
from pathlib import Path

_project = Path(__file__).parent.parent
sys.path.insert(0, str(_project))

import mujoco
from electronbot_mujoco.robot import ElectronBotRobot


def main():
    xml_path = str(_project / "electronbot_mujoco" / "assets" / "electronbot_inline.xml")
    robot = ElectronBotRobot(xml_path=xml_path)

    print("="*70)
    print("  重力影响测试")
    print("="*70)

    target_deg = np.array([0, 0, 0, 0, 120, 25])
    target_rad = np.radians(target_deg)

    # ── 测试1: 有重力 ──
    print("\n" + "─"*70)
    print("  🧪 测试1: 有重力 (默认)")
    print("─"*70)

    robot.reset()
    robot.model.opt.gravity[2] = -9.81  # 正常重力
    print(f"  重力: {robot.model.opt.gravity}")

    Kp, Kd = 200.0, 20.0
    for step in range(500):
        qpos = robot.get_joint_positions()
        qvel = robot.get_joint_velocities()
        error = target_rad - qpos
        torques = np.clip(Kp * error - Kd * qvel, -50, 50)
        robot.send_torque_command(torques)
        robot.step()

    pos_with_g = np.degrees(robot.get_joint_positions())
    err_with_g = np.abs(target_deg - pos_with_g)
    print(f"  最终位置: {pos_with_g.astype(int)}°")
    print(f"  最大误差: {np.max(err_with_g):.1f}°")

    # ── 测试2: 无重力 ──
    print("\n" + "─"*70)
    print("  🧪 测试2: 无重力")
    print("─"*70)

    robot.reset()
    robot.model.opt.gravity[2] = 0.0  # 关闭重力
    print(f"  重力: {robot.model.opt.gravity}")

    for step in range(500):
        qpos = robot.get_joint_positions()
        qvel = robot.get_joint_velocities()
        error = target_rad - qpos
        torques = np.clip(Kp * error - Kd * qvel, -50, 50)
        robot.send_torque_command(torques)
        robot.step()

    pos_no_g = np.degrees(robot.get_joint_positions())
    err_no_g = np.abs(target_deg - pos_no_g)
    print(f"  最终位置: {pos_no_g.astype(int)}°")
    print(f"  最大误差: {np.max(err_no_g):.1f}°")

    # ── 对比 ──
    print("\n" + "="*70)
    print("  📊 对比结果")
    print("="*70)
    print(f"\n  {'关节':<25s} {'有重力':<15s} {'无重力':<15s}")
    print(f"  {'-'*55}")
    for i, name in enumerate(robot.JOINT_NAMES):
        print(f"  {name:<25s} {pos_with_g[i]:>+8.1f}°     {pos_no_g[i]:>+8.1f}°")

    if np.max(err_no_g) < 10:
        print(f"\n  ✅ 无重力时可以到达目标位置！")
        print(f"     → 问题确认是重力太大或连杆太重")
        print(f"     → 解决方案: 减小几何体密度或增大力矩上限")
    else:
        print(f"\n  ❌ 无重力时也不行，问题不在重力")


if __name__ == "__main__":
    main()