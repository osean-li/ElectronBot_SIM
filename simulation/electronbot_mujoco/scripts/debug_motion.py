#!/usr/bin/env python3
"""
ElectronBot 运动调试脚本 - 定位"不动"问题的根本原因

测试内容:
1. 直接设置qpos（绕过执行器）
2. 使用send_position_command（通过执行器）
3. 对比两种方式的差异
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
    print("  ElectronBot 运动调试 - 定位'不动'问题")
    print("="*70)

    # 测试目标: 右肩抬起120°
    target_deg = np.array([0, 0, 0, 0, 120, 25])  # wave动作
    target_rad = np.radians(target_deg)

    print(f"\n📌 测试目标角度 (度): {target_deg}")
    print(f"📌 测试目标角度 (弧度): {target_rad}")
    print(f"   [body, head, L_pitch, L_roll, R_pitch, R_roll]")

    # ── 测试1: 直接设置 qpos (绕过执行器) ──
    print("\n" + "─"*70)
    print("  🧪 测试1: 直接设置 qpos (绕过执行器)")
    print("─"*70)

    robot.reset()
    initial_pos = np.degrees(robot.get_joint_positions())
    print(f"  初始位置: {initial_pos.astype(int)}°")

    # 直接设置目标位置
    robot.set_joint_positions(target_rad)
    mujoco.mj_forward(robot.model, robot.data)
    after_set = np.degrees(robot.get_joint_positions())
    print(f"  设置后位置: {after_set.astype(int)}°")

    # 做几步物理仿真看看会不会被弹回来
    for i in range(100):
        robot.step()
        if i % 20 == 0:
            pos = np.degrees(robot.get_joint_positions())
            print(f"  步进{i:3d}后: {pos.astype(int)}°")

    final_direct = np.degrees(robot.get_joint_positions())
    print(f"  最终位置: {final_direct.astype(int)}°")
    error_direct = np.abs(target_deg - final_direct)
    print(f"  目标误差: {error_direct.astype(int)}°")

    # ── 测试2: 使用 send_position_command (通过执行器) ──
    print("\n" + "─"*70)
    print("  🧪 测试2: 使用 send_position_command (通过Position Actuator)")
    print("─"*70)

    robot.reset()
    initial_pos2 = np.degrees(robot.get_joint_positions())
    print(f"  初始位置: {initial_pos2.astype(int)}°")

    # 发送位置命令
    robot.send_position_command(target_rad)
    print(f"  已发送位置命令...")

    # 检查ctrl值是否正确设置
    ctrl_values = robot.data.ctrl.copy()
    print(f"  执行器ctrl值 (弧度): {ctrl_values}")
    print(f"  执行器ctrl值 (度): {np.degrees(ctrl_values).astype(int)}°")

    # 步进并观察
    print("\n  开始步进物理仿真...")
    for i in range(500):
        robot.step()
        if i % 50 == 0:
            pos = np.degrees(robot.get_joint_positions())
            error = np.abs(target_deg - pos)
            print(f"  步进{i:3d}后: {pos.astype(int)}° | 误差: {error.astype(int)}°")

    final_actuator = np.degrees(robot.get_joint_positions())
    print(f"\n  最终位置: {final_actuator.astype(int)}°")
    error_actuator = np.abs(target_deg - final_actuator)
    print(f"  目标误差: {error_actuator.astype(int)}°")

    # ── 测试3: 检查执行器参数 ──
    print("\n" + "─"*70)
    print("  🧪 测试3: 检查执行器和关节配置")
    print("─"*70)

    print(f"\n  关节限位 (range, 弧度):")
    for i, name in enumerate(robot.JOINT_NAMES):
        jid = robot._joint_ids[i]
        range_min = robot.model.jnt_range[jid, 0]
        range_max = robot.model.jnt_range[jid, 1]
        print(f"    {name:25s}: [{np.degrees(range_min):7.1f}°, {np.degrees(range_max):7.1f}°]")

    print(f"\n  执行器控制范围 (ctrlrange, 弧度):")
    for i, name in enumerate(robot.ACTUATOR_NAMES):
        aid = robot._actuator_ids[i]
        if robot.model.actuator_ctrllimited[aid]:
            range_min = robot.model.actuator_ctrlrange[aid, 0]
            range_max = robot.model.actuator_ctrlrange[aid, 1]
            limited = "✓"
        else:
            range_min = -float('inf')
            range_max = float('inf')
            limited = "✗ (无限制)"
        print(f"    {name:25s}: [{np.degrees(range_min):7.1f}°, {np.degrees(range_max):7.1f}°] {limited}")

    print(f"\n  执行器增益参数:")
    for i, name in enumerate(robot.ACTUATOR_NAMES):
        aid = robot._actuator_ids[i]
        kp = robot.model.actuator_gainprm[aid, 0]
        kv = robot.model.actuator_biasprm[aid, 1]
        force_min = robot.model.actuator_forcerange[aid, 0]
        force_max = robot.model.actuator_forcerange[aid, 1]
        print(f"    {name:25s}: kp={kp:6.1f}, kv={kv:5.2f}, force=[{force_min:+.1f}, {force_max:+.1f}] Nm")

    # ── 结果总结 ──
    print("\n" + "="*70)
    print("  📊 调试结论")
    print("="*70)

    if np.max(error_direct) < 5:
        print("  ✅ 测试1成功: 直接设置qpos可以到达目标位置")
        print("     → 问题在执行器层（kp/forcerange/控制逻辑）")
    else:
        print("  ❌ 测试1失败: 即使直接设置qpos也无法保持")
        print("     → 问题在物理约束（关节限位/碰撞/质量太大）")

    if np.max(error_actuator) < 10:
        print("  ✅ 测试2成功: 通过执行器可以到达目标位置")
        print("     → 原来的visual_demo应该能工作，可能是时间不够")
    else:
        print("  ❌ 测试2失败: 通过执行器无法到达目标位置")
        print("     → 需要增大kp/forcerange或延长稳定时间")


if __name__ == "__main__":
    main()