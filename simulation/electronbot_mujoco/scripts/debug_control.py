---#!/usr/bin/env python3
"""
ElectronBot 控制链路诊断工具

诊断项目:
1. 执行器ID映射是否正确
2. 控制信号(ctrl)是否正确设置
3. 执行器输出力矩(actuator_force)是否正常
4. 关节位置(qpos)是否在更新
5. 物理仿真步进是否有效
"""

import sys
import numpy as np
from pathlib import Path

_project = Path(__file__).parent.parent
sys.path.insert(0, str(_project))

import mujoco
from electronbot_mujoco.robot import ElectronBotRobot


def diagnose_control_chain():
    """完整诊断控制链路"""
    xml_path = str(_project / "electronbot_mujoco" / "assets" / "electronbot_inline.xml")
    robot = ElectronBotRobot(xml_path=xml_path)

    print("=" * 70)
    print("  ElectronBot 控制链路诊断")
    print("=" * 70)

    # ── 1. 检查模型和执行器信息 ──
    print("\n[1] 模型基本信息:")
    print(f"  关节数量: {robot.model.nq}")
    print(f"  执行器数量: {robot.model.nu}")
    print(f"  时间步长: {robot.model.opt.timestep*1000:.2f} ms")

    print("\n[2] 关节列表:")
    for i, name in enumerate(robot.JOINT_NAMES):
        jid = robot._joint_ids[i]
        joint_range = robot.model.jnt_range[jid]
        print(f"  [{i}] {name:25s} (id={jid:2d}) | "
              f"range=[{np.degrees(joint_range[0]):+6.1f}°, {np.degrees(joint_range[1]):+6.1f}°]")

    print("\n[3] 执行器列表:")
    for i, name in enumerate(robot.ACTUATOR_NAMES):
        aid = robot._actuator_ids[i]
        kp = robot.model.actuator_gainprm[aid, 0]
        kv = robot.model.actuator_gainprm[aid, 1]
        ctrl_range = robot.model.actuator_ctrlrange[aid]
        force_range = robot.model.actuator_forcerange[aid]
        print(f"  [{i}] {name:25s} (id={aid:2d}) | "
              f"kp={kp:5.1f} kv={kv:4.1f} | "
              f"ctrl=[{np.degrees(ctrl_range[0]):+6.2f}°, {np.degrees(ctrl_range[1]):+6.2f}°] | "
              f"force=[{force_range[0]:+5.1f}, {force_range[1]:+5.1f}] Nm")

    # ── 2. 测试控制信号传递 ──
    print("\n" + "=" * 70)
    print("[4] 测试控制信号传递 (目标: 右肩90°)")
    print("=" * 70)

    # 重置到零位
    mujoco.mj_resetData(robot.model, robot.data)
    robot.send_position_command(np.zeros(6))
    mujoco.mj_forward(robot.model, robot.data)

    target_angles_deg = np.array([0, 0, 0, 0, 90, 15])  # 右肩90°, 右roll 15°
    target_angles_rad = np.radians(target_angles_deg)

    print(f"\n  目标角度 (deg): {target_angles_deg.astype(int)}")
    print(f"  目标角度 (rad): {target_angles_rad.round(4)}")

    # 设置控制信号
    robot.send_position_command(target_angles_rad)

    # 检查ctrl是否正确设置
    print("\n[4.1] 控制信号 (data.ctrl) 检查:")
    for i, name in enumerate(robot.JOINT_NAMES):
        aid = robot._actuator_ids[i]
        ctrl_val = robot.data.ctrl[aid]
        expected = target_angles_rad[i]
        status = "✅" if abs(ctrl_val - expected) < 0.01 else "❌"
        print(f"  {status} {name:25s}: ctrl={ctrl_val:+.4f} rad ({np.degrees(ctrl_val):+.2f}°) "
              f"(期望 {expected:+.4f} rad)")

    # ── 3. 测试单步物理仿真 ──
    print("\n[4.2] 单步物理仿真后状态:")
    robot.step()

    for i, name in enumerate(robot.JOINT_NAMES):
        jid = robot._joint_ids[i]
        aid = robot._actuator_ids[i]

        qpos = np.degrees(robot.data.qpos[jid])
        qvel = np.degrees(robot.data.qvel[jid])
        act_force = robot.data.actuator_force[aid]

        print(f"  {name:25s}: qpos={qpos:+7.2f}° | qvel={qvel:+7.2f}°/s | force={act_force:+7.3f} Nm")

    # ── 4. 多步仿真观察收敛过程 ──
    print("\n" + "=" * 70)
    print("[5] 多步物理仿真 - 收敛过程 (每10步打印一次)")
    print("=" * 70)

    mujoco.mj_resetData(robot.model, robot.data)
    robot.send_position_command(target_angles_rad)
    mujoco.mj_forward(robot.model, robot.data)

    print(f"\n  {'步骤':>6s} | {'R_shoulder':>12s} | {'R_roll':>12s} | {'R_shoulder_f':>14s} | {'R_roll_f':>12s}")
    print(f"  {'':6s} | {'(度)':>12s} | {'(度)':>12s} | {'(Nm)':>14s} | {'(Nm)':>12s}")
    print(f"  {'-'*70}")

    for step in range(201):
        if step % 10 == 0 or step < 5:
            r_shoulder_qpos = np.degrees(robot.data.qpos[robot._joint_ids[4]])
            r_roll_qpos = np.degrees(robot.data.qpos[robot._joint_ids[5]])
            r_shoulder_force = robot.data.actuator_force[robot._actuator_ids[4]]
            r_roll_force = robot.data.actuator_force[robot._actuator_ids[5]]

            marker = " ← 初始" if step == 0 else ""
            print(f"  {step:6d} | {r_shoulder_qpos:+12.2f}° | {r_roll_qpos:+12.2f}° | "
                  f"{r_shoulder_force:+14.4f} Nm | {r_roll_force:+12.4f} Nm{marker}")

        robot.step()

    # 最终状态
    print(f"\n  最终状态 (200步后):")
    final_angles = np.degrees(robot.get_joint_positions())
    error = np.abs(target_angles_deg - final_angles)
    max_error_idx = np.argmax(error)

    print(f"  目标:   {target_angles_deg.astype(int)}°")
    print(f"  实际:   {final_angles.round(1).astype(int)}°")
    print(f"  误差:   {error.round(1).astype(int)}°")
    print(f"  最大误差: {error[max_error_idx]:.1f}° @ {robot.JOINT_NAMES[max_error_idx]}")

    if error[max_error_idx] > 5:
        print(f"\n  ❌ 诊断发现问题! 最大误差过大")
        print(f"  可能原因:")
        print(f"    1. 执行器力矩不足 (forcerange太小)")
        print(f"    2. 增益参数过低 (kp/kv太小)")
        print(f"    3. 关节限位冲突 (target超出range)")
        print(f"    4. 物理参数异常 (质量/惯性过大)")
    else:
        print(f"\n  ✅ 控制链路正常!")

    # ── 5. 检查关节限位 ──
    print("\n" + "=" * 70)
    print("[6] 关节限位 vs 目标角度检查")
    print("=" * 70)

    for i, name in enumerate(robot.JOINT_NAMES):
        jid = robot._joint_ids[i]
        joint_range = robot.model.jnt_range[jid]
        target_deg = target_angles_deg[i]

        in_range = joint_range[0] <= np.radians(target_deg) <= joint_range[1]
        status = "✅ 在范围内" if in_range else "❌ 超出范围!"

        print(f"  {name:25s}: target={target_deg:+6.1f}° | "
              f"range=[{np.degrees(joint_range[0]):+6.1f}°, {np.degrees(joint_range[1]):+6.1f}°] | "
              f"{status}")

    print("\n" + "=" * 70)
    print("  诊断完成")
    print("=" * 70)


if __name__ == "__main__":
    diagnose_control_chain()