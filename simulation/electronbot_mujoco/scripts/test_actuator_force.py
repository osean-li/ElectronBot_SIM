#!/usr/bin/env python3
"""
测试执行器实际输出的力矩，定位为什么500步只移动了-5°
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
    print("  执行器力矩输出测试")
    print("="*70)

    # 设置目标: 右肩120°
    target_rad = np.zeros(6)
    target_rad[4] = np.radians(120)  # right_shoulder

    robot.reset()
    robot.send_position_command(target_rad)

    print(f"\n目标: right_shoulder = 120°")
    print(f"初始位置: {np.degrees(robot.get_joint_positions()).astype(int)}°")

    # 检查执行器原始参数
    aid = robot._actuator_ids[4]  # act_right_shoulder
    print(f"\n执行器原始参数:")
    print(f"  gainprm = {robot.model.actuator_gainprm[aid]}")
    print(f"  biasprm = {robot.model.actuator_biasprm[aid]}")
    print(f"  forcerange = {robot.model.actuator_forcerange[aid]}")

    # 手动计算期望的力矩
    ctrl = robot.data.ctrl[aid]
    qpos = robot.data.qpos[robot._joint_ids[4]]
    qvel = robot.data.qvel[robot._joint_ids[4]]

    kp = robot.model.actuator_gainprm[aid, 0]
    kv_raw = robot.model.actuator_biasprm[aid, 1]

    print(f"\n当前状态:")
    print(f"  ctrl (目标) = {np.degrees(ctrl):.1f}°")
    print(f"  qpos (当前) = {np.degrees(qpos):.1f}°")
    print(f"  qvel (速度) = {np.degrees(qvel):.1f}°/s")
    print(f"  误差 = {np.degrees(ctrl - qpos):.1f}°")

    # MuJoCo position actuator 力矩公式:
    # force = gainprm[0] * (ctrl - qpos) + biasprm[1] * qvel
    force_pos = kp * (ctrl - qpos)
    force_vel = kv_raw * qvel
    force_total = force_pos + force_vel

    print(f"\n力矩分解:")
    print(f"  位置力矩 = kp * (ctrl - qpos) = {kp:.1f} * {np.degrees(ctrl-qpos):.1f}° = {force_pos:.4f} Nm")
    print(f"  速度力矩 = kv * qvel = {kv_raw:.1f} * {np.degrees(qvel):.1f}°/s = {force_vel:.4f} Nm")
    print(f"  总力矩 = {force_total:.4f} Nm")
    print(f"  力矩限制 = [{robot.model.actuator_forcerange[aid][0]:.1f}, {robot.model.actuator_forcerange[aid][1]:.1f}] Nm")

    # 执行一步并查看实际输出
    print("\n执行1步物理仿真...")
    mujoco.mj_step(robot.model, robot.data)

    # 查看实际施加的力矩
    actual_force = robot.data.qfrc_actuator[robot._joint_ids[4]]
    new_qpos = robot.data.qpos[robot._joint_ids[4]]
    new_qvel = robot.data.qvel[robot._joint_ids[4]]

    print(f"\n步进后状态:")
    print(f"  实际施加力矩 = {actual_force:.6f} Nm")
    print(f"  新位置 = {np.degrees(new_qpos):.2f}°")
    print(f"  新速度 = {np.degrees(new_qvel):.2f}°/s")

    # 连续多步观察
    print("\n连续20步观察...")
    forces = []
    positions = []
    for i in range(20):
        mujoco.mj_step(robot.model, robot.data)
        f = robot.data.qfrc_actuator[robot._joint_ids[4]]
        p = np.degrees(robot.data.qpos[robot._joint_ids[4]])
        forces.append(f)
        positions.append(p)

    print(f"\n步骤 | 力矩(Nm) | 位置(度)")
    print("-"*40)
    for i in range(0, 20, 5):
        print(f"  {i:3d} | {forces[i]:8.4f} | {positions[i]:7.2f}")

    avg_force = np.mean(forces)
    print(f"\n平均力矩: {avg_force:.4f} Nm")
    print(f"结论: {'✅ 力矩足够' if abs(avg_force) > 0.01 else '❌ 力矩几乎为零!'}")


if __name__ == "__main__":
    main()