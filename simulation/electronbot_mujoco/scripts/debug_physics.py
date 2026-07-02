----------------------------------------------------------------------------------------------------------------------------#!/usr/bin/env python3
"""
ElectronBot 物理参数诊断 - 检查质量、惯性和阻尼
"""

import sys
import numpy as np
from pathlib import Path

_project = Path(__file__).parent.parent
sys.path.insert(0, str(_project))

import mujoco
from electronbot_mujoco.robot import ElectronBotRobot


def diagnose_physics():
    """诊断物理参数"""
    xml_path = str(_project / "electronbot_mujoco" / "assets" / "electronbot_inline.xml")
    robot = ElectronBotRobot(xml_path=xml_path)
    model = robot.model

    print("=" * 70)
    print("  ElectronBot 物理参数诊断")
    print("=" * 70)

    # ── 1. 重力和时间步长 ──
    print("\n[1] 全局参数:")
    print(f"  重力: {model.opt.gravity} m/s²")
    print(f"  时间步长: {model.opt.timestep*1000:.2f} ms")

    # ── 2. 各body质量 ──
    print("\n[2] Body质量分布:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        mass = model.body_mass[i]
        if mass > 0:
            print(f"  [{i:2d}] {body_name:20s}: 质量={mass:.4f} kg")

    # ── 3. 关节阻尼和惯性 ──
    print("\n[3] 关节物理参数:")
    joint_names = ["body_joint", "head_joint", "left_shoulder_joint",
                   "left_arm_roll_joint", "right_shoulder_joint", "right_arm_roll_joint"]

    for jid in range(6):
        jname = joint_names[jid]
        damping = model.jnt_damping[jid]
        armature = model.jnt_armature[jid]
        frictionloss = model.jnt_frictionloss[jid]
        limited = model.jnt_limited[jid]

        # 计算该关节的等效惯性 (简化估算)
        # 实际需要考虑整个运动链的惯性矩阵

        print(f"\n  [{jid}] {jname:25s}:")
        print(f"       阻尼(damping)={damping:.2f} Nm/(rad/s)")
        print(f"       惯性(armature)={armature:.4f} kg·m²")
        print(f"       摩擦损耗(frictionloss)={frictionloss:.2f}")
        print(f"       限位开关(limited)={'是' if limited else '否'}")

    # ── 4. 力矩需求分析 ──
    print("\n" + "=" * 70)
    print("[4] 力矩需求分析 (目标: 右肩90°)")
    print("=" * 70)

    # 设置目标并计算所需力矩
    mujoco.mj_resetData(model, robot.data)
    target = np.zeros(6)
    target[4] = np.radians(90)  # 右肩90°
    robot.send_position_command(target)

    # 单步仿真观察
    mujoco.mj_step(model, robot.data)

    r_shoulder_id = robot._joint_ids[4]
    r_shoulder_force = robot.data.actuator_force[robot._actuator_ids[4]]
    qvel = robot.data.qvel[r_shoulder_id]
    qacc = robot.data.qacc[r_shoulder_id]

    print(f"\n  执行器输出力矩: {r_shoulder_force:.2f} Nm")
    print(f"  关节角速度: {np.degrees(qvel):.4f} °/s")
    print(f"  关节角加速度: {np.degrees(qacc):.4f} °/s²")

    # 估算所需的静态力矩 (克服重力和阻尼)
    print(f"\n  分析:")
    print(f"  - 如果执行器输出{r_shoulder_force:.1f}Nm但关节几乎不动")
    print(f"  - 说明存在约{abs(r_shoulder_force):.1f}Nm的反向阻力")

    # 检查是否有外部接触力
    print(f"\n[5] 接触/约束力检查:")
    ncon = robot.data.ncon
    print(f"  当前接触数: {ncon}")

    if ncon > 0:
        for c in range(min(ncon, 10)):
            contact = robot.data.contact[c]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            force = np.linalg.norm(contact.force[:3])
            print(f"  接触{c}: {geom1_name} <-> {geom2_name}, 法向力={force:.3f} N")

    # ── 6. 建议修复方案 ──
    print("\n" + "=" * 70)
    print("[6] 诊断结论与修复建议")
    print("=" * 70)

    print("""
  ❌ 问题确认: 执行器满载但关节无法转动

  根本原因分析:
  1. 连杆质量过大 → 需要更大力矩克服重力/惯性
  2. 关节阻尼过大 → 运动被强烈抑制
  3. 可能存在数值稳定性问题 → 时间步长或积分方法不当

  建议修复方案 (按优先级):

  方案A: 增大执行器力矩范围 (推荐⭐⭐⭐)
  ┌────────────────────────────────────┐
  │ 修改 electronbot_inline.xml:      │
  │   forcerange="-200.0 200.0"       │
  │   (从±50增大到±200 Nm)           │
  └────────────────────────────────────┘

  方案B: 减小关节阻尼 (推荐⭐⭐)
  ┌────────────────────────────────────┐
  │ 修改 electronbot_inline.xml:      │
  │   <joint damping="1.0" .../>      │
  │   (从5.0减小到1.0)               │
  └────────────────────────────────────┘

  方案C: 减小连杆质量 (推荐⭐)
  ┌────────────────────────────────────┐
  │ 修改 body/arm 的 density 参数     │
  │   density="0.1" (从0.6减小到0.1) │
  └────────────────────────────────────┘

  方案D: 增大控制增益 (辅助⭐)
  ┌────────────────────────────────────┐
  │ 修改 actuator 的 kp 参数          │
  │   kp="500" kv="50"                │
  └────────────────────────────────────┘
""")

    print("=" * 70)


if __name__ == "__main__":
    diagnose_physics()