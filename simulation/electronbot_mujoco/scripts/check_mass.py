#!/usr/bin/env python3
"""
检查模型质量和惯性参数
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

    print("="*80)
    print("  模型质量与惯性分析")
    print("="*80)

    # 检查所有body的质量
    print(f"\n📦 各部件质量:")
    print(f"{'Body名称':<25s} {'质量(kg)':<10} {'惯性矩阵对角项':<40}")
    print("-"*75)

    total_mass = 0.0
    for bid in range(robot.model.nbody):
        bname = mujoco.mj_id2name(robot.model, mujoco.mjtObj.mjOBJ_BODY, bid)
        if bname is None:
            continue
        mass = robot.model.body_mass[bid]
        inertia = robot.body_inertia[bid] if hasattr(robot, 'body_inertia') else None
        if inertia is None:
            inertia_diag = "N/A"
        else:
            inertia_diag = f"[{inertia[0]:.4f}, {inertia[1]:.4f}, {inertia[2]:.4f}]"

        total_mass += mass
        if mass > 0.001:  # 只显示有质量的body
            print(f"  {bname:<23s} {mass:<10.3f} {str(inertia_diag):<40}")

    print(f"\n  总质量: {total_mass:.3f} kg")

    # 检查手臂连杆的具体信息
    print(f"\n💪 手臂连杆详情:")
    for body_name in ["left_arm", "right_arm", "left_shoulder", "right_shoulder"]:
        bid = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid >= 0:
            mass = robot.model.body_mass[bid]
            print(f"  {body_name}:")
            print(f"    质量: {mass:.3f} kg ({mass*1000:.1f} g)")

            # 找到该body下的geom
            for gid in range(robot.model.ngeom):
                if robot.model.geom_bodyid[gid] == bid:
                    gname = mujoco.mj_id2name(robot.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
                    if gname:
                        gmass = robot.model.geom_mass[gid]
                        gdensity = robot.model.geom_density[gid]
                        gtype = ['plane', 'sphere', 'capsule', 'ellipsoid', 'cylinder',
                                 'box', 'mesh'][robot.model.geom_type[gid]]
                        print(f"    几何体 '{gname}': type={gtype}, mass={gmass:.3f}kg, density={gdensity:.0f}kg/m³")

    # 计算需要的力矩
    print(f"\n⚙️  力矩需求估算:")
    print(f"  假设: 手臂长度=0.055m (从site pos看出), 质量=右臂质量kg")
    right_arm_id = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
    if right_arm_id >= 0:
        arm_mass = robot.model.body_mass[right_arm_id]
        arm_length = 0.055  # 米
        gravity_torque = arm_mass * 9.81 * arm_length / 2  # 重力力矩 (近似)
        print(f"    右臂质量: {arm_mass:.3f} kg")
        print(f"    手臂长度: {arm_length*100:.1f} cm")
        print(f"    克服重力所需力矩: ~{gravity_torque:.2f} Nm")
        print(f"    当前执行器上限: 50 Nm")
        print(f"    {'✅ 足够' if 50 > gravity_torque else '❌ 不够'} 克服重力")


if __name__ == "__main__":
    main()