#!/usr/bin/env python3
"""
检查所有6个关节的实际配置
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
    print("  所有关节配置详情")
    print("="*80)

    type_names = {0: 'FREE', 1: 'SLIDE', 2: 'HINGE', 3: 'BALL'}

    print(f"\n{'索引':<4} {'名称':<25} {'类型':<7} {'轴':<15} {'范围(°)':<20} {'qpos地址':<8} {'qvel地址':<8}")
    print("-"*80)

    for i, name in enumerate(robot.JOINT_NAMES):
        jid = robot._joint_ids[i]
        jtype = robot.model.jnt_type[jid]
        axis = robot.model.jnt_axis[jid]
        range_min = np.degrees(robot.model.jnt_range[jid, 0])
        range_max = np.degrees(robot.model.jnt_range[jid, 1])
        qpos_addr = robot.model.jnt_qposadr[jid]
        qvel_addr = robot.model.jnt_dofadr[jid]

        print(f"{i:<4} {name:<25} {type_names[jtype]:<7} [{axis[0]:.0f}, {axis[1]:.0f}, {axis[2]:.0f}]     "
              f"[{range_min:7.1f}, {range_max:7.1f}]   {qpos_addr:<8} {qvel_addr:<8}")

    # 检查总自由度数
    print(f"\n📊 模型统计:")
    print(f"   总关节数 (njnt): {robot.model.njnt}")
    print(f"   总自由度数 (nv): {robot.model.nv}")
    print(f"   总位置数 (nq): {robot.model.nq}")

    # 如果nv > 6，说明有多余的自由度
    if robot.model.nv > 6:
        print(f"\n⚠️  警告: 自由度数({robot.model.nv}) > 关节数(6)!")
        print("   这意味着某些关节可能有多个自由度（如ball joint）")
        print("\n   所有非预期关节:")
        for jid in range(robot.model.njnt):
            jname = mujoco.mj_id2name(robot.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname not in robot.JOINT_NAMES:
                jtype = robot.model.jnt_type[jid]
                print(f"     - {jname} (type={type_names[jtype]})")


if __name__ == "__main__":
    main()