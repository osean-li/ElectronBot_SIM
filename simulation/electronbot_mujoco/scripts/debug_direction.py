#!/usr/bin/env python3
"""
深度诊断: 检查为什么大力矩下运动依然缓慢
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
    print("  深度诊断: 力矩 vs 运动方向")
    print("="*70)

    # 获取模型信息
    timestep = robot.model.opt.timestep
    print(f"\n⏱️  仿真时间步长: {timestep*1000:.1f} ms ({timestep} s)")
    print(f"   500步 = {500*timestep:.2f} 秒仿真时间")

    # 测试右肩关节 (joint_id=4)
    joint_id = robot._joint_ids[4]  # right_shoulder_joint
    actuator_id = robot._actuator_ids[4]  # act_right_shoulder

    print(f"\n🔧 右肩关节信息:")
    print(f"   关节名称: {robot.JOINT_NAMES[4]}")
    print(f"   关节类型: {['free', 'slide', 'hinge', 'ball'][robot.model.jnt_type[joint_id]]}")
    print(f"   关节轴: {robot.model.jnt_axis[joint_id]}")
    print(f"   关节范围: [{np.degrees(robot.model.jnt_range[joint_id][0]):.1f}°, {np.degrees(robot.model.jnt_range[joint_id][1]):.1f}°]")

    print(f"\n⚙️  执行器信息:")
    print(f"   执行器名称: {robot.ACTUATOR_NAMES[4]}")
    print(f"   执行器类型: ['motor', 'position', 'velocity'][robot.model.actuator_trnid[actuator_id, 0]]")
    print(f"   gear ratio: {robot.model.actuator_gear[actuator_id, :3]}")
    print(f"   forcerange: [{robot.model.actuator_forcerange[actuator_id][0]:.1f}, {robot.model.actuator_forcerange[actuator_id][1]:.1f}] Nm")

    # ── 测试A: 正向 ctrl → 观察运动方向 ──
    print("\n" + "─"*70)
    print("  🧪 测试A: 发送正向命令 (+120°)")
    print("─"*70)

    robot.reset()
    target = np.zeros(6)
    target[4] = np.radians(120)  # 正向120°
    robot.send_position_command(target)

    positions_a = []
    velocities_a = []
    forces_a = []

    for i in range(200):
        forces_a.append(robot.data.qfrc_actuator[joint_id])
        positions_a.append(np.degrees(robot.data.qpos[joint_id]))
        velocities_a.append(np.degrees(robot.data.qvel[joint_id]))
        robot.step()

    print(f"   初始位置: {positions_a[0]:.2f}°")
    print(f"   最终位置: {positions_a[-1]:.2f}°")
    print(f"   总位移: {positions_a[-1] - positions_a[0]:.2f}°")
    print(f"   平均速度: {np.mean(velocities_a[100:]):.2f}°/s")
    print(f"   平均力矩: {np.mean(forces_a[:50]):.2f} Nm")

    # ── 测试B: 负向 ctrl → 观察运动方向 ──
    print("\n" + "─"*70)
    print("  🧪 测试B: 发送负向命令 (-20°)")
    print("─"*70)

    robot.reset()
    target = np.zeros(6)
    target[4] = np.radians(-20)  # 负向20°（在允许范围内）
    robot.send_position_command(target)

    positions_b = []
    velocities_b = []
    forces_b = []

    for i in range(200):
        forces_b.append(robot.data.qfrc_actuator[joint_id])
        positions_b.append(np.degrees(robot.data.qpos[joint_id]))
        velocities_b.append(np.degrees(robot.data.qvel[joint_id]))
        robot.step()

    print(f"   初始位置: {positions_b[0]:.2f}°")
    print(f"   最终位置: {positions_b[-1]:.2f}°")
    print(f"   总位移: {positions_b[-1] - positions_b[0]:.2f}°")
    print(f"   平均速度: {np.mean(velocities_b[100:]):.2f}°/s")
    print(f"   平均力矩: {np.mean(forces_b[:50]):.2f} Nm")

    # ── 对比分析 ──
    print("\n" + "="*70)
    print("  📊 方向分析")
    print("="*70)

    disp_a = positions_a[-1] - positions_a[0]
    disp_b = positions_b[-1] - positions_b[0]

    print(f"\n  正向命令(+120°): 位移 = {disp_a:+.2f}°")
    print(f"  负向命令(-20°): 位移 = {disp_b:+.2f}°")

    if abs(disp_a) < 1 and abs(disp_b) < 1:
        print("\n  ❌ 两个方向都几乎不动！")
        print("     可能原因:")
        print("     1. 连杆质量/惯性极大")
        print("     2. 关节阻尼(damping)过大")
        print("     3. 存在其他约束限制运动")
        print("\n  💡 建议: 检查几何体质量属性和关节阻尼")
    elif disp_a * disp_b > 0:
        print("\n  ⚠️  两个方向运动方向相同！执行器方向可能有误")
    else:
        print(f"\n  ✅ 方向正确: 正向命令→{disp_a:+.1f}°, 负向命令→{disp_b:+.1f}°")


if __name__ == "__main__":
    main()