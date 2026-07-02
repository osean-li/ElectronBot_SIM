#!/usr/bin/env python3
"""
终极测试: 使用PD控制器手动计算力矩，绕过Position Actuator
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
    print("  手动PD控制测试 (绕过Position Actuator)")
    print("="*70)

    # PD增益
    Kp = 200.0  # 位置增益
    Kd = 20.0   # 阻尼增益

    target_deg = np.array([0, 0, 0, 0, 120, 25])
    target_rad = np.radians(target_deg)

    print(f"\n🎯 目标角度: {target_deg}°")
    print(f"⚙️  PD增益: Kp={Kp}, Kd={Kd}")

    robot.reset()
    initial_pos = np.degrees(robot.get_joint_positions())
    print(f"📍 初始位置: {initial_pos.astype(int)}°")

    # 使用motor actuator施加力矩
    print("\n开始PD控制...")

    positions_history = []
    for step in range(1000):  # 2秒仿真时间
        # 获取当前状态
        qpos = robot.get_joint_positions()  # rad
        qvel = robot.get_joint_velocities()  # rad/s

        # PD控制律: torque = Kp * (target - current) - Kd * velocity
        error = target_rad - qpos
        torques = Kp * error - Kd * qvel

        # 限制力矩范围
        torques = np.clip(torques, -50, 50)

        # 施加力矩
        robot.send_torque_command(torques)

        # 步进物理仿真
        robot.step()

        if step % 100 == 0:
            pos_deg = np.degrees(qpos)
            err_deg = np.abs(target_deg - pos_deg)
            max_torque = np.max(np.abs(torques))
            print(f"  步进{step:4d}: pos=[{pos_deg[0]:6.1f} {pos_deg[1]:6.1f} {pos_deg[2]:6.1f} {pos_deg[3]:6.1f} {pos_deg[4]:6.1f} {pos_deg[5]:6.1f}]° | err_max={np.max(err_deg):5.1f}° | torque_max={max_torque:5.1f}Nm")
            positions_history.append(pos_deg.copy())

    final_pos = np.degrees(robot.get_joint_positions())
    final_error = np.abs(target_deg - final_pos)

    print(f"\n{'='*70}")
    print(f"  📊 最终结果:")
    print(f"     目标: {target_deg.astype(int)}°")
    print(f"     实际: {final_pos.astype(int)}°")
    print(f"     误差: {final_error.astype(int)}°")
    print(f"     最大误差关节: {robot.JOINT_NAMES[np.argmax(final_error)]} ({np.max(final_error):.1f}°)")

    if np.max(final_error) < 5:
        print(f"\n  ✅ 成功! 手动PD控制可以驱动机器人到达目标位置")
        print(f"     → 问题确认在Position Actuator的配置或使用方式")
    else:
        print(f"\n  ❌ 失败! 即使手动PD控制也无法到达目标")
        print(f"     → 问题可能在模型质量/惯性参数")


if __name__ == "__main__":
    main()