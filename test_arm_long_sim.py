#!/usr/bin/env python3
"""
测试arm控制，增加仿真步数
"""

import mujoco
import numpy as np
import time

def test_arm_with_more_steps():
    # 加载模型
    xml_path = "assets/mjcf/electronbot_mesh.xml"
    print(f"加载模型: {xml_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        print("模型加载成功")
    except Exception as e:
        print(f"加载模型失败: {e}")
        return
    
    # 初始化
    data.qpos[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    
    # 测试左臂控制
    print("\n测试左臂控制 (更多步数)...")
    
    # 找到左臂执行器索引
    left_arm_index = -1
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if act_name and "left_arm" in act_name:
            left_arm_index = i
            break
    
    if left_arm_index >= 0:
        print(f"左臂执行器索引: {left_arm_index}")
        
        # 测试45度位置
        target_pos = 0.7854  # 45度
        print(f"\n设置左臂到 {np.degrees(target_pos):.1f}度 ({target_pos:.4f}弧度)")
        data.ctrl[left_arm_index] = target_pos
        
        # 运行更多仿真步
        for step in range(500):
            mujoco.mj_step(model, data)
            
            # 每50步打印一次
            if step % -1 == 0:
                current_qpos = data.qpos.copy()
                left_arm_qpos = current_qpos[3]  # left_arm_roll_joint
                error = np.degrees(target_pos - left_arm_qpos)
                print(f"  步数 {step}: 关节角度 = {np.degrees(left_arm_qpos):.2f}度, 误差 = {error:.2f}度")
            
            # 检查是否接近目标
            if step % 50 == 0:
                current_qpos = data.qpos.copy()
                left_arm_qpos = current_qpos[3]
                error = np.degrees(target_pos - left_arm_qpos)
                print(f"  步数 {step}: 关节角度 = {np.degrees(left_arm_qpos):.2f}度, 误差 = {error:.2f}度")
        
        # 最终位置
        current_qpos = data.qpos.copy()
        left_arm_qpos = current_qpos[3]
        final_error = np.degrees(target_pos - left_arm_qpos)
        print(f"  最终: 关节角度 = {np.degrees(left_arm_qpos):.2f}度, 误差 = {final_error:.2f}度")
    
    print("\n测试完成")

if __name__ == "__main__":
    test_arm_with_more_steps()