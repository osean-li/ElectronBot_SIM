#!/usr/bin/env python3
"""
测试arm控制，无需viewer
"""

import mujoco
import numpy as np
import time

def test_arm_control_no_viewer():
    # 加载模型
    xml_path = "assets/mjcf/electronbot_full_arm.xml"
    print(f"加载模型: {xml_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        print("模型加载成功")
    except Exception as e:
        print(f"加载模型失败: {e}")
        return
    
    print(f"执行器数量: {model.nu}")
    print(f"关节数量: {model.njnt}")
    
    # 打印执行器信息
    print("\n执行器信息:")
    for i in range(model.nu):
        jnt_id = model.actuator_trnid[i, 0]
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  执行器 {i}: {act_name if act_name else 'unnamed'} -> 关节 {jnt_name if jnt_name else 'unnamed'}")
    
    # 初始化
    data.qpos[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    
    # 获取初始关节位置
    initial_qpos = data.qpos.copy()
    print(f"\n初始关节位置: {np.degrees(initial_qpos[:6])}")
    
    # 测试左臂控制
    print("\n测试左臂控制...")
    
    # 找到左臂执行器索引
    left_arm_index = -1
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if act_name and "left_arm" in act_name:
            left_arm_index = i
            break
    
    if left_arm_index >= 0:
        print(f"左臂执行器索引: {left_arm_index}")
        
        # 测试不同位置
        test_positions = [-0.7854, -0.5, -0.2, 0.0, 0.2, 0.5, 0.7854]  # 弧度
        
        for pos in test_positions:
            print(f"\n设置左臂到 {np.degrees(pos):.1f}度 ({pos:.4f}弧度)")
            data.ctrl[left_arm_index] = pos
            
            # 运行仿真
            for step in range(100):
                mujoco.mj_step(model, data)
                
                # 每20步打印一次
                if step % 20 == 0:
                    current_qpos = data.qpos.copy()
                    left_arm_qpos = current_qpos[3]  # left_arm_roll_joint 是第4个关节
                    print(f"  步数 {step}: 关节角度 = {np.degrees(left_arm_qpos):.2f}度")
            
            time.sleep(0.5)
    else:
        print("错误: 未找到左臂执行器")
    
    # 测试右臂控制
    print("\n测试右臂控制...")
    
    # 找到右臂执行器索引
    right_arm_index = -1
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if act_name and "right_arm" in act_name:
            right_arm_index = i
            break
    
    if right_arm_index >= 0:
        print(f"右臂执行器索引: {right_arm_index}")
        
        # 测试不同位置
        for pos in test_positions:
            print(f"\n设置右臂到 {np.degrees(pos):.1f}度 ({pos:.4f}弧度)")
            data.ctrl[right_arm_index] = pos
            
            # 运行仿真
            for step in range(100):
                mujoco.mj_step(model, data)
                
                # 每20步打印一次
                if step % 20 == 0:
                    current_qpos = data.qpos.copy()
                    right_arm_qpos = current_qpos[5]  # right_arm_roll_joint 是第6个关节
                    print(f"  步数 {step}: 关节角度 = {np.degrees(right_arm_qpos):.2f}度")
            
            time.sleep(0.5)
    else:
        print("错误: 未找到右臂执行器")
    
    print("\n测试完成")

if __name__ == "__main__":
    test_arm_control_no_viewer()