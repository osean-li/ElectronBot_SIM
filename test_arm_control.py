#!/usr/bin/env python3
"""
测试Mujoco viewer中的arm控制
"""

import mujoco
import numpy as np
import time

def test_arm_control():
    # 加载模型
    xml_path = "assets/mjcf/electronbot_mesh.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print(f"模型加载成功: {xml_path}")
    print(f"执行器数量: {model.nu}")
    print(f"关节数量: {model.njnt}")
    print(f"控制维度: {model.nu}")
    
    # 打印执行器信息
    print("\n执行器信息:")
    for i in range(model.nu):
        jnt_id = model.actuator_trnid[i, 0]
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  执行器 {i}: {act_name} -> 关节 {jnt_name}")
    
    # 打印关节信息
    print("\n关节信息:")
    for i in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_type = model.jnt_type[i]
        type_str = "free" if jnt_type == 0 else "ball" if jnt_type == 1 else "slide" if jnt_type == 2 else "hinge" if jnt_type == 3 else "unknown"
        print(f"  关节 {i}: {jnt_name} (类型: {type_str})")
    
    # 启动viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        
        print("\n控制说明:")
        print("1. 将依次测试每个关节的控制")
        print("2. 每个关节会移动到极限位置")
        print("3. 按Ctrl+C退出")
        
        # 获取执行器索引
        actuator_names = ["act_body", "act_head", "act_left_shoulder", "act_left_arm", "act_right_shoulder", "act_right_arm"]
        actuator_indices = {}
        for i, name in enumerate(actuator_names):
            actuator_indices[name] = i

        # 获取执行器索引
        actuator_names = ["act_body", "act_head", "act_left_shoulder", "act_left_arm", "act_right_shoulder", "act_right_arm"]
        actuator_indices = {}
        for i, name in enumerate(actuator_names):
            actuator_indices[name] = i
        
        try:
            while viewer.is_running():
                # 测试左臂控制
                print("\n测试左臂控制...")
                
                # 将左臂移动到最大角度 (45度 = 0.7854弧度)
                data.ctrl[actuator_indices["act_left_arm"]] = 0.7854
                print(f"  设置左臂到 45度 (0.7854弧度)")
                
                for _ in range(100):  # 运行100步
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.001)
                
                # 将左臂移动到最小角度 (-45度 = -0.7854弧度)
                data.ctrl[actuator_indices["act_left_arm"]] = -0.7854
                print(f"  设置左臂到 -45度 (-0.7854弧度)")
                
                for _ in range(100):  # 运行100步
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.001)
                
                # 将左臂移回中间位置
                data.ctrl[actuator_indices["act_left_arm"]] = 0.0
                print(f"  设置左臂到 0度")
                
                for _ in range(100):  # 运行100步
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.001)
                
                # 测试右臂控制
                print("\n测试右臂控制...")
                
                # 将右臂移动到最大角度
                data.ctrl[actuator_indices["act_right_arm"]] = 0.7854
                print(f"  设置右臂到 45度 (0.7854弧度)")
                
                for _ in range(100):
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.001)
                
                # 将右臂移动到最小角度
                data.ctrl[actuator_indices["act_right_arm"]] = -0.7854
                print(f"  设置右臂到 -45度 (-0.7854弧度)")
                
                for _ in range(100):
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.001)
                
                # 将右臂移回中间位置
                data.ctrl[actuator_indices["act_right_arm"]] = 0.0
                print(f"  设置右臂到 0度")
                
                for _ in range(100):
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.001)
                
                print("\n测试完成，按Ctrl+C退出...")
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\n用户中断，退出...")
    
    print("测试完成")

if __name__ == "__main__":
    test_arm_control()