#!/usr/bin/env python3
"""
简单的arm控制测试
"""

import mujoco
import numpy as np
import time

def main():
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
    
    # 打印关节信息
    print("\n关节信息:")
    for i in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if jnt_name:
            print(f"  关节 {i}: {jnt_name}")
    
    # 启动viewer
    print("\n启动viewer...")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 0.3
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            
            print("Viewer启动成功")
            print("按Ctrl+C退出")
            
            # 初始化控制信号
            data.ctrl[:] = 0.0
            
            # 测试循环
            try:
                while viewer.is_running():
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
                        # 移动到45度
                        data.ctrl[left_arm_index] = 0.7854
                        print(f"设置左臂到45度 (0.7854弧度)")
                        
                        for _ in range(50):
                            mujoco.mj_step(model, data)
                            viewer.sync()
                            time.sleep(0.01)
                        
                        # 移动到-45度
                        data.ctrl[left_arm_index] = -0.7854
                        print(f"设置左臂到-45度 (-0.7854弧度)")
                        
                        for _ in range(50):
                            mujoco.mj_step(model, data)
                            viewer.sync()
                            time.sleep(0.01)
                        
                        # 回到0度
                        data.ctrl[left_arm_index] = 0.0
                        print(f"设置左臂到0度")
                        
                        for _ in range(50):
                            mujoco.mj_step(model, data)
                            viewer.sync()
                            time.sleep(0.01)
                    else:
                        print("未找到左臂执行器")
                    
                    time.sleep(1)
                    
            except KeyboardInterrupt:
                print("\n用户中断")
    
    except Exception as e:
        print(f"Viewer启动失败: {e}")

if __name__ == "__main__":
    main()