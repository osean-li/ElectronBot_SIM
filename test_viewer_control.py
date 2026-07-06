#!/usr/bin/env python3
"""
测试Mujoco viewer中手动控制arm关节的可行性
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_full_arm.xml')
    data = mujoco.MjData(model)
    
    print("=== Mujoco Viewer 手动控制测试 ===")
    print(f"模型加载成功，执行器数量: {model.nu}")
    print(f"关节数量: {model.nq}")
    print()
    
    # 打印执行器信息
    print("执行器列表:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[i][0])
        print(f"  执行器 {i}: {actuator_name} -> 关节: {joint_name}")
    
    print()
    print("关节列表:")
    for i in range(model.nq):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            print(f"  关节 {i}: {joint_name}")
    
    print()
    print("=== 控制模式 ===")
    print("在Mujoco viewer中，您可以使用以下方式进行控制:")
    print("1. 键盘控制: 默认情况下，viewer支持键盘控制")
    print("2. 鼠标控制: 选择身体并拖动")
    print("3. 程序控制: 通过data.ctrl数组设置控制信号")
    
    print()
    print("=== 测试程序控制 ===")
    
    # 启动viewer
    print("正在启动Mujoco viewer...")
    print("注意: 如果出现DISPLAY错误，请在图形环境中运行此脚本")
    print()
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("Viewer已启动!")
            print("在viewer窗口中，您可以:")
            print("1. 按空格键暂停/继续仿真")
            print("2. 按'R'键重置仿真")
            print("3. 使用鼠标选择身体并拖动")
            print("4. 按'Ctrl+C'退出")
            print()
            
            # 设置控制信号
            print("设置初始控制信号...")
            # 设置左臂到45度 (0.7854弧度)
            data.ctrl[3] = 0.7854  # act_left_arm
            # 设置右臂到-30度 (-0.5236弧度)
            data.ctrl[5] = -0.5236  # act_right_arm
            
            print(f"左臂控制信号: {data.ctrl[3]:.4f} 弧度 ({np.degrees(data.ctrl[3]):.2f}度)")
            print(f"右臂控制信号: {data.ctrl[5]:.4f} 弧度 ({np.degrees(data.ctrl[5]):.2f}度)")
            print()
            
            # 运行仿真
            print("开始仿真...")
            print("观察viewer中的arm关节是否移动")
            print()
            
            # 运行1000步
            for i in range(1000):
                # 每100步更新一次控制信号
                if i % 200 == 0:
                    # 交替移动左右臂
                    if i % 400 == 0:
                        data.ctrl[3] = 0.7854  # 左臂45度
                        data.ctrl[5] = -0.5236  # 右臂-30度
                        print(f"步数 {i}: 左臂45度, 右臂-30度")
                    else:
                        data.ctrl[3] = -0.5236  # 左臂-30度
                        data.ctrl[5] = 0.7854   # 右臂45度
                        print(f"步数 {i}: 左臂-30度, 右臂45度")
                
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 慢速运行以便观察
                time.sleep(0.01)
                
            print()
            print("仿真完成!")
            
    except Exception as e:
        print(f"启动viewer时出错: {e}")
        print("这可能是因为没有图形显示环境")
        print("您可以尝试以下解决方案:")
        print("1. 在有图形界面的环境中运行")
        print("2. 设置DISPLAY环境变量: export DISPLAY=:0")
        print("3. 使用EGL渲染器: MUJOCO_GL=egl python test_viewer_control.py")
        print("4. 使用无头模式运行仿真")

if __name__ == "__main__":
    main()