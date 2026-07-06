#!/usr/bin/env python3
"""
测试Mujoco viewer键盘控制arm关节
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys

def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_full_arm.xml')
    data = mujoco.MjData(model)
    
    print("=== Mujoco Viewer 键盘控制测试 ===")
    print("执行器信息:")
    actuator_names = ["act_body", "act_head", "act_left_shoulder", "act_left_arm", "act_right_shoulder", "act_right_arm"]
    for i, name in enumerate(actuator_names):
        print(f"  {i}: {name}")
    
    print()
    print("=== 键盘控制说明 ===")
    print("在Mujoco viewer中，默认键盘控制映射:")
    print("  1. 空格键: 暂停/继续仿真")
    print("  2. 'R'键: 重置仿真")
    print("  3. '[' 和 ']'键: 减慢/加快仿真速度")
    print("  4. 'T'键: 切换透明模式")
    print("  5. 'V'键: 切换相机模式")
    print("  6. 'H'键: 显示帮助")
    print()
    print("=== 手动控制arm关节的方法 ===")
    print("Mujoco viewer本身不提供直接键盘控制关节的功能。")
    print("但您可以通过以下方式手动控制:")
    print("  1. 鼠标选择身体并拖动")
    print("  2. 编写Python代码通过data.ctrl数组控制")
    print("  3. 使用外部控制器或游戏手柄")
    print()
    
    # 尝试启动viewer
    print("正在尝试启动Mujoco viewer...")
    print("如果出现DISPLAY错误，说明当前环境无图形界面")
    print()
    
    try:
        # 设置初始姿势
        print("设置初始姿势...")
        data.ctrl[0] = 0.0  # body
        data.ctrl[1] = 0.0  # head
        data.ctrl[2] = 0.0  # left_shoulder
        data.ctrl[3] = 0.0  # left_arm
        data.ctrl[4] = 0.0  # right_shoulder
        data.ctrl[5] = 0.0  # right_arm
        
        mujoco.mj_forward(model, data)
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("✓ Viewer启动成功!")
            print()
            print("=== 在Viewer中手动控制arm的方法 ===")
            print("1. 鼠标控制:")
            print("   - 左键: 选择身体")
            print("   - 按住左键拖动: 移动身体")
            print("   - 右键: 旋转相机")
            print()
            print("2. 通过代码控制 (在此脚本中演示):")
            print("   - 程序将自动移动arm关节")
            print()
            
            # 运行仿真并自动控制arm
            print("开始自动控制演示...")
            print("左臂和右臂将交替移动")
            print()
            
            step = 0
            while viewer.is_running():
                # 每100步改变一次控制信号
                if step % 100 == 0:
                    # 计算正弦波控制信号
                    t = step / 100.0
                    
                    # 左臂: 正弦波，幅度45度
                    left_angle = 0.7854 * np.sin(t * 0.5)  # ±45度
                    data.ctrl[3] = left_angle
                    
                    # 右臂: 余弦波，相位偏移
                    right_angle = 0.7854 * np.cos(t * 0.5)  # ±45度
                    data.ctrl[5] = right_angle
                    
                    print(f"步数 {step}: 左臂={np.degrees(left_angle):.1f}°, 右臂={np.degrees(right_angle):.1f}°")
                
                # 步进仿真
                mujoco.mj_step(model, data)
                viewer.sync()
                
                step += 1
                time.sleep(0.01)  # 控制仿真速度
                
            print("Viewer已关闭")
            
    except Exception as e:
        print(f"✗ 启动viewer失败: {e}")
        print()
        print("=== 解决方案 ===")
        print("1. 在有图形界面的环境中运行此脚本")
        print("2. 设置DISPLAY环境变量:")
        print("   export DISPLAY=:0")
        print("3. 使用EGL渲染器:")
        print("   MUJOCO_GL=egl python test_keyboard_control.py")
        print("4. 使用VNC或远程桌面连接")
        print()
        print("=== 无图形环境的替代方案 ===")
        print("您可以使用以下方法在没有图形界面的环境中控制arm:")
        print("1. 使用无头模式运行仿真 (如visual_demo.py)")
        print("2. 编写Python脚本通过data.ctrl数组控制")
        print("3. 保存仿真结果并离线渲染")

if __name__ == "__main__":
    main()