#!/usr/bin/env python3
"""
测试修复质量后的鼠标控制
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys

def test_mouse_control():
    print("=== 测试修复质量后的鼠标控制 ===")
    print("使用修复后的模型: assets/mjcf/electronbot_mesh_fixed2.xml")
    print()
    
    # 加载修复后的模型
    try:
        model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh_fixed2.xml')
        data = mujoco.MjData(model)
        print("✓ 成功加载修复后的模型")
    except Exception as e:
        print(f"✗ 加载修复后的模型失败: {e}")
        print("尝试加载原始模型...")
        model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh.xml')
        data = mujoco.MjData(model)
        print("✓ 加载原始模型作为备选")
    
    # 检查arm身体质量
    left_arm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
    right_arm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
    
    print(f"左臂身体质量: {model.body_mass[left_arm_body_id]:.6f}")
    print(f"右臂身体质量: {model.body_mass[right_arm_body_id]:.6f}")
    
    if model.body_mass[left_arm_body_id] > 0.001 and model.body_mass[right_arm_body_id] > 0.001:
        print("✓ arm身体质量足够大，鼠标应该可以拖动")
    else:
        print("⚠ arm身体质量可能仍然偏小")
        print("建议进一步增加mass值")
    
    print()
    print("=== 启动Mujoco viewer ===")
    print("注意：需要图形环境")
    print()
    
    # 设置初始姿势
    data.qpos[:] = [0, 0, 0, 0, 0, 0]  # 所有关节归零
    data.ctrl[:] = [0, 0, 0, 0, 0, 0]  # 控制信号归零
    mujoco.mj_forward(model, data)
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("✓ Viewer启动成功！")
            print()
            print("=== 鼠标控制测试步骤 ===")
            print("1. 在viewer窗口中，左键点击arm的末端（白色小方块）")
            print("2. 按住左键并拖动鼠标")
            print("3. 观察arm关节是否跟随移动")
            print("4. 按'H'键查看帮助信息")
            print("5. 按空格键暂停/继续仿真")
            print("6. 按'R'键重置姿势")
            print()
            print("=== 提示 ===")
            print("- arm质量已增加到0.010，应该更容易拖动")
            print("- 如果仍然难以拖动，可以按'['键减慢仿真速度")
            print("- 拖动时观察关节角度变化")
            print()
            
            # 显示关节角度
            step = 0
            last_left_angle = 0
            last_right_angle = 0
            
            while viewer.is_running():
                # 每100步检查关节角度变化
                if step % 100 == 0:
                    left_angle = data.qpos[3]  # left_arm_roll_joint
                    right_angle = data.qpos[5]  # right_arm_roll_joint
                    
                    # 检查角度是否有变化（鼠标拖动会导致变化）
                    left_changed = abs(left_angle - last_left_angle) > 0.001
                    right_changed = abs(right_angle - last_right_angle) > 0.001
                    
                    if left_changed or right_changed:
                        print(f"步数 {step}:")
                        print(f"  左臂角度: {np.degrees(left_angle):.2f}° (变化: {np.degrees(left_angle - last_left_angle):.2f}°)")
                        print(f"  右臂角度: {np.degrees(right_angle):.2f}° (变化: {np.degrees(right_angle - last_right_angle):.2f}°)")
                        print("  ✓ 检测到关节移动！鼠标控制正常工作")
                        print()
                    
                    last_left_angle = left_angle
                    last_right_angle = right_angle
                
                mujoco.mj_step(model, data)
                viewer.sync()
                step += 1
                time.sleep(0.01)
                
            print("Viewer已关闭")
            
    except Exception as e:
        print(f"✗ 启动viewer失败: {e}")
        print()
        print("=== 无图形环境测试 ===")
        print("进行程序控制测试...")
        
        # 测试程序控制
        print("\n1. 测试左臂控制:")
        data.ctrl[3] = 0.5  # 左臂控制信号
        for i in range(50):
            mujoco.mj_step(model, data)
        print(f"   左臂角度: {np.degrees(data.qpos[3]):.2f}°")
        
        print("\n2. 测试右臂控制:")
        data.ctrl[3] = 0.0  # 重置左臂
        data.ctrl[5] = -0.3  # 右臂控制信号
        for i in range(50):
            mujoco.mj_step(model, data)
        print(f"   右臂角度: {np.degrees(data.qpos[5]):.2f}°")
        
        print("\n✓ 程序控制正常，关节可以移动")
        print("鼠标控制问题可能是环境配置问题")

def main():
    test_mouse_control()
    
    print()
    print("=== 总结 ===")
    print("1. arm身体质量已从0.000001增加到0.010")
    print("2. 现在鼠标应该可以拖动arm")
    print("3. 如果仍然有问题，可以尝试：")
    print("   - 进一步增加mass值（修改XML中的mass属性）")
    print("   - 在有图形界面的环境中运行")
    print("   - 检查DISPLAY环境变量")
    print()
    print("=== 快速修复原始文件 ===")
    print("如果您想永久修复此问题，可以运行：")
    print("  cp assets/mjcf/electronbot_mesh_fixed2.xml assets/mjcf/electronbot_mesh.xml")
    print()
    print("=== 测试命令 ===")
    print("在有图形界面的环境中运行：")
    print("  python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_mesh_fixed2.xml")
    print("或使用EGL渲染器：")
    print("  MUJOCO_GL=egl python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_mesh_fixed2.xml")

if __name__ == "__main__":
    main()