#!/usr/bin/env python3
"""
诊断Mujoco viewer鼠标控制arm不动的问题
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_full_arm.xml')
    data = mujoco.MjData(model)
    
    print("=== Mujoco Viewer 鼠标控制诊断 ===")
    print()
    
    # 打印身体和关节信息
    print("身体层次结构:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name:
            # 获取父身体
            parent_id = model.body_parentid[i]
            parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id) if parent_id >= 0 else "world"
            print(f"  身体 {i}: {body_name} (父: {parent_name})")
    
    print()
    print("关节信息:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        joint_qposadr = model.jnt_qposadr[i]
        joint_dofadr = model.jnt_dofadr[i]
        if joint_name:
            type_str = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(joint_type, "unknown")
            print(f"  关节 {i}: {joint_name} (类型: {type_str}, qpos索引: {joint_qposadr}, dof索引: {joint_dofadr})")
    
    print()
    print("执行器信息:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        joint_id = model.actuator_trnid[i][0]
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        print(f"  执行器 {i}: {actuator_name} -> 关节: {joint_name}")
    
    print()
    print("=== 物理属性检查 ===")
    
    # 检查arm身体的质量属性
    left_arm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
    right_arm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
    
    if left_arm_body_id >= 0:
        print(f"左臂身体ID: {left_arm_body_id}")
        print(f"  质量: {model.body_mass[left_arm_body_id]:.6f}")
        print(f"  惯性: {model.body_inertia[left_arm_body_id * 3:(left_arm_body_id * 3) + 3]}")
    
    if right_arm_body_id >= 0:
        print(f"右臂身体ID: {right_arm_body_id}")
        print(f"  质量: {model.body_mass[right_arm_body_id]:.6f}")
        print(f"  惯性: {model.body_inertia[right_arm_body_id * 3:(right_arm_body_id * 3) + 3]}")
    
    print()
    print("=== 可能的鼠标控制问题 ===")
    print("1. 质量太小: arm身体质量可能太小，鼠标拖动时移动不明显")
    print("2. 关节限制: 关节范围限制可能阻止移动")
    print("3. 几何体位置: 几何体可能不在鼠标选择区域")
    print("4. 鼠标选择模式: 需要选择正确的身体层级")
    
    print()
    print("=== 测试鼠标控制 ===")
    print("启动viewer后，请尝试:")
    print("1. 左键点击arm的末端（手部几何体）")
    print("2. 按住左键并拖动")
    print("3. 观察joint角度是否变化")
    print("4. 按'H'键查看帮助信息")
    
    # 设置初始姿势
    print()
    print("设置初始姿势...")
    data.qpos[:] = [0, 0, 0, 0, 0, 0]  # 所有关节归零
    mujoco.mj_forward(model, data)
    
    try:
        print("正在启动Mujoco viewer...")
        print("注意: 如果出现DISPLAY错误，请在图形环境中运行")
        print()
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("✓ Viewer启动成功!")
            print()
            print("=== 诊断步骤 ===")
            print("1. 在viewer中，按'H'键查看帮助")
            print("2. 尝试选择不同的身体:")
            print("   - left_arm (手臂身体)")
            print("   - left_hand_geom (手部几何体)")
            print("   - left_shoulder (肩部身体)")
            print("3. 拖动选中的身体，观察关节角度变化")
            print("4. 按空格键暂停仿真，更容易观察")
            print()
            
            # 添加一些控制信号，使arm有初始位置
            data.ctrl[3] = 0.5  # 左臂
            data.ctrl[5] = -0.3  # 右臂
            
            step = 0
            while viewer.is_running():
                # 每200步打印一次关节角度
                if step % 200 == 0:
                    print(f"\n步数 {step}:")
                    print(f"  左臂关节角度: {data.qpos[3]:.4f} 弧度 ({np.degrees(data.qpos[3]):.2f}度)")
                    print(f"  右臂关节角度: {data.qpos[5]:.4f} 弧度 ({np.degrees(data.qpos[5]):.2f}度)")
                    print("  当前控制信号:")
                    print(f"    左臂控制: {data.ctrl[3]:.4f}")
                    print(f"    右臂控制: {data.ctrl[5]:.4f}")
                
                mujoco.mj_step(model, data)
                viewer.sync()
                step += 1
                time.sleep(0.01)
                
            print("Viewer已关闭")
            
    except Exception as e:
        print(f"✗ 启动viewer失败: {e}")
        print()
        print("=== 无图形环境的替代测试 ===")
        print("您可以通过以下方式测试控制:")
        print("1. 使用程序控制测试关节响应")
        print("2. 检查关节是否被锁定")
        print()
        
        # 无viewer测试
        print("执行无viewer测试...")
        
        # 重置姿势
        data.qpos[:] = [0, 0, 0, 0, 0, 0]
        data.ctrl[:] = [0, 0, 0, 0, 0, 0]
        mujoco.mj_forward(model, data)
        
        print("初始关节角度:")
        print(f"  左臂: {data.qpos[3]:.4f} 弧度")
        print(f"  右臂: {data.qpos[5]:.4f} 弧度")
        
        # 施加控制信号
        print("\n施加控制信号 (左臂0.5弧度, 右臂-0.3弧度)...")
        data.ctrl[3] = 0.5
        data.ctrl[5] = -0.3
        
        # 运行100步
        for i in range(100):
            mujoco.mj_step(model, data)
            if i % 20 == 0:
                print(f"步数 {i}: 左臂={data.qpos[3]:.4f}, 右臂={data.qpos[5]:.4f}")
        
        print("\n最终关节角度:")
        print(f"  左臂: {data.qpos[3]:.4f} 弧度 ({np.degrees(data.qpos[3]):.2f}度)")
        print(f"  右臂: {data.qpos[5]:.4f} 弧度 ({np.degrees(data.qpos[5]):.2f}度)")
        
        if abs(data.qpos[3] - 0.5) < 0.1 and abs(data.qpos[5] + 0.3) < 0.1:
            print("✓ 关节响应正常，控制信号有效")
            print("问题可能是鼠标选择或viewer交互问题")
        else:
            print("✗ 关节响应异常，可能关节被锁定或物理属性有问题")

if __name__ == "__main__":
    main()