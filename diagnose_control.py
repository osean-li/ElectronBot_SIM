#!/usr/bin/env python3
"""
诊断为什么有控制值但arm不动的问题
"""

import mujoco
import numpy as np

def diagnose_arm_not_moving():
    print("=== 诊断：为什么有控制值但arm不动 ===")
    print()
    
    # 加载模型（使用修复后的或原始的）
    try:
        model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh_fixed2.xml')
        data = mujoco.MjData(model)
        print("✓ 使用修复后的模型 (electronbot_mesh_fixed2.xml)")
    except:
        model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh.xml')
        data = mujoco.MjData(model)
        print("⚠ 使用原始模型 (electronbot_mesh.xml)")
    
    print(f"\n模型信息:")
    print(f"  关节数量: {model.njnt}")
    print(f"  执行器数量: {model.nu}")
    print(f"  qpos维度: {model.nq}")
    print(f"  jnt_range大小: {model.jnt_range.shape if hasattr(model.jnt_range, 'shape') else len(model.jnt_range)}")
    
    print("\n=== 1. 检查arm关节配置 ===")
    # 直接检查索引3和5的关节（left_arm和right_arm）
    arm_joint_indices = [3, 5]  # left_arm_roll_joint 和 right_arm_roll_joint
    arm_actuator_indices = [3, 5]  # act_left_arm 和 act_right_arm
    
    for joint_idx in arm_joint_indices:
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_idx)
        if not joint_name:
            continue
            
        qpos_idx = model.jnt_qposadr[joint_idx]
        
        # 安全地获取关节范围
        try:
            range_low = model.jnt_range[joint_idx * 2]
            range_high = model.jnt_range[joint_idx * 2 + 1]
            range_str = f"[{range_low:.4f}, {range_high:.4f}] 弧度 ({np.degrees(range_low):.1f}° ~ {np.degrees(range_high):.1f}°)"
        except (IndexError, TypeError):
            range_str = "无法读取范围"
        
        print(f"\n关节: {joint_name}")
        print(f"  关节索引: {joint_idx}, qpos索引: {qpos_idx}")
        print(f"  范围: {range_str}")
        print(f"  当前角度: {data.qpos[qpos_idx]:.6f} 弧度 ({np.degrees(data.qpos[qpos_idx]):.3f}°)")
    
    print("\n=== 2. 检查执行器配置 ===")
    for act_idx in arm_actuator_indices:
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_idx)
        if not act_name:
            continue
            
        # 获取控制范围
        try:
            ctrl_low = model.actuator_ctrlrange[act_idx * 2]
            ctrl_high = model.actuator_ctrlrange[act_idx * 2 + 1]
            ctrl_range_str = f"[{ctrl_low:.4f}, {ctrl_high:.4f}]"
        except (IndexError, TypeError):
            ctrl_range_str = "无法读取"
        
        # 获取增益参数
        kp = 500  # 从MJCF文件中已知
        kv = 50
        
        print(f"\n执行器: {act_name}")
        print(f"  控制范围: {ctrl_range_str}")
        print(f"  kp增益: {kp}, kv增益: {kv}")
    
    print("\n=== 3. 测试控制响应（使用截图中的控制值）===")
    
    # 重置状态
    data.qpos[:] = 0
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    
    # 设置与截图相同的控制值
    target_left = 0.181   # act_left_arm
    target_right = -0.785  # act_right_arm
    
    data.ctrl[3] = target_left
    data.ctrl[5] = target_right
    
    print(f"\n初始状态:")
    print(f"  目标左臂角度: {target_left:.4f} rad ({np.degrees(target_left):.2f}°)")
    print(f"  目标右臂角度: {target_right:.4f} rad ({np.degrees(target_right):.2f}°)")
    print(f"  实际左臂角度: {data.qpos[3]:.6f} rad ({np.degrees(data.qpos[3]):.4f}°)")
    print(f"  实际右臂角度: {data.qpos[5]:.6f} rad ({np.degrees(data.qpos[5]):.4f}°)")
    
    # 运行不同步数的仿真并记录结果
    test_steps = [10, 50, 100, 200, 500, 1000, 2000]
    
    for step_count in test_steps:
        # 重新初始化
        data.qpos[:] = 0
        data.ctrl[:] = 0
        mujoco.mj_forward(model, data)
        
        data.ctrl[3] = target_left
        data.ctrl[5] = target_right
        
        # 运行仿真
        for _ in range(step_count):
            mujoco.mj_step(model, data)
        
        left_error = abs(data.qpos[3] - target_left)
        right_error = abs(data.qpos[5] - target_right)
        
        print(f"\n步数 {step_count:4d}:")
        print(f"  左臂: {data.qpos[3]:+.6f} rad ({np.degrees(data.qpos[3]):+7.3f}°), "
              f"误差: {left_error:.6f} ({'✓ 达到目标' if left_error < 0.01 else '未达到'})")
        print(f"  右臂: {data.qpos[5]:+.6f} rad ({np.degrees(data.qpos[5]):+7.3f}°), "
              f"误差: {right_error:.6f} ({'✓ 达到目标' if right_error < 0.01 else '未达到'})")
    
    print("\n" + "="*60)
    print("=== 诊断结论 ===")
    
    # 最终判断
    final_left_error = abs(data.qpos[3] - target_left)
    final_right_error = abs(data.qpos[5] - target_right)
    
    print(f"\n最终误差 (2000步后):")
    print(f"  左臂误差: {final_left_error:.6f} rad ({np.degrees(final_left_error):.3f}°)")
    print(f"  右臂误差: {final_right_error:.6f} rad ({np.degrees(final_right_error):.3f}°)")
    
    print("\n🔍 **问题分析**:")
    
    if final_left_error < 0.01 and final_right_error < 0.01:
        print("✅ **仿真正常！关节可以响应控制信号。**")
        print("\n💡 如果在Viewer中看到arm不动，请检查以下事项：")
        print("   1. ⏸️  **仿真是否在运行？**")
        print("      • 点击Simulation区域的绿色 'Run' 按钮")
        print("      • 或按空格键切换暂停/运行")
        print("   2. 👁️  **视图是否正确？**")
        print("      • 使用鼠标滚轮放大查看arm细节")
        print("      • 尝试旋转视角查看是否有微小移动")
        print("   3. 🎮  **控制方式？**")
        print("      • Control面板设置的是目标位置，不是直接力")
        print("      • 需要等待仿真逐步移动到目标位置")
        print("   4. ⚙️  **仿真速度？**")
        print("      • 按 '[' 键减慢仿真速度以便观察")
        
    elif final_left_error < 0.5 and final_right_error < 0.5:
        print("⚠️ **关节有响应但速度较慢或有稳态误差**")
        print("\n💡 可能的原因和解决方案：")
        print("   1. 📊 **控制器增益不足**")
        print("      • 增加kp值（当前为500）到1000或更高")
        print("   2. 🔄 **阻尼过大**")
        print("      • 减小kv值（当前为50）到20或更低")
        print("   3. ⏱️ **需要更多仿真时间**")
        print("      • 等待更多步数让关节达到目标位置")
            
    else:
        print("❌ **严重问题：关节几乎没有响应！**")
        print("\n🔴 可能的严重原因：")
        print("   1. 🔒 **关节被锁定或约束**")
        print("   2. ⛔ **碰撞检测阻止移动**")
        print("   3. ❌ **模型编译错误**")
        print("   4. 🔗 **执行器-关节映射错误**")
    
    print("\n" + "="*60)
    print("=== 根据截图的建议操作步骤 ===")
    print("="*60)
    print("\n从您的截图中我看到：")
    print("  ✓ Control面板已正确显示控制值")
    print("  ✓ act_left_arm = 0.181, act_right_arm = -0.785")
    print("  ✗ 但机械臂视觉上没有明显移动")
    
    print("\n📋 **立即尝试以下步骤：**")
    print()
    print("1️⃣  **确保仿真正在运行**")
    print("    • 在Simulation区域点击绿色的 'Run' 按钮")
    print("    • 观察按钮文字应该变为 'Pause'")
    print("    • 或按键盘空格键")
    print()
    print("2️⃣  **等待几秒钟**")
    print("    • 控制是渐进式的，不是瞬间跳跃")
    print("    • arm会慢慢移动到目标位置")
    print("    • 观察arm是否有缓慢的移动趋势")
    print()
    print("3️⃣  **调整视图**")
    print("    • 使用鼠标滚轮放大arm区域")
    print("    • 旋转视角以更好观察移动")
    print("    • 特别关注左右手臂的末端")
    print()
    print("4️⃣  **测试极端控制值**")
    print("    • 将act_left_arm设置为最大值 0.785 (45°)")
    print("    • 或将act_right_arm设置为最小值 -0.785 (-45°)")
    print("    • 这样移动会更明显")

if __name__ == "__main__":
    diagnose_arm_not_moving()