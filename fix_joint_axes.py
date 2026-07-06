#!/usr/bin/env python3
"""
修复Mujoco关节轴定义以匹配FreeCAD

主要修改:
1. left/right_shoulder_joint: axis X(1,0,0) → Y(0,1,0) 匹配FreeCAD的Pitch
2. left/right_arm_roll_joint: axis Z(0,0,1) → X(1,0,0) 匹配FreeCAD的Roll
"""

import shutil
import os

def fix_joint_axes():
    print("="*70)
    print("修复Mujoco关节轴定义 - 匹配FreeCAD")
    print("="*70)
    
    # 使用稳定版本作为基础
    input_path = 'assets/mjcf/electronbot_mesh_stable.xml'
    output_path = 'assets/mjcf/electronbot_freecad_compat.xml'
    
    # 如果稳定版本不存在，使用原始版本
    if not os.path.exists(input_path):
        input_path = 'assets/mjcf/electronbot_mesh.xml'
        print(f"⚠ 稳定版本不存在，使用原始文件")
    
    with open(input_path, 'r') as f:
        content = f.read()
    
    print(f"\n读取: {input_path}")
    
    original_content = content
    
    # ===== 修改1: shoulder关节轴 X→Y =====
    print("\n[修改1] shoulder关节: axis X(1,0,0) → Y(0,1,0)")
    
    count_left_shoulder = 0
    count_right_shoulder = 0
    
    # 左肩
    if '<joint name="left_shoulder_joint" type="hinge" axis="1 0 0"' in content:
        content = content.replace(
            '<joint name="left_shoulder_joint" type="hinge" axis="1 0 0"',
            '<joint name="left_shoulder_joint" type="hinge" axis="0 1 0"'
        )
        count_left_shoulder += 1
        print("  ✓ left_shoulder_joint: (1,0,0) → (0,1,0)")
    
    # 右肩
    if '<joint name="right_shoulder_joint" type="hinge" axis="1 0 0"' in content:
        content = content.replace(
            '<joint name="right_shoulder_joint" type="hinge" axis="1 0 0"',
            '<joint name="right_shoulder_joint" type="hinge" axis="0 1 0"'
        )
        count_right_shoulder += 1
        print("  ✓ right_shoulder_joint: (1,0,0) → (0,1,0)")
    
    # ===== 修改2: arm_roll关节轴 Z→X =====
    print("\n[修改2] arm_roll关节: axis Z(0,0,1) → X(1,0,0)")
    
    count_left_arm = 0
    count_right_arm = 0
    
    # 左臂roll
    if '<joint name="left_arm_roll_joint" type="hinge" axis="0 0 1"' in content:
        content = content.replace(
            '<joint name="left_arm_roll_joint" type="hinge" axis="0 0 1"',
            '<joint name="left_arm_roll_joint" type="hinge" axis="1 0 0"'
        )
        count_left_arm += 1
        print("  ✓ left_arm_roll_joint: (0,0,1) → (1,0,0)")
    
    # 右臂roll
    if '<joint name="right_arm_roll_joint" type="hinge" axis="0 0 1"' in content:
        content = content.replace(
            '<joint name="right_arm_roll_joint" type="hinge" axis="0 0 1"',
            '<joint name="right_arm_roll_joint" type="hinge" axis="1 0 0"'
        )
        count_right_arm += 1
        print("  ✓ right_arm_roll_joint: (0,0,1) → (1,0,0)")
    
    total_changes = count_left_shoulder + count_right_shoulder + count_left_arm + count_right_arm
    
    if total_changes == 0:
        print("\n⚠️ 未找到需要修改的内容！可能已经修改过或格式不同")
        return False
    
    # 保存修改后的文件
    print(f"\n保存到: {output_path}")
    with open(output_path, 'w') as f:
        f.write(content)
    print("✓ 文件已保存")
    
    # 验证修改
    print("\n" + "="*70)
    print("验证修改结果...")
    print("="*70)
    
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(output_path)
        
        print("\n✓ 修改后的模型加载成功!")
        
        # 打印关节信息
        print("\n关节轴验证:")
        joint_names_to_check = [
            'left_shoulder_joint',
            'right_shoulder_joint',
            'left_arm_roll_joint',
            'right_arm_roll_joint'
        ]
        
        for jname in joint_names_to_check:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                axis = model.jnt_axis[jid]
                print(f"  {jname}: axis = ({axis[0]}, {axis[1]}, {axis[2]})")
                
                # 预期值检查
                if 'shoulder' in jname:
                    expected = (0, 1, 0)  # Y轴
                else:  # arm_roll
                    expected = (1, 0, 0)  # X轴
                    
                if tuple(axis) == expected:
                    print(f"    ✓ 正确! 匹配预期 {expected}")
                else:
                    print(f"    ❌ 错误! 期望 {expected}, 实际 {tuple(axis)}")
        
        # 简单稳定性测试
        print("\n快速稳定性测试...")
        data = mujoco.MjData(model)
        data.qpos[:] = 0
        data.ctrl[3] = 0.5  # 测试左arm
        
        stable = True
        for _ in range(2000):
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)):
                stable = False
                break
        
        if stable:
            print(f"✓ 稳定性测试通过! 左臂角度={data.qpos[3]:.4f}rad")
        else:
            print("❌ 稳定性测试失败")
            
        print("\n" + "="*70)
        print("✅✅✅ 修复完成! 请测试新模型 ✅✅✅")
        print("="*70)
        print(f"""
📋 使用新模型 (与FreeCAD兼容):

  python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_freecad_compat.xml

🎯 对比测试:

  FreeCAD设置:          对应Mujoco设置:
  ─────────────         ────────────────
  左臂 Pitch [Y]=38°  →  act_left_shoulder = 0.66
  左臂 Roll  [X]=-45° →  act_left_arm = -0.785
  
  现在两个系统的运动方向应该一致了!

⚠️ 注意:
  • 这是基于FreeCAD脚本的修改
  • 如果真实硬件使用不同的约定，请告知
  • 可以保留原文件作为备份
""")
        
        return True
        
    except Exception as e:
        print(f"\n❌ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    import numpy as np  # 用于稳定性测试
    fix_joint_axes()