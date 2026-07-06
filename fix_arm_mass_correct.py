#!/usr/bin/env python3
"""
正确修复arm身体质量的方法
在Mujoco中，body的质量通过其子几何体(geom)定义
"""

import mujoco
import numpy as np

def main():
    print("=== 正确修复arm身体质量 ===")
    print("在Mujoco中，body的质量由其子几何体(geom)的质量决定")
    print("要增加arm身体的质量，需要修改其几何体的质量")
    print()
    
    # 读取原始XML
    with open('assets/mjcf/electronbot_mesh.xml', 'r') as f:
        lines = f.readlines()
    
    print("分析当前arm几何体定义...")
    
    # 查找left_hand_geom和right_hand_geom
    left_hand_found = False
    right_hand_found = False
    
    for i, line in enumerate(lines):
        if 'left_hand_geom' in line:
            print(f"行 {i+1}: {line.strip()}")
            left_hand_found = True
            # 检查是否有mass属性
            if 'mass=' in line:
                print("  ✓ 已有质量属性")
            else:
                print("  ✗ 缺少质量属性")
                
        elif 'right_hand_geom' in line:
            print(f"行 {i+1}: {line.strip()}")
            right_hand_found = True
            # 检查是否有mass属性
            if 'mass=' in line:
                print("  ✓ 已有质量属性")
            else:
                print("  ✗ 缺少质量属性")
    
    print()
    print("当前left_hand_geom定义:")
    print("  <geom name=\"left_hand_geom\" type=\"box\" size=\"0.008 0.008 0.012\" material=\"mat_arm\"/>")
    print()
    print("问题：没有指定质量，使用默认值（可能太小）")
    print()
    print("=== 修复方案 ===")
    print("在left_hand_geom和right_hand_geom中添加mass属性")
    print()
    
    # 创建修复后的XML
    fixed_lines = []
    for line in lines:
        # 修改left_hand_geom
        if 'left_hand_geom' in line and 'type="box"' in line and 'mass=' not in line:
            # 在material属性前添加mass
            if 'material="mat_arm"' in line:
                line = line.replace('material="mat_arm"', 'mass="0.010" material="mat_arm"')
                print("✓ 已为left_hand_geom添加mass=\"0.010\"")
        
        # 修改right_hand_geom
        elif 'right_hand_geom' in line and 'type="box"' in line and 'mass=' not in line:
            # 在material属性前添加mass
            if 'material="mat_arm"' in line:
                line = line.replace('material="mat_arm"', 'mass="0.010" material="mat_arm"')
                print("✓ 已为right_hand_geom添加mass=\"0.010\"")
        
        fixed_lines.append(line)
    
    # 保存修复后的XML
    temp_xml_path = 'assets/mjcf/electronbot_mesh_fixed2.xml'
    with open(temp_xml_path, 'w') as f:
        f.writelines(fixed_lines)
    
    print(f"\n✓ 已保存修复后的XML到: {temp_xml_path}")
    
    # 测试修复后的模型
    print("\n=== 测试修复后的模型 ===")
    try:
        model_fixed = mujoco.MjModel.from_xml_path(temp_xml_path)
        data_fixed = mujoco.MjData(model_fixed)
        
        # 获取arm身体ID
        left_arm_body_id = mujoco.mj_name2id(model_fixed, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
        right_arm_body_id = mujoco.mj_name2id(model_fixed, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
        
        print(f"左臂身体质量: {model_fixed.body_mass[left_arm_body_id]:.6f}")
        print(f"右臂身体质量: {model_fixed.body_mass[right_arm_body_id]:.6f}")
        
        if (model_fixed.body_mass[left_arm_body_id] > 0.001 and 
            model_fixed.body_mass[right_arm_body_id] > 0.001):
            print("✓ 质量修复成功！")
            print(f"  左臂质量: {model_fixed.body_mass[left_arm_body_id]:.6f}")
            print(f"  右臂质量: {model_fixed.body_mass[right_arm_body_id]:.6f}")
        else:
            print("✗ 质量仍然太小，可能需要更大的质量值")
            print("尝试增加mass值到0.050...")
            
            # 进一步增加质量
            fixed_lines2 = []
            for line in fixed_lines:
                if 'left_hand_geom' in line and 'mass="0.010"' in line:
                    line = line.replace('mass="0.010"', 'mass="0.050"')
                elif 'right_hand_geom' in line and 'mass="0.010"' in line:
                    line = line.replace('mass="0.010"', 'mass="0.050"')
                fixed_lines2.append(line)
            
            temp_xml_path2 = 'assets/mjcf/electronbot_mesh_fixed3.xml'
            with open(temp_xml_path2, 'w') as f:
                f.writelines(fixed_lines2)
            
            # 测试更大质量
            model_fixed2 = mujoco.MjModel.from_xml_path(temp_xml_path2)
            print(f"增加质量后左臂身体质量: {model_fixed2.body_mass[left_arm_body_id]:.6f}")
            print(f"增加质量后右臂身体质量: {model_fixed2.body_mass[right_arm_body_id]:.6f}")
            
    except Exception as e:
        print(f"✗ 测试修复后的模型时出错: {e}")
        print("尝试直接修改原始文件...")
        
        # 备份原始文件
        import shutil
        shutil.copy2('assets/mjcf/electronbot_mesh.xml', 'assets/mjcf/electronbot_mesh.xml.backup')
        
        # 直接修改原始文件
        with open('assets/mjcf/electronbot_mesh.xml', 'w') as f:
            f.writelines(fixed_lines)
        
        print("✓ 已直接修改原始文件")
        print("请重新运行测试")
    
    print("\n=== 使用说明 ===")
    print("1. 使用修复后的模型:")
    print(f"   python3 -m mujoco.viewer --mjcf={temp_xml_path}")
    print()
    print("2. 在viewer中测试鼠标控制:")
    print("   - 左键点击arm末端（手部几何体）")
    print("   - 拖动鼠标移动arm")
    print("   - 应该能感觉到阻力并移动")
    print()
    print("3. 如果仍然难以移动，可以尝试:")
    print("   - 进一步增加mass值（如0.050或0.100）")
    print("   - 减小控制器增益（kp和kv）")
    print("   - 在viewer中按'['键减慢仿真速度")

if __name__ == "__main__":
    main()