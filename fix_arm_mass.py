#!/usr/bin/env python3
"""
修复arm身体质量太小导致鼠标控制不动的问题
"""

import mujoco
import numpy as np

def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh.xml')
    data = mujoco.MjData(model)
    
    print("=== 修复arm身体质量 ===")
    print()
    
    # 获取arm身体ID
    left_arm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
    right_arm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
    
    print(f"左臂身体ID: {left_arm_body_id}, 当前质量: {model.body_mass[left_arm_body_id]:.6f}")
    print(f"右臂身体ID: {right_arm_body_id}, 当前质量: {model.body_mass[right_arm_body_id]:.6f}")
    
    # 增加质量（从0.000001增加到0.010）
    new_mass = 0.010
    
    print(f"\n将arm身体质量增加到 {new_mass:.6f}...")
    
    # 注意：不能直接修改model.body_mass，需要重新编译模型
    # 但我们可以创建一个修改后的XML文件
    
    print("\n=== 解决方案 ===")
    print("需要修改MJCF文件，为left_arm和right_arm身体添加质量属性")
    print()
    print("当前MJCF结构:")
    print("  <body name=\"left_arm\" pos=\"0 0.03 0\">")
    print("    <joint name=\"left_arm_roll_joint\" .../>")
    print("    <geom name=\"left_hand_geom\" .../>  <!-- 质量在几何体上 -->")
    print("  </body>")
    print()
    print("问题：left_arm身体本身没有质量属性")
    print()
    print("修复方法1：为arm身体添加质量属性")
    print("  <body name=\"left_arm\" pos=\"0 0.03 0\" mass=\"0.010\">")
    print("    ...")
    print("  </body>")
    print()
    print("修复方法2：增加手部几何体的质量")
    print("  <geom name=\"left_hand_geom\" type=\"box\" size=\"0.008 0.008 0.012\" mass=\"0.010\" material=\"mat_arm\"/>")
    print()
    
    # 测试修改后的效果
    print("\n=== 测试修改效果 ===")
    print("创建临时修改的模型进行测试...")
    
    # 读取原始XML内容
    with open('assets/mjcf/electronbot_mesh.xml', 'r') as f:
        xml_content = f.read()
    
    # 修改left_arm身体，添加质量属性
    if '<body name="left_arm" pos="0 0.03 0">' in xml_content:
        xml_content = xml_content.replace(
            '<body name="left_arm" pos="0 0.03 0">',
            '<body name="left_arm" pos="0 0.03 0" mass="0.010">'
        )
        print("✓ 已修改left_arm身体添加质量属性")
    
    # 修改right_arm身体，添加质量属性
    if '<body name="right_arm" pos="0 0.03 0">' in xml_content:
        xml_content = xml_content.replace(
            '<body name="right_arm" pos="0 0.03 0">',
            '<body name="right_arm" pos="0 0.03 0" mass="0.010">'
        )
        print("✓ 已修改right_arm身体添加质量属性")
    
    # 保存修改后的XML
    temp_xml_path = 'assets/mjcf/electronbot_mesh_fixed.xml'
    with open(temp_xml_path, 'w') as f:
        f.write(xml_content)
    
    print(f"✓ 已保存修改后的XML到: {temp_xml_path}")
    
    # 测试修改后的模型
    print("\n=== 测试修改后的模型 ===")
    try:
        model_fixed = mujoco.MjModel.from_xml_path(temp_xml_path)
        data_fixed = mujoco.MjData(model_fixed)
        
        left_arm_body_id_fixed = mujoco.mj_name2id(model_fixed, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
        right_arm_body_id_fixed = mujoco.mj_name2id(model_fixed, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
        
        print(f"修改后左臂身体质量: {model_fixed.body_mass[left_arm_body_id_fixed]:.6f}")
        print(f"修改后右臂身体质量: {model_fixed.body_mass[right_arm_body_id_fixed]:.6f}")
        
        if (model_fixed.body_mass[left_arm_body_id_fixed] > 0.001 and 
            model_fixed.body_mass[right_arm_body_id_fixed] > 0.001):
            print("✓ 质量修复成功！")
            print("现在鼠标应该可以拖动arm了")
        else:
            print("✗ 质量修复失败，可能需要其他方法")
            
    except Exception as e:
        print(f"✗ 测试修改后的模型时出错: {e}")
    
    print("\n=== 使用方法 ===")
    print("1. 使用修改后的XML文件:")
    print(f"   python3 -m mujoco.viewer --mjcf={temp_xml_path}")
    print()
    print("2. 或永久修改原始文件:")
    print("   cp assets/mjcf/electronbot_mesh_fixed.xml assets/mjcf/electronbot_mesh.xml")
    print()
    print("3. 在viewer中测试鼠标控制:")
    print("   - 左键点击arm身体")
    print("   - 拖动鼠标移动arm")
    print("   - 观察关节角度变化")

if __name__ == "__main__":
    main()