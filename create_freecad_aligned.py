#!/usr/bin/env python3
"""
生成 electronbot_freecad_aligned.xml
- 去掉 shoulder 层，只保留 arm 的 pitch(Y) + roll(X)
- 完全对齐 FreeCAD electronbot_joints.FCMacro
- 保持稳定参数: RK4 integrator, 50 iterations
"""

import re
import os

INPUT_FILE = "assets/mjcf/electronbot_freecad_compat.xml"
OUTPUT_FILE = "assets/mjcf/electronbot_freecad_aligned.xml"

def main():
    with open(INPUT_FILE, 'r') as f:
        content = f.read()
    
    # ============================================================
    # 1. 替换左臂 body 结构: 
    #    old: left_shoulder(joint Y-axis) → left_arm_geom + left_arm(joint X-axis) → hand
    #    new: left_arm_pitch(joint Y-axis) → left_arm_geom + left_arm_roll(joint X-axis) → hand
    # ============================================================
    
    # 匹配旧的左臂结构
    old_left_arm = r'''<body name="left_shoulder" pos="0\.025 0 0\.065">
            <joint name="left_shoulder_joint" type="hinge" axis="0 1 0" range="-1\.5708 1\.5708" limited="true"/>
            <!-- arm mesh 挂在 shoulder 上: X轴Pitch旋转时上臂随之转动 -->
            <geom name="left_arm_geom" type="mesh" mesh="left_arm" mass="0\.010"/>
            <body name="left_arm" pos="0 0\.03 0">
              <joint name="left_arm_roll_joint" type="hinge" axis="1 0 0" range="-0\.7854 0\.7854" limited="true"/>
              <geom name="left_hand_geom" type="box" size="0\.008 0\.008 0\.012" mass="0\.005" material="mat_arm"/>'''
    
    new_left_arm = '''<body name="left_arm" pos="0.025 0 0.065">
            <!-- FreeCAD对齐: Pitch绕Y轴 (±90°), 与FreeCAD LEFT_ARM_PITCH_Y 一致 -->
            <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
            <geom name="left_arm_geom" type="mesh" mesh="left_arm" mass="0.010"/>
            <!-- FreeCAD对齐: Roll绕X轴 (±45°), 与FreeCAD LEFT_ARM_ROLL_X 一致 -->
            <body name="left_hand" pos="0 0.03 0">
              <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
              <geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>'''
    
    content = re.sub(old_left_arm, new_left_arm, content)
    
    # ============================================================
    # 2. 替换右臂 body 结构 (同理)
    # ============================================================
    old_right_arm = r'''<body name="right_shoulder" pos="-0\.025 0 0\.065">
            <joint name="right_shoulder_joint" type="hinge" axis="0 1 0" range="-1\.5708 1\.5708" limited="true"/>
            <!-- arm mesh 挂在 shoulder 上 -->
            <geom name="right_arm_geom" type="mesh" mesh="right_arm" mass="0\.010"/>
            <body name="right_arm" pos="0 0\.03 0">
              <joint name="right_arm_roll_joint" type="hinge" axis="1 0 0" range="-0\.7854 0\.7854" limited="true"/>
              <geom name="right_hand_geom" type="box" size="0\.008 0\.008 0\.012" mass="0\.005" material="mat_arm"/>'''
    
    new_right_arm = '''<body name="right_arm" pos="-0.025 0 0.065">
            <!-- FreeCAD对齐: Pitch绕Y轴 (±90°) -->
            <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
            <geom name="right_arm_geom" type="mesh" mesh="right_arm" mass="0.010"/>
            <!-- FreeCAD对齐: Roll绕X轴 (±45°) -->
            <body name="right_hand" pos="0 0.03 0">
              <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
              <geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>'''
    
    content = re.sub(old_right_arm, new_right_arm, content)
    
    # ============================================================
    # 3. 更新 actuator 定义
    #    old: act_left_shoulder, act_left_arm, ...
    #    new: act_left_pitch, act_left_roll, ...  (与 FreeCAD 一致)
    # ============================================================
    old_actuators = '''<position name="act_left_shoulder" joint="left_shoulder_joint" ctrlrange="-1.5708 1.5708" kp="500" kv="50"/>
    <position name="act_left_arm" joint="left_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>
    <position name="act_right_shoulder" joint="right_shoulder_joint" ctrlrange="-1.5708 1.5708" kp="500" kv="50"/>
    <position name="act_right_arm" joint="right_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>'''
    
    new_actuators = '''<!-- FreeCAD对齐: act_left_pitch(Y轴±90°), act_left_roll(X轴±45°) -->
    <position name="act_left_pitch" joint="left_pitch_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="80"/>
    <position name="act_left_roll" joint="left_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>
    <position name="act_right_pitch" joint="right_pitch_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="80"/>
    <position name="act_right_roll" joint="right_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>'''
    
    content = content.replace(old_actuators, new_actuators)
    
    # ============================================================
    # 4. 更新 sensor 定义
    # ============================================================
    old_sensors = '''<jointpos name="jpos_left_shoulder" joint="left_shoulder_joint"/>
    <jointpos name="jpos_left_arm" joint="left_arm_roll_joint"/>
    <jointpos name="jpos_right_shoulder" joint="right_shoulder_joint"/>
    <jointpos name="jpos_right_arm" joint="right_arm_roll_joint"/>'''
    
    new_sensors = '''<jointpos name="jpos_left_pitch" joint="left_pitch_joint"/>
    <jointpos name="jpos_left_roll" joint="left_roll_joint"/>
    <jointpos name="jpos_right_pitch" joint="right_pitch_joint"/>
    <jointpos name="jpos_right_roll" joint="right_roll_joint"/>'''
    
    content = content.replace(old_sensors, new_sensors)
    
    # ============================================================
    # 5. 更新注释和 keyframe
    # ============================================================
    content = content.replace(
        '按 DOF 顺序排列, 对齐参考项目 zhihui electronbot.xml:\n         body(Z轴±90°), head(Y轴±15°), L/R_shoulder(X轴Pitch±90°), L/R_arm(Z轴Roll±45°)\n         shoulder与head同级(都是body的子节点)',
        '''完全对齐 FreeCAD electronbot_joints.FCMacro:
         body(Z轴±90°), head(Y轴±15°), L/R_pitch(Y轴±90°=FreeCAD Pitch), L/R_roll(X轴±45°=FreeCAD Roll)
         无shoulder层! 只有arm的pitch+roll'''
    )
    
    # 更新 keyframe 注释 (DOF 数量不变，还是 6 个)
    content = content.replace('<key name="home" qpos="0 0 0 0 0 0"/>',
                              '<key name="home" qpos="0 0 0 0 0 0"/>\n    <!-- qpos: body head Lpitch Lroll Rpitch Rroll -->')
    
    # 写入输出文件
    with open(OUTPUT_FILE, 'w') as f:
        f.write(content)
    
    print(f"✅ 已生成: {OUTPUT_FILE}")
    print("\n📋 新模型结构 (完全对齐FreeCAD):")
    print("=" * 55)
    print("  body")
    print("  ├── body_joint     [Z轴 ±90°]  腰部旋转")
    print("  │   └── head")
    print("  │       └── head_joint   [Y轴 ±15°]  头部俯仰")
    print("  ├── left_arm")
    print("  │   ├── left_pitch_joint [Y轴 ±90°]  ← FreeCAD left_pitch")
    print("  │   │   └── left_arm_geom (mesh)")
    print("  │   └── left_hand")
    print("  │       └── left_roll_joint [X轴 ±45°] ← FreeCAD left_roll")
    print("  │           └── left_hand_geom (box)")
    print("  └── right_arm")
    print("      ├── right_pitch_joint [Y轴 ±90°]  ← FreeCAD right_pitch")
    print("      │   └── right_arm_geom (mesh)")
    print("      └── right_hand")
    print("          └── right_roll_joint [X轴 ±45°] ← FreeCAD right_roll")
    print("              └── right_hand_geom (box)")
    print("=" * 55)
    print("\n🎮 Control 面板中的执行器名称:")
    print("  act_body, act_head,")
    print("  act_left_pitch, act_left_roll,")
    print("  act_right_pitch, act_right_roll")

if __name__ == "__main__":
    main()
