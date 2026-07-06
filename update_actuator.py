"""
重写 electronbot_full_arm.xml actuator，对齐舵机规格
"""
import re

xml_path = '/mnt/data2/projects/xiaozhi/ElectronBot_SIM/assets/mjcf/electronbot_full_arm.xml'
with open(xml_path) as f:
    xml = f.read()

# actuator 重写 - 按舵机力度
old_actuator = r'''  <actuator>
    <position name="act_body" joint="body_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="40"/>
    <position name="act_head" joint="head_joint" ctrlrange="-0.2618 0.2618" kp="500" kv="50"/>
    <position name="act_left_pitch" joint="left_pitch_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="80"/>
    <position name="act_left_roll" joint="left_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>
    <position name="act_right_pitch" joint="right_pitch_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="80"/>
    <position name="act_right_roll" joint="right_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>
  </actuator>'''

new_actuator = '''  <actuator>
    <!-- 腰部 SG90 -->
    <position name="act_body"          joint="body_joint"           ctrlrange="-1.5708 1.5708" kp="80"  kv="20"/>
    <!-- 头部 2g 舵机 -->
    <position name="act_head"          joint="head_joint"           ctrlrange="-0.2618 0.2618" kp="40"  kv="10"/>
    <!-- 左臂 Pitch 2g 舵机 -->
    <position name="act_left_pitch"    joint="left_pitch_joint"     ctrlrange="-1.5708 1.5708" kp="60"  kv="15"/>
    <!-- 左臂 Roll 2g 舵机 -->
    <position name="act_left_roll"     joint="left_roll_joint"      ctrlrange="-0.7854 0.7854" kp="30"  kv="8"/>
    <!-- 右臂 Pitch 2g 舵机 -->
    <position name="act_right_pitch"   joint="right_pitch_joint"    ctrlrange="-1.5708 1.5708" kp="60"  kv="15"/>
    <!-- 右臂 Roll 2g 舵机 -->
    <position name="act_right_roll"    joint="right_roll_joint"     ctrlrange="-0.7854 0.7854" kp="30"  kv="8"/>
  </actuator>'''

xml = xml.replace(old_actuator, new_actuator)

# 更新 default 中的 joint damping（降低以配合低 kp）
xml = xml.replace('<joint damping="8.0" armature="0.2" frictionloss="1.0"/>',
                  '<joint damping="2.0" armature="0.1" frictionloss="0.5"/>')

with open(xml_path, 'w') as f:
    f.write(xml)

print('Done: actuator kp/kv aligned with servo specs')
