#!/usr/bin/env python3
from pathlib import Path
xml_path = Path("/home/maple/数据盘/projects/xiaozhi/ElectronBot_SIM/assets/mjcf/electronbot_full_arm.xml")
with open(xml_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. 纹理定义插入 <asset> 之后
old1 = '  <asset>\n\n    <mesh name="base_link"'
new1 = '''  <asset>

    <!-- 棋盘格地面纹理 -->
    <texture name="tex_grid" type="2d" builtin="checker"
             rgb1="0.93 0.95 1" rgb2="0.25 0.4 0.6"
             width="512" height="512"/>
    <!-- 天空盒渐变背景 -->
    <texture name="tex_sky" type="skybox" builtin="gradient"
             rgb1="0.75 0.85 1" rgb2="0.35 0.55 0.75"
             width="512" height="512"/>

    <mesh name="base_link"'''
if old1 in content:
    content = content.replace(old1, new1, 1)
    print("STEP1 OK")
else:
    print("STEP1 FAILED")

# 2. 材质 mat_grid
old2 = '    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>\n'
new2 = '    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>\n    <material name="mat_grid" texture="tex_grid" texrepeat="4 4" texuniform="true"/>\n'
if old2 in content:
    content = content.replace(old2, new2, 1)
    print("STEP2 OK")
else:
    print("STEP2 FAILED")

# 3. 地面
old3 = '  <worldbody>\n    <light name="light1"'
new3 = '''  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01" pos="0 0 -0.01" material="mat_grid"/>
    <light name="light1"'''
if old3 in content:
    content = content.replace(old3, new3, 1)
    print("STEP3 OK")
else:
    print("STEP3 FAILED")

with open(xml_path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
print("ALL DONE")
