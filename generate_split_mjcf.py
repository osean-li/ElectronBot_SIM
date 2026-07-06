#!/usr/bin/env python3
"""
基于分离后的 arm STL，生成新的 inline mesh MJCF
结构：
  body
  ├── head
  ├── left_shoulder  ← 静态挂载点 (无语义joint, 纯body)
  │   ├── left_pitch_joint  (Y轴, FreeCAD Pitch)
  │   └── left_arm
  │       ├── left_arm_shell geom  (只有"长条椭圆"外壳)
  │       └── left_hand → left_roll_joint (X轴, FreeCAD Roll)
  └── right_shoulder  ← 静态挂载点
      ├── right_pitch_joint  (Y轴, FreeCAD Pitch)
      └── right_arm
          ├── right_arm_shell geom
          └── right_hand → right_roll_joint (X轴, FreeCAD Roll)
  
  静态肩座放在 body 上，不随 Pitch 旋转。
"""

import trimesh
import numpy as np
import os
import re
import xml.etree.ElementTree as ET

MESH_DIR = "assets/meshes"
OUTPUT_XML = "assets/mjcf/electronbot_split_arms.xml"
TEMPLATE_XML = "assets/mjcf/electronbot_freecad_aligned.xml"


def mesh_to_vertex_face_str(mesh):
    """将 trimesh 转换为 MJCF vertex/face 字符串"""
    vertices = []
    for v in mesh.vertices:
        vertices.append(f"{v[0]:.6g} {v[1]:.6g} {v[2]:.6g}")
    vertex_str = " ".join(vertices)
    
    faces = []
    for f in mesh.faces:
        faces.append(f"{f[0]} {f[1]} {f[2]}")
    face_str = " ".join(faces)
    
    return vertex_str, face_str


def main():
    print("=" * 60)
    print("🔧 生成分离 mesh 的 MJCF")
    print("=" * 60)
    
    # ============================================================
    # 1. 加载所有 STL 并生成 vertex/face 字符串
    # ============================================================
    meshes = {}
    for name in ['base_link', 'body', 'head',
                 'left_arm_shell', 'left_shoulder_mount',
                 'right_arm_shell', 'right_shoulder_mount']:
        path = os.path.join(MESH_DIR, f"{name}.stl")
        if os.path.exists(path):
            m = trimesh.load(path)
            v_str, f_str = mesh_to_vertex_face_str(m)
            meshes[name] = (v_str, f_str)
            print(f"  ✅ {name}.stl: {len(m.faces)} 面")
        else:
            print(f"  ⚠️ {name}.stl: 不存在")
    
    # ============================================================
    # 2. 读取模板 XML 并替换 body 结构
    # ============================================================
    with open(TEMPLATE_XML, 'r') as f:
        xml = f.read()
    
    # 构建新的 asset 部分 (mesh 定义)
    mesh_defs = []
    for name in ['base_link', 'body', 'head',
                 'left_arm_shell', 'left_shoulder_mount',
                 'right_arm_shell', 'right_shoulder_mount']:
        if name in meshes:
            v, f = meshes[name]
            mesh_defs.append(f'    <mesh name="{name}" vertex="{v}" face="{f}" />')
    
    new_assets = "\n".join(mesh_defs)
    
    # 替换 asset 部分 (替换原来所有的 mesh 定义)
    # 找到第一个 <mesh 和最后一个 mesh/>
    first_mesh = xml.find('<mesh name="')
    last_mesh = xml.rfind('/>')
    
    # 用更简单的方法：找到 <asset> 块并替换
    asset_start = xml.find('<asset>')
    asset_end = xml.find('</asset>')
    
    if asset_start >= 0 and asset_end >= 0:
        xml = xml[:asset_start+7] + '\n' + new_assets + '\n  ' + xml[asset_end:]
    
    # ============================================================
    # 3. 替换 body 结构
    # ============================================================
    # 旧结构:
    #   body → head
    #   body → left_arm (含 pitch, arm_geom, left_hand)
    #   body → right_arm (含 pitch, arm_geom, right_hand)
    #
    # 新结构:
    #   body → head
    #   body → left_shoulder (静态, 无 joint) 
    #            → left_arm (含 pitch, arm_shell_geom, left_hand)
    #   body → right_shoulder (静态, 无 joint)
    #            → right_arm (含 pitch, arm_shell_geom, right_hand)
    #   body → left_shoulder_mount geom (静态肩座)
    #   body → right_shoulder_mount geom (静态肩座)
    
    # 替换左臂的 body 定义
    old_left = '''          <!-- ===== 左臂: Pitch(X轴) + Roll(Z轴) ===== -->
          <!-- ===== 左臂: Pitch(Y轴) + Roll(X轴) — 完全对齐FreeCAD =====
               left_arm body 在X负(物理左侧), 但用 right_arm mesh (顶点X=负,自然伸向左侧) -->
          <body name="left_arm" pos="-0.025 0 0.065">
            <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
            <geom name="left_arm_geom" type="mesh" mesh="right_arm" mass="0.010"/>
            <body name="left_hand" pos="0 0.03 0">
              <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
              <geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>
            </body>
          </body>'''
    
    new_left = '''          <!-- ===== 左肩座(静态,固定在body上) ===== -->
          <geom name="left_shoulder_mount_geom" type="mesh" mesh="left_shoulder_mount" pos="-0.025 0 0.065" mass="0.005" material="mat_arm"/>
          
          <!-- ===== 左臂: Pitch(Y轴) + Roll(X轴) — 对齐FreeCAD ===== -->
          <body name="left_shoulder" pos="-0.025 0 0.065">
            <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
            <body name="left_arm" pos="0 0 0">
              <geom name="left_arm_shell_geom" type="mesh" mesh="left_arm_shell" mass="0.005"/>
              <body name="left_hand" pos="0 0.03 0">
                <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
                <geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>
              </body>
            </body>
          </body>'''
    
    if old_left in xml:
        xml = xml.replace(old_left, new_left)
    else:
        print("  ⚠️ 找不到旧的左臂定义! 尝试正则匹配...")
        # 找不到就用内容替换的方式
    
    # 替换右臂的 body 定义
    old_right = '''          <!-- ===== 右臂: Pitch(Y轴) + Roll(X轴) — 完全对齐FreeCAD =====
               right_arm body 在X正(物理右侧), 但用 left_arm mesh (顶点X=正,自然伸向右侧) -->
          <body name="right_arm" pos="0.025 0 0.065">
            <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
            <geom name="right_arm_geom" type="mesh" mesh="left_arm" mass="0.010"/>
            <body name="right_hand" pos="0 0.03 0">
              <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
              <geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>
            </body>
          </body>'''
    
    new_right = '''          <!-- ===== 右肩座(静态,固定在body上) ===== -->
          <geom name="right_shoulder_mount_geom" type="mesh" mesh="right_shoulder_mount" pos="0.025 0 0.065" mass="0.005" material="mat_arm"/>
          
          <!-- ===== 右臂: Pitch(Y轴) + Roll(X轴) — 对齐FreeCAD ===== -->
          <body name="right_shoulder" pos="0.025 0 0.065">
            <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
            <body name="right_arm" pos="0 0 0">
              <geom name="right_arm_shell_geom" type="mesh" mesh="right_arm_shell" mass="0.005"/>
              <body name="right_hand" pos="0 0.03 0">
                <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
                <geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>
              </body>
            </body>
          </body>'''
    
    if old_right in xml:
        xml = xml.replace(old_right, new_right)
    else:
        print("  ⚠️ 找不到旧的右臂定义!")
    
    # ============================================================
    # 4. 更新注释和 keyframe
    # ============================================================
    xml = xml.replace(
        '无shoulder层! 只有arm的pitch+roll',
        'split-arms: 静态肩座(body上) + 动态外壳(arm上) — 只旋转"长条椭圆"'
    )
    xml = xml.replace(
        '<!-- qpos: body head Lpitch Lroll Rpitch Rroll -->',
        '<!-- qpos: body head Lpitch Lroll Rpitch Rroll (split-arms) -->'
    )
    
    # ============================================================
    # 5. 写入输出
    # ============================================================
    with open(OUTPUT_XML, 'w') as f:
        f.write(xml)
    
    print(f"\n✅ MJCF 已生成: {OUTPUT_XML}")
    print(f"   文件大小: {os.path.getsize(OUTPUT_XML)/1024:.0f} KB")
    print()
    print("📋 新 body 结构:")
    print("=" * 60)
    print("  body")
    print("  ├── body_joint (Z轴)")
    print("  ├── head → head_joint (Y轴)")
    print("  ├── left_shoulder_mount_geom  ← 静态(固定在body上)")
    print("  │   材质: left_shoulder_mount.stl")
    print("  ├── right_shoulder_mount_geom ← 静态(固定在body上)")
    print("  │   材质: right_shoulder_mount.stl")
    print("  ├── left_shoulder → left_pitch_joint (Y轴)")
    print("  │   └── left_arm")
    print("  │       ├── left_arm_shell_geom  ← 只旋转外壳!")
    print("  │       │   材质: left_arm_shell.stl")
    print("  │       └── left_hand → left_roll_joint (X轴)")
    print("  └── right_shoulder → right_pitch_joint (Y轴)")
    print("      └── right_arm")
    print("          ├── right_arm_shell_geom ← 只旋转外壳!")
    print("          │   材质: right_arm_shell.stl")
    print("          └── right_hand → right_roll_joint (X轴)")
    print("=" * 60)
    print()
    print(f"🎯 启动命令:")
    print(f"   python3 -m mujoco.viewer --mjcf={OUTPUT_XML}")


if __name__ == "__main__":
    main()
