#!/usr/bin/env python3
"""
从 STL 文件生成 Inline mesh 版 MJCF

读取 simulation/electronbot_description/meshes/*.stl
将顶点/面数据嵌入到 <mesh vertex="..." face="..."/> 字符串
生成 electronbot_inline.xml, 路径无关, 单文件可移植

背景: 当前 mujoco pip wheel 不支持 mesh 加载, 但支持 inline mesh 数据

用法:
  python generate_inline_mesh.py
"""

import os
import numpy as np
import trimesh

PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MESHES_DIR = os.path.join(PROJECT, "simulation/electronbot_description/meshes")
OUTPUT = os.path.join(PROJECT, "simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot_inline.xml")

# STL → geom name 映射
NAME_MAP = {
    "base_link.stl": "base_geom",
    "body.stl":      "body_geom",
    "head.stl":      "head_geom",
    "left_arm.stl":  "left_arm_geom",
    "right_arm.stl": "right_arm_geom",
}


def encode_mesh_data(stl_path):
    """STL → inline mesh vertex/face 字符串"""
    m = trimesh.load(stl_path)
    v_str = " ".join(f"{x:.6f}" for x in m.vertices.flatten())
    f_str = " ".join(str(int(x)) for x in m.faces.flatten())
    return v_str, f_str, len(m.vertices), len(m.faces)


def build_xml():
    """构建完整 inline mesh MJCF"""
    print(f"读取 meshes: {MESHES_DIR}")
    meshes_xml = ""
    for stl_name, _ in NAME_MAP.items():
        stl_path = os.path.join(MESHES_DIR, stl_name)
        if not os.path.exists(stl_path):
            print(f"  [SKIP] {stl_name} (not found)")
            continue
        v_str, f_str, nv, nf = encode_mesh_data(stl_path)
        mesh_name = stl_name.replace(".stl", "")
        meshes_xml += f'    <mesh name="{mesh_name}" vertex="{v_str}" face="{f_str}"/>\n'
        print(f"  [OK] {stl_name}: {nv} verts, {nf} faces")

    xml = """<mujoco model="electronbot">
  <compiler angle="radian" autolimits="true"/>
  <default>
    <joint damping="5" armature="0.005" frictionloss="0.1"/>
    <position kp="5" kv="1" forcerange="-0.01 0.01" ctrllimited="true"/>
    <motor ctrllimited="true"/>
    <geom contype="1" conaffinity="1" condim="3" friction="0.5 0.1 0.1"/>
  </default>
  <asset>
"""
    xml += meshes_xml
    xml += """    <material name="mat_base" rgba="0.2 0.2 0.2 1.0"/>
    <material name="mat_body" rgba="0.85 0.85 0.85 1.0"/>
    <material name="mat_head" rgba="0.3 0.3 0.3 1.0"/>
    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>
    <material name="mat_camera" rgba="0.0 0.0 0.0 1.0"/>
  </asset>
  <worldbody>
    <body name="base_link" pos="0 0 0.015">
      <geom name="base_geom" type="mesh" mesh="base_link"/>
      <site name="base_site" pos="0 0 0" size="0.005"/>
      <body name="body" pos="0 0 0.03">
        <joint name="body_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
        <geom name="body_geom" type="mesh" mesh="body"/>
        <site name="body_site" pos="0 0 0.07" size="0.005"/>
        <body name="head" pos="0 0 0.07">
          <joint name="head_joint" type="hinge" axis="1 0 0" range="-0.2618 0.2618" limited="true"/>
          <geom name="head_geom" type="mesh" mesh="head"/>
          <site name="head_site" pos="0 0 0.04" size="0.005"/>
          <site name="imu_site" pos="0 0 0.02" size="0.003"/>
          <body name="camera_body" pos="0 0.0175 0.025">
            <geom name="camera_geom" type="box" size="0.0125 0.0075 0.005" material="mat_camera"/>
            <site name="camera_site" pos="0 0 0.005" size="0.003"/>
            <camera name="d435_camera" pos="0 0 0.005" zaxis="0 0 1" resolution="240 240" fovy="60"/>
          </body>
        </body>
        <body name="left_shoulder" pos="0.025 0 0.065">
          <joint name="left_shoulder_joint" type="hinge" axis="1 0 0" range="-0.3491 3.1416" limited="true"/>
          <geom name="left_shoulder_geom" type="box" size="0.01 0.015 0.015" pos="0 0.01 0" material="mat_arm"/>
          <site name="left_shoulder_site" pos="0 0.03 0" size="0.005"/>
          <body name="left_arm" pos="0 0.03 0">
            <joint name="left_arm_roll_joint" type="hinge" axis="0 0 1" range="0 0.5236" limited="true"/>
            <geom name="left_arm_geom" type="mesh" mesh="left_arm"/>
            <site name="left_ee_site" pos="0 -0.055 0" size="0.008"/>
          </body>
        </body>
        <body name="right_shoulder" pos="-0.025 0 0.065">
          <joint name="right_shoulder_joint" type="hinge" axis="1 0 0" range="-0.3491 3.1416" limited="true"/>
          <geom name="right_shoulder_geom" type="box" size="0.01 0.015 0.015" pos="0 0.01 0" material="mat_arm"/>
          <site name="right_shoulder_site" pos="0 0.03 0" size="0.005"/>
          <body name="right_arm" pos="0 0.03 0">
            <joint name="right_arm_roll_joint" type="hinge" axis="0 0 1" range="0 0.5236" limited="true"/>
            <geom name="right_arm_geom" type="mesh" mesh="right_arm"/>
            <site name="right_ee_site" pos="0 -0.055 0" size="0.008"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="act_body" joint="body_joint" kp="5" kv="1" forcerange="-0.01 0.01" ctrlrange="-1.5708 1.5708"/>
    <position name="act_head" joint="head_joint" kp="3" kv="1" forcerange="-0.01 0.01" ctrlrange="-0.2618 0.2618"/>
    <position name="act_left_shoulder" joint="left_shoulder_joint" kp="5" kv="1" forcerange="-0.01 0.01" ctrlrange="-0.3491 3.1416"/>
    <position name="act_left_arm_roll" joint="left_arm_roll_joint" kp="5" kv="1" forcerange="-0.01 0.01" ctrlrange="0 0.5236"/>
    <position name="act_right_shoulder" joint="right_shoulder_joint" kp="5" kv="1" forcerange="-0.01 0.01" ctrlrange="-0.3491 3.1416"/>
    <position name="act_right_arm_roll" joint="right_arm_roll_joint" kp="5" kv="1" forcerange="-0.01 0.01" ctrlrange="0 0.5236"/>
    <motor name="motor_body" joint="body_joint" ctrlrange="-0.5 0.5"/>
    <motor name="motor_head" joint="head_joint" ctrlrange="-0.25 0.25"/>
    <motor name="motor_left_shoulder" joint="left_shoulder_joint" ctrlrange="-0.5 0.5"/>
    <motor name="motor_left_arm_roll" joint="left_arm_roll_joint" ctrlrange="-1.0 1.0"/>
    <motor name="motor_right_shoulder" joint="right_shoulder_joint" ctrlrange="-0.5 0.5"/>
    <motor name="motor_right_arm_roll" joint="right_arm_roll_joint" ctrlrange="-1.0 1.0"/>
  </actuator>
  <sensor>
    <accelerometer name="imu_accel" site="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <jointpos name="jpos_body" joint="body_joint"/>
    <jointpos name="jpos_head" joint="head_joint"/>
    <jointpos name="jpos_left_shoulder" joint="left_shoulder_joint"/>
    <jointpos name="jpos_left_arm_roll" joint="left_arm_roll_joint"/>
    <jointpos name="jpos_right_shoulder" joint="right_shoulder_joint"/>
    <jointpos name="jpos_right_arm_roll" joint="right_arm_roll_joint"/>
    <framepos name="left_ee_pos" objtype="site" objname="left_ee_site"/>
    <framepos name="right_ee_pos" objtype="site" objname="right_ee_site"/>
  </sensor>
</mujoco>
"""
    return xml


def main():
    print("=" * 60)
    print("Generate Inline Mesh MJCF")
    print("=" * 60)
    print()
    xml = build_xml()
    with open(OUTPUT, "w") as f:
        f.write(xml)
    size_mb = os.path.getsize(OUTPUT) / 1024 / 1024
    print(f"\nSaved: {OUTPUT}")
    print(f"Size:  {size_mb:.2f} MB")
    print()
    print("Usage:")
    print(f"  MUJOCO_GL=egl python -m mujoco.viewer --mjcf={OUTPUT}")


if __name__ == "__main__":
    main()
