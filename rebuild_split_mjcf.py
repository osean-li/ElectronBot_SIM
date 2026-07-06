#!/usr/bin/env python3
"""
重新生成 electronbot_split_arms.xml
- arm_shell 顶点平移后在 shoulder 关节处自然定位
- 使用正确的 geom pos 偏移
"""
import trimesh, numpy as np, os

MESH_DIR = "assets/meshes"
OUT_XML = "assets/mjcf/electronbot_split_arms.xml"

meshes = {}
for name, fname, shift_x in [
    ('base_link', 'base_link.stl', 0),
    ('body', 'body.stl', 0),
    ('head', 'head.stl', 0),
    # 左臂外壳: 用 right_arm_shell, 平移使 mesh 在 shoulder (X=-25) 向外
    # right_arm_shell 原始 X=[-27.8,0], 中心=-13.9
    # 目标: 在 shoulder X=-25 处，mesh 外端在 X≈-50
    # mesh 宽度=27.8mm, 外端在 -25-27.8=-52.8, 内端在 -25
    # 需要平移: 使新 mesh 顶点在 [-52.8, -25]
    # 原始 [-27.8, 0] + dx = [-52.8, -25] → dx = -25mm (两种方式得到相同结果)
    ('left_arm_shell', 'right_arm_shell.stl', -25),
    # 右臂外壳: 用 left_arm_shell, 平移使 mesh 在 shoulder (X=+25) 向外
    # left_arm_shell 原始 X=[0,27.8], 中心=13.9
    # 目标: mesh 在 X=[+25, +52.8]
    # 需要平移: dx = +25
    ('right_arm_shell', 'left_arm_shell.stl', +25),
    # 肩座同理
    ('left_shoulder_mount', 'right_shoulder_mount.stl', -25),
    ('right_shoulder_mount', 'left_shoulder_mount.stl', +25),
]:
    m = trimesh.load(os.path.join(MESH_DIR, fname))
    if shift_x != 0:
        m.vertices[:, 0] += shift_x
    v = " ".join([f"{x:.6g} {y:.6g} {z:.6g}" for x,y,z in m.vertices])
    f = " ".join([f"{a} {b} {c}" for a,b,c in m.faces])
    meshes[name] = (v, f)
    print(f"  {name:25s} X=[{m.bounds[0][0]:.1f},{m.bounds[1][0]:.1f}]mm ← {fname} shift={shift_x:+d}mm")

# 生成 MJCF
mesh_xml = "\n".join(f'    <mesh name="{n}" vertex="{v}" face="{f}" />' for n, (v, f) in meshes.items())

xml = f'''<!-- AUTO-GENERATED: arm外壳与肩座分离 -->
<mujoco model="electronbot">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="RK4" iterations="50" cone="elliptic"/>
  <default>
    <joint damping="8.0" armature="0.2" frictionloss="1.0"/>
    <position kp="300" kv="100" forcerange="-50 50" ctrllimited="true"/>
    <motor ctrllimited="true"/>
    <geom contype="0" conaffinity="0" condim="3" friction="0.8 0.3 0.1" density="0.1"/>
  </default>

  <asset>
    <material name="mat_base" rgba="0.2 0.2 0.2 1.0"/>
    <material name="mat_body" rgba="0.85 0.85 0.85 1.0"/>
    <material name="mat_head" rgba="0.3 0.3 0.3 1.0"/>
    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>
{mesh_xml}
  </asset>

  <worldbody>
    <light name="light1" pos="0 0 1" dir="0 0 -1" directional="true"/>
    <light name="light2" pos="0.1 0.1 0.3"/>

    <body name="base_link" pos="0 0 0.015">
      <geom name="base_geom" type="mesh" mesh="base_link" mass="0.045"/>

      <body name="body" pos="0 0 0.03">
        <joint name="body_joint" type="hinge" axis="0 0 1" range="-1.5708 1.5708" limited="true"/>
        <geom name="body_geom" type="mesh" mesh="body" mass="0.060"/>

        <body name="head" pos="0 0 0.07">
          <joint name="head_joint" type="hinge" axis="0 1 0" range="-0.2618 0.2618" limited="true"/>
          <geom name="head_geom" type="mesh" mesh="head" mass="0.030"/>
        </body>

        <!-- 静态肩座: 固定在 body 上, Pitch时不动 -->
        <geom name="left_shoulder_mount_geom" type="mesh" mesh="left_shoulder_mount" pos="-0.025 0 0.065" mass="0.003" material="mat_arm"/>
        <geom name="right_shoulder_mount_geom" type="mesh" mesh="right_shoulder_mount" pos="0.025 0 0.065" mass="0.003" material="mat_arm"/>

        <!-- 左臂 Pitch (Y轴) — FreeCAD对齐 -->
        <body name="left_shoulder" pos="-0.025 0 0.065">
          <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
          <body name="left_arm" pos="0 0 0">
            <geom name="left_arm_shell_geom" type="mesh" mesh="left_arm_shell" mass="0.005" material="mat_arm"/>
            <!-- 左臂 Roll (X轴) — FreeCAD对齐 -->
            <body name="left_hand" pos="0 0.03 0">
              <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
              <geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>
            </body>
          </body>
        </body>

        <!-- 右臂 Pitch (Y轴) — FreeCAD对齐 -->
        <body name="right_shoulder" pos="0.025 0 0.065">
          <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
          <body name="right_arm" pos="0 0 0">
            <geom name="right_arm_shell_geom" type="mesh" mesh="right_arm_shell" mass="0.005" material="mat_arm"/>
            <!-- 右臂 Roll (X轴) — FreeCAD对齐 -->
            <body name="right_hand" pos="0 0.03 0">
              <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
              <geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="act_body" joint="body_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="40"/>
    <position name="act_head" joint="head_joint" ctrlrange="-0.2618 0.2618" kp="500" kv="50"/>
    <position name="act_left_pitch" joint="left_pitch_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="80"/>
    <position name="act_left_roll" joint="left_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>
    <position name="act_right_pitch" joint="right_pitch_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="80"/>
    <position name="act_right_roll" joint="right_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"/>
  </actuator>

  <sensor>
    <jointpos name="jpos_body" joint="body_joint"/>
    <jointpos name="jpos_head" joint="head_joint"/>
    <jointpos name="jpos_left_pitch" joint="left_pitch_joint"/>
    <jointpos name="jpos_left_roll" joint="left_roll_joint"/>
    <jointpos name="jpos_right_pitch" joint="right_pitch_joint"/>
    <jointpos name="jpos_right_roll" joint="right_roll_joint"/>
  </sensor>

  <keyframe>
    <key name="home" qpos="0 0 0 0 0 0"/>
  </keyframe>
</mujoco>'''

with open(OUT_XML, 'w') as f:
    f.write(xml)

print(f"\n✅ 已生成: {OUT_XML}")
print(f"🎯 运行: python3 -m mujoco.viewer --mjcf={OUT_XML}")
