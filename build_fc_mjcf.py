#!/usr/bin/env python3
"""用 FreeCAD 导出的正确 arm STL 生成 MJCF - 最终版
roll 关节在 arm body 上，不是 hand 上
"""
import trimesh, numpy as np, os

MESH_DIR = "assets/meshes"

# 加载所有 STL
meshes = {}
for name in ['base_link', 'body', 'head']:
    m = trimesh.load(os.path.join(MESH_DIR, f'{name}.stl'))
    meshes[name] = m

left_arm = trimesh.load(os.path.join(MESH_DIR, 'left_arm_fc.stl'))
right_arm = trimesh.load(os.path.join(MESH_DIR, 'right_arm_fc.stl'))

# 平移 arm 顶点：使根部在 X=0
left_arm.vertices[:, 0] += 1.0
right_arm.vertices[:, 0] -= 1.0

# 生成 inline mesh
def mesh_str(m):
    v = " ".join([f"{x:.6g} {y:.6g} {z:.6g}" for x,y,z in m.vertices])
    f = " ".join([f"{a} {b} {c}" for a,b,c in m.faces])
    return v, f

mesh_defs = []
for name, m in [
    ('base_link', meshes['base_link']),
    ('body', meshes['body']),
    ('head', meshes['head']),
    ('left_arm_fc', left_arm),
    ('right_arm_fc', right_arm),
]:
    v, f = mesh_str(m)
    mesh_defs.append(f'    <mesh name="{name}" vertex="{v}" face="{f}" />')

mesh_xml = "\n".join(mesh_defs)

body_bounds = meshes['body'].bounds
body_left = body_bounds[0][0]
body_right = body_bounds[1][0]
left_arm_pos_x = body_left / 1000.0 - 0.001
right_arm_pos_x = body_right / 1000.0 + 0.001

xml = f'''<!-- FreeCAD导出的arm STL - 最终版 -->
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

        <!-- 左臂 (FreeCAD LEFT_ARM_PARTS) -->
        <body name="left_arm" pos="{left_arm_pos_x:.4f} 0 0.065">
          <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
          <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
          <geom name="left_arm_geom" type="mesh" mesh="left_arm_fc" mass="0.005" material="mat_arm"/>
          <body name="left_hand" pos="0 0.03 0">
            <geom name="left_hand_geom" type="box" size="0.006 0.006 0.010" mass="0.003" material="mat_arm"/>
          </body>
        </body>

        <!-- 右臂 (FreeCAD RIGHT_ARM_PARTS) -->
        <body name="right_arm" pos="{right_arm_pos_x:.4f} 0 0.065">
          <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
          <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
          <geom name="right_arm_geom" type="mesh" mesh="right_arm_fc" mass="0.005" material="mat_arm"/>
          <body name="right_hand" pos="0 0.03 0">
            <geom name="right_hand_geom" type="box" size="0.006 0.006 0.010" mass="0.003" material="mat_arm"/>
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

out_path = "assets/mjcf/electronbot_fc_arm.xml"
with open(out_path, 'w') as f:
    f.write(xml)

print(f"✅ 已生成: {out_path}")
print(f"🎯 运行: python3 -m mujoco.viewer --mjcf={out_path}")
