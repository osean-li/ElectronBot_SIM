#!/usr/bin/env python3
"""
将 electronbot_full_arm.xml 拆分为：
  1. electronbot.xml         — 机器人模型（不含场景）
  2. electronbot_scene.xml   — 场景文件（含地面、灯光，include 机器人）
"""
import re
from pathlib import Path

MJCF_DIR = Path(__file__).resolve().parent.parent / "assets" / "mjcf"
SRC = MJCF_DIR / "electronbot_full_arm.xml"

src_text = SRC.read_text(encoding="utf-8")

# =============================================
# 1. 提取 <asset> 中的材料 + 内联 mesh
# =============================================
m_asset = re.search(
    r'(?s)<asset>\s*(.*?)</asset>',
    src_text,
)
asset_inner = m_asset.group(1).strip()

# asset 内只有 material + mesh，直接全拿
# =============================================
# 2. 提取 <worldbody> 中的 robot body 树（去掉 ground / light）
# =============================================
m_world = re.search(
    r'(?s)<worldbody>(.*?)</worldbody>',
    src_text,
)
world_inner = m_world.group(1)

# 去掉 ground / light 等场景元素，只保留 body 树
world_inner = re.sub(
    r'<geom[^>]*?name="ground"[^>]*?/>\s*',
    '',
    world_inner,
)
world_inner = re.sub(
    r'<light[^>]*?/>\s*',
    '',
    world_inner,
)
robot_body_tree = world_inner.strip()

# =============================================
# 3. 提取 actuator / sensor / keyframe
# =============================================

def extract_tag(name: str) -> str:
    m = re.search(rf'(?s)<{name}>(.*?)</{name}>', src_text)
    return m.group(0) if m else f"<{name}>\n</{name}>"


actuator_block = extract_tag("actuator")
sensor_block = extract_tag("sensor")
keyframe_block = extract_tag("keyframe")

# =============================================
# 写出 electronbot.xml
# =============================================
ROBOT_XML = f"""<?xml version="1.0"?>
<mujoco model="electronbot">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="RK4" iterations="50" cone="elliptic"/>

  <default>
    <joint damping="4.0" armature="0.1" frictionloss="0.5"/>
    <geom contype="0" conaffinity="0" condim="3" friction="0.8 0.3 0.1" density="0.1"/>
  </default>

  <asset>
{asset_inner}
  </asset>

  <worldbody>
    <body name="base_link" pos="0 0 0.015">
      <geom name="base_geom" type="mesh" mesh="base_link" mass="0.045"/>

      <body name="body" pos="0 0 0.03">
        <joint name="body_joint" type="hinge" axis="0 0 1" range="-1.5708 1.5708" limited="true"/>
        <geom name="body_geom" type="mesh" mesh="body" mass="0.060"/>

        <body name="head" pos="0 0 0.07">
          <joint name="head_joint" type="hinge" axis="0 1 0" range="-0.2618 0.2618" limited="true"/>
          <geom name="head_geom" type="mesh" mesh="head" mass="0.030"/>
        </body>

        <!-- 左臂 (纯 LEFT_ARM_PARTS, 不含身体外壳) -->
        <body name="left_arm" pos="-0.0180 0 0.065">
          <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
          <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
          <geom name="left_arm_geom" type="mesh" mesh="left_arm" pos="-0.0256 0 0" mass="0.005" material="mat_arm"/>
          <body name="left_hand" pos="0 0.03 0">
            <geom name="left_hand_geom" type="box" size="0.006 0.006 0.010" mass="0.003" material="mat_arm"/>
          </body>
        </body>

        <!-- 右臂 (纯 RIGHT_ARM_PARTS, 不含身体外壳) -->
        <body name="right_arm" pos="0.0180 0 0.065">
          <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>
          <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>
          <geom name="right_arm_geom" type="mesh" mesh="right_arm" pos="0.0256 0 0" mass="0.005" material="mat_arm"/>
          <body name="right_hand" pos="0 0.03 0">
            <geom name="right_hand_geom" type="box" size="0.006 0.006 0.010" mass="0.003" material="mat_arm"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

{actuator_block}

{sensor_block}

{keyframe_block}
</mujoco>
"""

ROBOT_PATH = MJCF_DIR / "electronbot.xml"
ROBOT_PATH.write_text(ROBOT_XML, encoding="utf-8")
print(f"✔ 已生成: {ROBOT_PATH}")

# =============================================
# 4. 生成 electronbot_scene.xml
# =============================================
SCENE_XML = """<?xml version="1.0"?>
<mujoco model="electronbot scene">
  <include file="electronbot.xml"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
             markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""

SCENE_PATH = MJCF_DIR / "electronbot_scene.xml"
SCENE_PATH.write_text(SCENE_XML, encoding="utf-8")
print(f"✔ 已生成: {SCENE_PATH}")
