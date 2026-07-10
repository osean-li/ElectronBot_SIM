#!/usr/bin/env python3
"""build_electronbot_xml.py — 从 STL 生成 electronbot.xml.

STL 文件 → inline mesh XML (保持原始 STL 单位, 不缩放).
所有位置参数硬编码, 与当前工作模型 electronbot.xml 一致.

用法:
  python3 scripts/build_electronbot_xml.py
  输出: assets/mjcf/electronbot.xml
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict

import numpy as np
import trimesh

ROOT = Path(__file__).parent.parent
MESH_DIR = ROOT / "assets" / "meshes"
OUT_PATH = ROOT / "assets" / "mjcf" / "electronbot.xml"

# body 名称 → STL 文件名
MESH_FILES = {
    "base_link": "base_link",
    "body": "body",
    "head": "head",
    "left_arm": "left_arm",
    "right_arm": "right_arm",
}

# 换行符 (Linux 环境 checkout 后为 LF)
LF = "\n"


def load_stl(name: str) -> trimesh.Trimesh:
    p = MESH_DIR / f"{name}.stl"
    if not p.exists():
        raise FileNotFoundError(str(p))
    return trimesh.load(str(p))


def mesh_to_inline(name: str, m: trimesh.Trimesh) -> str:
    """返回 <mesh name="..." vertex="..." face="..." /> 单行 XML."""
    verts = m.vertices  # 不缩放, 保持 STL 原始单位
    v_str = " ".join(f"{v:.6g}" for v in verts.flatten())
    f_str = " ".join(str(int(i)) for i in m.faces.flatten())
    nm = len(verts)
    nf = len(m.faces)
    print(f"  {name:20s} {nm} verts, {nf} faces")
    return f'    <mesh name="{name}" vertex="{v_str}" face="{f_str}" />'


def main():
    print("加载 STL 文件...")
    meshes: Dict[str, trimesh.Trimesh] = {}
    for xml_name, stl_name in MESH_FILES.items():
        meshes[xml_name] = load_stl(stl_name)

    # 生成 inline mesh
    mesh_lines = [mesh_to_inline(name, m) for name, m in meshes.items()]

    # ── 构建 XML (精确匹配目标格式) ──
    lines = []
    L = lines.append

    L('<?xml version="1.0"?>')
    L('<mujoco model="electronbot">')
    L('  <compiler angle="radian" autolimits="true"/>')
    L('  <option timestep="0.002" integrator="RK4" iterations="50" cone="elliptic"/>')
    L('')
    L('  <default>')
    L('    <joint damping="4.0" armature="0.1" frictionloss="0.5"/>')
    L('    <geom contype="0" conaffinity="0" condim="3" friction="0.8 0.3 0.1" density="0.1"/>')
    L('  </default>')
    L('')
    L('  <asset>')
    L('<material name="mat_base" rgba="0.2 0.2 0.2 1.0"/>')
    L('    <material name="mat_body" rgba="0.85 0.85 0.85 1.0"/>')
    L('    <material name="mat_head" rgba="0.3 0.3 0.3 1.0"/>')
    L('    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>')
    for ml in mesh_lines:
        L(ml)
    L('  </asset>')
    L('')
    L('  <worldbody>')
    L('    <body name="base_link" pos="0 0 47.0">')
    L('      <geom name="base_geom" type="mesh" mesh="base_link" mass="0.045"/>')
    L('')
    L('      <body name="body" pos="0 0 0.03">')
    L('        <joint name="body_joint" type="hinge" axis="0 0 1" range="-1.5708 1.5708" limited="true"/>')
    L('        <geom name="body_geom" type="mesh" mesh="body" mass="0.060"/>')
    L('')
    L('        <body name="head" pos="0 0 0.07">')
    L('          <joint name="head_joint" type="hinge" axis="0 1 0" range="-0.2618 0.2618" limited="true"/>')
    L('          <geom name="head_geom" type="mesh" mesh="head" mass="0.030"/>')
    L('        </body>')
    L('')
    L('        <!-- 左臂 (纯 LEFT_ARM_PARTS, 不含身体外壳) -->')
    L('        <body name="left_arm" pos="-0.0180 0 0.065">')
    L('          <joint name="left_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>')
    L('          <joint name="left_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>')
    L('          <geom name="left_arm_geom" type="mesh" mesh="left_arm" pos="-0.0256 0 0" mass="0.005" material="mat_arm"/>')
    L('          <body name="left_hand" pos="0 0.03 0">')
    L('            <geom name="left_hand_geom" type="box" size="0.006 0.006 0.010" mass="0.003" material="mat_arm"/>')
    L('          </body>')
    L('        </body>')
    L('')
    L('        <!-- 右臂 (纯 RIGHT_ARM_PARTS, 不含身体外壳) -->')
    L('        <body name="right_arm" pos="0.0180 0 0.065">')
    L('          <joint name="right_pitch_joint" type="hinge" axis="0 1 0" range="-1.5708 1.5708" limited="true"/>')
    L('          <joint name="right_roll_joint" type="hinge" axis="1 0 0" range="-0.7854 0.7854" limited="true"/>')
    L('          <geom name="right_arm_geom" type="mesh" mesh="right_arm" pos="0.0256 0 0" mass="0.005" material="mat_arm"/>')
    L('          <body name="right_hand" pos="0 0.03 0">')
    L('            <geom name="right_hand_geom" type="box" size="0.006 0.006 0.010" mass="0.003" material="mat_arm"/>')
    L('          </body>')
    L('        </body>')
    L('      </body>')
    L('    </body>')
    L('  </worldbody>')
    L('')
    L('<actuator>')
    L('    <position name="act_body"          joint="body_joint"           ctrlrange="-1.5708 1.5708" kp="80" kv="20"/>')
    L('    <position name="act_head"          joint="head_joint"           ctrlrange="-0.2618 0.2618" kp="40" kv="10"/>')
    L('    <position name="act_left_pitch"    joint="left_pitch_joint"     ctrlrange="-1.5708 1.5708" kp="60" kv="15"/>')
    L('    <position name="act_left_roll"     joint="left_roll_joint"      ctrlrange="-0.7854 0.7854" kp="30" kv="8"/>')
    L('    <position name="act_right_pitch"   joint="right_pitch_joint"    ctrlrange="-1.5708 1.5708" kp="60" kv="15"/>')
    L('    <position name="act_right_roll"    joint="right_roll_joint"     ctrlrange="-0.7854 0.7854" kp="30" kv="8"/>')
    L('  </actuator>')
    L('')
    L('<sensor>')
    L('    <jointpos name="jpos_body" joint="body_joint"/>')
    L('    <jointpos name="jpos_head" joint="head_joint"/>')
    L('    <jointpos name="jpos_left_pitch" joint="left_pitch_joint"/>')
    L('    <jointpos name="jpos_left_roll" joint="left_roll_joint"/>')
    L('    <jointpos name="jpos_right_pitch" joint="right_pitch_joint"/>')
    L('    <jointpos name="jpos_right_roll" joint="right_roll_joint"/>')
    L('  </sensor>')
    L('')
    L('<keyframe>')
    L('    <key name="home" qpos="0 0 0 0 0 0"/>')
    L('  </keyframe>')
    L('</mujoco>')
    L('')

    xml = LF.join(lines)
    OUT_PATH.write_bytes(xml.encode("utf-8"))
    print(f"\n✅ 已生成: {OUT_PATH}")

    # 验证
    import mujoco
    try:
        model = mujoco.MjModel.from_xml_path(str(OUT_PATH))
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        print(f"   stat.center = {model.stat.center}")
        print(f"   stat.extent = {model.stat.extent:.3f}")
        print(f"   njnt = {model.njnt}, nbody = {model.nbody}")
        print(f"   body_mass = {[round(m, 4) for m in model.body_mass[:8]]}")
        print(f"   dof_damping = {[round(d, 4) for d in model.dof_damping[:6]]}")
        print(f"   ✅ 模型加载验证通过")
    except Exception as e:
        print(f"   ❌ 模型验证失败: {e}")


if __name__ == "__main__":
    main()
