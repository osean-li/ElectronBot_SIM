#!/usr/bin/env python3
"""generate_inline_mesh.py — STL → MuJoCo inline mesh 生成器。

参考 ElectronBot_zhihui 的 electronbot_inline.xml 风格:
  - STL 原始单位 (mm) 直接作为 vertex 值
  - 简化结构: base → torso → (head, left_arm, right_arm)
  - 无 visual/collision 分离
  - geom type="mesh" + mass 直接定义
  - material 定义颜色

命令行入口:
  python scripts/generate_inline_mesh.py \
      [--input assets/meshes] \
      [--output assets/mjcf/electronbot_mesh.xml]

依赖: trimesh>=4.0, numpy>=1.24
"""
from __future__ import annotations

import argparse
import logging
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("generate_inline_mesh")

# 跳过的关键词 (坐标轴、平面、原点等辅助文件)
SKIP_KEYWORDS = ['axis', '轴', 'plane', '平面', 'origin', '原点']

# STL 文件名到身体部位的映射规则 (优先级从高到低)
# 注意：box_base 不参与合并（参考文件未使用）
BODY_MAPPING_RULES = [
    # 底座 — 只用 base_link
    (r'^base_link$', 'base', 10),
    # 身体/躯干
    (r'^body$', 'torso', 20),
    # 头部
    (r'^head$', 'head', 30),
    # 左臂
    (r'left_arm', 'left_arm', 40),
    # 右臂
    (r'right_arm', 'right_arm', 50),
]


def stl_to_inline_mesh(stl_path: str, scale: float = 0.001) -> Tuple[str, str, int, int]:
    """STL → MuJoCo 空格分隔 vertex/face 字符串.

    Args:
        stl_path: STL 文件路径
        scale: 缩放因子 (默认 1.0，STL原始单位直接使用)

    Returns:
        (vertex_str, face_str, vertex_count, face_count)
    """
    import trimesh

    p = Path(stl_path)
    if not p.exists():
        raise FileNotFoundError(f"STL 文件不存在: {stl_path}")

    try:
        mesh = trimesh.load(str(p))
    except Exception as e:
        raise ValueError(f"STL 解析失败 ({stl_path}): {e}") from e

    # 处理 Scene 对象 (多几何体 STL)
    if hasattr(mesh, "geometry") and not hasattr(mesh, "vertices"):
        all_geoms = []
        for _name, geom in mesh.geometry.items():
            if hasattr(geom, "vertices") and len(geom.vertices) > 0:
                all_geoms.append(geom)
        if not all_geoms:
            raise ValueError(f"STL 无有效几何体 ({stl_path})")
        mesh = trimesh.util.concatenate(all_geoms)

    if len(mesh.vertices) == 0 or len(mesh.faces) == 0:
        raise ValueError(f"STL 为空 ({stl_path})")

    # 缩放到米单位
    vertices = mesh.vertices * scale

    # 使用紧凑格式 (6 位有效数字)
    verts_str = " ".join(f"{v:.6g}" for v in vertices.flatten())
    faces_str = " ".join(str(int(f)) for f in mesh.faces.flatten())

    logger.info("  [%s] vertices=%d faces=%d", p.stem, len(mesh.vertices), len(mesh.faces))
    return verts_str, faces_str, len(mesh.vertices), len(mesh.faces)


def sanitize_mesh_name(name: str) -> str:
    """生成合法的 MuJoCo mesh 名称."""
    safe = re.sub(r'[^a-zA-Z0-9]', '_', name).strip('_')
    if safe and safe[0].isdigit():
        safe = "m_" + safe
    return safe or "unnamed"


def classify_stl(stem: str) -> Optional[str]:
    """根据文件名判断 STL 属于哪个身体部位."""
    stem_lower = stem.lower()
    for pattern, body_part, _order in BODY_MAPPING_RULES:
        if re.search(pattern, stem_lower):
            return body_part
    return None


def merge_meshes_for_body(stl_files: List[Path], scale: float = 1.0,
                          center: bool = False) -> Tuple[str, str, np.ndarray]:
    """合并多个 STL 为一个 inline mesh (顶点合并).

    Args:
        scale: 顶点缩放因子 (0.001 = mm→m)
        center: True 时顶点居中到原点 (旋转轴 = body 原点)

    Returns:
        (vertex_str, face_str, center_offset) — offset 是居中前的质心(米)
    """
    import trimesh

    all_vertices = []
    all_faces = []
    vertex_offset = 0

    for stl_path in sorted(stl_files):
        try:
            mesh = trimesh.load(str(stl_path))

            if hasattr(mesh, "geometry") and not hasattr(mesh, "vertices"):
                geoms = [g for g in mesh.geometry.values() if hasattr(g, "vertices") and len(g.vertices) > 0]
                if geoms:
                    mesh = trimesh.util.concatenate(geoms)
                else:
                    continue

            if len(mesh.vertices) == 0:
                continue

            verts_scaled = mesh.vertices * scale
            all_vertices.append(verts_scaled)

            # 面索引加上偏移
            faces_offset = mesh.faces + vertex_offset
            all_faces.append(faces_offset)
            vertex_offset += len(mesh.vertices)

            logger.info("    + %s (%d verts)", stl_path.stem, len(mesh.vertices))

        except Exception as e:
            logger.warning("    跳过 %s: %s", stl_path.name, e)

    if not all_vertices:
        raise ValueError("无有效 mesh 数据")

    merged_vertices = np.vstack(all_vertices)
    merged_faces = np.vstack(all_faces)

    centroid = merged_vertices.mean(axis=0)
    if center:
        merged_vertices = merged_vertices - centroid
        logger.info("  居中: 减去centroid=(%.4f, %.4f, %.4f)m", *centroid)

    verts_str = " ".join(f"{v:.6g}" for v in merged_vertices.flatten())
    faces_str = " ".join(str(int(f)) for f in merged_faces.flatten())

    logger.info("  合并结果: %d vertices, %d faces, centroid=(%.4f, %.4f, %.4f)m",
                len(merged_vertices), len(merged_faces), *centroid)
    return verts_str, faces_str, centroid


def generate_mjcf(
    body_meshes: Dict[str, Tuple[str, str, np.ndarray]],
    timestamp: str,
) -> str:
    """生成 MJCF XML 内容.

    Args:
        body_meshes: {body_name: (vertex_str, face_str, centroid_offset_m)}
        timestamp: 生成时间戳
    """

    def get_mesh_xml(name: str, v: str, f: str) -> str:
        return f'    <mesh name="{name}" vertex="{v}" face="{f}" />\n'

    # 生成 asset 中的 mesh 定义 (名称匹配参考文件)
    # base→base_link, torso→body
    MESH_NAME_MAP = {"base": "base_link", "torso": "body"}
    mesh_assets = ""
    for body_name in ["base", "torso", "head", "left_arm", "right_arm"]:
        if body_name in body_meshes:
            v, f, _ = body_meshes[body_name]
            mesh_name = MESH_NAME_MAP.get(body_name, body_name)
            mesh_assets += get_mesh_xml(mesh_name, v, f)

    # 质量估计 (基于 CAD 体积计算 + 电子件附加质量, 目标总质量 ~160g)
    mass_map = {
        "base": "0.045",     # PLA 37g + PCB/电池 8g ≈ 45g
        "torso": "0.060",    # PLA 67g - 外壳实际较薄 ≈ 60g
        "head": "0.030",     # PLA 32g - 简化 ≈ 30g
        "left_arm": "0.010", # PLA ~10g
        "right_arm": "0.010",# PLA ~10g
    }

    return f'''<!-- AUTO-GENERATED by generate_inline_mesh.py
  ElectronBot MJCF — inline mesh 版
  生成时间: {timestamp}
  body/geom pos=米(m), vertex=STL原始坐标(mm→m), mass=kg, gravity=m/s², joint=弧度
-->
<mujoco model="electronbot">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic"/>
  <default>
    <joint damping="4.0" armature="0.1" frictionloss="0.5"/>
    <position kp="500" kv="50" forcerange="-100 100" ctrllimited="true"/>
    <motor ctrllimited="true"/>
    <geom contype="0" conaffinity="0" condim="3" friction="0.8 0.3 0.1" density="0.1"/>
  </default>

  <asset>
{mesh_assets}    <material name="mat_base" rgba="0.2 0.2 0.2 1.0"/>
    <material name="mat_body" rgba="0.85 0.85 0.85 1.0"/>
    <material name="mat_head" rgba="0.3 0.3 0.3 1.0"/>
    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>
  </asset>

  <worldbody>
    <light name="light1" pos="0 0 1" dir="0 0 -1" directional="true"/>
    <light name="light2" pos="0.1 0.1 0.3"/>

    <!-- ===== 底座 (固定) — 与参考zhihui项目一致 ===== -->
    <body name="base_link" pos="0 0 0.015">
{'      <geom name="base_geom" type="mesh" mesh="base_link" mass="' + mass_map.get("base", "0.1") + '"/>' if "base" in body_meshes else '      <!-- 无 base mesh -->'}

      <!-- ===== 身体 (绕 Z 轴旋转腰部, ±90°) ===== -->
      <body name="body" pos="0 0 0.03">
        <joint name="body_joint" type="hinge" axis="0 0 1" range="-1.5708 1.5708" limited="true"/>
{'        <geom name="body_geom" type="mesh" mesh="body" mass="' + mass_map.get("torso", "0.1") + '"/>' if "torso" in body_meshes else '        <!-- 无 torso mesh -->'}

        <!-- ===== 头部 (绕 Y 轴俯仰, ±15°) ===== -->
        <body name="head" pos="0 0 0.07">
          <joint name="head_joint" type="hinge" axis="0 1 0" range="-0.2618 0.2618" limited="true"/>
{'          <geom name="head_geom" type="mesh" mesh="head" mass="' + mass_map.get("head", "0.05") + '"/>' if "head" in body_meshes else '          <!-- 无 head mesh -->'}
        </body>

          <!-- ===== 左臂: Pitch(X轴) + Roll(Z轴) ===== -->
          <body name="left_shoulder" pos="0.025 0 0.065">
            <joint name="left_shoulder_joint" type="hinge" axis="1 0 0" range="-1.5708 1.5708" limited="true"/>
            <!-- arm mesh 挂在 shoulder 上: X轴Pitch旋转时上臂随之转动 -->
{'            <geom name="left_arm_geom" type="mesh" mesh="left_arm" mass="' + mass_map.get("left_arm", "0.01") + '"/>' if "left_arm" in body_meshes else '            <!-- 无 left_arm mesh -->'}
            <body name="left_arm" pos="0 0.03 0">
              <joint name="left_arm_roll_joint" type="hinge" axis="0 0 1" range="-0.7854 0.7854" limited="true"/>
              <!-- Roll 关节(Z轴): 手部/末端小几何体 -->
              <geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" material="mat_arm"/>
            </body>
          </body>

          <!-- ===== 右臂: Pitch(X轴) + Roll(Z轴) ===== -->
          <body name="right_shoulder" pos="-0.025 0 0.065">
            <joint name="right_shoulder_joint" type="hinge" axis="1 0 0" range="-1.5708 1.5708" limited="true"/>
            <!-- arm mesh 挂在 shoulder 上: X轴Pitch旋转时上臂随之转动 -->
{'            <geom name="right_arm_geom" type="mesh" mesh="right_arm" mass="' + mass_map.get("right_arm", "0.01") + '"/>' if "right_arm" in body_meshes else '            <!-- 无 right_arm mesh -->'}
            <body name="right_arm" pos="0 0.03 0">
              <joint name="right_arm_roll_joint" type="hinge" axis="0 0 1" range="-0.7854 0.7854" limited="true"/>
              <!-- Roll 关节(Z轴): 手部/末端小几何体 -->
              <geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" material="mat_arm"/>
            </body>
          </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- 按 DOF 顺序排列, 对齐参考项目 zhihui electronbot.xml:
         body(Z轴±90°), head(Y轴±15°), L/R_shoulder(X轴Pitch±90°), L/R_arm(Z轴Roll±45°)
         shoulder与head同级(都是body的子节点) -->
    <position name="act_body" joint="body_joint" ctrlrange="-1.5708 1.5708" kp="300" kv="40"/>
    <position name="act_head" joint="head_joint" ctrlrange="-0.2618 0.2618" kp="500" kv="50"/>
    <position name="act_left_shoulder" joint="left_shoulder_joint" ctrlrange="-1.5708 1.5708" kp="500" kv="50"/>
    <position name="act_left_arm" joint="left_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="500" kv="50"/>
    <position name="act_right_shoulder" joint="right_shoulder_joint" ctrlrange="-1.5708 1.5708" kp="500" kv="50"/>
    <position name="act_right_arm" joint="right_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="500" kv="50"/>
  </actuator>

  <sensor>
    <jointpos name="jpos_body" joint="body_joint"/>
    <jointpos name="jpos_head" joint="head_joint"/>
    <jointpos name="jpos_left_shoulder" joint="left_shoulder_joint"/>
    <jointpos name="jpos_left_arm" joint="left_arm_roll_joint"/>
    <jointpos name="jpos_right_shoulder" joint="right_shoulder_joint"/>
    <jointpos name="jpos_right_arm" joint="right_arm_roll_joint"/>
  </sensor>

  <keyframe>
    <key name="home" qpos="0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
'''


SCENE_MESH_TEMPLATE = '''<!--
  ElectronBot 场景 — include mesh 版
  生成时间: {timestamp}
-->
<mujoco model="electronbot_scene_mesh">
  <include file="electronbot_mesh.xml" />
  <worldbody>
    <light name="top" pos="0 0 1" dir="0 0 -1" directional="true"/>
    <camera name="view" pos="0.3 0 0.2" xyaxes="-1 0 0 0 0 1" fovy="60"/>
  </worldbody>
</mujoco>
'''


def main():
    parser = argparse.ArgumentParser(description="STL → MuJoCo inline mesh XML")
    parser.add_argument("--input", default="assets/meshes")
    parser.add_argument("--output", default="assets/mjcf/electronbot_full_arm.xml")
    parser.add_argument("--no-scene", action="store_true", help="不生成 scene 文件")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s][%(levelname)s][%(name)s] %(message)s"
    )

    mesh_dir = Path(args.input)
    if not mesh_dir.exists():
        logger.error("输入目录不存在: %s", mesh_dir)
        sys.exit(2)

    # 收集并分类 STL 文件
    stl_files = sorted(mesh_dir.glob("*.stl"))
    if not stl_files:
        logger.error("无 STL 文件")
        sys.exit(2)

    # 过滤掉辅助文件并分类
    classified: Dict[str, List[Path]] = {
        "base": [],
        "torso": [],
        "head": [],
        "left_arm": [],
        "right_arm": [],
    }
    unclassified: List[Path] = []

    for stl in stl_files:
        stem = stl.stem
        # 跳过辅助文件
        if any(kw in stem.lower() for kw in SKIP_KEYWORDS):
            logger.debug("跳过辅助文件: %s", stl.name)
            continue

        body_part = classify_stl(stem)
        if body_part and body_part in classified:
            classified[body_part].append(stl)
            logger.debug("[%s] => %s", stem, body_part)
        else:
            unclassified.append(stl)
            logger.warning("未分类: %s (将尝试自动分配)", stem)

    # 输出分类统计
    logger.info("=" * 50)
    logger.info("STL 分类结果:")
    for part, files in classified.items():
        logger.info("  %-12s: %d 个文件", part, len(files))
        for f in files:
            logger.info("              - %s (%.1f KB)", f.name, f.stat().st_size / 1024)
    if unclassified:
        logger.info("  未分类: %d 个文件", len(unclassified))
        for f in unclassified:
            logger.info("              - %s", f.name)

    # 合并每个部位的 STL 为单个 mesh
    logger.info("=" * 50)
    logger.info("正在合并 STL 并生成 inline mesh...")
    body_meshes: Dict[str, Tuple[str, str, np.ndarray]] = {}

    for body_name, files in classified.items():
        if not files:
            logger.warning("[%s] 无 STL 文件，跳过", body_name)
            continue
        try:
            logger.info("\n[%s] 合并 %d 个文件:", body_name, len(files))
            v, f, centroid = merge_meshes_for_body(files, scale=1.0, center=False)  # 保持STL原始mm单位(与参考zhihui项目一致)
            body_meshes[body_name] = (v, f, centroid)
            logger.info("  centroid: (%.4f, %.4f, %.4f)m", *centroid)
        except Exception as e:
            logger.error("[%s] 合并失败: %s", body_name, e)

    if not body_meshes:
        logger.error("无有效 mesh 数据")
        sys.exit(3)

    # 生成 MJCF
    timestamp = datetime.now().isoformat(timespec="seconds")
    xml = generate_mjcf(body_meshes, timestamp)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml, encoding="utf-8")

    file_size_kb = output.stat().st_size / 1024
    logger.info("=" * 50)
    logger.info("✅ 输出: %s (%.1f KB, %d 个 body mesh)", output, file_size_kb, len(body_meshes))

    if not args.no_scene:
        scene_path = output.parent / "scene_mesh.xml"
        scene_path.write_text(SCENE_MESH_TEMPLATE.format(timestamp=timestamp), encoding="utf-8")
        logger.info("✅ 输出: %s", scene_path)


if __name__ == "__main__":
    main()
