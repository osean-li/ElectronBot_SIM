#!/usr/bin/env python3
"""
step_to_mjcf.py — ElectronBot.step → MJCF XML 直接转换

流程:
  1. cascadio 把 STEP 转 GLB (保留零件边界和色彩)
  2. trimesh 读取 GLB 场景图 (scene.graph → 每个 node 有 mesh + transform)
  3. 遍历 scene graph, 生成 MJCF <body> + <geom> + <joint>
  4. 所有位置从 transforms 矩阵提取 (单位: mm → m, ×0.001)
  5. 6 个舵机关节从 robot-structure.md 手动配置 (臂肩/臂滚/腰/头)

依赖:
  pip install cascadio trimesh numpy pillow mujoco

用法:
  python3 scripts/step_to_mjcf.py
  python3 scripts/step_to_mjcf.py --step assets/cad/ElectronBot.step --out assets/mjcf/electronbot_full_arm_meters.xml

⚠️ 首次运行会很慢 (cascadio 内部需编译 OpenCASCADE 着色器, ~30s-2min)
   后续运行缓存 GLB 到 assets/meshes/electronbot_scene.glb
"""

from __future__ import annotations

import argparse
import logging
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("step_to_mjcf")

# ── MuJoCo 关节映射 (来自 robot-structure.md) ──
# body → joint axis, range, actuator
JOINT_CONFIG = {
    "body": {
        "joint_name": "joint_body",
        "actuator_name": "act_body",
        "type": "hinge",
        "axis": "0 0 1",
        "range": "-1.5708 1.5708",  # ±90°
        "kp": "80", "kv": "20",
    },
    "head": {
        "joint_name": "joint_head",
        "actuator_name": "act_head",
        "type": "hinge",
        "axis": "0 1 0",
        "range": "-0.2618 0.2618",  # ±15°
        "kp": "40", "kv": "10",
    },
    "left_arm": {
        "pitch": {"joint_name": "joint_lp", "actuator_name": "act_lp",
                  "type": "hinge", "axis": "0 1 0",
                  "range": "-1.5708 1.5708", "kp": "60", "kv": "15"},
        "roll": {"joint_name": "joint_lr", "actuator_name": "act_lr",
                 "type": "hinge", "axis": "1 0 0",
                 "range": "-0.7854 0.7854", "kp": "30", "kv": "8"},
    },
    "right_arm": {
        "pitch": {"joint_name": "joint_rp", "actuator_name": "act_rp",
                  "type": "hinge", "axis": "0 -1 0",
                  "range": "-1.5708 1.5708", "kp": "60", "kv": "15"},
        "roll": {"joint_name": "joint_rr", "actuator_name": "act_rr",
                 "type": "hinge", "axis": "1 0 0",
                 "range": "-0.7854 0.7854", "kp": "30", "kv": "8"},
    },
}

# Part name pattern → body group mapping (基于 FreeCAD dev-notes)
PART_TO_BODY = {
    # 身体
    "body": r"(?i)(body|身体|底座|控制板|舵机|齿轮|轴承|推杆轴|挡球|microservo|sg90|pcb|user_library|sot-23|sw3dps|ffc_fpc|sh07w_korp|shxx_pin|open_cascade_step_translator|scr|mark|mid_body|p_|p )",
    # 头部
    "head": r"(?i)(head|头部|摄像头)",
    # 颈部
    "neck": r"(?i)(neck|颈部)",
    # 肩部
    "shoulder": r"(?i)(shoulder|肩部)",
    # 左臂
    "left_arm": r"(?i)(left.*arm|left.*hand|手臂.*左|推杆.*左)",
    # 右臂
    "right_arm": r"(?i)(right.*arm|right.*hand|手臂.*右|手臂.*镜像|推杆.*右|推杆.*镜像)",
    # 底座
    "base": r"(?i)(base|底座)",
}


def load_step_scene(step_path: str) -> "trimesh.Scene":
    """用 cascadio + trimesh 加载 STEP 为 scene 对象."""
    step_file = Path(step_path)
    glb_cache = step_file.with_suffix(".glb")
    glb_cache = Path("assets/meshes") / f"{step_file.stem}_scene.glb"

    import trimesh

    if glb_cache.exists():
        logger.info("  从缓存加载 GLB: %s", glb_cache)
        scene = trimesh.load(glb_cache, file_type="glb")
        return scene

    logger.info("  cascadio 转换 STEP → GLB (首次, 预计 30s-2min)...")
    try:
        from cascadio import step_to_glb
        glb_cache.parent.mkdir(parents=True, exist_ok=True)
        step_to_glb(str(step_path), str(glb_cache))
        logger.info("  GLB 已缓存: %s", glb_cache)
        scene = trimesh.load(glb_cache, file_type="glb")
        return scene
    except Exception as e:
        logger.error("cascadio 转换失败: %s", e)
        raise


def extract_mesh_nodes(
    scene: "trimesh.Scene", out_dir: str
) -> List[dict]:
    """
    遍历 scene graph → 提取每个 mesh node 的:
      - mesh (STL 文件路径)
      - transform 矩阵 (4x4, 单位: m)
      - name (原始节点名)
      - category (映射到的 body group)
    """
    import trimesh

    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    meshes: Dict[str, trimesh.Trimesh] = {}
    nodes: List[dict] = []

    # -- 拿到所有几何 --
    if hasattr(scene, "geometry") and isinstance(scene.geometry, dict):
        meshes = {k: v for k, v in scene.geometry.items()
                  if isinstance(v, trimesh.Trimesh)}

    logger.info("  共 %d 个 mesh 几何体", len(meshes))

    # 调试：打印 scene graph 信息
    logger.info("  scene.graph.nodes_geometry: %d 个节点", len(scene.graph.nodes_geometry))
    if scene.graph.nodes_geometry:
        logger.info("  前 5 个节点: %s", list(scene.graph.nodes_geometry)[:5])

    if not meshes:
        # fallback: 尝试 dump 为单个 mesh
        combined = scene.dump(concatenate=True)
        if isinstance(combined, trimesh.Trimesh):
            meshes["combined"] = combined
            logger.info("  使用合并 mesh")

    # -- 遍历 scene graph 节点 --
    count = 0
    name_counter = {}  # 用于确保名称唯一

    for node_name in scene.graph.nodes_geometry:
        # scene.graph[node] 返回 (transform, geometry_name)
        result = scene.graph[node_name]
        if isinstance(result, tuple) and len(result) == 2:
            transform, geo_name = result
        else:
            continue

        mesh = meshes.get(geo_name)
        if mesh is None:
            continue

        # 获取世界 transform
        if isinstance(transform, np.ndarray) and transform.shape == (4, 4):
            world_matrix = transform.copy()
        else:
            world_matrix = np.eye(4)

        # GLB 顶点已经是米为单位，不需要额外缩放

        # 分类到 body group
        category = _classify_node(geo_name)

        # 导出 STL (保留中文字符，确保名称唯一)
        base_name = re.sub(r"[^\w]", "_", geo_name)[:40]
        if base_name in name_counter:
            name_counter[base_name] += 1
            safe_name = f"{base_name}_{name_counter[base_name]}"
        else:
            name_counter[base_name] = 0
            safe_name = base_name

        stl_path = out_path / f"step_{safe_name}.stl"

        # 将 world transform 应用到 mesh 顶点 (让 STL 在 CAD 世界坐标)
        mesh_copy = mesh.copy()
        mesh_copy.apply_transform(world_matrix)

        # 过滤退化 mesh (某个维度范围太小)
        extents = mesh_copy.extents
        if any(e < 1e-6 for e in extents):
            continue  # 跳过退化 mesh

        mesh_copy.export(stl_path)

        # 使用变换后 mesh 的质心作为零件世界位置（用于后续坐标本地化）
        mesh_centroid = mesh_copy.centroid.tolist()
        
        # 计算 mesh 顶点的最低 z 坐标（用于后续让模型坐地）
        min_vertex_z = float(mesh_copy.vertices[:, 2].min())

        nodes.append({
            "name": safe_name,
            "stl": stl_path.name,
            "category": category,
            "world_pos": mesh_centroid,  # 使用 mesh 质心
            "min_z": min_vertex_z,  # mesh 顶点的最低 z 坐标
            "world_matrix": world_matrix,
            "mesh": mesh_copy,
        })
        count += 1
        if count % 20 == 0:
            logger.info("    ... %d 个零件已导出", count)

    logger.info("  ✅ 共导出 %d 个零件", count)
    return nodes


def _classify_node(name: str) -> str:
    """根据节点名匹配到 robot body group (支持中文名称)."""
    # 头部
    if any(kw in name for kw in ["头部", "摄像头"]):
        return "head"
    
    # 颈部
    if "颈部" in name:
        return "neck"
    
    # 左臂 (手臂但不含"镜像"，或含"左")
    if any(kw in name for kw in ["手臂", "推杆"]):
        if "镜像" in name or "右" in name:
            return "right_arm"
        else:
            return "left_arm"
    
    # 右臂 (手臂且含"镜像"或"右")
    if "手臂" in name and ("镜像" in name or "右" in name):
        return "right_arm"
    
    # 肩部
    if "肩部" in name:
        return "shoulder"
    
    # 身体
    if any(kw in name for kw in ["身体", "底座", "控制板"]):
        return "body"
    
    # 舵机、齿轮、轴承等机械部件归到身体
    if any(kw in name for kw in ["舵机", "齿轮", "轴承", "推杆轴", "挡球"]):
        return "body"
    
    # 英文名称分类
    name_lower = name.lower()
    
    # 舵机 (SG90)
    if "microservo" in name_lower or "sg90" in name_lower:
        return "body"
    
    # 电路板/电子元件
    if any(kw in name_lower for kw in ["pcb", "user_library", "sot-23", "sw3dps", "ffc_fpc", "sh07w_korp", "shxx_pin"]):
        return "body"
    
    # 轴承/齿轮 (OpenCASCADE 自动生成的名称)
    if "open_cascade_step_translator" in name_lower:
        return "body"
    
    # 屏幕/标记
    if any(kw in name_lower for kw in ["scr", "mark", "mid_body"]):
        return "body"
    
    # 其他小零件
    if any(kw in name_lower for kw in ["p_", "p "]):
        return "body"
    
    return "unknown"


def group_nodes_by_category(nodes: List[dict]) -> Dict[str, List[dict]]:
    """将节点按类别分组."""
    groups: Dict[str, List[dict]] = {c: [] for c in PART_TO_BODY}
    groups["unknown"] = []
    for n in nodes:
        cat = n["category"]
        if cat not in groups:
            cat = "unknown"
        groups[cat].append(n)
    # dedup
    groups = {k: v for k, v in groups.items() if v}
    return groups


def _convert_stl_to_local(stl_path: Path, offset: np.ndarray) -> Path:
    """将 STL 从世界坐标转换为本地坐标（减去 offset），返回本地 STL 路径."""
    import trimesh
    mesh = trimesh.load(stl_path)
    mesh.vertices -= offset
    local_path = stl_path.with_name(stl_path.stem + "_local.stl")
    mesh.export(local_path)
    return local_path


def generate_mjcf(
    groups: Dict[str, List[dict]],
    output_xml: str,
    stl_dir: str = "assets/meshes/step_parts",
) -> None:
    """生成 MJCF XML (6-DOF).

    关键修正：
    - 使用 body 组中心作为 body 的世界位置
    - 其他零件组用各自中心做本地化
    - 每个零件的 geom.pos 设为其世界位置相对于 body 中心的偏移
    """
    import trimesh

    stl_base = Path(stl_dir)

    # 找到所有零件顶点的最低 z 坐标（用于让模型坐地）
    all_min_z = [p["min_z"] for parts in groups.values() for p in parts if "min_z" in p]
    min_z = min(all_min_z) if all_min_z else 0
    logger.info("  最低顶点 z = %.4f", min_z)

    # 世界原点 → 最低点偏移 (让机器人坐在地上)
    z_shift = -min_z
    logger.info("  Z shift = %.4f (让 robot 坐地)", z_shift)

    # 分组 (使用 _classify_node 返回的类别名)
    body_parts = groups.get("body", [])
    head_parts = groups.get("head", [])
    neck_parts = groups.get("neck", [])
    shoulder_parts = groups.get("shoulder", [])
    left_arm_parts = groups.get("left_arm", [])
    right_arm_parts = groups.get("right_arm", [])
    base_parts = groups.get("base", [])
    unknown_parts = groups.get("unknown", [])

    # 计算各 body 的世界坐标中心
    def _group_center(parts):
        if not parts:
            return np.array([0, 0, 0])
        return np.mean([p["world_pos"] for p in parts], axis=0)

    body_world_raw = _group_center(body_parts) if body_parts else np.array([0, 0, 0])
    head_world_raw = _group_center(head_parts)
    left_arm_world_raw = _group_center(left_arm_parts)
    right_arm_world_raw = _group_center(right_arm_parts)
    base_world_raw = _group_center(base_parts)

    # body 的世界位置（加 z_shift 让机器人坐地）
    body_world = body_world_raw.copy()
    body_world[2] += z_shift

    # 计算各子 body 的世界位置（加 z_shift）
    neck_world_raw = _group_center(neck_parts) if neck_parts else body_world_raw
    neck_world = neck_world_raw.copy()
    neck_world[2] += z_shift

    head_world = head_world_raw.copy()
    head_world[2] += z_shift

    shoulder_world_raw = _group_center(shoulder_parts) if shoulder_parts else body_world_raw
    shoulder_world = shoulder_world_raw.copy()
    shoulder_world[2] += z_shift

    left_arm_world = left_arm_world_raw.copy()
    left_arm_world[2] += z_shift

    right_arm_world = right_arm_world_raw.copy()
    right_arm_world[2] += z_shift

    base_world = base_world_raw.copy()
    base_world[2] += z_shift

    # 关键修正：STL 顶点转换为 local 坐标（减去父 body 世界位置）
    # geom.pos = 0，最终位置 = body.pos + 0 + localVertex = worldVertex + z_shift
    def _localize_parts(parts, parent_world_pos):
        """将 STL 顶点从世界坐标转换为相对于父 body 的 local 坐标
        注意：parent_world_pos 已经包含 z_shift，所以 local 坐标 = worldVertex - parent_world_pos
        最终显示位置 = body.pos + localVertex = parent_world_pos + (worldVertex - parent_world_pos) = worldVertex
        但 worldVertex 需要加 z_shift 才能坐地，所以 local 坐标 = (worldVertex + z_shift) - parent_world_pos
        """
        for p in parts:
            stl_path = stl_base / p["stl"]
            # 读取 STL，将顶点加 z_shift，再减去 parent_world_pos
            import trimesh
            mesh = trimesh.load(stl_path)
            # 顶点加 z_shift（让模型坐地）
            mesh.vertices[:, 2] += z_shift
            # 转换为 local 坐标（减去父 body 世界位置）
            mesh.vertices -= parent_world_pos
            local_path = stl_path.with_name(stl_path.stem + "_local.stl")
            mesh.export(local_path)
            p["local_stl"] = local_path.name
        return parts

    _localize_parts(body_parts, body_world)
    _localize_parts(unknown_parts, body_world)
    _localize_parts(neck_parts, neck_world)
    _localize_parts(head_parts, head_world)
    _localize_parts(shoulder_parts, shoulder_world)
    _localize_parts(left_arm_parts, left_arm_world)
    _localize_parts(right_arm_parts, right_arm_world)
    _localize_parts(base_parts, base_world)

    # 开始构建 XML
    xml_lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<mujoco model="ElectronBot_step">',
        "",
        f'  <compiler meshdir="{stl_base.absolute()}" />',
        "",
        "  <option timestep=\"0.002\" gravity=\"0 0 -9.81\">",
        "    <flag warmstart=\"disable\" />",
        "  </option>",
        "",
        "  <default>",
        '    <geom contype="0" conaffinity="0" condim="3" friction="0.8 0.3 0.1" density="0.1"/>',
        "    <joint armature=\"0.001\" damping=\"0.01\" />",
        "  </default>",
        "",
        "  <asset>",
    ]

    # 注册所有 mesh（使用本地坐标 STL）
    for cat, parts in groups.items():
        for p in parts:
            local_stl = p.get("local_stl", p["stl"])
            xml_lines.append(
                f'    <mesh name="mesh_{p["name"]}" '
                f'file="{local_stl}" '
                f'scale="1.0 1.0 1.0" />'
            )
    xml_lines.extend([
        "  </asset>",
        "",
        "  <worldbody>",
        '    <light directional="true" pos="0.5 0.5 1.0" />',
    ])

    # body
    xml_lines.append(
        f'    <body name="body" pos="{body_world[0]:.4f} {body_world[1]:.4f} {body_world[2]:.4f}">'
    )
    xml_lines.append(
        f'      <joint name="body_joint" type="hinge" axis="0 0 1" '
        f'range="-1.5708 1.5708" limited="true" />'
    )
    # body geom: STL 已本地化，geom.pos = 0
    # 使用更合理的质量值（0.01 kg 每零件，body 部分较多）
    body_mass = 0.01
    for p in body_parts:
        gp = p.get("geom_pos", np.zeros(3))
        xml_lines.append(
            f'      <geom name="geom_{p["name"]}" type="mesh" '
            f'mesh="mesh_{p["name"]}" '
            f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
            f'mass="{body_mass}" rgba="0.5 0.5 0.6 1" />'
        )

    # neck
    if neck_parts:
        neck_world_raw = np.mean([p["world_pos"] for p in neck_parts], axis=0)
        neck_world = neck_world_raw.copy()
        neck_world[2] += z_shift
        neck_body_pos = neck_world - body_world
        xml_lines.append(
            f'      <body name="neck" '
            f'pos="{neck_body_pos[0]:.4f} {neck_body_pos[1]:.4f} {neck_body_pos[2]:.4f}">'
        )
        xml_lines.append(
            f'        <joint name="joint_neck" type="hinge" axis="0 0 1" '
            f'range="-0.5236 0.5236" limited="true" />'
        )
        neck_mass = 0.01
        for p in neck_parts:
            gp = p.get("geom_pos", np.zeros(3))
            xml_lines.append(
                f'        <geom name="geom_{p["name"]}" type="mesh" '
                f'mesh="mesh_{p["name"]}" '
                f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
                f'mass="{neck_mass}" rgba="0.5 0.5 0.5 1" />'
            )
        
        # head (作为 neck 的子 body)
        if head_parts:
            head_world = head_world_raw.copy()
            head_world[2] += z_shift
            head_neck_pos = head_world - neck_world
            xml_lines.append(
                f'        <body name="head" '
                f'pos="{head_neck_pos[0]:.4f} {head_neck_pos[1]:.4f} {head_neck_pos[2]:.4f}">'
            )
            xml_lines.append(
                f'          <joint name="joint_head" type="hinge" axis="0 1 0" '
                f'range="-0.2618 0.2618" limited="true" />'
            )
            head_mass = 0.02
            for p in head_parts:
                gp = p.get("geom_pos", np.zeros(3))
                xml_lines.append(
                    f'          <geom name="geom_{p["name"]}" type="mesh" '
                    f'mesh="mesh_{p["name"]}" '
                    f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
                    f'mass="{head_mass}" rgba="0.3 0.6 0.8 1" />'
                )
            xml_lines.append("        </body>")
        xml_lines.append("      </body>")

    # shoulder (左右肩)
    if shoulder_parts:
        shoulder_world_raw = np.mean([p["world_pos"] for p in shoulder_parts], axis=0)
        shoulder_world = shoulder_world_raw.copy()
        shoulder_world[2] += z_shift
        shoulder_body_pos = shoulder_world - body_world
        xml_lines.append(
            f'      <body name="shoulder" '
            f'pos="{shoulder_body_pos[0]:.4f} {shoulder_body_pos[1]:.4f} {shoulder_body_pos[2]:.4f}">'
        )
        shoulder_mass = 0.01
        for p in shoulder_parts:
            gp = p.get("geom_pos", np.zeros(3))
            xml_lines.append(
                f'        <geom name="geom_{p["name"]}" type="mesh" '
                f'mesh="mesh_{p["name"]}" '
                f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
                f'mass="{shoulder_mass}" rgba="0.5 0.5 0.5 1" />'
            )
        xml_lines.append("      </body>")

    # left arm
    _gen_arm_xml(xml_lines, "left_arm", left_arm_parts, body_world,
                 "joint_lp", "0 1 0", "joint_lr", "1 0 0", z_shift)

    # right arm
    _gen_arm_xml(xml_lines, "right_arm", right_arm_parts, body_world,
                 "joint_rp", "0 -1 0", "joint_rr", "1 0 0", z_shift)

    # base
    if base_parts:
        base_world = base_world_raw.copy()
        base_world[2] += z_shift
        base_body_pos = base_world - body_world
        xml_lines.append(
            f'      <body name="base" '
            f'pos="{base_body_pos[0]:.4f} {base_body_pos[1]:.4f} {base_body_pos[2]:.4f}">'
        )
        base_mass = 0.03
        for p in base_parts:
            gp = p.get("geom_pos", np.zeros(3))
            xml_lines.append(
                f'        <geom name="geom_{p["name"]}" type="mesh" '
                f'mesh="mesh_{p["name"]}" '
                f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
                f'mass="{base_mass}" rgba="0.6 0.4 0.2 1" />'
            )
        xml_lines.append("      </body>")

    # unknown
    if unknown_parts:
        logger.info("  还有 %d 个未分类零件放在 body 下", len(unknown_parts))
        unknown_mass = 0.01
        for p in unknown_parts:
            gp = p.get("geom_pos", np.zeros(3))
            xml_lines.append(
                f'      <geom name="geom_{p["name"]}" type="mesh" '
                f'mesh="mesh_{p["name"]}" '
                f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
                f'mass="{unknown_mass}" rgba="0.8 0.8 0.8 1" />'
            )

    xml_lines.extend([
        "    </body>",  # body
        "",
        "    <!-- ground plane -->",
        '    <geom name="floor" type="plane" size="0.5 0.5 0.025" '
        'pos="0 0 0" rgba="0.3 0.4 0.5 1" />',
        "  </worldbody>",
        "",
        "  <actuator>",
        '    <position name="act_body" joint="body_joint" ctrlrange="-1.5708 1.5708" kp="20" kv="1"/>',
    ])

    # 条件化添加 neck actuator
    if neck_parts:
        xml_lines.append('    <position name="act_neck" joint="joint_neck" ctrlrange="-0.5236 0.5236" kp="10" kv="0.5"/>')
        # head actuator 只有在 neck body 存在时才能添加（因为 head 是 neck 的子 body）
        if head_parts:
            xml_lines.append('    <position name="act_head" joint="joint_head" ctrlrange="-0.2618 0.2618" kp="10" kv="0.5"/>')

    xml_lines.extend([
        '    <position name="act_lp" joint="joint_lp" ctrlrange="-1.5708 1.5708" kp="20" kv="1"/>',
        '    <position name="act_lr" joint="joint_lr" ctrlrange="-0.7854 0.7854" kp="20" kv="1"/>',
        '    <position name="act_rp" joint="joint_rp" ctrlrange="-1.5708 1.5708" kp="20" kv="1"/>',
        '    <position name="act_rr" joint="joint_rr" ctrlrange="-0.7854 0.7854" kp="20" kv="1"/>',
        "  </actuator>",
        "",
        "</mujoco>",
    ])

    with open(output_xml, "w", encoding="utf-8") as f:
        f.write("\n".join(xml_lines))
    logger.info("✅ MJCF 输出: %s (%.1f KB)", output_xml,
                Path(output_xml).stat().st_size / 1024)


def _gen_arm_xml(
    lines: List[str],
    arm_name: str,
    parts: List[dict],
    body_world: np.ndarray,
    pitch_joint: str, pitch_axis: str,
    roll_joint: str, roll_axis: str,
    z_shift: float,
):
    """生成手臂 MJCF XML 块 (STL 已本地化，geom.pos = 0).

    结构: body(pitch) → child_body(roll + geoms)
    """
    if not parts:
        return

    arm_world_raw = np.mean([p["world_pos"] for p in parts], axis=0)
    arm_world = arm_world_raw.copy()
    arm_world[2] += z_shift
    arm_body_pos = arm_world - body_world

    # pitch 关节在父 body
    lines.append(
        f'      <body name="{arm_name}" '
        f'pos="{arm_body_pos[0]:.4f} {arm_body_pos[1]:.4f} {arm_body_pos[2]:.4f}">'
    )
    lines.append(
        f'        <inertial pos="0 0 0" mass="0.015" diaginertia="1e-6 1e-6 1e-6"/>'
    )
    lines.append(
        f'        <joint name="{pitch_joint}" type="hinge" '
        f'axis="{pitch_axis}" range="-1.5708 1.5708" limited="true" />'
    )

    # roll 关节在子 body
    lines.append(
        f'        <body name="{arm_name}_roll">'
    )
    lines.append(
        f'          <joint name="{roll_joint}" type="hinge" '
        f'axis="{roll_axis}" range="-0.7854 0.7854" limited="true" />'
    )

    # STL 保持世界坐标，geom.pos 使用计算出的相对偏移
    arm_mass = 0.015
    for p in parts:
        gp = p.get("geom_pos", np.zeros(3))
        lines.append(
            f'          <geom name="geom_{p["name"]}" type="mesh" '
            f'mesh="mesh_{p["name"]}" '
            f'pos="{gp[0]:.4f} {gp[1]:.4f} {gp[2]:.4f}" '
            f'mass="{arm_mass}" rgba="0.4 0.6 0.3 1" />'
        )
    lines.append("        </body>")  # roll body
    lines.append("      </body>")    # pitch body


def main():
    parser = argparse.ArgumentParser(
        description="ElectronBot.step → MJCF 直接转换"
    )
    parser.add_argument(
        "--step", default="assets/cad/ElectronBot.step",
        help="STEP 文件路径"
    )
    parser.add_argument(
        "--out", default="assets/mjcf/electronbot_step_meters.xml",
        help="输出 MJCF XML 路径"
    )
    parser.add_argument(
        "--stl-dir", default="assets/meshes/step_parts",
        help="STL 零件输出目录"
    )
    parser.add_argument(
        "--no-cache", action="store_true",
        help="跳过 GLB 缓存, 强制重新转换"
    )
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    step_path = str(project_root / args.step)
    out_xml = str(project_root / args.out)
    stl_dir = str(project_root / args.stl_dir)

    logger.info("=" * 60)
    logger.info("STEP → MJCF 直接转换")
    logger.info("  STEP: %s", step_path)
    logger.info("  输出: %s", out_xml)
    logger.info("  STL:  %s", stl_dir)

    # 1. 加载 STEP → scene
    logger.info("\n[1/4] 加载 STEP 场景...")
    scene = load_step_scene(step_path)

    # 2. 提取 mesh nodes
    logger.info("\n[2/4] 提取零件节点...")
    nodes = extract_mesh_nodes(scene, stl_dir)

    # 3. 分组
    logger.info("\n[3/4] 分组零件...")
    groups = group_nodes_by_category(nodes)
    for cat, parts in groups.items():
        logger.info("  %s: %d 个零件", cat, len(parts))

    # 4. 生成 MJCF
    logger.info("\n[4/4] 生成 MJCF XML...")
    generate_mjcf(groups, out_xml, stl_dir)

    logger.info("\n✅ 完成! MJCF: %s", out_xml)
    logger.info("\n验证: python3 -m electronbot_sim.mcp_server --render human --xml=%s",
                args.out)


if __name__ == "__main__":
    main()
