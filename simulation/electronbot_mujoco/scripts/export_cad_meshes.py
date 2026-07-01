#!/usr/bin/env python3
"""
从 ElectronBot.step 导出各连杆的 STL 网格

策略:
1. 用 OpenCascade 解析 STEP, 提取所有体积 > 100mm³ 的 solid
2. 按质心 Z(高度) + X(左右) 聚类成 7 个连杆
3. 合并聚类内 solid 为一个 compound → 三角化 → 导出 STL
4. 自动替换 MJCF 中的几何体引用

用法:
  python export_cad_meshes.py
"""

import os, sys, json, shutil
from collections import defaultdict
import numpy as np

# ── OpenCascade ──
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCP.StlAPI import StlAPI_Writer
from OCP.TopoDS import TopoDS_Shape, TopoDS_Compound
from OCP.BRep import BRep_Builder

_file = os.path.abspath(__file__)
PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_file))))
STEP_PATH = "/home/j6m/code/github/ElectronBot/4.CAD-Model/ElectronBot.step"
BOX_PATH  = "/home/j6m/code/github/ElectronBot/4.CAD-Model/Box.step"
EMOJI_DIR = "/home/j6m/code/github/ElectronBot/4.CAD-Model/Emoji"
OUT_DIR = f"{PROJECT}/simulation/electronbot_description/meshes"
EMOJI_OUT = f"{PROJECT}/data/emoji_animations"
MJCF_PATH = f"{PROJECT}/simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot.xml"
INLINE_PATH = f"{PROJECT}/simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot_inline.xml"
STL_REL_DIR = "../electronbot_description/meshes"


def load_solids(path):
    """加载 STEP 文件中所有体积 > 100mm³ 的 solid"""
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError(f"Cannot read {path}")
    reader.TransferRoots()
    shape = reader.OneShape()

    solids = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solid = explorer.Current()
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, props)
        vol = props.Mass()
        if vol > 100:
            com = props.CentreOfMass()
            solids.append({
                "shape": solid,
                "volume_mm3": vol,
                "com_xyz": np.array([com.X(), com.Y(), com.Z()]),
            })
        explorer.Next()
    return solids


def cluster_parts(solids):
    """按 COM 位置聚类成 7 个连杆"""
    # 7 个连杆的定义 (按 COM 的 Z 和 X 分)
    parts = {
        "base_link":       [],   # Z 最低, 中心
        "body":            [],   # Z 中等偏低, 中心
        "head":            [],   # Z 最高, 中心
        "left_arm":        [],   # X > 0 (左=正), Z 中等
        "right_arm":       [],   # X < 0 (右=负), Z 中等
        "left_shoulder":   [],   # X > 0, Z 较高
        "right_shoulder":  [],   # X < 0, Z 较高
    }

    for s in solids:
        x, y, z = s["com_xyz"]
        vol = s["volume_mm3"]

        if z < -15:
            parts["base_link"].append(s)                    # 底座: 最低
        elif z > 0 and abs(x) < 15:
            if z > 30:
                parts["head"].append(s)                     # 头部: 最高, 中心
            else:
                parts["body"].append(s)                     # 身体: 中间, 中心
        elif abs(x) > 8:
            target = "left_arm" if x > 0 else "right_arm"
            if z > 10:
                target = "left_shoulder" if x > 0 else "right_shoulder"
            parts[target].append(s)

    return parts


def merge_and_export(parts, out_dir):
    """合并每组的 solid → 三角化 → STL"""
    os.makedirs(out_dir, exist_ok=True)
    exported = {}

    for name, pieces in parts.items():
        if not pieces:
            print(f"  [SKIP] {name}: no parts")
            continue

        # 合并所有 solid
        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)

        for p in pieces:
            builder.Add(compound, p["shape"])

        # 三角化
        mesh = BRepMesh_IncrementalMesh(compound, 1.0)
        mesh.Perform()

        # 导出 STL
        stl_path = os.path.join(out_dir, f"{name}.stl")
        writer = StlAPI_Writer()
        writer.Write(compound, stl_path)

        if os.path.exists(stl_path):
            size_kb = os.path.getsize(stl_path) / 1024
            total_vol = sum(p["volume_mm3"] for p in pieces)
            print(f"  [OK] {name}.stl: {len(pieces)} parts, {total_vol:.0f}mm³, {size_kb:.0f}KB")
            exported[name] = {"path": stl_path, "volume_mm3": total_vol, "n_parts": len(pieces)}
        else:
            print(f"  [FAIL] {name}: STL write failed")

    return exported


def update_mjcf(exported, mjcf_path, rel_dir):
    """生成 mesh 版 MJCF (另存为 electronbot_mesh.xml，不覆盖原文件)"""
    geom_to_stl = {
        "base_geom":  "base_link",
        "body_geom":  "body",
        "head_geom":  "head",
        "left_arm_geom": "left_arm",
        "right_arm_geom": "right_arm",
    }

    with open(mjcf_path) as f:
        content = f.read()

    mesh_path = os.path.join(os.path.dirname(mjcf_path), "electronbot_mesh.xml")

    # 更新 meshdir 为绝对路径
    abs_meshdir = os.path.abspath(os.path.join(os.path.dirname(mjcf_path),
                                               "../../../electronbot_description/meshes"))
    content = content.replace(
        'meshdir="../../electronbot_description/meshes"',
        f'meshdir="{abs_meshdir}"'
    )

    for geom_name, stl_name in geom_to_stl.items():
        if stl_name not in exported:
            continue
        mesh_file = f'{stl_name}.stl'
        if f'name="{geom_name}"' not in content:
            continue
        lines = content.split('\n')
        new_lines = []
        for line in lines:
            if f'name="{geom_name}"' in line:
                if 'type="box"' in line:
                    line = line.replace('type="box"', f'type="mesh" mesh="{mesh_file}"')
                elif 'type="cylinder"' in line:
                    line = line.replace('type="cylinder"', f'type="mesh" mesh="{mesh_file}"')
                # 去掉 mesh geom 的 size 属性
                if 'type="mesh"' in line:
                    import re
                    line = re.sub(r'\s+size="[^"]*"', '', line)
            new_lines.append(line)
        content = '\n'.join(new_lines)
        print(f"  [MJCF] {geom_name} → mesh={mesh_file}")

    with open(mesh_path, 'w') as f:
        f.write(content)

    print(f"[INFO] Mesh MJCF saved: {mesh_path}")
    print(f"[INFO]   → MuJoCo 3.1+ mesh loading REQUIRES libassimp: sudo apt install libassimp-dev")
    print(f"[INFO]   → Without libassimp, scene.xml (geometry primitives) works fine:")
    print(f"[INFO]     MUJOCO_GL=egl python -m mujoco.viewer --mjcf={MJCF_PATH}")
    print(f"[INFO]   → With libassimp installed:")
    print(f"[INFO]     MUJOCO_GL=egl python -m mujoco.viewer --mjcf={mesh_path}")


def export_box_stl(box_path, out_dir):
    """导出 Box.step → box_base.stl"""
    if not os.path.exists(box_path):
        print(f"  [SKIP] Box.step not found: {box_path}")
        return None

    solids = load_solids(box_path)
    if not solids:
        return None

    # Merge all solids into one compound
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for s in solids:
        builder.Add(compound, s["shape"])

    BRepMesh_IncrementalMesh(compound, 1.0).Perform()

    stl_path = os.path.join(out_dir, "box_base.stl")
    StlAPI_Writer().Write(compound, stl_path)

    if os.path.exists(stl_path):
        total_vol = sum(s["volume_mm3"] for s in solids)
        size_kb = os.path.getsize(stl_path) / 1024
        print(f"  [OK] box_base.stl: {len(solids)} parts, {total_vol:.0f}mm³, {size_kb:.0f}KB")
        return stl_path
    return None


def catalog_emoji_videos(emoji_dir, out_dir):
    """整理 Emoji 情绪动画视频 → 元数据 JSON, 供情绪策略参考"""
    if not os.path.isdir(emoji_dir):
        print(f"  [SKIP] Emoji dir not found: {emoji_dir}")
        return

    os.makedirs(out_dir, exist_ok=True)
    emoji_map = {}

    for folder in sorted(os.listdir(emoji_dir)):
        folder_path = os.path.join(emoji_dir, folder)
        if not os.path.isdir(folder_path):
            continue
        videos = sorted([f for f in os.listdir(folder_path) if f.endswith('.mp4')])
        emoji_map[folder] = {
            "emotion": folder,
            "videos": videos,
            "note": "参考动作 → emotional_reward.py 的 reward shaping"
        }
        for v in videos:
            src = os.path.join(folder_path, v)
            dst = os.path.join(out_dir, f"{folder}_{v}")
            if not os.path.exists(dst):
                shutil.copy2(src, dst)

    meta_path = os.path.join(out_dir, "emoji_catalog.json")
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(emoji_map, f, indent=2, ensure_ascii=False)

    print(f"  [OK] {len(emoji_map)} emoji folders → {out_dir}")
    print(f"  [INFO]   用于 emotional_reward.py 的动作参考")


def main():
    print("=" * 60)
    print("ElectronBot CAD → STL → MJCF")
    print("=" * 60)

    # ── ElectronBot body ──
    if not os.path.exists(STEP_PATH):
        print(f"[ERROR] ElectronBot STEP not found: {STEP_PATH}")
    else:
        print(f"\n[1/5] Loading ElectronBot STEP...")
        solids = load_solids(STEP_PATH)
        print(f"  Found {len(solids)} solids > 100mm³")

        print(f"\n[2/5] Clustering into 7 links...")
        parts = cluster_parts(solids)
        for name, ps in parts.items():
            print(f"  {name}: {len(ps)} parts")

        print(f"\n[3/5] Exporting ElectronBot STL meshes...")
        exported = merge_and_export(parts, OUT_DIR)

        if exported:
            print(f"\n[4/5] Generating mesh MJCF...")
            update_mjcf(exported, MJCF_PATH, STL_REL_DIR)

    # ── Box base ──
    print(f"\n[5/5] Extra assets...")
    box_stl = export_box_stl(BOX_PATH, OUT_DIR)

    # ── Emoji videos ──
    catalog_emoji_videos(EMOJI_DIR, EMOJI_OUT)

    # ── Summary ──
    stl_count = len([f for f in os.listdir(OUT_DIR) if f.endswith('.stl')]) if os.path.isdir(OUT_DIR) else 0
    print(f"\n{'='*60}")
    print(f"Done! {stl_count} STL files in {OUT_DIR}")
    if box_stl:
        print(f"  box_base.stl → can replace base_link in scene")
    if os.path.isdir(EMOJI_OUT):
        video_count = sum(1 for f in os.listdir(EMOJI_OUT) if f.endswith('.mp4'))
        print(f"  {video_count} emoji videos in {EMOJI_OUT}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
