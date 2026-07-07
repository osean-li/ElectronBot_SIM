#!/usr/bin/env python3
"""
分离 arm STL：手臂外壳 (arm_shell) vs 肩座/舵机 (shoulder_mount)
- 按连通分量自动分离
- 生成新的 STL 文件
- 更新 MJCF 模型
"""

import trimesh
import numpy as np
import os
import re

MESH_DIR = "assets/meshes"
OUTPUT_DIR = MESH_DIR  # 直接放在 meshes 目录
TEMPLATE_XML = "assets/mjcf/electronbot_freecad_aligned.xml"
OUTPUT_XML = "assets/mjcf/electronbot_split_arms.xml"


def split_arm_mesh(stl_path, arm_side):
    """
    分离手臂 STL 为外壳 + 肩座
    外壳 = 最大的连通分量 (体积最大)
    肩座 = 其余所有分量
    """
    m = trimesh.load(stl_path)
    splits = m.split()
    
    if len(splits) <= 1:
        print(f"  ⚠️ {arm_side}: 只有一个分量，无法自动分离")
        return None, None
    
    # 按体积排序，最大的 = 外壳
    volumes = [(s.volume, s) for s in splits]
    volumes.sort(key=lambda x: -x[0])
    
    # 外壳 = 体积最大的分量
    shell = volumes[0][1]
    shell_vol = volumes[0][0]
    
    # 肩座 = 其余分量合并
    mount_parts = [s for _, s in volumes[1:]]
    mount = trimesh.util.concatenate(mount_parts) if mount_parts else None
    
    print(f"  {arm_side}:")
    print(f"    外壳 (arm_shell): {len(shell.faces)} 面, 体积={shell_vol*1e9:.0f} mm³")
    if mount:
        print(f"    肩座 (shoulder_mount): {len(mount.faces)} 面, 体积={mount.volume*1e9:.0f} mm³")
    else:
        print(f"    肩座: 无其他分量")
    
    return shell, mount


def generate_inline_mesh(mesh, name):
    """生成 MJCF inline mesh 的 vertex/face 字符串"""
    if mesh is None:
        return None
    
    # 顶点 (mm 单位, 保持原始)
    vertex_str = " ".join([f"{v[0]:.6g} {v[1]:.6g} {v[2]:.6g}" for v in mesh.vertices])
    # 面索引
    face_str = " ".join([f"{f[0]} {f[1]} {f[2]}" for f in mesh.faces])
    
    return f'<mesh name="{name}" vertex="{vertex_str}" face="{face_str}" />'


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print("=" * 60)
    print("🔧 分离手臂 STL")
    print("=" * 60)
    
    # ============================================================
    # 1. 分离左手
    # ============================================================
    left_shell, left_mount = split_arm_mesh(
        os.path.join(MESH_DIR, "left_arm.stl"), "左手"
    )
    
    # 如果分离成功，导出新的 STL
    if left_shell:
        left_shell_path = os.path.join(OUTPUT_DIR, "left_arm_shell.stl")
        left_shell.export(left_shell_path)
        print(f"  ✅ 已导出: {left_shell_path}")
    if left_mount:
        left_mount_path = os.path.join(OUTPUT_DIR, "left_shoulder_mount.stl")
        left_mount.export(left_mount_path)
        print(f"  ✅ 已导出: {left_mount_path}")
    
    # ============================================================
    # 2. 分离右手
    # ============================================================
    right_shell, right_mount = split_arm_mesh(
        os.path.join(MESH_DIR, "right_arm.stl"), "右手"
    )
    
    if right_shell:
        right_shell_path = os.path.join(OUTPUT_DIR, "right_arm_shell.stl")
        right_shell.export(right_shell_path)
        print(f"  ✅ 已导出: {right_shell_path}")
    if right_mount:
        right_mount_path = os.path.join(OUTPUT_DIR, "right_shoulder_mount.stl")
        right_mount.export(right_mount_path)
        print(f"  ✅ 已导出: {right_mount_path}")
    
    # ============================================================
    # 3. 检查包围盒
    # ============================================================
    print("\n📐 分离后包围盒 (m):")
    for fname in ['left_arm_shell', 'left_shoulder_mount', 'right_arm_shell', 'right_shoulder_mount']:
        path = os.path.join(OUTPUT_DIR, f"{fname}.stl")
        if os.path.exists(path):
            m = trimesh.load(path)
            b = m.bounds * 0.001
            print(f"  {fname:25s}: X=[{b[0][0]:.4f},{b[1][0]:.4f}], "
                  f"Y=[{b[0][1]:.4f},{b[1][1]:.4f}], Z=[{b[0][2]:.4f},{b[1][2]:.4f}]")
    
    # ============================================================
    # 4. 读取现有 MJCF 模板并重建
    # ============================================================
    print("\n📝 生成新的 MJCF...")
    
    # 读取模板获取 asset 定义 (mesh vertex/face)
    # 我们不需要完整重建，只需要替换 body 结构
    # 让我读模板并做简单替换
    
    with open(TEMPLATE_XML, 'r') as f:
        xml = f.read()
    
    # ============================================================
    # 5. 重建 MJCF
    # ============================================================
    
    # 读取原始 electronbot_mesh.xml 的 asset 定义
    # (包含所有 mesh 的 vertex/face)
    # 我们沿用 freecad_aligned.xml 的 asset, 但添加新的分离 mesh
    
    # 由于 inline mesh 太大，我们用 STL 文件引用方式
    # 首先检查 generate_inline_mesh.py 是如何工作的
    
    # 更简单的方案：在 MJCF 中使用 <mesh> 引用 STL 文件
    # 这需要 stl->msh 转换，或者直接使用 asset
    # 我们用 python-mujoco 可以直接加载 STL 吗？试试
    
    # 实际上 mujoco 的 <mesh> 在 MJCF 中可以用 file 属性引用 STL/OBJ
    # 但需要正确的路径
    
    # 让我们使用 inline mesh 方式（vertex face），这样不需要额外的文件
    
    # 为简单起见，先构建一个没有 inline mesh 的版本
    # 然后手动生成 inline mesh
    
    # 实际上最简单的方式：读取 electronbot_mesh.xml，
    # 修改 body 结构来包含分离的 mesh
    
    print("\n✅ STL 文件已分离，接下来生成 inline mesh MJCF...")
    print("=" * 60)


if __name__ == "__main__":
    main()
