#!/usr/bin/env python3
"""
从 FreeCAD 导出 LEFT_ARM_PARTS 和 RIGHT_ARM_PARTS 为 STL
保持 FreeCAD 原始坐标系
"""

import FreeCAD, Mesh, os

CAD_PATH = '/mnt/data2/projects/xiaozhi/xiaozhi-electronbot-docs/docs/cad/cadelectron.FCStd'
OUT_DIR = '/mnt/data2/projects/xiaozhi/ElectronBot_SIM/assets/meshes'

doc = FreeCAD.open(CAD_PATH)

# LEFT_ARM_PARTS 和 RIGHT_ARM_PARTS
LEFT_ARM_PARTS = [
    "Part__Feature042",  # 左手
    "Part__Feature045",  # 左手镜
    "Part__Feature046",  # 臂件
    "Part__Feature047",  # 臂件
    "Part__Feature048",  # 臂件
    "Part__Feature049",  # 臂件
]

RIGHT_ARM_PARTS = [
    "Part__Feature027",  # 齿轮
    "Part__Feature028",  # 小齿轮
    "Part__Feature029",  # 右手
    "Part__Feature030",  # 臂件
    "Part__Feature031",  # 臂件
    "Part__Feature032",  # 臂件
    "Part__Feature033",  # 臂件
]

def export_parts_as_stl(part_names, output_name, label):
    """合并多个零件为一个 STL"""
    shapes = []
    for name in part_names:
        obj = doc.getObject(name)
        if obj and hasattr(obj, 'Shape'):
            shapes.append(obj.Shape)
            print(f"  + {name}")
    
    if not shapes:
        print(f"  ERROR: no shapes for {label}")
        return
    
    # 合并所有 Shape
    compound = shapes[0]
    for s in shapes[1:]:
        compound = compound.fuse(s)
    
    # 导出 STL
    out_path = os.path.join(OUT_DIR, output_name)
    Mesh.export([compound], out_path)
    
    bb = compound.BoundBox
    print(f"  → {out_path}")
    print(f"    BB: X=[{bb.XMin:.1f},{bb.XMax:.1f}] Y=[{bb.YMin:.1f},{bb.YMax:.1f}] Z=[{bb.ZMin:.1f},{bb.ZMax:.1f}] mm")
    print(f"    Volume: {compound.Volume:.0f} mm³")

print("=" * 60)
print("Exporting LEFT arm from FreeCAD...")
export_parts_as_stl(LEFT_ARM_PARTS, "left_arm_fc.stl", "LEFT_ARM")

print()
print("Exporting RIGHT arm from FreeCAD...")
export_parts_as_stl(RIGHT_ARM_PARTS, "right_arm_fc.stl", "RIGHT_ARM")

print()
print("Done! STL files saved in:", OUT_DIR)

FreeCAD.closeDocument(doc.Name)
