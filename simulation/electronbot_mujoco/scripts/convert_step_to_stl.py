#!/usr/bin/env python3
"""
从 ElectronBot STEP 装配体文件导出各连杆为 STL 网格文件。

使用 FreeCAD Python API 解析 STEP 装配体，遍历零部件树，
按名称匹配分离出 7 个独立连杆，导出为 STL 并测算理论质量/质心/惯性张量。

层级结构 (来自 Unity Prefab 分析):
  Base -> Body -> Head
             -> LeftShoulder -> LeftArm (末端)
             -> RightShoulder -> RightArm (末端)

导出目标:
  ElectronBot_SIM/simulation/electronbot_description/meshes/
    ├── base_link.stl
    ├── body.stl
    ├── head.stl
    ├── left_shoulder.stl
    ├── left_arm.stl
    ├── right_shoulder.stl
    └── right_arm.stl
"""

import sys
import os
import json

# FreeCAD 路径
FREECAD_PATHS = [
    "/usr/lib/freecad/lib",
    "/usr/lib/freecad-python3/lib",
    "/usr/lib64/freecad/lib",
]

def setup_freecad():
    """设置 FreeCAD Python 路径"""
    for p in FREECAD_PATHS:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    try:
        import FreeCAD
        return True
    except ImportError:
        return False

def find_body_parts(step_path):
    """解析 STEP 装配体，提取零部件层级"""
    import FreeCAD
    import ImportGui
    import Mesh

    doc = FreeCAD.newDocument("ElectronBot")
    ImportGui.insert(step_path, doc.Name)

    print(f"[INFO] 加载 {step_path}")
    print(f"[INFO] 文档对象数量: {len(doc.Objects)}")

    # 列出所有对象
    objects = []
    for obj in doc.Objects:
        obj_info = {
            "name": obj.Name,
            "label": obj.Label,
            "type": obj.TypeId,
            "shape_type": None,
            "has_shape": hasattr(obj, 'Shape') and obj.Shape is not None,
        }
        if obj_info["has_shape"]:
            obj_info["volume"] = obj.Shape.Volume
            obj_info["center_of_mass"] = list(obj.Shape.CenterOfMass)
            # 简化惯性估算
            try:
                matrix = obj.Shape.MatrixOfInertia
                obj_info["inertia"] = {
                    "A": matrix.A11, "B": matrix.A22, "C": matrix.A33,
                }
            except Exception:
                pass

        objects.append(obj_info)
        print(f"  [{obj_info['type']}] {obj_info['label']} (name={obj_info['name']})"
              + (f" vol={obj_info.get('volume', 'N/A'):.1f}" if obj_info.get('volume') else ""))

    return doc, objects

def match_part(obj_label, obj_name):
    """根据名称匹配零部件对应哪个连杆"""
    label_lower = (obj_label + obj_name).lower()

    # 匹配规则（基于 ElectronBot 的命名习惯）
    mapping = [
        ("head", "head"),
        ("body", "body"),
        ("base", "base_link"),
        ("leftarm", "left_arm"),
        ("left_arm", "left_arm"),
        ("rightarm", "right_arm"),
        ("right_arm", "right_arm"),
        ("leftshoulder", "left_shoulder"),
        ("left_shoulder", "left_shoulder"),
        ("rightshoulder", "right_shoulder"),
        ("right_shoulder", "right_shoulder"),
    ]

    for keyword, part_name in mapping:
        if keyword in label_lower:
            return part_name
    return None

def export_stl_meshes(doc, objects, output_dir):
    """导出匹配到的零部件为 STL 文件"""
    import FreeCAD
    import Mesh

    os.makedirs(output_dir, exist_ok=True)

    exported = {}
    unmatched = []

    for obj_info in objects:
        part_name = match_part(obj_info["label"], obj_info["name"])
        if part_name:
            obj = doc.getObject(obj_info["name"])
            if obj and hasattr(obj, 'Shape'):
                stl_path = os.path.join(output_dir, f"{part_name}.stl")
                Mesh.export([obj], stl_path)
                file_size = os.path.getsize(stl_path)
                exported[part_name] = {
                    "stl_path": stl_path,
                    "file_size_kb": file_size / 1024,
                    "volume_mm3": obj_info.get("volume", 0),
                    "com": obj_info.get("center_of_mass", []),
                    "inertia": obj_info.get("inertia", {}),
                }
                print(f"  [EXPORT] {part_name}.stl ({file_size/1024:.1f} KB)")
            else:
                unmatched.append(f"{obj_info['label']} (no shape)")
        else:
            if obj_info["has_shape"]:
                unmatched.append(obj_info["label"])

    # 保存元数据
    meta_path = os.path.join(output_dir, "part_metadata.json")
    metadata = {
        "source": "ElectronBot.step",
        "exported_parts": {k: {kk: vv for kk, vv in v.items() if kk != "stl_path"}
                           for k, v in exported.items()},
        "unmatched_parts": unmatched,
        "notes": "体积单位 mm³, 质心单位 mm, 需转换为 m/kg 单位制用于 MJCF/URDF",
    }
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)
    print(f"[INFO] 元数据已保存: {meta_path}")

    return exported, metadata

def create_fallback_primitives(output_dir):
    """
    如果 STEP 导出失败，使用 Unity Prefab 中的变换信息
    创建基于几何基元的替代模型
    """
    import json

    os.makedirs(output_dir, exist_ok=True)

    # 从 Unity Prefab 分析得到的几何参数
    # 位置来自 Transform 层级，尺寸来自 BoxCollider / 经验估算
    parts_info = {
        "base_link": {
            "description": "底座",
            "mass_kg": 0.15,
            "dimensions": {"x": 0.07, "y": 0.03, "z": 0.064},  # box approx
            "com": [0, 0, 0],
            "inertia": "auto",
        },
        "body": {
            "description": "躯干 (含腰部旋转)",
            "mass_kg": 0.12,
            "dimensions": {"x": 0.05, "y": 0.07, "z": 0.05},
            "com": [0, 0.035, 0],
            "inertia": "auto",
        },
        "head": {
            "description": "头部 (含 LCD 屏幕)",
            "mass_kg": 0.08,
            "dimensions": {"x": 0.04, "y": 0.04, "z": 0.035},
            "com": [0, 0.02, 0],
            "inertia": "auto",
        },
        "left_shoulder": {
            "description": "左肩",
            "mass_kg": 0.03,
            "dimensions": {"x": 0.02, "y": 0.03, "z": 0.02},
            "com": [0, 0, 0],
            "inertia": "auto",
        },
        "left_arm": {
            "description": "左臂 (末端)",
            "mass_kg": 0.05,
            "dimensions": {"x": 0.015, "y": 0.06, "z": 0.015},
            "com": [0, -0.03, 0],
            "inertia": "auto",
        },
        "right_shoulder": {
            "description": "右肩",
            "mass_kg": 0.03,
            "dimensions": {"x": 0.02, "y": 0.03, "z": 0.02},
            "com": [0, 0, 0],
            "inertia": "auto",
        },
        "right_arm": {
            "description": "右臂 (末端)",
            "mass_kg": 0.05,
            "dimensions": {"x": 0.015, "y": 0.06, "z": 0.015},
            "com": [0, -0.03, 0],
            "inertia": "auto",
        },
    }

    meta_path = os.path.join(output_dir, "fallback_part_metadata.json")
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(parts_info, f, indent=2, ensure_ascii=False)

    print(f"[INFO] Fallback 几何元数据已保存: {meta_path}")
    print("[WARNING] 使用几何基元替代 STL 网格，建议后续用实际 STL 替换")

    return parts_info


def main():
    step_path = os.path.expanduser(
        "/home/j6m/code/github/ElectronBot/4.CAD-Model/ElectronBot.step"
    )
    output_dir = os.path.expanduser(
        "/home/j6m/code/github/ElectronBot_SIM/simulation/electronbot_description/meshes"
    )

    if not os.path.exists(step_path):
        print(f"[ERROR] STEP 文件不存在: {step_path}")
        print("[INFO] 使用几何基元方案作为替代...")
        create_fallback_primitives(output_dir)
        return

    # 尝试使用 FreeCAD
    if not setup_freecad():
        print("[WARNING] FreeCAD Python API 不可用")
        print("[INFO] 尝试安装: sudo apt-get install freecad")
        print("[INFO] 使用几何基元方案作为替代...")
        create_fallback_primitives(output_dir)
        return

    print("[INFO] FreeCAD 已加载，开始解析 STEP...")
    try:
        doc, objects = find_body_parts(step_path)
        exported, metadata = export_stl_meshes(doc, objects, output_dir)

        print(f"\n[SUCCESS] 成功导出 {len(exported)} 个 STL 文件")
        print(f"[INFO] 未匹配的零件: {len(metadata['unmatched_parts'])}")

    except Exception as e:
        print(f"[ERROR] STEP 处理失败: {e}")
        import traceback
        traceback.print_exc()
        print("\n[INFO] 使用几何基元方案作为替代...")
        create_fallback_primitives(output_dir)


if __name__ == "__main__":
    main()
