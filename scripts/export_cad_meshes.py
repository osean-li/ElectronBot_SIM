#!/usr/bin/env python3
"""export_cad_meshes.py — FreeCAD 自动化 STL 导出脚本。

对齐 docs/tasks/01-CAD-to-MJCF §3 Step 1 + §5 调试命令速查。
从 ElectronBot.step (或 .FCStd) 装配体中提取 24 个零件, 导出为 STL 文件
到 assets/meshes/ 目录。

两种使用模式:
  1. FreeCAD 内部模式 (推荐): 在 FreeCAD GUI 中 Macro → Run Script
     freecad --run scripts/export_cad_meshes.py ElectronBot.step
  2. headless 模式: freecadcmd scripts/export_cad_meshes.py ElectronBot.step

零件分组对照 §2.1:
  底座 (base)    2 件
  身体 (torso)   3 件
  头部 (head)    5 件
  左臂 (left)    6 件
  右臂 (right)   7 件
  合计 24 件 (注: 头部 5 = 3 已标注 + 2 自动计算)

依赖: FreeCAD 0.21+ (freecad / freecadcmd)
输出: assets/meshes/{part_name}.stl (mm 单位, 每个 < 500KB)
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("export_cad_meshes")

# ============================================================
# 24 零件命名表 (对齐 §2.1 五大运动组)
# key = FreeCAD 中零件标签 (Part__FeatureNNN)
# value = 导出 STL 文件名 (不含扩展名)
# ============================================================
# 注意: 实际零件标签取决于 CAD 文件, 此处为已知映射。
# 未在表中的零件按 "part_{index}" 自动命名。
KNOWN_PART_MAP: Dict[str, str] = {
    # 底座 (base) — 2 件
    "Part__Feature043": "base_top",
    "Part__Feature044": "base_bottom",
    # 身体 (torso) — 3 件
    "Part__Feature034": "torso_center",
    "Part__Feature035": "torso_right",
    "Part__Feature036": "torso_left",
    # 头部 (head) — 5 件 (3 已知 + 2 自动)
    "Part__Feature037": "head_front",
    "Part__Feature038": "head_top",
    "Part__Feature039": "head_shell",
}

# STL 导出参数 (对齐 §10.1 MAX_STL_SIZE_KB=500)
STL_MESH_DEVIATION = 0.1    # mm, 网格偏差 (越小越精细)
STL_MESH_ANGULAR = 0.5      # 度, 角度偏差
MAX_STL_SIZE_KB = 500       # 单文件大小上限 (§10.1)


def get_part_name(obj_label: str, index: int) -> str:
    """根据 FreeCAD 零件标签获取规范文件名, 未知零件自动命名。"""
    if obj_label in KNOWN_PART_MAP:
        return KNOWN_PART_MAP[obj_label]
    # 清理标签中的非法字符
    clean = obj_label.replace("__", "_").replace(" ", "_").lower()
    return f"part_{index:02d}_{clean}" if clean else f"part_{index:02d}"


def export_stl_from_freecad(cad_path: str, output_dir: str) -> List[str]:
    """在 FreeCAD 环境中执行: 加载 CAD → 遍历零件 → 导出 STL。

    参数:
        cad_path:   .step 或 .FCStd 文件路径
        output_dir: STL 输出目录

    返回:
        list[str]: 成功导出的 STL 文件路径列表

    异常:
        ImportError: FreeCAD 模块不可用 (不在 FreeCAD 环境中)
        RuntimeError: CAD 文件加载失败
    """
    import FreeCAD       # type: ignore
    import Mesh          # type: ignore

    cad_path = str(Path(cad_path).resolve())
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("加载 CAD 文件: %s", cad_path)
    is_fcstd = str(cad_path).lower().endswith(".fcstd")
    try:
        if is_fcstd:
            # .FCStd 是 FreeCAD 原生格式 → openDocument
            doc = FreeCAD.openDocument(cad_path)
        else:
            # .step/.iges 等 → Import 加载
            doc = FreeCAD.newDocument("electronbot_export")
            try:
                import ImportGui  # type: ignore
                ImportGui.insert(cad_path, doc.Name)
            except ImportError:
                import Import  # type: ignore
                Import.insert(cad_path, doc.Name)
        FreeCAD.ActiveDocument = doc
        doc.recompute()

        exported: List[str] = []
        index = 0

        exported: List[str] = []
        index = 0

        for obj in doc.Objects:
            # 只导出有形状 (Shape) 的零件
            if not hasattr(obj, "Shape") or obj.Shape is None:
                continue
            if obj.Shape.isNull():
                continue

            part_name = get_part_name(obj.Label, index)
            stl_path = output_dir / f"{part_name}.stl"
            index += 1

            try:
                # 导出 STL (mm 单位, 不缩放)
                Mesh.export(
                    [obj],
                    str(stl_path),
                    STL_MESH_DEVIATION,
                    STL_MESH_ANGULAR,
                )

                size_kb = stl_path.stat().st_size / 1024
                if size_kb > MAX_STL_SIZE_KB:
                    logger.warning(
                        "STL 过大 (%.0f KB > %d KB): %s — 考虑增大 deviation",
                        size_kb, MAX_STL_SIZE_KB, stl_path.name,
                    )

                logger.info(
                    "导出 %s (%.1f KB, label=%s)",
                    stl_path.name, size_kb, obj.Label,
                )
                exported.append(str(stl_path))
            except Exception as e:
                logger.error("导出失败 %s: %s", obj.Label, e)

        logger.info("完成: %d 个零件导出到 %s", len(exported), output_dir)
        return exported
    finally:
        FreeCAD.closeDocument(doc.Name)


def list_parts_info(cad_path: str) -> List[Tuple[str, float]]:
    """列出 CAD 文件中所有零件及其体积 (mm³)。

    返回:
        list[tuple[str, float]]: (零件名, 体积 mm³)
    """
    import FreeCAD       # type: ignore

    doc = FreeCAD.newDocument("electronbot_info")
    try:
        try:
            import ImportGui  # type: ignore
            ImportGui.insert(str(Path(cad_path).resolve()), doc.Name)
        except ImportError:
            import Import  # type: ignore
            Import.insert(str(Path(cad_path).resolve()), doc.Name)
        FreeCAD.ActiveDocument = doc
        doc.recompute()

        parts: List[Tuple[str, float]] = []
        idx = 0
        for obj in doc.Objects:
            if not hasattr(obj, "Shape") or obj.Shape is None:
                continue
            if obj.Shape.isNull():
                continue
            name = get_part_name(obj.Label, idx)
            volume = float(obj.Shape.Volume)  # mm³
            parts.append((name, volume))
            idx += 1

        return parts
    finally:
        FreeCAD.closeDocument(doc.Name)


def main():
    # FreeCAD 可能将自身参数混入 sys.argv, 过滤掉非本脚本的参数
    import sys as _sys
    _my_argv = [a for a in _sys.argv if not a.startswith("--render")]
    if "--" in _my_argv:
        _my_argv = _my_argv[_my_argv.index("--") + 1:]
    _sys.argv = _my_argv[:1] + _my_argv[1:]  # 保留脚本名

    parser = argparse.ArgumentParser(
        description="从 ElectronBot CAD 文件导出 24 个零件的 STL"
    )
    parser.add_argument(
        "cad_file",
        nargs="?",
        default="../xiaozhi-electronbot-docs/docs/cad/ElectronBot.step",
        help="CAD 文件路径 (.step / .FCStd)",
    )
    parser.add_argument(
        "--output", "-o",
        default="assets/meshes",
        help="STL 输出目录 (默认 assets/meshes)",
    )
    parser.add_argument(
        "--list-only", action="store_true",
        help="仅列出零件信息, 不导出 STL",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s][%(levelname)s][%(name)s] %(message)s",
    )

    # 检查 CAD 文件
    cad_path = Path(args.cad_file)
    if not cad_path.exists():
        logger.error("CAD 文件不存在: %s", cad_path)
        logger.info("常见路径: xiaozhi-electronbot-docs/docs/cad/ElectronBot.step")
        sys.exit(2)

    # 检查是否在 FreeCAD 环境中
    try:
        import FreeCAD  # type: ignore  # noqa: F401
    except ImportError:
        logger.error("此脚本必须在 FreeCAD 环境中运行:")
        logger.info("  GUI 模式:   freecad scripts/export_cad_meshes.py <cad_file>")
        logger.info("  命令行模式: freecadcmd scripts/export_cad_meshes.py <cad_file>")
        sys.exit(3)

    if args.list_only:
        parts = list_parts_info(str(cad_path))
        print(f"\n{'零件名':<20s} {'体积 (mm³)':>12s} {'质量 (g)':>10s}")
        print("-" * 44)
        total_vol = 0.0
        for name, vol in parts:
            mass_g = vol / 1000 * 1.24  # PLA 密度
            total_vol += vol
            print(f"{name:<20s} {vol:>12.1f} {mass_g:>10.2f}")
        print("-" * 44)
        print(f"{'合计':<20s} {total_vol:>12.1f} {total_vol/1000*1.24:>10.2f}")
        print(f"\n零件数: {len(parts)}")
        sys.exit(0)

    exported = export_stl_from_freecad(str(cad_path), args.output)
    if len(exported) < 24:
        logger.warning("导出 %d 个零件 (期望 24 个)", len(exported))
    print(f"\n✅ 导出完成: {len(exported)} 个 STL → {args.output}/")


if __name__ == "__main__":
    main()
