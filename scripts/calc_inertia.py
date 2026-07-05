#!/usr/bin/env python3
"""calc_inertia.py — 基于 STL 几何体与均质密度假设计算质量、质心、惯性张量。

对齐 docs/tasks/01-CAD-to-MJCF §7.1.2 接口设计。
命令行入口: python scripts/calc_inertia.py [--density 1.24e-6]
输出 markdown 表格供直接粘贴到 MJCF。

依赖: trimesh>=4.0, numpy>=1.24
"""
from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import numpy as np

logger = logging.getLogger("calc_inertia")

# PLA 密度: 1.24 g/cm³ = 1.24e-6 kg/mm³
PLA_DENSITY = 1.24e-6

# 24 个零件清单 (name → stl 相对路径)。实际项目从 assets/meshes 加载。
DEFAULT_MESHES: Dict[str, str] = {
    "base_top": "assets/meshes/base_top.stl",
    "base_bottom": "assets/meshes/base_bottom.stl",
    "torso_center": "assets/meshes/torso_center.stl",
    "torso_right": "assets/meshes/torso_right.stl",
    "torso_left": "assets/meshes/torso_left.stl",
    "head_front": "assets/meshes/head_front.stl",
    "head_top": "assets/meshes/head_top.stl",
    "head_shell": "assets/meshes/head_shell.stl",
    # ... 其余 16 个零件按需补充
}


@dataclass
class InertiaResult:
    """单个零件的惯性计算结果 (对齐 §8.1.2)"""

    name: str
    mass: float  # kg
    com: np.ndarray  # (3,) mm
    inertia_matrix: np.ndarray  # (3, 3) kg·mm², 关于质心


def calculate_inertia(stl_path: str, density: float = PLA_DENSITY) -> InertiaResult:
    """基于 STL 几何体与均质密度假设计算质量、质心、惯性张量。

    参数:
        stl_path: STL 文件路径 (单位: mm)
        density:  密度 (kg/mm³), 默认 PLA_DENSITY=1.24e-6 (即 1.24 g/cm³)

    返回:
        InertiaResult: 含 mass/com/inertia_matrix 的数据类

    异常:
        FileNotFoundError: STL 文件不存在
        ZeroDivisionError: 退化几何体 (体积为 0) 导致惯性张量不可计算
    """
    import trimesh

    p = Path(stl_path)
    if not p.exists():
        raise FileNotFoundError(f"STL 文件不存在: {stl_path}")

    mesh = trimesh.load(str(p))
    if mesh.volume <= 0:
        raise ZeroDivisionError(f"退化几何体 (体积为 0): {stl_path}")

    mass = mesh.volume * density  # kg
    com = np.asarray(mesh.center_mass, dtype=np.float64)  # mm
    # moment_inertia 单位 mm^5, 乘密度得 kg·mm²
    inertia = np.asarray(mesh.moment_inertia, dtype=np.float64) * density

    # 数值稳定性: 检测负定惯性矩阵 (数值误差), 取绝对值
    eigvals = np.linalg.eigvalsh(inertia)
    if np.any(eigvals < 0):
        logger.warning("惯性矩阵含负特征值 (%s), 取绝对值并 clamp", p.name)
        inertia = np.abs(inertia)
        eigvals_clamped = np.clip(np.linalg.eigvalsh(inertia), 1e-12, None)
        # 用特征值重建对称正定矩阵
        _, eigvecs = np.linalg.eigh(inertia)
        inertia = (eigvecs * eigvals_clamped) @ eigvecs.T

    return InertiaResult(
        name=p.stem,
        mass=float(mass),
        com=com,
        inertia_matrix=inertia,
    )


def batch_calculate(meshes: Dict[str, str], density: float = PLA_DENSITY) -> list:
    """批量计算, 单个失败时用占位惯性, 不阻塞下游流程 (对齐 §9.2.1)。"""
    results = []
    fail_count = 0
    for name, path in meshes.items():
        try:
            r = calculate_inertia(path, density)
            results.append(r)
            logger.info(
                "inertia computed name=%s mass=%.4f g com=%s",
                name, r.mass * 1000, np.round(r.com, 2).tolist(),
            )
        except (FileNotFoundError, ValueError, ZeroDivisionError) as e:
            fail_count += 1
            logger.warning("跳过 %s: %s", name, e)
            results.append(InertiaResult(
                name=name, mass=1e-6,
                com=np.zeros(3),
                inertia_matrix=np.eye(3) * 1e-9,
            ))

    # 超过 50% 失败 → 退出码 3
    if fail_count > len(meshes) // 2:
        logger.error("失败超过 50%% (%d/%d), 退出码 3", fail_count, len(meshes))
        sys.exit(3)
    return results


def to_markdown_table(results: list) -> str:
    """输出 markdown 表格供直接粘贴到 MJCF。"""
    lines = [
        "| 零件名 | 质量 (g) | 质心 x (mm) | 质心 y (mm) | 质心 z (mm) | Ixx | Iyy | Izz |",
        "|--------|---------:|------------:|------------:|------------:|----:|----:|----:|",
    ]
    for r in results:
        lines.append(
            f"| {r.name} | {r.mass*1000:.2f} | {r.com[0]:.2f} | {r.com[1]:.2f} | "
            f"{r.com[2]:.2f} | {r.inertia_matrix[0,0]:.2e} | "
            f"{r.inertia_matrix[1,1]:.2e} | {r.inertia_matrix[2,2]:.2e} |"
        )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="计算 STL 零件的惯性参数")
    parser.add_argument("--density", type=float, default=PLA_DENSITY,
                        help="密度 (kg/mm³), 默认 PLA 1.24e-6")
    parser.add_argument("--mesh-dir", default="assets/meshes",
                        help="STL 输入目录")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s][%(levelname)s][%(name)s] %(message)s",
    )

    # 动态扫描 mesh 目录
    mesh_dir = Path(args.mesh_dir)
    meshes = {}
    if mesh_dir.exists():
        for stl in sorted(mesh_dir.glob("*.stl")):
            meshes[stl.stem] = str(stl)
    if not meshes:
        logger.warning("mesh 目录为空或不存在 (%s), 使用 DEFAULT_MESHES", mesh_dir)
        meshes = DEFAULT_MESHES

    results = batch_calculate(meshes, args.density)
    print("\n" + to_markdown_table(results) + "\n")
    total_mass_g = sum(r.mass for r in results) * 1000
    print(f"总质量 (PLA 部分): {total_mass_g:.1f} g")
    print(f"  + 附加电子件 60g → 完整机器人 ≈ {total_mass_g + 60:.1f} g")


if __name__ == "__main__":
    main()
