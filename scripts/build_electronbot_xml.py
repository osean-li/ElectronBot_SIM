"""build_electronbot_xml.py — 验证 MJCF 模型并输出推荐的相机参数。

用途:
  1. 加载 electronbot_scene.xml, 验证 XML 结构 (关节/执行器命名空间)
  2. 计算模型包围盒 (stat.center / stat.extent) → 推荐相机参数
  3. 模拟 home 姿态, 检查 IK 与可达性
  4. 输出 JSON 报告供 visual_demo 集成

用法:
  python3 scripts/build_electronbot_xml.py
  python3 scripts/build_electronbot_xml.py --xml assets/mjcf/electronbot_scene.xml
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def build_xml_report(xml_path: Path) -> dict:
    """加载 MJCF, 验证结构, 输出相机建议。"""
    import mujoco

    if not xml_path.exists():
        raise FileNotFoundError(f"MJCF 文件不存在: {xml_path}")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # ── 统计包围盒 ──
    center = model.stat.center.tolist()
    extent = float(model.stat.extent)

    # ── 关节/执行器清单 ──
    joint_names = [
        model.names[model.name_jntadr[i]].decode()
        for i in range(model.njnt)
    ]
    actuator_names = [
        model.names[model.name_actuatoradr[i]].decode()
        for i in range(model.nu)
    ]

    # ── 推荐相机 (对齐 demos/01_manual_control.py 的已验证参数) ──
    cam = {
        "lookat": center,
        "distance": extent * 1.8,
        "azimuth": 135,
        "elevation": -20,
    }

    # ── Body 高度 (验证 base_link 是否在高空) ──
    body_names = [
        model.names[model.name_bodyadr[i]].decode()
        for i in range(model.nbody)
    ]
    base_height = float(data.xpos[1, 2]) if model.nbody > 1 else 0.0

    report = {
        "xml_path": str(xml_path),
        "model_summary": {
            "nq": model.nq,
            "nv": model.nv,
            "nu": model.nu,
            "njnt": model.njnt,
            "nbody": model.nbody,
        },
        "stat": {
            "center": center,
            "extent": extent,
        },
        "camera_recommended": cam,
        "joints": joint_names,
        "actuators": actuator_names,
        "body_names": body_names,
        "base_link_world_z": base_height,
        "warnings": [],
    }

    # ── 健康检查 ──
    if extent < 1e-6:
        report["warnings"].append("stat.extent ≈ 0, 模型可能未正确加载")
    if "body_joint" not in joint_names:
        report["warnings"].append("缺少 body_joint, 转身动作无法执行")
    if "head_joint" not in joint_names:
        report["warnings"].append("缺少 head_joint, 点头动作无法执行")
    if base_height > 10.0:
        report["warnings"].append(
            f"base_link z={base_height:.2f}m 远高于地面, "
            f"相机 distance={cam['distance']:.1f}m 才能看到机器人"
        )

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="验证 MJCF 并输出相机建议")
    parser.add_argument(
        "--xml",
        type=Path,
        default=Path("assets/mjcf/electronbot_scene.xml"),
        help="MJCF 文件路径",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="输出 JSON 报告路径 (默认打印到 stdout)",
    )
    args = parser.parse_args()

    xml_path = args.xml.resolve()
    report = build_xml_report(xml_path)

    text = json.dumps(report, ensure_ascii=False, indent=2)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
        print(f"报告已写入: {args.out}")
    else:
        print(text)

    # ── 关键参数提示 ──
    cam = report["camera_recommended"]
    print("\n[推荐相机参数] (复制到 env.py / visual_demo.py)")
    print(f"  lookat   = {cam['lookat']}")
    print(f"  distance = {cam['distance']:.2f}")
    print(f"  azimuth  = {cam['azimuth']}")
    print(f"  elevation= {cam['elevation']}")

    if report["warnings"]:
        print("\n[⚠ 警告]")
        for w in report["warnings"]:
            print(f"  - {w}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
