#!/usr/bin/env python3
"""benchmark.py — MuJoCo 模型 FPS 性能基准测试。

关键指标:
  - fps_mesh:      inline mesh 版 FPS (< 300 告警)
  - mesh_file_size_kb: mesh 文件大小 (> 2048 KB 告警, ~2MB 单文件)
  - total_mass_g:  总质量 (140-180g 合理)

命令行入口:
  python scripts/benchmark.py [assets/mjcf/scene_mesh.xml] [--steps 5000]
  python scripts/benchmark.py --json             # 输出 JSON (供 CI 采集)

依赖: mujoco>=3.0, numpy>=1.24
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List

logger = logging.getLogger("benchmark")

# 告警阈值
FPS_MESH_MIN = 300         # inline mesh 版 FPS 下限
MESH_FILE_MAX_KB = 2048    # mesh XML 文件大小上限 (~2MB)
MASS_MIN_G = 140           # 总质量下限
MASS_MAX_G = 180           # 总质量上限

DEFAULT_STEPS = 5000       # 默认仿真步数


def run_fps_benchmark(model_path: str, steps: int = DEFAULT_STEPS) -> Dict:
    """对指定 MJCF 模型运行 FPS 基准测试。"""
    import mujoco
    import numpy as np

    p = Path(model_path)
    logger.info("加载模型: %s", p)
    model = mujoco.MjModel.from_xml_path(str(p))
    data = mujoco.MjData(model)

    for _ in range(100):
        mujoco.mj_step(model, data)
    mujoco.mj_resetData(model, data)

    logger.info("开始 %d 步仿真...", steps)
    start = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - start

    fps = steps / elapsed if elapsed > 0 else 0.0
    timestep_ms = (elapsed / steps) * 1000 if steps > 0 else 0.0
    total_mass_g = float(model.body_mass.sum()) * 1000

    result = {
        "model": str(p),
        "steps": steps,
        "elapsed_s": round(elapsed, 4),
        "fps": round(fps, 1),
        "nbody": int(model.nbody),
        "njoint": int(model.njnt),
        "ngeom": int(model.ngeom),
        "nu": int(model.nu),
        "total_mass_g": round(total_mass_g, 1),
        "timestep_ms": round(timestep_ms, 3),
    }
    logger.info(
        "fps=%.1f  steps=%d  elapsed=%.2fs  mass=%.1fg",
        fps, steps, elapsed, total_mass_g,
    )
    return result


def check_file_size(file_path: str) -> Dict:
    """检查文件大小是否超标。"""
    p = Path(file_path)
    if not p.exists():
        return {"file": str(p), "size_kb": 0, "ok": False, "exists": False}

    size_kb = p.stat().st_size / 1024
    return {
        "file": str(p),
        "size_kb": round(size_kb, 1),
        "ok": size_kb <= MESH_FILE_MAX_KB,
        "exists": True,
    }


def evaluate_alerts(result: Dict) -> List[str]:
    """根据阈值评估告警。"""
    alerts: List[str] = []

    if result["fps"] < FPS_MESH_MIN:
        alerts.append(
            f"fps_mesh={result['fps']:.0f} < {FPS_MESH_MIN} (告警)"
        )

    mass = result["total_mass_g"]
    if mass < MASS_MIN_G or mass > MASS_MAX_G:
        alerts.append(
            f"total_mass_g={mass:.1f} 超出合理范围 [{MASS_MIN_G}, {MASS_MAX_G}]"
        )

    return alerts


def main():
    parser = argparse.ArgumentParser(
        description="ElectronBot MJCF 模型 FPS 性能基准测试"
    )
    parser.add_argument(
        "model", nargs="?", default="assets/mjcf/scene_mesh.xml",
        help="MJCF 模型路径 (默认 scene_mesh.xml)",
    )
    parser.add_argument(
        "--steps", type=int, default=DEFAULT_STEPS,
        help=f"仿真步数 (默认 {DEFAULT_STEPS})",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="输出 JSON 格式 (供 CI/监控采集)",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s][%(levelname)s][%(name)s] %(message)s",
    )

    results: List[Dict] = []
    all_alerts: List[str] = []

    r = run_fps_benchmark(args.model, args.steps)
    r["type"] = "mesh"
    results.append(r)
    alerts = evaluate_alerts(r)
    all_alerts.extend(alerts)
    for a in alerts:
        logger.warning("⚠️ %s", a)

    # 检查 mesh 文件大小
    mesh_xml = "assets/mjcf/electronbot_mesh.xml"
    size_info = check_file_size(mesh_xml)
    if size_info.get("exists") and not size_info["ok"]:
        all_alerts.append(
            f"mesh_file_size_kb={size_info['size_kb']:.0f} > {MESH_FILE_MAX_KB} (告警)"
        )
        logger.warning("⚠️ %s", all_alerts[-1])

    # 输出
    if args.json:
        output = {
            "results": results,
            "alerts": all_alerts,
            "thresholds": {
                "fps_mesh_min": FPS_MESH_MIN,
                "mesh_file_max_kb": MESH_FILE_MAX_KB,
                "mass_min_g": MASS_MIN_G,
                "mass_max_g": MASS_MAX_G,
            },
        }
        print(json.dumps(output, indent=2, ensure_ascii=False))
    else:
        print("\n" + "=" * 60)
        print("Benchmark Results")
        print("=" * 60)
        for r in results:
            print(f"\n[{r['type'].upper()}] {r['model']}")
            print(f"  FPS:        {r['fps']:>8.1f}")
            print(f"  Steps:      {r['steps']:>8d}")
            print(f"  Elapsed:    {r['elapsed_s']:>8.2f} s")
            print(f"  Timestep:   {r['timestep_ms']:>8.3f} ms")
            print(f"  Bodies:     {r['nbody']:>8d}")
            print(f"  Joints:     {r['njoint']:>8d}")
            print(f"  Geoms:      {r['ngeom']:>8d}")
            print(f"  Actuators:  {r['nu']:>8d}")
            print(f"  Mass:       {r['total_mass_g']:>8.1f} g")

        if all_alerts:
            print("\n" + "-" * 60)
            print("⚠️ 告警:")
            for a in all_alerts:
                print(f"  - {a}")
        else:
            print("\n✅ 无告警, 所有指标在合理范围内")
        print("=" * 60)

    sys.exit(1 if all_alerts else 0)


if __name__ == "__main__":
    main()
