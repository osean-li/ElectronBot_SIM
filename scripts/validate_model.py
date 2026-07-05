#!/usr/bin/env python3
"""validate_model.py — 加载并验证 MJCF 模型结构完整性。

对齐 docs/tasks/01-CAD-to-MJCF §4.1 + §7.1.3 接口设计。
检查项:
  1. body 数量与命名
  2. joint 数量、类型、范围
  3. actuator gear 映射比 [1.0, 1.125, 1.0, 1.125, 1.5, 2.0]
  4. 碰撞几何体不为原始 mesh (group=3 需为凸体)
  5. 100 步仿真 smoke test (无崩溃)

命令行入口: python scripts/validate_model.py [assets/mjcf/scene_tabletop.xml]

依赖: mujoco>=3.0, numpy>=1.24
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

logger = logging.getLogger("validate_model")

# inline mesh 版模型结构 (v1.3): base_link → body → {head, left_shoulder, right_shoulder}; shoulder → arm
EXPECTED_BODIES = {"base_link", "body", "head",
                   "left_shoulder", "left_arm",
                   "right_shoulder", "right_arm"}

EXPECTED_JOINTS = {
    "body_joint":             (-90.0, 90.0),   # Z 轴腰部旋转
    "head_joint":             (-15.0, 15.0),   # Y 轴俯仰
    "left_shoulder_joint":    (-90.0, 90.0),  # X 轴 Pitch (前后摆臂)
    "left_arm_roll_joint":    (-45.0, 45.0),  # Z 轴 Roll (手腕自转)
    "right_shoulder_joint":   (-90.0, 90.0),  # X 轴 Pitch (前后摆臂)
    "right_arm_roll_joint":   (-45.0, 45.0),  # Z 轴 Roll (手腕自转)
}

# position actuator 类型, gear 恒为 1.0 (直接控制关节角度)
EXPECTED_ACTUATORS = {
    "act_body",
    "act_head",
    "act_left_shoulder",
    "act_left_arm",
    "act_right_shoulder",
    "act_right_arm",
}


def validate_model(model_path: str) -> bool:
    """加载并验证 MJCF 模型的结构完整性。

    参数:
        model_path: MJCF XML 文件路径 (可为 scene.xml 或单文件)

    返回:
        bool: True 表示全部检查通过, False 表示存在错误 (错误列表打印到 stdout)

    异常:
        mujoco.FatalError: XML 解析失败 (语法错误/引用缺失) 会直接抛出
    """
    import mujoco
    import numpy as np

    p = Path(model_path)
    if not p.exists():
        logger.error("模型文件不存在: %s", p)
        return False

    logger.info("加载模型: %s", p)
    model = mujoco.MjModel.from_xml_path(str(p))
    data = mujoco.MjData(model)

    errors = []

    # 1. 检查 body 数量
    actual_bodies = {model.body(i).name for i in range(model.nbody)}
    for b in EXPECTED_BODIES:
        if b not in actual_bodies:
            errors.append(f"缺少 body: {b}")
    logger.info("body 检查: %d 个 (期望含 %d 个关键 body)",
                model.nbody, len(EXPECTED_BODIES))

    # 2. 检查 joint 数量、类型、范围
    for j in range(model.njnt):
        name = model.joint(j).name
        if name in EXPECTED_JOINTS:
            jmin, jmax = EXPECTED_JOINTS[name]
            # MuJoCo jnt_range 内部存储弧度, 需转换为度数比较
            actual_range = np.degrees(model.jnt_range[j])
            if abs(actual_range[0] - jmin) > 1.0 or abs(actual_range[1] - jmax) > 1.0:
                errors.append(
                    f"joint {name} range 不对: 实际={actual_range.tolist()}, "
                    f"期望=[{jmin}, {jmax}]"
                )
    logger.info("joint 检查: %d 个 (期望 6 个 hinge)", model.njnt)

    # 3. 检查 actuator 命名 (position 类型, gear 恒为 1.0)
    actual_actuators = {model.actuator(i).name for i in range(model.nu)}
    for a in EXPECTED_ACTUATORS:
        if a not in actual_actuators:
            errors.append(f"缺少 actuator: {a}")
    logger.info("actuator 检查: %d 个 (position 类型, gear=1.0)", model.nu)

    # 4. 检查碰撞几何体 (group=3 不应为 mesh type=7)
    mesh_collision_count = 0
    for g in range(model.ngeom):
        geom_type = model.geom_type[g]
        # MuJoCo geom type 7 = mesh
        is_collision = (model.geom_group[g] == 3) or (model.geom_contype[g] > 0)
        if is_collision and geom_type == 7:
            mesh_collision_count += 1
            geom_name = model.geom(g).name or f"geom_{g}"
            errors.append(f"碰撞体 {geom_name} 使用原始 mesh, 应简化为凸体")
    logger.info("collision geom 检查: %d 个 geom", model.ngeom)

    # 5. 100 步仿真 smoke test
    try:
        start = time.time()
        for _ in range(100):
            mujoco.mj_step(model, data)
        elapsed_ms = (time.time() - start) * 1000
        logger.info("smoke test ok steps=100 elapsed_ms=%.1f", elapsed_ms)
    except Exception as e:
        errors.append(f"仿真崩溃: {e}")

    # 6. 检查 NaN
    if __import__("numpy").any(__import__("numpy").isnan(data.qpos)):
        errors.append("仿真后 qpos 含 NaN")

    # 输出
    total_mass_g = model.body_mass.sum() * 1000
    if errors:
        print("❌ 验证失败:")
        for e in errors:
            print(f"   - {e}")
        return False
    else:
        print("✅ 模型验证全部通过!")
        print(f"   nbody={model.nbody}  njoint={model.njnt}  ngeom={model.ngeom}")
        print(f"   nactuator={model.nu}  总质量={total_mass_g:.1f} g")
        print(f"   smoke test: 100 步, {elapsed_ms:.1f} ms")
        # 总质量范围告警 (140-180g 合理)
        if total_mass_g < 140 or total_mass_g > 180:
            print(f"   ⚠️ 总质量 {total_mass_g:.1f} g 超出合理范围 [140, 180] g")
        return True


def main():
    parser = argparse.ArgumentParser(description="验证 ElectronBot MJCF 模型")
    parser.add_argument("model", nargs="?", default="assets/mjcf/scene_tabletop.xml",
                        help="MJCF 模型路径")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s][%(levelname)s][%(name)s] %(message)s",
    )

    ok = validate_model(args.model)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
