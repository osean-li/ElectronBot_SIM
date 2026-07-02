#!/usr/bin/env python3
"""
验证关节类型并尝试修复
"""

import sys
import numpy as np
from pathlib import Path

_project = Path(__file__).parent.parent
sys.path.insert(0, str(_project))

import mujoco


def main():
    xml_path = str(_project / "electronbot_mujoco" / "assets" / "electronbot_inline.xml")

    print("="*70)
    print("  关节类型验证")
    print("="*70)

    # 读取原始XML并打印关节定义
    print("\n📄 XML中的关节定义:")
    with open(xml_path, 'r') as f:
        for i, line in enumerate(f, 1):
            if '<joint' in line.lower() and 'name=' in line:
                print(f"  行{i}: {line.strip()}")

    # 加载模型并检查类型
    print("\n🔍 MuJoCo解析后的关节类型 (原始数值):")
    model = mujoco.MjModel.from_xml_path(xml_path)

    type_map = {mujoco.mjtJoint.mjJNT_HINGE: 'HINGE (2)',
                mujoco.mjtJoint.mjJNT_BALL: 'BALL (3)',
                mujoco.mjtJoint.mjJNT_SLIDE: 'SLIDE (1)',
                mujoco.mjtJoint.mjJNT_FREE: 'FREE (0)'}

    for jid in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        jtype_raw = model.jnt_type[jid]
        jtype_name = type_map.get(jtype_raw, f'UNKNOWN({jtype_raw})')
        print(f"  {jid}: {jname:<25s} → raw={jtype_raw} ({jtype_name})")

    # 检查自由度详情
    print(f"\n📊 自由度分析:")
    print(f"   nq (位置数) = {model.nq}")
    print(f"   nv (速度数) = {model.nv}")
    print(f"   njnt (关节数) = {model.njnt}")

    if model.nq == model.nv == 6:
        print(f"\n  ✅ nq=nv=6，说明实际上是1D关节（可能是显示问题）")
        print(f"     BALL类型通常有3个自由度，但这里只有6个总自由度")
        print(f"     → 这些关节实际上可能是特殊的受限BALL或HINGE")
    else:
        print(f"\n  ❌ 自由度不匹配！")


if __name__ == "__main__":
    main()