#!/usr/bin/env python3
"""渲染诊断 v2：加强光照，生成对比 GIF，并保存图片。"""
from __future__ import annotations

import os
import sys
import logging
from pathlib import Path

os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("diagnose2")

PROJECT = Path(__file__).resolve().parent.parent


def render_scene(model, data, lookat, distance, azimuth, elevation,
                 width=480, height=480):
    """渲染场景一帧。"""
    renderer = mujoco.Renderer(model, width, height)

    # 加强光照
    model.vis.headlight.active = True
    model.vis.headlight.ambient[:] = [0.3, 0.3, 0.35]
    model.vis.headlight.diffuse[:] = [0.6, 0.6, 0.65]

    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation

    opt = mujoco.MjvOption()
    mujoco.mjv_updateScene(
        model, data, opt, mujoco.MjvPerturb(),
        cam, mujoco.mjtCatBit.mjCAT_ALL, renderer.scene,
    )
    frame = renderer.render()
    renderer.close()
    return frame


def analyze_frame(frame, label):
    h, w, _ = frame.shape
    total = h * w
    max_ch = np.max(frame, axis=2)
    min_ch = np.min(frame, axis=2)
    gray_mask = (max_ch - min_ch) < 15
    gray_ratio = float(np.mean(gray_mask))
    r_std, g_std, b_std = float(frame[..., 0].std()), float(frame[..., 1].std()), float(frame[..., 2].std())
    overall_std = (r_std + g_std + b_std) / 3
    r_mean, g_mean, b_mean = float(frame[..., 0].mean()), float(frame[..., 1].mean()), float(frame[..., 2].mean())
    dark_ratio = float(np.mean(max_ch < 30))
    bright_ratio = float(np.mean(max_ch > 200))

    logger.info(
        "  [%-25s] gray=%.0f%%  std=%5.0f  mean_rgb=(%3d,%3d,%3d)  dark=%.0f%%  bright=%.0f%%",
        label, gray_ratio * 100, overall_std,
        int(r_mean), int(g_mean), int(b_mean), dark_ratio * 100, bright_ratio * 100,
    )
    return {"gray_ratio": gray_ratio, "std": overall_std, "mean_rgb": (r_mean, g_mean, b_mean),
            "dark_ratio": dark_ratio, "bright_ratio": bright_ratio}


def main():
    output_dir = Path("/tmp/electronbot_render_diagnose2")
    output_dir.mkdir(parents=True, exist_ok=True)

    # 直接加载 MJCF (不通过 env wrapper，避免域随机化干扰)
    xml_path = PROJECT / "assets" / "mjcf" / "scene_tabletop.xml"
    if not xml_path.exists():
        xml_path = PROJECT / "assets" / "mjcf" / "electronbot_scene.xml"
    logger.info("加载模型: %s", xml_path)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # 设置 home 姿态
    from electronbot_sim.env import HOME_QPOS
    joint_names = ["joint_rp", "joint_rr", "joint_lp", "joint_lr", "joint_body", "joint_head"]

    # 找正确的 joint 名字 (MJCF 里用的名字可能不同)
    # electronbot.xml 中 joint 名字: joint_body, joint_head, joint_lp, joint_lr, joint_rr, joint_rr_pitch
    mjcf_joint_names = ["joint_rr_pitch", "joint_rr", "joint_lp", "joint_lr", "joint_body", "joint_head"]
    # 对应 HOME_QPOS 顺序: [RP, RR, LP, LR, BODY, HEAD]

    for i, jname in enumerate(mjcf_joint_names):
        try:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                addr = model.jnt_qposadr[jid]
                data.qpos[addr] = np.radians(HOME_QPOS[i])
                logger.info("  设置 %s qpos_addr=%d → %.1f°", jname, addr, HOME_QPOS[i])
        except Exception:
            pass

    mujoco.mj_forward(model, data)

    # 稳定仿真
    for _ in range(50):
        mujoco.mj_step(model, data)

    # 关节角度
    for i, jname in enumerate(mjcf_joint_names):
        try:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                addr = model.jnt_qposadr[jid]
                deg = np.degrees(data.qpos[addr])
                logger.info("  %s = %.1f°", jname, deg)
        except Exception:
            pass

    # ─── 测试系列 ───
    configs = [
        # (label, lookat, distance, azimuth, elevation, desc)
        ("env_default_4cm",   [0, 0, 0.055], 0.04, 180, -35, "env.py 默认 4cm 太近"),
        ("10cm_front",        [0, 0, 0.05],  0.10, 180, -15, "正前方 10cm"),
        ("15cm_diagonal",     [0, 0, 0.05],  0.15, 135, -20, "斜角 15cm"),
        ("20cm_front",        [0, 0, 0.04],  0.20, 180, -10, "正前方 20cm"),
        ("25cm_3q_view",      [0, 0, 0.03],  0.25, 135, -25, "四分之三视角 25cm"),
        ("30cm_top",          [0, 0, 0.05],  0.30,   0, -60, "俯视 30cm"),
        ("35cm_wide",         [0, 0, 0.03],  0.35, 150, -15, "宽视角 35cm"),
        ("50cm_iso",          [0, 0, 0.04],  0.50, 135, -30, "等轴视角 50cm"),
    ]

    print("\n" + "=" * 70)
    print("  ElectronBot 渲染诊断 v2 — 加强光照")
    print("=" * 70)
    print(f"\n模型: {xml_path.name}")
    print(f"nbody={model.nbody}, ngeom={model.ngeom}, njnt={model.njnt}")
    print()

    results = []
    for label, lookat, dist, az, el, desc in configs:
        logger.info("▶ %s (%s)", label, desc)
        frame = render_scene(model, data, lookat, dist, az, el)
        stats = analyze_frame(frame, label)
        score = stats["std"] * (1 - stats["gray_ratio"]) * (1 - stats["dark_ratio"]) + stats["bright_ratio"] * 10
        results.append((label, score, stats, desc))

        # 保存图片
        try:
            from PIL import Image
            Image.fromarray(frame).save(output_dir / f"{label}.png")
        except ImportError:
            pass

    # ─── 排名 ───
    results.sort(key=lambda x: x[1], reverse=True)
    print("\n" + "=" * 70)
    print("  排名 (分数越高越好)")
    print("=" * 70)
    for i, (label, score, stats, desc) in enumerate(results):
        marker = "⭐" if i == 0 else "  "
        print(f"  {marker} #{i+1}: {label:20s} score={score:7.1f}  "
              f"gray={stats['gray_ratio']*100:3.0f}%  std={stats['std']:5.0f}  "
              f"dark={stats['dark_ratio']*100:2.0f}%  bright={stats['bright_ratio']*100:2.0f}%")
        print(f"        {desc}")

    # ─── 保存 GIF ───
    try:
        from PIL import Image
        frames = []
        for label, _, _, _ in results:
            p = output_dir / f"{label}.png"
            if p.exists():
                frames.append(Image.open(p))
        if frames:
            gif_path = output_dir / "all_views.gif"
            frames[0].save(str(gif_path), save_all=True, append_images=frames[1:],
                           duration=1500, loop=0)
            logger.info("\nGIF: %s", gif_path)
    except ImportError:
        pass

    # ─── 结论 ───
    print("\n" + "=" * 70)
    print("  诊断结论")
    print("=" * 70)

    best = results[0]
    default = [r for r in results if r[0] == "env_default_4cm"][0]

    if default[2]["gray_ratio"] > 0.50:
        print(f"  ❌ 当前 env.py 默认相机 (4cm) 灰色占比 {default[2]['gray_ratio']*100:.0f}%")
        print(f"     视点太近，画面被桌面/底座占据大部分。")
    elif default[2]["dark_ratio"] > 0.40:
        print(f"  ⚠️ 当前相机偏暗，暗像素 {default[2]['dark_ratio']*100:.0f}%")

    if best[0] != "env_default_4cm":
        print(f"  💡 推荐改用: {best[0]} (score={best[1]:.1f})")
        print(f"     {best[3]}")
    else:
        print(f"  ✅ 当前 4cm 即为最佳 (score={default[1]:.1f})")
        print(f"     但 gray={default[2]['gray_ratio']*100:.0f}% 偏高，需检查实际画面")

    print(f"\n  📁 所有帧: {output_dir}/")


if __name__ == "__main__":
    main()
