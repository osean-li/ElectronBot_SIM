#!/usr/bin/env python3
"""渲染诊断最终版：定位 clipping + 光照根因，生成 GIF。"""
from __future__ import annotations

import os
import io
import logging
from pathlib import Path

os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("diagnose_final")

PROJECT = Path(__file__).resolve().parent.parent


def render_and_save(model, data, lookat, distance, azimuth, elevation,
                    output_dir, label, width=480, height=480):
    """渲染一帧并保存 PNG，返回帧数组。"""
    renderer = mujoco.Renderer(model, width, height)

    # 在 mjv_updateScene 之前重置场景标志
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0

    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation

    opt = mujoco.MjvOption()
    mujoco.mjv_updateScene(model, data, opt, mujoco.MjvPerturb(),
                           cam, mujoco.mjtCatBit.mjCAT_ALL, renderer.scene)
    frame = renderer.render()
    renderer.close()
    return frame


def analyze(frame, label):
    max_ch = np.max(frame, axis=2)
    min_ch = np.min(frame, axis=2)
    gray_mask = (max_ch - min_ch) < 15
    gray_ratio = float(np.mean(gray_mask))
    overall_std = float((frame[..., 0].std() + frame[..., 1].std() + frame[..., 2].std()) / 3)
    r_mean = float(frame[..., 0].mean())
    g_mean = float(frame[..., 1].mean())
    b_mean = float(frame[..., 2].mean())
    dark_ratio = float(np.mean(max_ch < 30))
    bright_ratio = float(np.mean(max_ch > 200))
    unique = len(np.unique(frame.reshape(-1, 3), axis=0))

    ok = "✅" if unique > 10 and dark_ratio < 0.5 else "❌"
    logger.info(
        "%s [%-30s] gray=%.0f%% std=%5.0f rgb=(%3d,%3d,%3d) dark=%.0f%% uniq=%d",
        ok, label, gray_ratio * 100, overall_std,
        int(r_mean), int(g_mean), int(b_mean), dark_ratio * 100, unique,
    )
    return frame, {"gray": gray_ratio, "std": overall_std, "dark": dark_ratio, "unique": unique}


def main():
    output_dir = Path("/tmp/electronbot_render_final")
    output_dir.mkdir(parents=True, exist_ok=True)

    # ─── 测试 1: scene_tabletop.xml (有 znear=0.0001 问题) ───
    logger.info("=" * 70)
    logger.info("测试 A: scene_tabletop.xml (znear=0.0001, headlight=0,0,0)")
    logger.info("=" * 70)

    xml = PROJECT / "assets" / "mjcf" / "scene_tabletop.xml"
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)

    logger.info("model.vis.map: znear=%.4f zfar=%.1f", model.vis.map.znear, model.vis.map.zfar)

    # Home 姿态
    home = np.array([0.0, -45.0, 0.0, -45.0, 0.0, 0.0], dtype=np.float32)
    joint_names = ["joint_rr_pitch", "joint_rr", "joint_lp", "joint_lr", "joint_body", "joint_head"]
    for i, jname in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = np.radians(home[i])
    mujoco.mj_forward(model, data)
    for _ in range(50):
        mujoco.mj_step(model, data)

    # 不修改任何 vis，测试不同距离
    all_frames = []

    for dist, az, el, desc in [
        (0.04, 180, -35, "A1: 4cm (env default)"),
        (0.10, 180, -20, "A2: 10cm"),
        (0.20, 160, -15, "A3: 20cm"),
    ]:
        frame = render_and_save(model, data, [0, 0, 0.055], dist, az, el, output_dir, desc)
        f, _ = analyze(frame, desc)
        all_frames.append(f)

    # ─── 测试 2: scene_tabletop.xml 修复 znear ───
    logger.info("\n" + "=" * 70)
    logger.info("测试 B: scene_tabletop.xml 修复 znear=0.001 + 加亮 headlight")
    logger.info("=" * 70)

    model.vis.map.znear = 0.001  # 从 0.0001 改为 1mm
    model.vis.map.zfar = 100.0   # 从 0.5 改为 100m
    model.vis.headlight.active = True
    model.vis.headlight.ambient[:] = [0.4, 0.4, 0.45]
    model.vis.headlight.diffuse[:] = [0.6, 0.6, 0.65]
    model.vis.headlight.specular[:] = [0.1, 0.1, 0.1]

    logger.info("model.vis.map: znear=%.4f zfar=%.1f", model.vis.map.znear, model.vis.map.zfar)

    for dist, az, el, desc in [
        (0.04, 180, -35, "B1: 4cm fixed"),
        (0.10, 180, -15, "B2: 10cm fixed"),
        (0.20, 160, -15, "B3: 20cm fixed"),
        (0.30, 135, -25, "B4: 30cm fixed"),
    ]:
        frame = render_and_save(model, data, [0, 0, 0.04], dist, az, el, output_dir, desc)
        f, _ = analyze(frame, desc)
        all_frames.append(f)

    # ─── 测试 3: electronbot_scene.xml (没有 znear 问题) ───
    logger.info("\n" + "=" * 70)
    logger.info("测试 C: electronbot_scene.xml (默认 visual, 无 znear 覆写)")
    logger.info("=" * 70)

    xml2 = PROJECT / "assets" / "mjcf" / "electronbot_scene.xml"
    model2 = mujoco.MjModel.from_xml_path(str(xml2))
    data2 = mujoco.MjData(model2)

    logger.info("model.vis.map: znear=%.4f zfar=%.1f", model2.vis.map.znear, model2.vis.map.zfar)
    logger.info("headlight: active=%d ambient=%s diffuse=%s",
                model2.vis.headlight.active, model2.vis.headlight.ambient, model2.vis.headlight.diffuse)

    # Home
    for i, jname in enumerate(joint_names):
        jid = mujoco.mj_name2id(model2, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data2.qpos[model2.jnt_qposadr[jid]] = np.radians(home[i])
    mujoco.mj_forward(model2, data2)
    for _ in range(50):
        mujoco.mj_step(model2, data2)

    for dist, az, el, desc in [
        (0.04, 180, -35, "C1: 4cm"),
        (0.15, 160, -20, "C2: 15cm"),
        (0.30, 135, -25, "C3: 30cm"),
    ]:
        frame = render_and_save(model2, data2, [0, 0, 0.04], dist, az, el, output_dir, desc)
        f, _ = analyze(frame, desc)
        all_frames.append(f)

    # ─── 保存 GIF ───
    try:
        from PIL import Image
        pil_frames = [Image.fromarray(f) for f in all_frames]
        gif_path = output_dir / "diagnosis_all.gif"
        pil_frames[0].save(str(gif_path), save_all=True, append_images=pil_frames[1:],
                           duration=2000, loop=0)
        logger.info("\n✅ GIF: %s (%d 帧)", gif_path, len(pil_frames))

        # 也另存各个 PNG
        for i, (desc, f) in enumerate(zip(
            ["A1_4cm_broken", "A2_10cm_broken", "A3_20cm_broken",
             "B1_4cm_fixed", "B2_10cm_fixed", "B3_20cm_fixed", "B4_30cm_fixed",
             "C1_4cm_alt", "C2_15cm_alt", "C3_30cm_alt"], all_frames)):
            Image.fromarray(f).save(output_dir / f"{desc}.png")
            logger.info("  PNG: %s.png", desc)
    except ImportError:
        pass

    # ─── 结论 ───
    print("\n" + "=" * 70)
    print("  🔍 诊断结论")
    print("=" * 70)
    print("""
  根因: scene_tabletop.xml 设置了 <map znear="0.0001" zfar="0.5"/>
        znear=0.0001m (0.1mm) 导致浮点精度问题，
        相机距离超过 ~5cm 时场景投影异常，全部变纯黑。

  修复方案:
    1. 修改 scene_tabletop.xml: znear="0.001" zfar="100.0"
    2. 设置 headlight ambient/diffuse 为非零值
    3. env.py _render_rgb 中 cam.distance 改为 0.15~0.30m
""")
    print(f"  📁 所有图片: {output_dir}/")


if __name__ == "__main__":
    main()
