#!/usr/bin/env python3
"""生成修复前后对比 GIF。"""
from __future__ import annotations

import os, logging, copy
from pathlib import Path

os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("compare")

PROJECT = Path(__file__).resolve().parent.parent


def render_scene(model, data, lookat, dist, az, el, label, output_dir):
    """渲染一帧并保存。"""
    renderer = mujoco.Renderer(model, 480, 480)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = dist
    cam.azimuth = az
    cam.elevation = el
    opt = mujoco.MjvOption()
    mujoco.mjv_updateScene(model, data, opt, mujoco.MjvPerturb(),
                           cam, mujoco.mjtCatBit.mjCAT_ALL, renderer.scene)
    frame = renderer.render()
    renderer.close()

    # 统计
    max_ch = np.max(frame, axis=2)
    min_ch = np.min(frame, axis=2)
    gray_mask = (max_ch - min_ch) < 15
    gray_pct = float(np.mean(gray_mask)) * 100
    dark_pct = float(np.mean(max_ch < 30)) * 100
    overall_std = float(np.mean([frame[..., c].std() for c in range(3)]))
    uniq = len(np.unique(frame.reshape(-1, 3), axis=0))

    logger.info("  %s: gray=%.0f%% std=%.0f dark=%.0f%% uniq=%d", label, gray_pct, overall_std, dark_pct, uniq)

    try:
        from PIL import Image
        Image.fromarray(frame).save(output_dir / f"{label}.png")
    except ImportError:
        pass
    return frame


def main():
    output_dir = Path("/tmp/electronbot_compare")
    output_dir.mkdir(parents=True, exist_ok=True)

    # --- 1. BEFORE: 原始的 broken 配置 ---
    xml = PROJECT / "assets" / "mjcf" / "scene_tabletop.xml"
    model_before = mujoco.MjModel.from_xml_path(str(xml))

    # 模拟修复前的 vis 设置
    model_before.vis.headlight.active = True
    model_before.vis.headlight.ambient[:] = [0, 0, 0]
    model_before.vis.headlight.diffuse[:] = [0, 0, 0]
    model_before.vis.map.znear = 0.0001
    model_before.vis.map.zfar = 0.5

    data_before = mujoco.MjData(model_before)
    home = np.array([0.0, -45.0, 0.0, -45.0, 0.0, 0.0])
    jnames = ["joint_rr_pitch", "joint_rr", "joint_lp", "joint_lr", "joint_body", "joint_head"]
    for i, jn in enumerate(jnames):
        jid = mujoco.mj_name2id(model_before, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data_before.qpos[model_before.jnt_qposadr[jid]] = np.radians(home[i])
    mujoco.mj_forward(model_before, data_before)
    for _ in range(50):
        mujoco.mj_step(model_before, data_before)

    # --- 2. AFTER: 修复后的配置 (与新 XML 一致) ---
    model_after = mujoco.MjModel.from_xml_path(str(xml))
    model_after.vis.headlight.active = True
    model_after.vis.headlight.ambient[:] = [0.3, 0.3, 0.35]
    model_after.vis.headlight.diffuse[:] = [0.6, 0.6, 0.65]
    model_after.vis.headlight.specular[:] = [0.1, 0.1, 0.1]
    model_after.vis.map.znear = 0.001
    model_after.vis.map.zfar = 100.0

    data_after = mujoco.MjData(model_after)
    for i, jn in enumerate(jnames):
        jid = mujoco.mj_name2id(model_after, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data_after.qpos[model_after.jnt_qposadr[jid]] = np.radians(home[i])
    mujoco.mj_forward(model_after, data_after)
    for _ in range(50):
        mujoco.mj_step(model_after, data_after)

    # --- 渲染对比 ---
    print("\n" + "=" * 70)
    print("  修复前 vs 修复后 对比")
    print("=" * 70)

    all_frames = []

    # 原配置 cam=4cm (唯一能渲染的距离)
    logger.info("\n--- 修复前 (znear=0.0001, headlight=0, cam=4cm) ---")
    f1 = render_scene(model_before, data_before, [0, 0, 0.055], 0.04, 180, -35,
                      "01_before_4cm", output_dir)
    all_frames.append(f1)

    # 原配置 + 正常距离 (无法渲染)
    logger.info("\n--- 修复前 (znear=0.0001, headlight=0, cam=25cm) ---")
    f2 = render_scene(model_before, data_before, [0, 0, 0.04], 0.25, 145, -25,
                      "02_before_25cm_BROKEN", output_dir)
    all_frames.append(f2)

    # 修复后 + 正常距离
    logger.info("\n--- 修复后 (znear=0.001, bright headlight, cam=25cm) ---")
    f3 = render_scene(model_after, data_after, [0, 0, 0.04], 0.25, 145, -25,
                      "03_after_25cm_FIXED", output_dir)
    all_frames.append(f3)

    # 修复后 + 更远视角
    logger.info("\n--- 修复后 (cam=35cm wide view) ---")
    f4 = render_scene(model_after, data_after, [0, 0, 0.03], 0.35, 150, -20,
                      "04_after_35cm", output_dir)
    all_frames.append(f4)

    # GIF
    try:
        from PIL import Image
        pil = [Image.fromarray(f) for f in all_frames]
        gif_path = output_dir / "before_after_compare.gif"
        pil[0].save(str(gif_path), save_all=True, append_images=pil[1:],
                    duration=3000, loop=0)
        logger.info("\n✅ 对比 GIF: %s", gif_path)
    except ImportError:
        pass

    print(f"\n📁 所有文件: {output_dir}/")


if __name__ == "__main__":
    main()
