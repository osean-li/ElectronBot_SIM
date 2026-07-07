#!/usr/bin/env python3
"""渲染诊断 v3：修复光照时序 + 增加 debug 输出。"""
from __future__ import annotations

import os
import logging
from pathlib import Path

os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("diagnose3")

PROJECT = Path(__file__).resolve().parent.parent


def analyze_frame(frame, label):
    max_ch = np.max(frame, axis=2)
    min_ch = np.min(frame, axis=2)
    gray_mask = (max_ch - min_ch) < 15
    gray_ratio = float(np.mean(gray_mask))
    r_std = float(frame[..., 0].std())
    g_std = float(frame[..., 1].std())
    b_std = float(frame[..., 2].std())
    overall_std = (r_std + g_std + b_std) / 3
    r_mean = float(frame[..., 0].mean())
    g_mean = float(frame[..., 1].mean())
    b_mean = float(frame[..., 2].mean())
    dark_ratio = float(np.mean(max_ch < 30))
    bright_ratio = float(np.mean(max_ch > 200))
    unique_colors = len(np.unique(frame.reshape(-1, 3), axis=0))

    logger.info(
        "  [%-25s] gray=%.0f%% std=%5.0f rgb=(%3d,%3d,%3d) dark=%.0f%% bright=%.0f%% unique=%d",
        label, gray_ratio * 100, overall_std,
        int(r_mean), int(g_mean), int(b_mean),
        dark_ratio * 100, bright_ratio * 100, unique_colors,
    )
    return {
        "gray_ratio": gray_ratio, "std": overall_std,
        "mean_rgb": (r_mean, g_mean, b_mean),
        "dark_ratio": dark_ratio, "bright_ratio": bright_ratio,
        "unique_colors": unique_colors,
    }


def main():
    output_dir = Path("/tmp/electronbot_render_diagnose3")
    output_dir.mkdir(parents=True, exist_ok=True)

    xml_path = PROJECT / "assets" / "mjcf" / "scene_tabletop.xml"
    logger.info("加载: %s", xml_path)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Home 姿态
    mjcf_joint_names = ["joint_rr_pitch", "joint_rr", "joint_lp", "joint_lr", "joint_body", "joint_head"]
    HOME_QPOS = np.array([0.0, -45.0, 0.0, -45.0, 0.0, 0.0], dtype=np.float32)
    for i, jname in enumerate(mjcf_joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = np.radians(HOME_QPOS[i])

    mujoco.mj_forward(model, data)
    for _ in range(50):
        mujoco.mj_step(model, data)

    # ─── 打印 model.vis 默认值 ───
    logger.info("\n--- model.vis 初始状态 ---")
    logger.info("headlight.active=%s", model.vis.headlight.active)
    logger.info("headlight.ambient=%s", model.vis.headlight.ambient)
    logger.info("headlight.diffuse=%s", model.vis.headlight.diffuse)
    logger.info("headlight.specular=%s", model.vis.headlight.specular)
    logger.info("map.znear=%.6f  zfar=%.3f", model.vis.map.znear, model.vis.map.zfar)

    # ─── 方案 A: 不改 vis, 只用场景里定义的三盏灯 + 环境光 ───
    # ─── 方案 B: 大幅提高模型 vis 设置 ───
    # 先测试不改任何设置

    configs = [
        ("default_no_change",    [0, 0, 0.055], 0.04, 180, -35),
        ("default_10cm",         [0, 0, 0.055], 0.10, 180, -35),
        ("default_20cm",         [0, 0, 0.05],  0.20, 180, -20),
        ("default_40cm",         [0, 0, 0.04],  0.40, 135, -25),
    ]

    print("\n" + "=" * 70)
    print("  测试 1: 使用 XML 默认光照 (scene_tabletop 把 headlight 清零了!)")
    print("=" * 70)

    for label, lookat, dist, az, el in configs:
        # ⚠️ 关键: 先设置 model.vis 再创建 Renderer
        # 不修改 vis, 看原始效果
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
        analyze_frame(frame, label)
        try:
            from PIL import Image
            Image.fromarray(frame).save(output_dir / f"{label}.png")
        except ImportError:
            pass
        renderer.close()

    # ─── 测试 2: 手动添加光照 ───
    print("\n" + "=" * 70)
    print("  测试 2: 手动设置 model.vis.headlight BEFORE 创建 Renderer")
    print("=" * 70)

    model.vis.headlight.active = True
    model.vis.headlight.ambient[:] = [0.4, 0.4, 0.45]
    model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.85]
    model.vis.headlight.specular[:] = [0.2, 0.2, 0.2]

    configs2 = [
        ("bright_4cm",    [0, 0, 0.055], 0.04, 180, -35),
        ("bright_10cm",   [0, 0, 0.055], 0.10, 180, -20),
        ("bright_20cm",   [0, 0, 0.05],  0.20, 160, -15),
        ("bright_30cm",   [0, 0, 0.04],  0.30, 135, -25),
        ("bright_50cm",   [0, 0, 0.04],  0.50, 135, -30),
    ]

    for label, lookat, dist, az, el in configs2:
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
        analyze_frame(frame, label)
        try:
            from PIL import Image
            Image.fromarray(frame).save(output_dir / f"{label}.png")
        except ImportError:
            pass
        renderer.close()

    # ─── 测试 3: 直接修改 renderer.scene 的光照 ───
    print("\n" + "=" * 70)
    print("  测试 3: 修改 renderer.scene 的光照 (headlight enabled)")
    print("=" * 70)

    for label, lookat, dist, az, el in configs2:
        label = "scene_" + label
        renderer = mujoco.Renderer(model, 480, 480)
        # 直接在 scene 上设置
        renderer.scene.enabletransform = True
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True

        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance = dist
        cam.azimuth = az
        cam.elevation = el
        opt = mujoco.MjvOption()
        mujoco.mjv_updateScene(model, data, opt, mujoco.MjvPerturb(),
                               cam, mujoco.mjtCatBit.mjCAT_ALL, renderer.scene)
        frame = renderer.render()
        analyze_frame(frame, label)
        try:
            from PIL import Image
            Image.fromarray(frame).save(output_dir / f"{label}.png")
        except ImportError:
            pass
        renderer.close()

    # ─── 生成汇总 GIF ───
    try:
        from PIL import Image
        frames = []
        for fname in sorted(os.listdir(output_dir)):
            if fname.endswith(".png"):
                p = output_dir / fname
                frames.append(Image.open(p))
        if frames:
            gif_path = output_dir / "diagnosis_final.gif"
            # 只取前 12 张
            frames[:12][0].save(str(gif_path), save_all=True,
                                append_images=frames[1:12], duration=2000, loop=0)
            logger.info("\nGIF: %s (%d frames)", gif_path, len(frames[:12]))
    except ImportError:
        pass

    print(f"\n📁 所有帧: {output_dir}/")


if __name__ == "__main__":
    main()
