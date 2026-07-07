#!/usr/bin/env python3
"""生成修复后的演示 GIF。"""
from __future__ import annotations

import os
import logging
from pathlib import Path

os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("gen_gif")

PROJECT = Path(__file__).resolve().parent.parent


def main():
    output_dir = Path("/tmp/electronbot_fixed_gif")
    output_dir.mkdir(parents=True, exist_ok=True)

    # 使用修复后的 scene_tabletop.xml
    xml_path = PROJECT / "assets" / "mjcf" / "scene_tabletop.xml"
    logger.info("加载: %s", xml_path)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    home = np.array([0.0, -45.0, 0.0, -45.0, 0.0, 0.0], dtype=np.float32)
    joint_names = ["joint_rr_pitch", "joint_rr", "joint_lp", "joint_lr", "joint_body", "joint_head"]
    for i, jname in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = np.radians(home[i])

    # actuator names 对应 ctrl 索引
    act_names = ["act_rp", "act_rr", "act_lp", "act_lr", "act_body", "act_head"]
    act_ids = []
    for name in act_names:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        act_ids.append(aid)

    logger.info("actuator ids: %s", list(zip(act_names, act_ids)))

    mujoco.mj_forward(model, data)
    for _ in range(100):
        mujoco.mj_step(model, data)

    # ─── 动作序列 (更丰富的演示) ───
    sequence = [
        # (动作描述, 目标角度 [RP, RR, LP, LR, BODY, HEAD], 持续步数)
        ("home",       [  0, -45,   0, -45,   0,   0], 30),
        ("挥手(右)",    [ 30, -45,   0, -45,   0,   0], 15),
        ("挥手(右)",    [-10, -45,   0, -45,   0,   0], 15),
        ("挥手(右)",    [ 30, -45,   0, -45,   0,   0], 15),
        ("挥手(右)",    [-10, -45,   0, -45,   0,   0], 15),
        ("home",       [  0, -45,   0, -45,   0,   0], 20),
        ("点头",       [  0, -45,   0, -45,   0,  10], 10),
        ("点头",       [  0, -45,   0, -45,   0, -10], 10),
        ("点头",       [  0, -45,   0, -45,   0,  10], 10),
        ("home",       [  0, -45,   0, -45,   0,   0], 20),
        ("挥手(左)",    [  0, -45,  30, -45,   0,   0], 15),
        ("挥手(左)",    [  0, -45, -10, -45,   0,   0], 15),
        ("挥手(左)",    [  0, -45,  30, -45,   0,   0], 15),
        ("挥手(左)",    [  0, -45, -10, -45,   0,   0], 15),
        ("home",       [  0, -45,   0, -45,   0,   0], 20),
        ("转身右",      [  0, -45,   0, -45,  40,   0], 20),
        ("转身左",      [  0, -45,   0, -45, -40,   0], 20),
        ("home",       [  0, -45,   0, -45,   0,   0], 20),
        ("兴奋举双手",   [ 45, -20,  45, -20,  20,  10], 25),
        ("home",       [  0, -45,   0, -45,   0,   0], 30),
    ]

    renderer = mujoco.Renderer(model, 480, 480)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.04]
    cam.distance = 0.25
    cam.azimuth = 145
    cam.elevation = -25

    opt = mujoco.MjvOption()
    frames = []
    frame_idx = 0

    for desc, target_deg, steps in sequence:
        target_rad = np.radians(np.array(target_deg, dtype=np.float32))

        # 平滑插值
        current_ctrl = np.zeros(6)
        for i, aid in enumerate(act_ids):
            if aid >= 0:
                current_ctrl[i] = data.ctrl[aid]

        for s in range(steps):
            alpha = min(1.0, (s + 1) / max(5, steps // 3))
            # ease-in-out
            alpha = alpha * alpha * (3 - 2 * alpha)
            interp = current_ctrl + (target_rad - current_ctrl) * alpha

            for i, aid in enumerate(act_ids):
                if aid >= 0:
                    data.ctrl[aid] = interp[i]

            for _ in range(10):  # 子步
                mujoco.mj_step(model, data)

            # 每 2 帧渲染一次
            if s % 2 == 0:
                mujoco.mjv_updateScene(model, data, opt, mujoco.MjvPerturb(),
                                       cam, mujoco.mjtCatBit.mjCAT_ALL, renderer.scene)
                frame = renderer.render()
                frames.append(frame)
                frame_idx += 1

    renderer.close()

    logger.info("生成了 %d 帧", len(frames))

    # 保存 GIF
    try:
        from PIL import Image
        pil_frames = [Image.fromarray(f) for f in frames]
        gif_path = output_dir / "electronbot_fixed_demo.gif"
        pil_frames[0].save(str(gif_path), save_all=True, append_images=pil_frames[1:],
                           duration=50, loop=0)
        logger.info("✅ GIF: %s", gif_path)

        # 也保存首帧 PNG
        Image.fromarray(frames[0]).save(output_dir / "first_frame.png")
        logger.info("✅ 首帧: %s/first_frame.png", output_dir)
    except ImportError:
        logger.warning("Pillow 未安装")

    print(f"\n📁 输出: {output_dir}/")


if __name__ == "__main__":
    main()
