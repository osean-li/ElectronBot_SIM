#!/usr/bin/env python3
"""渲染诊断脚本：用不同相机参数生成 GIF 并自动分析像素分布。

分析指标：
  - 灰色主导率 (gray_dominance)：像素中灰色调占比 (> 70% 表示视角卡在模型内部)
  - 有效像素率 (effective_pixel_rate)：非背景色的像素占比
  - 对比度 (contrast)：图像标准差 / 均值

理想输出：能清晰看到桌面上的机器人和彩色物体，而不是一个灰色方块。
"""
from __future__ import annotations

import os
import sys
import time
import logging
from pathlib import Path

import numpy as np

# 确保使用 EGL 后端（无头渲染）
os.environ["MUJOCO_GL"] = "egl"

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
logger = logging.getLogger("diagnose")


def analyze_frame(frame: np.ndarray, label: str) -> dict:
    """分析单帧 RGB 图像的像素分布。"""
    h, w, _ = frame.shape
    total = h * w

    # 1. 灰度判断：R/G/B 偏差 < 15 的像素
    max_ch = np.max(frame, axis=2)
    min_ch = np.min(frame, axis=2)
    gray_mask = (max_ch - min_ch) < 15
    gray_ratio = float(np.mean(gray_mask))

    # 2. 颜色多样性 (像素标准差)
    r_std, g_std, b_std = float(frame[..., 0].std()), float(frame[..., 1].std()), float(frame[..., 2].std())
    overall_std = (r_std + g_std + b_std) / 3

    # 3. 各个颜色通道的均值
    r_mean, g_mean, b_mean = float(frame[..., 0].mean()), float(frame[..., 1].mean()), float(frame[..., 2].mean())

    # 4. 暗像素比例 (值 < 30)
    dark_ratio = float(np.mean(np.max(frame, axis=2) < 30))

    logger.info(
        "  [%s] gray=%.1f%%  std=%.1f  mean_rgb=(%d,%d,%d)  dark=%.1f%%",
        label, gray_ratio * 100, overall_std,
        int(r_mean), int(g_mean), int(b_mean), dark_ratio * 100,
    )

    return {
        "gray_ratio": gray_ratio,
        "std": overall_std,
        "mean_rgb": (r_mean, g_mean, b_mean),
        "dark_ratio": dark_ratio,
    }


def render_with_camera(model, data, lookat, distance, azimuth, elevation, label: str):
    """用指定相机参数渲染一帧。"""
    import mujoco

    # NOTE: 缩放模型以改善可见性
    # model 和 data 是从 env 借用的，不做修改

    renderer = mujoco.Renderer(model, 480, 480)

    # 适度头灯
    model.vis.headlight.active = True
    model.vis.headlight.ambient[:] = [0.1, 0.1, 0.12]
    model.vis.headlight.diffuse[:] = [0.3, 0.3, 0.35]

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


def _compute_distribution_stats(frames: list[np.ndarray]) -> dict:
    """计算帧序列的统计摘要。"""
    if not frames:
        return {}
    arr = np.stack(frames)
    gray_ratios = []
    stds = []
    for f in frames:
        max_ch = np.max(f, axis=2)
        min_ch = np.min(f, axis=2)
        gray_mask = (max_ch - min_ch) < 15
        gray_ratios.append(float(np.mean(gray_mask)))
        stds.append(float((f[..., 0].std() + f[..., 1].std() + f[..., 2].std()) / 3))
    return {
        "gray_mean": float(np.mean(gray_ratios)),
        "gray_max": float(np.max(gray_ratios)),
        "std_mean": float(np.mean(stds)),
        "std_min": float(np.min(stds)),
    }


def save_gif(frames: list[np.ndarray], path: str, duration_ms: int = 50):
    """保存帧序列为 GIF。"""
    try:
        from PIL import Image
        pil_frames = [Image.fromarray(f) for f in frames]
        pil_frames[0].save(
            path, save_all=True, append_images=pil_frames[1:],
            duration=duration_ms, loop=0,
        )
        logger.info("  GIF 已保存: %s (%d 帧)", path, len(frames))
    except ImportError:
        logger.warning("  Pillow 未安装, 跳过 GIF 保存")


def main():
    output_dir = Path("/tmp/electronbot_render_diagnose")
    output_dir.mkdir(parents=True, exist_ok=True)

    # ─── 使用 ElectronBotEnv 加载场景 ───
    from electronbot_sim.env import ElectronBotEnv

    env = ElectronBotEnv(render_mode="rgb_array")
    env.reset()

    model = env.model
    data = env.data

    # ─── 测试多个相机参数组合 ───
    # 机器人大小: base_geom 半径12mm 高15mm, body 16x20x36mm, head 14x14x10mm
    # 总高约 70mm, 最宽约 24mm(手臂)
    # 场景: table 80x60x4mm, floor plane 1m x 1m
    # 单位: 米

    configs = [
        # (label, lookat, distance, azimuth, elevation, desc)
        ("current_env_default", [0, 0, 0.055], 0.04, 180, -35, "env.py 当前默认 (4cm, 太近!)"),
        ("medium_15cm", [0, 0, 0.05], 0.15, 180, -20, "中等距离 15cm"),
        ("medium_25cm", [0, 0, 0.05], 0.25, 160, -15, "中等距离 25cm"),
        ("wide_40cm", [0, 0, 0.04], 0.40, 150, -20, "远距离 40cm 俯视"),
        ("top_down_30cm", [0, 0, 0.05], 0.30, 0, -89, "正上方 30cm"),
        ("front_25cm", [0, 0, 0.04], 0.25, 180, -5, "正前方 25cm 眼睛高度"),
        ("side_20cm", [0, 0, 0.05], 0.20, 90, -10, "侧面 20cm"),
        ("diagonal_30cm", [0, 0, 0.05], 0.30, 135, -25, "对角 30cm"),
    ]

    print("\n" + "=" * 70)
    print("  ElectronBot 渲染诊断")
    print("=" * 70)
    print(f"\n模型信息: nbody={model.nbody}, njnt={model.njnt}, ngeom={model.ngeom}")
    print(f"关节角度 (度): {np.round(env.get_joint_positions(), 1).tolist()}")
    print()

    # 步进几帧让模型稳定
    for _ in range(10):
        env.step(np.zeros(6, dtype=np.float32))

    best_config = None
    best_score = -1
    results = []

    for label, lookat, dist, az, el, desc in configs:
        print(f"▶ 测试: {label} ({desc})")
        frame = render_with_camera(model, data, lookat, dist, az, el, label)
        stats = analyze_frame(frame, label)

        # 保存帧
        save_path = output_dir / f"frame_{label}.png"
        try:
            from PIL import Image
            Image.fromarray(frame).save(save_path)
        except ImportError:
            pass

        # 评分：对比度越高越好，灰度越少越好
        score = stats["std"] * (1.0 - stats["gray_ratio"]) * (1.0 - stats["dark_ratio"])
        results.append((label, score, stats, save_path, desc))

        if score > best_score:
            best_score = score
            best_config = (label, save_path)
        print()

    # ─── 排名 ───
    results.sort(key=lambda x: x[1], reverse=True)
    print("=" * 70)
    print("  渲染质量排名 (分数越高越好)")
    print("=" * 70)
    for i, (label, score, stats, _path, desc) in enumerate(results):
        marker = "⭐" if i == 0 else "  "
        print(f"  {marker} #{i+1}: {label:25s}  score={score:.1f}  std={stats['std']:.0f}  "
              f"gray={stats['gray_ratio']*100:.0f}%  dark={stats['dark_ratio']*100:.0f}%")
        print(f"        {desc}")

    # ─── 诊断结论 ───
    print("\n" + "=" * 70)
    print("  诊断结论")
    print("=" * 70)

    current = [r for r in results if r[0] == "current_env_default"][0]
    gray_pct = current[2]["gray_ratio"] * 100
    dark_pct = current[2]["dark_ratio"] * 100
    std_val = current[2]["std"]

    if gray_pct > 60:
        problem = (
            f"❌ 当前默认相机配置有问题！\n"
            f"   灰色占比 {gray_pct:.0f}% (正常应 < 30%)\n"
            f"   对比度 {std_val:.0f} (正常应 > 20)\n"
            f"   原因: cam.distance=0.04m 太近，视角大概率卡在模型/桌板内部\n"
        )
    elif dark_pct > 50:
        problem = (
            f"⚠️ 当前相机偏暗\n"
            f"   暗像素 {dark_pct:.0f}% (正常应 < 20%)\n"
            f"   可能需要调整 lookat 或增加 distance\n"
        )
    else:
        problem = "✅ 当前默认相机似乎正常，但需要人工确认图像内容"

    print(f"  {problem}")
    print(f"\n  推荐相机参数: {results[0][0]} (score={results[0][1]:.1f})")
    print(f"  {results[0][4]}")

    env.close()

    # ─── 生成对比 GIF：展示不同距离下的画面 ───
    print(f"\n  所有帧已保存到: {output_dir}/")
    print(f"  建议用图片查看器打开 frame_current_env_default.png")
    print(f"  和 frame_{results[0][0]}.png 对比")

    # 生成一个多角度动画 GIF
    configs_for_gif = [r[0] for r in results[:5]]  # top 5
    frames_for_gif = []
    for label in configs_for_gif:
        p = output_dir / f"frame_{label}.png"
        if p.exists():
            try:
                from PIL import Image
                frames_for_gif.append(Image.open(p))
            except ImportError:
                break

    if frames_for_gif:
        gif_path = output_dir / "comparison.gif"
        frames_for_gif[0].save(
            str(gif_path), save_all=True, append_images=frames_for_gif[1:],
            duration=1500, loop=0,
        )
        print(f"  对比 GIF: {gif_path}")


if __name__ == "__main__":
    main()
