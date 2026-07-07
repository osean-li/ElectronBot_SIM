"""ElectronBot 自动动作演示脚本。

对齐 Phase 2 §3 Step 3: 与 mujoco.viewer 的互补。
典型动作序列: home → 挥手 → 点头 → 转身 → home, 循环播放。

用法:
    python -m electronbot_sim.visual_demo              # 有渲染模式
    python -m electronbot_sim.visual_demo --headless    # 无头模式, 帧存 /tmp/
    python -m electronbot_sim.visual_demo --cycles 3    # 播放 3 个循环
    python -m electronbot_sim.visual_demo --gif output  # 生成 GIF
"""
from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path

import numpy as np

logger = logging.getLogger("electronbot_sim.visual_demo")


# ═══════════════════════════════════════════════════════════════════
#  预设动作序列 — 每个动作持续固定帧数
#  动作: (关节索引, delta_deg, 持续帧数)
#  关节顺序: [RP(0), RR(1), LP(2), LR(3), BODY(4), HEAD(5)]
# ═══════════════════════════════════════════════════════════════════

def _build_demo_sequence() -> list[tuple[np.ndarray, int]]:
    """构建演示动作序列。

    每个元素: (action(6,), hold_frames)
    - action: 6 维角度增量 (度/帧)
    - hold_frames: 保持帧数
    """
    action_home = np.zeros(6, dtype=np.float32)
    d = 2.0  # 默认步进角度 (度)

    sequence = [
        # === 1. 从 home 起始 ===
        (action_home, 20),

        # === 2. 点头 (头部俯仰) ===
        (np.array([0, 0, 0, 0, 0, +d], dtype=np.float32), 10),   # 头下
        (np.array([0, 0, 0, 0, 0, -d], dtype=np.float32), 10),   # 头上
        (np.array([0, 0, 0, 0, 0, -d], dtype=np.float32), 10),   # 头下
        (np.array([0, 0, 0, 0, 0, +d], dtype=np.float32), 10),   # 头上
        (action_home, 15),

        # === 3. 挥手 (右臂 Pitch 摆动) ===
        (np.array([+d, 0, 0, 0, 0, 0], dtype=np.float32), 8),
        (np.array([-d, 0, 0, 0, 0, 0], dtype=np.float32), 8),
        (np.array([-d, 0, 0, 0, 0, 0], dtype=np.float32), 8),
        (np.array([+d, 0, 0, 0, 0, 0], dtype=np.float32), 8),
        (np.array([-d, 0, 0, 0, 0, 0], dtype=np.float32), 8),
        (np.array([+d, 0, 0, 0, 0, 0], dtype=np.float32), 8),
        (action_home, 15),

        # === 4. 左臂挥手 ===
        (np.array([0, 0, +d, 0, 0, 0], dtype=np.float32), 8),
        (np.array([0, 0, -d, 0, 0, 0], dtype=np.float32), 8),
        (np.array([0, 0, -d, 0, 0, 0], dtype=np.float32), 8),
        (np.array([0, 0, +d, 0, 0, 0], dtype=np.float32), 8),
        (np.array([0, 0, -d, 0, 0, 0], dtype=np.float32), 8),
        (np.array([0, 0, +d, 0, 0, 0], dtype=np.float32), 8),
        (action_home, 15),

        # === 5. 转身 (身体旋转) ===
        (np.array([0, 0, 0, 0, +d, 0], dtype=np.float32), 15),
        (np.array([0, 0, 0, 0, -d, 0], dtype=np.float32), 15),
        (np.array([0, 0, 0, 0, -d, 0], dtype=np.float32), 15),
        (np.array([0, 0, 0, 0, +d, 0], dtype=np.float32), 15),
        (action_home, 20),

        # === 6. 双臂同时 Roll ===
        (np.array([0, +d, 0, -d, 0, 0], dtype=np.float32), 10),
        (np.array([0, -d, 0, +d, 0, 0], dtype=np.float32), 10),
        (np.array([0, -d, 0, +d, 0, 0], dtype=np.float32), 10),
        (np.array([0, +d, 0, -d, 0, 0], dtype=np.float32), 10),
        (action_home, 20),
    ]
    return sequence


def run_visual_demo(
    env=None,
    cycles: int = -1,
    headless: bool = False,
    save_gif: str | None = None,
    fps: int = 30,
) -> None:
    """自动动作演示主循环。

    参数:
        env: 可选的 ElectronBotEnv 实例, 为 None 时自动创建。
        cycles: 播放循环次数, -1 表示无限循环。
        headless: True 时将帧保存到 /tmp/ 而非显示。
        save_gif: GIF 输出路径 (不含扩展名)。
        fps: 渲染帧率。
    """
    render_mode = "rgb_array" if (headless or save_gif) else "human"

    if env is None:
        from electronbot_sim.env import ElectronBotEnv
        env = ElectronBotEnv(render_mode=render_mode)

    sequence = _build_demo_sequence()
    total_frames = sum(hold for _, hold in sequence)
    logger.info(
        "动作序列: %d 个动作阶段, 共 %d 帧 (约 %.1f 秒/循环 @%dHz)",
        len(sequence), total_frames, total_frames / fps, fps,
    )

    frames: list[np.ndarray] = []  # 收集帧用于生成 GIF
    frame_output_dir = Path("/tmp/electronbot_demo_frames") if headless else None
    if frame_output_dir:
        frame_output_dir.mkdir(parents=True, exist_ok=True)

    try:
        env.reset()
        cycle_count = 0

        while cycles == -1 or cycle_count < cycles:
            logger.info("循环 %d/%s 开始", cycle_count + 1,
                        str(cycles) if cycles > 0 else "∞")

            for action, hold_frames in sequence:
                for _ in range(hold_frames):
                    obs, reward, terminated, truncated, info = env.step(action)
                    if terminated:
                        logger.warning("仿真终止, 自动 reset")
                        env.reset()

                    # 渲染
                    if render_mode == "rgb_array":
                        frame = env.render()
                        if save_gif or headless:
                            frames.append(frame)
                    elif render_mode == "human":
                        env.render()

                # 实时步进 (50Hz 控制)
                time.sleep(0.02)

            cycle_count += 1

    except KeyboardInterrupt:
        logger.info("演示被用户中断, 共完成 %d 个循环", cycle_count)
    finally:
        env.close()

        # 保存 GIF
        if save_gif and frames:
            _save_gif(frames, save_gif, fps=fps)
        elif headless and frames:
            _save_frames(frames, frame_output_dir)

        logger.info("演示结束, 共 %d 循环, %d 帧", cycle_count, len(frames))


def _save_frames(frames: list[np.ndarray], output_dir: Path) -> None:
    """保存帧为 PNG 图片。"""
    try:
        from PIL import Image
        for i, frame in enumerate(frames):
            img = Image.fromarray(frame)
            img.save(output_dir / f"frame_{i:06d}.png")
        logger.info("%d 帧已保存到 %s", len(frames), output_dir)
    except ImportError:
        logger.warning("Pillow 未安装, 无法保存帧图片。pip install Pillow")


def _save_gif(frames: list[np.ndarray], output_path: str, fps: int = 30) -> None:
    """保存帧序列为 GIF。"""
    try:
        from PIL import Image
        pil_frames = [Image.fromarray(f) for f in frames]
        gif_path = output_path if output_path.endswith(".gif") else f"{output_path}.gif"

        # 调整间隔: 控制步 50Hz → 渲染帧 30fps
        duration = int(1000 / fps)
        pil_frames[0].save(
            gif_path,
            save_all=True,
            append_images=pil_frames[1:],
            duration=duration,
            loop=0,
            optimize=False,
        )
        logger.info("GIF 已保存: %s (%d 帧, %d ms/帧)", gif_path, len(frames), duration)
    except ImportError:
        logger.warning("Pillow 未安装, 无法生成 GIF。pip install Pillow")


# ─── 独立运行入口 ───
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-7s | %(name)s | %(message)s",
    )

    headless = "--headless" in sys.argv
    save_gif = None
    cycles = -1

    # 解析命令行参数
    for i, arg in enumerate(sys.argv):
        if arg == "--gif" and i + 1 < len(sys.argv):
            save_gif = sys.argv[i + 1]
        if arg == "--cycles" and i + 1 < len(sys.argv):
            try:
                cycles = int(sys.argv[i + 1])
            except ValueError:
                pass

    from electronbot_sim.env import ElectronBotEnv

    env = ElectronBotEnv(render_mode="human" if not (headless or save_gif) else "rgb_array")
    run_visual_demo(env, cycles=cycles, headless=headless, save_gif=save_gif)
