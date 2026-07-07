"""ElectronBot 键盘交互控制脚本。

对齐 Phase 2 §1.1-1.2 功能清单, 提供 6 关节实时键盘控制。
用法:
    python -m electronbot_sim.interactive
    python -m electronbot_sim.interactive --headless  # 无渲染模式 (仅打印状态)
"""
from __future__ import annotations

import logging
import os
import sys
import time

import numpy as np

os.environ.setdefault("MUJOCO_GL", "glfw")

logger = logging.getLogger("electronbot_sim.interactive")

# ─── 按键→关节索引/方向映射 ───
KEY_MAPPING = {
    # (关节索引, delta_方向)
    # 头部: 1=+俯, 2=-俯
    "1": (5, +1.0),
    "2": (5, -1.0),
    # 身体: 3=-转, 4=+转
    "3": (4, -1.0),
    "4": (4, +1.0),
    # 左臂 Pitch: Q=+上, A=-下
    "q": (2, +1.0),
    "a": (2, -1.0),
    # 左臂 Roll: W=+左, S=-右
    "w": (3, +1.0),
    "s": (3, -1.0),
    # 右臂 Pitch: E=+上, D=-下
    "e": (0, +1.0),
    "d": (0, -1.0),
    # 右臂 Roll: R=+左, F=-右
    "r": (1, +1.0),
    "f": (1, -1.0),
}


def _print_status(env, step_deg: float = 3.0) -> None:
    """打印当前控制说明和关节状态。"""
    print("\033[2J\033[H")  # 清屏
    print("=" * 55)
    print("  \U0001f3ae ElectronBot 键盘控制模式")
    print("=" * 55)
    print()
    print("  [1/2] 头部俯仰 (head)       [3/4] 身体旋转 (body)")
    print("  [Q/A] 左臂 Pitch (lp)       [W/S] 左臂 Roll (lr)")
    print("  [E/D] 右臂 Pitch (rp)       [R/F] 右臂 Roll (rr)")
    print(f"  [Space] 复位 home           [Esc] 退出")
    print(f"  [+/-] 步长调整 (当前: {step_deg:.0f}°/步)")
    print()

    angles = env.get_joint_positions()
    # 重排显示顺序: head(5), body(4), lp(2), lr(3), rp(0), rr(1)
    labels = ["head", "body", "lp", "lr", "rp", "rr"]
    idx_order = [5, 4, 2, 3, 0, 1]
    status = "  ".join(
        f"{lbl}={angles[i]:+6.1f}°" for lbl, i in zip(labels, idx_order)
    )
    print(f"  当前状态:")
    print(f"  {status}")
    print("-" * 55)


def run_interactive(env=None, headless: bool = False) -> None:
    """键盘交互主循环。

    参数:
        env: 可选的已创建 ElectronBotEnv 实例, 为 None 时自动创建。
        headless: 为 True 时不启动 mujoco viewer, 仅打印控制台状态。
    """
    render_mode = None if headless else "human"

    if env is None:
        from electronbot_sim.env import ElectronBotEnv
        env = ElectronBotEnv(render_mode=render_mode)

    step_deg = 3.0
    running = True
    reset_requested = False

    # 非阻塞键盘检测 (pygame)
    try:
        import pygame
        import pygame.locals as pg

        pygame.init()
        # 创建隐藏窗口用于捕获键盘事件
        pygame.display.set_mode((200, 100))
        pygame.display.set_caption("ElectronBot Control (focus this window)")

        _print_status(env, step_deg)
        clock = pygame.time.Clock()

        while running:
            # 处理事件
            for event in pygame.event.get():
                if event.type == pg.QUIT:
                    running = False
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_ESCAPE:
                        running = False
                    elif event.key == pg.K_SPACE:
                        reset_requested = True
                    elif event.key in (pg.K_PLUS, pg.K_EQUALS):
                        step_deg = min(15.0, step_deg + 1.0)
                    elif event.key == pg.K_MINUS:
                        step_deg = max(1.0, step_deg - 1.0)

            if not running:
                break

            # 处理 reset
            if reset_requested:
                obs, info = env.reset()
                reset_requested = False
                _print_status(env, step_deg)

            # 轮询按键
            keys = pygame.key.get_pressed()
            action = np.zeros(6, dtype=np.float32)
            for key_str, (joint_idx, direction) in KEY_MAPPING.items():
                key_attr = getattr(pg, f"K_{key_str}", None)
                if key_attr is not None and keys[key_attr]:
                    action[joint_idx] = direction * step_deg

            if np.any(np.abs(action) > 0.01):
                obs, reward, terminated, truncated, info = env.step(action)
                if terminated:
                    logger.info("Episode 终止 (auto_reset_reason=%s)",
                                info.get("auto_reset_reason", "unknown"))
                    env.reset()
                env.render()
                _print_status(env, step_deg)

            # 必要时更新 mujoco viewer
            if not headless:
                env.render()

            clock.tick(50)  # 50Hz

    except ImportError:
        logger.error("pygame 未安装, 无法启用键盘交互。pip install pygame")
        running = False

    except KeyboardInterrupt:
        logger.info("用户中断 Ctrl+C")

    finally:
        env.close()
        try:
            import pygame
            pygame.quit()
        except Exception:
            pass


# ─── 独立运行入口 ───
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-7s | %(name)s | %(message)s",
    )

    headless_flag = "--headless" in sys.argv or os.environ.get(
        "ELECTRONBOT_SIM_HEADLESS") == "1"

    from electronbot_sim.env import ElectronBotEnv

    env = ElectronBotEnv(render_mode="human" if not headless_flag else None)
    run_interactive(env, headless=headless_flag)
