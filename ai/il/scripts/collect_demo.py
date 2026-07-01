#!/usr/bin/env python3
"""
ElectronBot 键盘遥操作演示采集器

控制映射:
  W/S     → 头部俯仰 (head_pitch)   +/- 5°
  A/D     → 腰部旋转 (body_yaw)     +/- 5°
  Q/E     → 左臂俯仰 (left_pitch)   +/- 5°
  R/F     → 左臂roll  (left_roll)   +/- 3°
  U/J     → 右臂俯仰 (right_pitch)  +/- 5°
  I/K     → 右臂roll  (right_roll)  +/- 3°
  Z       → 所有关节归零
  Space   → 开始/停止录制
  ESC     → 退出

输出: HDF5 格式 (obs, action) 序列

用法:
  python collect_demo.py --task wave --arm right --output demo_wave.h5
"""

import os
import sys
import argparse
import numpy as np
import h5py
from pathlib import Path
from collections import deque

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation" / "electronbot_mujoco"))

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False
    print("[WARN] pygame 未安装，使用 readchar 模式 (pip install pygame)")

try:
    import readchar
    HAS_READCHAR = True
except ImportError:
    HAS_READCHAR = False

from electronbot_mujoco.robot import ElectronBotRobot
from electronbot_mujoco.utils import JOINT_MODEL_MIN, JOINT_MODEL_MAX


class KeyboardController:
    """键盘遥操作控制器"""

    # 按键 → (关节索引, 增量度数)
    KEY_MAP = {
        # 头部
        'w': (1,  5),   # head +
        's': (1, -5),   # head -
        # 腰部
        'a': (0,  5),   # body +
        'd': (0, -5),   # body -
        # 左臂 pitch
        'q': (2,  5),   # left_pitch +
        'e': (2, -5),   # left_pitch -
        # 左臂 roll
        'r': (3,  3),   # left_roll +
        'f': (3, -3),   # left_roll -
        # 右臂 pitch
        'u': (4,  5),   # right_pitch +
        'j': (4, -5),   # right_pitch -
        # 右臂 roll
        'i': (5,  3),   # right_roll +
        'k': (5, -3),   # right_roll -
    }

    def __init__(self, robot: ElectronBotRobot):
        self.robot = robot
        self.angles = np.zeros(6)  # 当前角度 (度)
        self.keys_pressed = set()

    def apply_key(self, key: str):
        """应用按键"""
        key = key.lower()
        if key == 'z':
            # 归零
            self.angles = np.zeros(6)
        elif key in self.KEY_MAP:
            joint_idx, delta_deg = self.KEY_MAP[key]
            self.angles[joint_idx] += delta_deg
            # Clip to joint limits
            self.angles[joint_idx] = np.clip(
                self.angles[joint_idx],
                JOINT_MODEL_MIN[joint_idx],
                JOINT_MODEL_MAX[joint_idx]
            )

    def sync_to_robot(self):
        """同步角度到 MuJoCo"""
        target_rad = np.radians(self.angles)
        self.robot.send_position_command(target_rad)


class DemoRecorder:
    """演示数据录制器"""

    def __init__(self, output_path: str):
        self.output_path = output_path
        self.episodes = []
        self.current_episode = {"observations": [], "actions": []}
        self.is_recording = False

    def start_recording(self):
        self.is_recording = True
        self.current_episode = {"observations": [], "actions": []}
        print("[REC] 开始录制...")

    def stop_recording(self):
        self.is_recording = False
        if len(self.current_episode["observations"]) > 10:
            self.episodes.append(self.current_episode)
            print(f"[REC] 录制完成: {len(self.current_episode['observations'])} 步")
        else:
            print("[REC] 太短，已丢弃")

    def record_step(self, observation: np.ndarray, action: np.ndarray):
        if self.is_recording:
            self.current_episode["observations"].append(observation.copy())
            self.current_episode["actions"].append(action.copy())

    def save(self):
        """保存所有 episode 到 HDF5"""
        if not self.episodes:
            print("[WARN] 没有数据可保存")
            return

        print(f"\n[SAVE] 保存 {len(self.episodes)} 个 episode 到 {self.output_path}")
        with h5py.File(self.output_path, "w") as f:
            for i, ep in enumerate(self.episodes):
                grp = f.create_group(f"episode_{i:03d}")
                grp.create_dataset("observations", data=np.array(ep["observations"]))
                grp.create_dataset("actions", data=np.array(ep["actions"]))
                grp.attrs["steps"] = len(ep["observations"])

        total_steps = sum(len(ep["observations"]) for ep in self.episodes)
        print(f"[SAVE] 总计 {total_steps} 步, {len(self.episodes)} 个 episode")
        print(f"[SAVE] 文件大小: {os.path.getsize(self.output_path) / 1024:.1f} KB")


def run_pygame_mode(robot, recorder):
    """pygame 图形界面模式"""
    pygame.init()
    screen = pygame.display.set_mode((300, 400))
    pygame.display.set_caption("ElectronBot Teleop - WASD控制")

    controller = KeyboardController(robot)
    clock = pygame.time.Clock()
    running = True

    print("\n" + "=" * 60)
    print("键盘遥操作控制")
    print("  W/S  头部俯仰  A/D  腰部旋转")
    print("  Q/E  左臂俯仰  R/F  左臂roll")
    print("  U/J  右臂俯仰  I/K  右臂roll")
    print("  Z    归零        Space 开始/停止录制")
    print("  ESC  退出")
    print("=" * 60)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    if recorder.is_recording:
                        recorder.stop_recording()
                    else:
                        recorder.start_recording()
                else:
                    key = pygame.key.name(event.key)
                    controller.apply_key(key)

        # 同步到机器人
        controller.sync_to_robot()
        robot.step()

        # 录制
        obs = robot.get_observation()
        action = np.radians(controller.angles)
        recorder.record_step(obs, action)

        # 显示状态
        screen.fill((240, 240, 240))
        font = pygame.font.Font(None, 24)
        text_lines = [
            f"Head:  {controller.angles[1]:6.1f}°",
            f"Body:  {controller.angles[0]:6.1f}°",
            f"LeftP: {controller.angles[2]:6.1f}°  LeftR: {controller.angles[3]:6.1f}°",
            f"RightP:{controller.angles[4]:6.1f}°  RightR:{controller.angles[5]:6.1f}°",
            "",
            f"REC: {'ON' if recorder.is_recording else 'OFF'}",
            f"Steps: {len(recorder.current_episode['observations'])}",
        ]
        for i, line in enumerate(text_lines):
            surf = font.render(line, True, (0, 0, 0))
            screen.blit(surf, (20, 20 + i * 28))
        pygame.display.flip()

        clock.tick(50)

    pygame.quit()
    recorder.save()


def run_terminal_mode(robot, recorder):
    """终端 readchar 模式"""
    if not HAS_READCHAR:
        print("[ERROR] readchar 未安装: pip install readchar")
        return

    controller = KeyboardController(robot)

    print("\n" + "=" * 60)
    print("键盘遥操作控制 (终端模式)")
    print("  按键即时生效, Space=录制, ESC=退出")
    print("=" * 60)

    import threading, time

    running = True

    def input_thread():
        nonlocal running
        while running:
            key = readchar.readkey()
            if key == readchar.key.ESC:
                running = False
                break
            elif key == readchar.key.SPACE:
                if recorder.is_recording:
                    recorder.stop_recording()
                else:
                    recorder.start_recording()
            else:
                controller.apply_key(key)
            # 实时同步
            controller.sync_to_robot()

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    print("\n开始控制 (ESC 退出)...")
    while running:
        robot.step()
        obs = robot.get_observation()
        action = np.radians(controller.angles)
        recorder.record_step(obs, action)
        time.sleep(0.02)  # 50Hz

    recorder.save()


def main():
    parser = argparse.ArgumentParser(description="ElectronBot 演示采集")
    parser.add_argument("--output", type=str, default="demos.h5",
                        help="输出 HDF5 文件")
    parser.add_argument("--mode", type=str, default="auto",
                        choices=["auto", "pygame", "terminal"])
    args = parser.parse_args()

    print("[INFO] 初始化 ElectronBot MuJoCo...")
    robot = ElectronBotRobot()
    robot.reset()

    recorder = DemoRecorder(args.output)

    # 选择模式
    mode = args.mode
    if mode == "auto":
        mode = "pygame" if HAS_PYGAME else "terminal"

    if mode == "pygame" and HAS_PYGAME:
        run_pygame_mode(robot, recorder)
    else:
        run_terminal_mode(robot, recorder)

    print(f"\n[DONE] 演示数据已保存: {args.output}")


if __name__ == "__main__":
    main()
