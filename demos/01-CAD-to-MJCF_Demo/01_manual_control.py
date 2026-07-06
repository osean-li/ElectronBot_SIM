#!/usr/bin/env python3
"""
Demo 1: 手动控制 — 启动 MuJoCo viewer 拖动 actuator slider

用法:
  # SSH/服务器 (无桌面)
  MUJOCO_GL=egl python demos/01-CAD-to-MJCF_Demo/01_manual_control.py

  # 有桌面
  python demos/01-CAD-to-MJCF_Demo/01_manual_control.py

效果:
  - 打开 MuJoCo viewer 窗口
  - 右侧 Control 面板有 6 个 actuator slider：
      act_body        → 腰部旋转 (Z轴, ±90°)
      act_head        → 头部俯仰 (Y轴, ±15°)
      act_left_pitch  → 左臂 Pitch (Y轴, ±90°)
      act_left_roll   → 左臂 Roll  (X轴, ±45°)
      act_right_pitch → 右臂 Pitch (Y轴, ±90°)
      act_right_roll  → 右臂 Roll  (X轴, ±45°)
  - 拖动 slider 即可控制对应关节

快捷键:
  空格  暂停/继续仿真
  R     重置
  V     切换透明模式
  H     显示帮助
  滚轮  缩放
  右键  拖拽旋转视角
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import mujoco
import mujoco.viewer


def main():
    # ── 模型路径 ──
    project = Path(__file__).resolve().parent.parent.parent
    xml_path = project / "assets" / "mjcf" / "electronbot_full_arm.xml"

    if not xml_path.exists():
        print(f"错误: 模型文件不存在: {xml_path}")
        sys.exit(1)

    # 自动检测桌面环境
    if not os.environ.get("DISPLAY"):
        # 尝试常见 display 和 Xauthority
        candidates = [
            (":0", "/run/user/1000/gdm/Xauthority"),
            (":0", os.path.expanduser("~/.Xauthority")),
            (":1", "/run/user/1000/gdm/Xauthority"),
            (":99", None),  # Xvfb
        ]
        found = False
        for display, auth in candidates:
            auth_path = auth if auth and Path(auth).exists() else None
            if auth_path:
                os.environ["XAUTHORITY"] = auth_path
            os.environ["DISPLAY"] = display
            # 快速测试能否连接
            try:
                from subprocess import run, DEVNULL
                result = run(["xdpyinfo", "-display", display],
                             capture_output=True, timeout=2)
                if result.returncode == 0:
                    found = True
                    print(f"[自动检测] DISPLAY={display}, XAUTHORITY={auth_path}")
                    break
            except Exception:
                continue

        if not found:
            print("=" * 60)
            print("  无法打开交互窗口: 未检测到可用的 X11 显示")
            print()
            print("  替代方案:")
            print("    1. 启动 Xvfb 虚拟桌面:")
            print("       Xvfb :99 -screen 0 1024x768x24 &")
            print("       DISPLAY=:99 python3 demos/01-CAD-to-MJCF_Demo/01_manual_control.py")
            print()
            print("    2. 程序控制演示 (无需桌面):")
            print("       MUJOCO_GL=egl python3 demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py")
            print("=" * 60)
            return

    print("=" * 60)
    print("  ElectronBot 手动控制演示")
    print(f"  模型: {xml_path.name}")
    print()
    print("  右侧 Control 面板 → 拖动 actuator slider 控制关节")
    print("  空格键 暂停/继续")
    print("  右键+拖拽 旋转视角 | 滚轮 缩放")
    print("=" * 60)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        msg = str(e)
        if "GLFW" in msg or "DISPLAY" in msg or "could not initialize" in msg:
            print()
            print("=" * 60)
            print("  无法打开交互窗口")
            print("  mujoco.viewer 需要桌面环境 (X11/Wayland)")
            print()
            print("  替代方案:")
            print("    1. 在有桌面的机器上运行此脚本")
            print("    2. 用程序控制演示 (无需桌面):")
            print("       MUJOCO_GL=egl python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py")
            print("    3. 直接用 mujoco.viewer 命令行:")
            print("       python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml")
            print("=" * 60)
        else:
            raise
