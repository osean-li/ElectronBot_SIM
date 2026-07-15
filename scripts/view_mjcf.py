#!/usr/bin/env python3
"""简单的 MJCF 模型可视化脚本"""

import argparse
import os
from pathlib import Path
import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description="可视化 MJCF 模型")
    parser.add_argument("--xml", default="assets/mjcf/electronbot_step_meters.xml",
                        help="MJCF XML 文件路径")
    args = parser.parse_args()

    # 切换到项目根目录
    project_root = Path(__file__).parent.parent
    os.chdir(project_root)

    xml_path = Path(args.xml)
    print(f"加载模型: {xml_path}")
    print(f"工作目录: {os.getcwd()}")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    print(f"模型信息:")
    print(f"  body 数量: {model.nbody}")
    print(f"  joint 数量: {model.njnt}")
    print(f"  geom 数量: {model.ngeom}")
    print(f"  actuator 数量: {model.nu}")

    print("\n启动可视化窗口...")
    print("  - 鼠标左键拖动旋转视角")
    print("  - 鼠标右键拖动平移视角")
    print("  - 滚轮缩放")
    print("  - 按 ESC 或关闭窗口退出")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 可选：在这里添加控制逻辑
            pass


if __name__ == "__main__":
    main()
