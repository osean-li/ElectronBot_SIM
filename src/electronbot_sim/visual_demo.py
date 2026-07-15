"""visual_demo — 精简对齐版，初始化代码与 01_manual_control.py 逐行一致。"""
import sys, time, logging
from pathlib import Path
import mujoco, mujoco.viewer, numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# ┄┄ 与 manual_control 完全一致的初始化 ┄┄
project = Path(__file__).resolve().parent.parent.parent
xml_path = project / "assets" / "mjcf" / "electronbot_scene.xml"
model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)

act_ids = np.array([
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    for n in ("act_right_pitch","act_right_roll","act_left_pitch",
              "act_left_roll","act_body","act_head")
], dtype=int)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = model.stat.center
    viewer.cam.distance = model.stat.extent * 1.8
    viewer.cam.azimuth = 90   # 正面视角（避免背面手臂重叠造成的视觉误穿模）
    viewer.cam.elevation = -20

    # ┄┄ 在此之前的代码与 manual_control 100% 一致 ┄┄
    logging.info("qpos  = %s", data.qpos[:6])
    logging.info("ctrl  = %s", data.ctrl)
    logging.info("stat  = center %s  extent %.2f", model.stat.center, model.stat.extent)

    # 动作序列
    d = 2.0
    seq = [
        (np.zeros(6), 40),  # 先停2秒确认画面
        (np.array([0,0,0,0,0,+d]), 10),
        (np.array([0,0,0,0,0,-d]), 10),
        (np.array([0,0,0,0,0,-d]), 10),
        (np.array([0,0,0,0,0,+d]), 10),
        (np.array([+d,0,0,0,0,0]), 8),
        (np.array([-d,0,0,0,0,0]), 8),
        (np.array([-d,0,0,0,0,0]), 8),
        (np.array([+d,0,0,0,0,0]), 8),
    ]

    joint_min = np.rad2deg(np.array([model.actuator_ctrlrange[i][0] for i in act_ids]))
    joint_max = np.rad2deg(np.array([model.actuator_ctrlrange[i][1] for i in act_ids]))
    angles = np.zeros(6, dtype=np.float32)

    while viewer.is_running():
        for action, hold in seq:
            for _ in range(hold):
                angles = np.clip(angles + action, joint_min, joint_max)
                data.ctrl[act_ids] = np.radians(angles.astype(np.float64))
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.02)
