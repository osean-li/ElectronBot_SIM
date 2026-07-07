#!/usr/bin/env python3
"""根据缩放后的 mesh 质心重新计算 body/geom 位置，使零件正确连接。"""
import re
import numpy as np
from pathlib import Path

MJCF_DIR = Path(__file__).resolve().parent.parent / "assets" / "mjcf"
ROBOT_PATH = MJCF_DIR / "electronbot.xml"

txt = ROBOT_PATH.read_text()

if 'scale="0.001' not in txt:
    raise RuntimeError("mesh 缺少 scale")

# ── 提取 BBox ──
def bbox(name, scale=0.001):
    m = re.search(f'<mesh[^>]+name="{name}"[^>]+vertex="([^"]+)"', txt)
    v = np.array([float(x) for x in m.group(1).split()]).reshape(-1, 3) * scale
    return v.min(0), v.max(0)

bmin, bmax = {}, {}
for n in ["base_link", "body", "head", "left_arm", "right_arm"]:
    bmin[n], bmax[n] = bbox(n)

# ── 计算 body 位置 ──
# base_link: mesh bottom at world z=0
base_z = -bmin["base_link"][2]
print(f"base_link    z ={base_z:.5f}  (visual z=[{bmin['base_link'][2]+base_z:.4f}, {bmax['base_link'][2]+base_z:.4f}])")

# body: bottom touches base top  (所有 z 用相对偏移)
body_z_rel = bmax["base_link"][2] - bmin["body"][2]
print(f"body         z ={body_z_rel:.5f}  (world vis z=[{bmin['body'][2]+base_z+body_z_rel:.4f}, {bmax['body'][2]+base_z+body_z_rel:.4f}])")

# head: bottom touches body top
body_world = base_z + body_z_rel
head_z_rel = bmax["body"][2] - bmin["head"][2]
print(f"head         z ={head_z_rel:.5f}  (world vis z=[{bmin['head'][2]+body_world+head_z_rel:.4f}, {bmax['head'][2]+body_world+head_z_rel:.4f}])")

# 臂：body 高度约 0.05m，挂在前 60% 处
arm_z_rel = (bmax["body"][2] - bmin["body"][2]) * 0.6 + bmin["body"][2]
left_arm_x = -0.030
right_arm_x = 0.030
print(f"left_arm     pos=({left_arm_x}, 0, {arm_z_rel:.5f})")
print(f"right_arm    pos=({right_arm_x}, 0, {arm_z_rel:.5f})")

# hand pos（相对于 arm body）
left_arm_len = bmax["left_arm"][0] - bmin["left_arm"][0]
right_arm_len = bmax["right_arm"][0] - bmin["right_arm"][0]
left_hand_pos = f"{left_arm_len:.5f} 0 0"
right_hand_pos = f"{-right_arm_len:.5f} 0 0"
print(f"left_hand    pos=({left_hand_pos})")
print(f"right_hand   pos=({right_hand_pos})")

# ── 替换 XML ──
def rpl(pattern, repl):
    global txt
    txt, n = re.subn(pattern, repl, txt)
    if n == 0:
        print(f"  WARNING: pattern not matched: {pattern[:60]}...")

rpl(r'<body name="base_link" pos="[^"]*"',
    f'<body name="base_link" pos="0 0 {base_z:.5f}"')
rpl(r'<body name="body" pos="[^"]*"',
    f'<body name="body" pos="0 0 {body_z_rel:.5f}"')
rpl(r'<body name="head" pos="[^"]*"',
    f'<body name="head" pos="0 0 {head_z_rel:.5f}"')
rpl(r'<body name="left_arm" pos="[^"]*"',
    f'<body name="left_arm" pos="{left_arm_x:.5f} 0 {arm_z_rel:.5f}"')
rpl(r'<body name="right_arm" pos="[^"]*"',
    f'<body name="right_arm" pos="{right_arm_x:.5f} 0 {arm_z_rel:.5f}"')
rpl(r'<body name="left_hand" pos="[^"]*"',
    f'<body name="left_hand" pos="{left_hand_pos}"')
rpl(r'<body name="right_hand" pos="[^"]*"',
    f'<body name="right_hand" pos="{right_hand_pos}"')

# 清除 arm geom 显式 pos（由 body 定位）
rpl(r'<geom name="left_arm_geom" type="mesh" mesh="left_arm" pos="[^"]*"',
    '<geom name="left_arm_geom" type="mesh" mesh="left_arm"')
rpl(r'<geom name="right_arm_geom" type="mesh" mesh="right_arm" pos="[^"]*"',
    '<geom name="right_arm_geom" type="mesh" mesh="right_arm"')

ROBOT_PATH.write_text(txt)
print("\n✔ electronbot.xml 已更新")
