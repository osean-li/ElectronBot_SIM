#!/usr/bin/env python3
"""临时脚本：修正 electronbot_full_arm.xml 中的 body/head 位置"""
from pathlib import Path

xml_path = Path(__file__).parent / "assets" / "mjcf" / "electronbot_full_arm.xml"
text = xml_path.read_text(encoding="utf-8")

# body: 腰部在基座上方
text = text.replace(
    '<body name="body" pos="0 0 0.03">',
    '<body name="body" pos="0 0 0.053">'
)
text = text.replace(
    '<body name="body" pos="0 0 0.04">',
    '<body name="body" pos="0 0 0.053">'
)

# head: 坐落在身体壳体之间（低于 body 质心）
text = text.replace(
    '<body name="head" pos="0 0 0.07">',
    '<body name="head" pos="0 0 -0.005">'
)
text = text.replace(
    '<body name="head" pos="0 0 0.046">',
    '<body name="head" pos="0 0 -0.005">'
)
text = text.replace(
    '<body name="head" pos="0 0 0.02">',
    '<body name="head" pos="0 0 -0.005">'
)

xml_path.write_text(text, encoding="utf-8")
print("XML positions updated: body=0.053, head=-0.005")
print("Now run: MUJOCO_GL=egl python3 demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py")
