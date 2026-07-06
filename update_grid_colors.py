from pathlib import Path
import sys

xml_path = Path("/home/maple/数据盘/projects/xiaozhi/ElectronBot_SIM/assets/mjcf/electronbot_full_arm.xml")
with open(xml_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace dark checkerboard colors with light blue/white
old_grid = (
    '    <texture name="tex_grid" type="2d" builtin="checker"\r\n'
    '             rgb1="0.25 0.35 0.45" rgb2="0.4 0.5 0.6"\r\n'
    '             width="512" height="512"/>\r\n'
)
new_grid = (
    '    <texture name="tex_grid" type="2d" builtin="checker"\r\n'
    '             rgb1="0.93 0.95 1" rgb2="0.25 0.4 0.6"\r\n'
    '             width="512" height="512"/>\r\n'
)
if old_grid not in content:
    print("ERROR: old_grid not found")
    sys.exit(1)
content = content.replace(old_grid, new_grid, 1)

with open(xml_path, 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(content)

print("Grid texture colors updated")
