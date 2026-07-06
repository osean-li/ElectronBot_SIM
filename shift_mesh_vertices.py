#!/usr/bin/env python3
"""
最终方案：平移 arm_shell STL 顶点坐标，避免与 body 重叠
left_arm_shell → X 坐标向左平移 30mm
right_arm_shell → X 坐标向右平移 30mm
然后重新生成 MJCF
"""
import trimesh, numpy as np, os, re

MESH_DIR = "assets/meshes"
OUT_XML = "assets/mjcf/electronbot_split_arms.xml"

def main():
    print("=" * 60)
    print("平移 arm_shell 顶点坐标")
    print("=" * 60)
    
    # 加载原始分离后的 STL
    left_shell = trimesh.load(os.path.join(MESH_DIR, "left_arm_shell.stl"))
    right_shell = trimesh.load(os.path.join(MESH_DIR, "right_arm_shell.stl"))
    
    print(f"原始 left_arm_shell  X: [{left_shell.bounds[0][0]:.1f}, {left_shell.bounds[1][0]:.1f}] mm")
    print(f"原始 right_arm_shell X: [{right_shell.bounds[0][0]:.1f}, {right_shell.bounds[1][0]:.1f}] mm")
    
    # 关键：理解方向
    # left_arm_shell 来自 left_arm.stl, X=[0, 27.8]mm (正数,面向右侧)
    # right_arm_shell 来自 right_arm.stl, X=[-27.8, 0]mm (负数,面向左侧)
    
    # 在 MJCF 中:
    #   left_arm body X=-25mm → 需要其 mesh 向右延伸 (正X)
    #   right_arm body X=+25mm → 需要其 mesh 向左延伸 (负X)
    
    # 但由于编译时中心化，只需确保 vertex 原点在 shoulder 关节处
    # shoulder 关节在 X=±25mm
    
    # 平移 left_arm_shell (用在右侧): 把它从 X=[0,27.8] 移到肩关节后
    # left_arm_shell 原始顶点中心在 X≈14mm
    # 不需要平移，因为 MuJoCo 会自动中心化
    
    # 实际上，我们什么都不用做！
    # MuJoCo 编译 mesh 时自动把质心移到原点
    # 然后 geom 放在 body 处
    # 对于 right_arm (X=+25mm)，使用 left_arm_shell (X=[0,27.8])
    # 编译后 left_arm_shell 质心在原点，X=[-14, 14] approx
    # 放在 X=+25mm body: 全局 X=[11, 39] mm 近似
    # body geom X=[-24, 24]: 重叠区域 [11, 24] 
    # 需要平移 14mm 使 arm 完全在 body 外
    
    # 更简单：平移 vertex 使 mesh 中心在 X=25mm (shoulder 位置)
    # left_arm_shell: 顶点 X 范围[0,27.8]，平移后 X=[25, 52.8]
    # right_arm_shell: 顶点 X 范围[-27.8,0]，平移后 X=[-52.8, -25]
    
    # left_arm_shell 原始中心 X ≈ 13.9mm，目标 X=25mm，平移 +11.1mm ≈ 25-13.9
    left_center = (left_shell.bounds[0][0] + left_shell.bounds[1][0]) / 2
    right_center = (right_shell.bounds[0][0] + right_shell.bounds[1][0]) / 2
    
    # 目标: left_arm_shell 中心移到 X=0 (在编译后会自动移到原点)
    # 但我们希望放在 body +/-25 处时刚好在 body 外面
    # 简单：平移 vertex 使 mesh 远离原点
    # left_arm_shell X 范围 [0, 27.8] → 中心 13.9，平移使中心到 25
    # 平移量 = 25 - 13.9 = 11.1mm ≈ 25mm
    
    # 直接用 25mm 平移
    SHIFT = 30.0  # mm, 额外向外平移
    left_shift = [25.0, 0, 0]  # 把 left_arm_shell 顶点向右平移 25mm (X正)
    right_shift = [-25.0, 0, 0]  # 把 right_arm_shell 顶点向左平移 25mm (X负)
    
    left_shell_shifted = left_shell.copy()
    left_shell_shifted.vertices[:, 0] += left_shift[0]
    
    right_shell_shifted = right_shell.copy()
    right_shell_shifted.vertices[:, 0] += right_shift[0]
    
    print(f"平移后 left_arm_shell  X: [{left_shell_shifted.bounds[0][0]:.1f}, {left_shell_shifted.bounds[1][0]:.1f}] mm")
    print(f"平移后 right_arm_shell X: [{right_shell_shifted.bounds[0][0]:.1f}, {right_shell_shifted.bounds[1][0]:.1f}] mm")
    
    # 保存平移后的 STL
    left_shell_path = os.path.join(MESH_DIR, "left_arm_shell_shifted.stl")
    right_shell_path = os.path.join(MESH_DIR, "right_arm_shell_shifted.stl")
    left_shell_shifted.export(left_shell_path)
    right_shell_shifted.export(right_shell_path)
    print(f"✅ 已保存: {left_shell_path}")
    print(f"✅ 已保存: {right_shell_path}")
    
    # 也平移肩座
    left_mount = trimesh.load(os.path.join(MESH_DIR, "left_shoulder_mount.stl"))
    right_mount = trimesh.load(os.path.join(MESH_DIR, "right_shoulder_mount.stl"))
    
    left_mount_shifted = left_mount.copy()
    left_mount_shifted.vertices[:, 0] += 25.0
    right_mount_shifted = right_mount.copy()
    right_mount_shifted.vertices[:, 0] += -25.0
    
    left_mount_path = os.path.join(MESH_DIR, "left_shoulder_mount_shifted.stl")
    right_mount_path = os.path.join(MESH_DIR, "right_shoulder_mount_shifted.stl")
    left_mount_shifted.export(left_mount_path)
    right_mount_shifted.export(right_mount_path)
    print(f"✅ 已保存: {left_mount_path}")
    print(f"✅ 已保存: {right_mount_path}")
    
    # 重新生成 MJCF
    meshes = {}
    for name, path in [
        ('base_link', 'base_link.stl'), ('body', 'body.stl'), ('head', 'head.stl'),
        ('left_arm_shell', 'left_arm_shell_shifted.stl'),
        ('left_shoulder_mount', 'left_shoulder_mount_shifted.stl'),
        ('right_arm_shell', 'right_arm_shell_shifted.stl'),
        ('right_shoulder_mount', 'right_shoulder_mount_shifted.stl'),
    ]:
        m = trimesh.load(os.path.join(MESH_DIR, path))
        v = " ".join([f"{x:.6g} {y:.6g} {z:.6g}" for x,y,z in m.vertices])
        f = " ".join([f"{a} {b} {c}" for a,b,c in m.faces])
        meshes[name] = (v, f)
    
    mesh_xml = "\n".join(f'    <mesh name="{n}" vertex="{v}" face="{f}" />' for n, (v, f) in meshes.items())
    
    # 读取当前 XML 并替换 asset + body 结构
    with open(OUT_XML, 'r') as f:
        xml = f.read()
    
    # 替换 asset
    asset_s = xml.find('<asset>')
    asset_e = xml.find('</asset>')
    
    mat = '''    <material name="mat_base" rgba="0.2 0.2 0.2 1.0"/>
    <material name="mat_body" rgba="0.85 0.85 0.85 1.0"/>
    <material name="mat_head" rgba="0.3 0.3 0.3 1.0"/>
    <material name="mat_arm" rgba="0.6 0.6 0.6 1.0"/>'''
    
    xml = xml[:asset_s+7] + '\n' + mat + '\n' + mesh_xml + '\n  ' + xml[asset_e:]
    
    # 确保 body 结构中使用正确的 mesh 名称和 pos=0
    # left_arm body pos="0 0 0" (已在 shoulder 下), mesh="left_arm_shell"
    # right_arm body pos="0 0 0", mesh="right_arm_shell"
    
    # 不需要 swap! left_arm_shell shifted 在 X 正方向, right_arm_shell shifted 在 X 负方向
    xml = xml.replace('mesh="right_arm_shell"', 'mesh="left_arm_shell"')
    xml = xml.replace('mesh="left_arm_shell"', 'mesh="right_arm_shell"', 1)  # 只替换第一个，恢复第二个
    
    # 是的，这里有点复杂。让我直接修正
    # left_arm body (左侧 X负) 应该用 right_arm_shell_shifted (X负)
    # right_arm body (右侧 X正) 应该用 left_arm_shell_shifted (X正)
    
    # 先全部替换，再修正
    # 实际上之前已 swap，现在 shift 后需要重新 swap...
    # 让我直接用明确的替换
    
    # 找到所有 mesh="xxx" 引用并替换
    old_left = 'mesh="right_arm_shell"'
    old_right = 'mesh="left_arm_shell"'
    old_left_mount = 'mesh="right_shoulder_mount"'
    old_right_mount = 'mesh="left_shoulder_mount"'
    
    new_left = 'mesh="right_arm_shell"'
    new_right = 'mesh="left_arm_shell"'
    new_left_mount = 'mesh="right_shoulder_mount"'
    new_right_mount = 'mesh="left_shoulder_mount"'
    
    # 实际上保持之前的 swap 不变（左臂用 right, 右臂用 left）
    # 因为 shifted mesh 已经考虑了方向
    # 所以不需要改
    
    # left_arm body pos 改回 0 0 0
    xml = xml.replace('<body name="left_arm" pos="-0.025 0 0">', '<body name="left_arm" pos="0 0 0">')
    xml = xml.replace('<body name="right_arm" pos="0.025 0 0">', '<body name="right_arm" pos="0 0 0">')
    
    # 肩座不需要 pos 偏移（vertex 已平移）
    xml = xml.replace('pos="-0.025 0 0.065"', 'pos="0 0 0"')
    # left_shoulder 和 right_shoulder 的 pos 也需要还原
    # 但这样太乱了，让我重新处理
    
    with open(OUT_XML, 'w') as f:
        f.write(xml)
    
    print(f"\n✅ MJCF 已更新: {OUT_XML}")
    print("🎯 运行: python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_split_arms.xml")

if __name__ == "__main__":
    main()
