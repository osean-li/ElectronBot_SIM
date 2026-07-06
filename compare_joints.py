#!/usr/bin/env python3
"""
对比Mujoco和FreeCAD的关节定义差异
"""

print("="*70)
print("对比分析: Mujoco vs FreeCAD 关节定义")
print("="*70)

print("\n【1】FreeCAD 关节定义 (来自 electronbot_joints.FCMacro)")
print("-"*70)
print("""
关节列表:
┌─────────────────┬──────────────┬──────────┬────────┬───────┐
│ 名称             │ 描述          │ 轴       │ 范围   │ 零件组 │
├─────────────────┼──────────────┼──────────┼────────┼───────┤
│ body_z           │ 腰部旋转      │ Z轴     │ ±90°   │ body  │
│ head_y           │ 头部俯仰      │ Y轴     │ ±30°   │ head  │
│ left_pitch       │ 左臂 Pitch    │ Y轴 ⚠️  │ ±90°   │ left_arm │
│ left_roll        │ 左臂 Roll     │ X轴 ⚠️  │ ±45°   │ left_arm │
│ right_pitch      │ 右臂 Pitch    │ Y轴     │ ±90°   │ right_arm│
│ right_roll       │ 右臂 Roll     │ X轴     │ ±45°   │ right_arm│
└─────────────────┴──────────────┴──────────┴────────┴───────┘

⚠️ 关键点:
  • 左/右臂的 Pitch 绕 Y轴
  • 左/右臂的 Roll 绕 X轴
  • 没有单独的 shoulder 层级！arm 组同时包含 pitch 和 roll
""")

print("\n【2】Mujoco 关节定义 (来自 electronbot_mesh.xml)")
print("-"*70)
print("""
身体层次结构:
world
 └─ base_link (固定底座)
     └─ body (身体)
         ├─ body_joint [hinge, Z轴, ±90°]        ← 对应 body_z ✓
         ├─ head
         │   ├─ head_joint [hinge, Y轴, ±15°]     ← 对应 head_y ✓
         ├─ left_shoulder                         ⚠️ 单独的shoulder层!
         │   ├─ left_shoulder_joint [hinge, X轴, ±90°]  ← 对应 left_pitch? 但轴是X不是Y!
         │   ├─ left_arm_geom (手臂mesh在这里!)
         │   └─ left_arm                           ⚠️ arm是shoulder的子节点!
         │       ├─ left_arm_roll_joint [hinge, Z轴, ±45°]  ← 对应 left_roll? 但轴是Z不是X!
         │       └─ left_hand_geom
         ├─ right_shoulder                        ⚠️ 同样结构!
         │   ├─ right_shoulder_joint [hinge, X轴]
         │   ├─ right_arm_geom
         │   └─ right_arm
         │       ├─ right_arm_roll_joint [hinge, Z轴]
         │       └─ right_hand_geom

执行器映射:
┌──────────────────┬───────────────────────┬───────┬───────┐
│ 执行器            │ 目标关节               │ 轴    │ 范围  │
├──────────────────┼───────────────────────┼───────┼───────┤
│ act_body         │ body_joint            │ Z    │ ±90° │
│ act_head         │ head_joint            │ Y    │ ±15° │
│ act_left_shoulder│ left_shoulder_joint   │ X⚠️  │ ±90° │  ← 应该是Y?
│ act_left_arm     │ left_arm_roll_joint   │ Z⚠️  │ ±45° │  ← 应该是X?
│ act_right_shoulder│ right_shoulder_joint  │ X    │ ±90° │
│ act_right_arm    │ right_arm_roll_joint  │ Z    │ ±45° │
└──────────────────┴───────────────────────┴───────┴───────┘
""")

print("\n【3】🚨 发现的关键问题")
print("="*70)
print("""
问题1: 旋转轴不匹配!
─────────────────────────────────────────────
                    FreeCAD          Mujoco        匹配?
左臂 Pitch (肩部)     Y轴              X轴           ❌ 不匹配!
左臂 Roll (肘部)      X轴              Z轴           ❌ 不匹配!

问题2: 身体层级结构不同!
─────────────────────────────────────────────
FreeCAD:
  • 没有 shoulder/arm 分层
  • 直接控制 left_arm 组 (包含所有臂部零件)
  • 一个组同时做 Pitch(Y) + Roll(X)

Mujoco:
  • 有 shoulder → arm 两层结构
  • shoulder 控制 Pitch (但轴错了?)
  • arm 控制 Roll (但轴也错了?)

问题3: 从截图观察到的现象!
─────────────────────────────────────────────
您的截图显示:
  FreeCAD: left_roll = -45° (X轴向负方向) → 臂向内收
  Mujoco:  act_left_arm = +0.675rad (+38.7°, Z轴正向) → 臂向外展?

如果视觉效果相似但符号相反，说明:
  ✅ 可能只是坐标系方向定义不同
  ✅ 或者正负号约定相反
""")

print("\n【4】验证假设: 检查实际旋转效果")
print("="*70)

import mujoco
import numpy as np

# 加载模型
model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh.xml')
data = mujoco.MjData(model)

print("""
测试: 设置正值的 act_left_arm 看实际运动方向
""")

# 重置并设置控制值
data.qpos[:] = 0
data.ctrl[3] = 0.5  # 正值，约28.6度

# 运行到稳态
for _ in range(3000):
    mujoco.mj_step(model, data)

left_arm_angle = data.qpos[3]

print(f"设置: act_left_arm = +0.5 rad (+28.6°)")
print(f"结果: left_arm_roll_joint = {left_arm_angle:.4f} rad ({np.degrees(left_arm_angle):.2f}°)")

# 获取位置信息判断运动方向
left_hand_pos = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_hand_geom")]
base_pos = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "body_geom")]

print(f"\n几何体位置:")
print(f"  body_geom中心: x={base_pos[0]:.4f}, y={base_pos[1]:.4f}")
print(f"  left_hand_geom中心: x={left_hand_pos[0]:.4f}, y={left_hand_pos[1]:.4f}")

# 计算相对位移 (应该能看到y方向的变化)
delta_y = left_hand_pos[1] - base_pos[1]
print(f"\n左手相对于身体的 Y偏移: {delta_y:.4f}m")

if delta_y > 0.001:
    print("  → 正值导致 hand 向 +Y 方向移动 (根据坐标系可能是外展或内收)")
elif delta_y < -0.001:
    print("  → 正值导致 hand 向 -Y 方向移动")

print("\n【5】解决方案建议")
print("="*70)
print("""
方案A: 修正Mujoco关节轴以匹配FreeCAD (推荐)
─────────────────────────────────────────────────
修改 electronbot_mesh.xml:

1. 将 left_shoulder_joint 的轴从 X(1,0,0) 改为 Y(0,1,0):
   <joint name="left_shoulder_joint" type="hinge" axis="0 1 0" .../>

2. 将 right_shoulder_joint 的轴从 X(1,0,0) 改为 Y(0,1,0):
   <joint name="right_shoulder_joint" type="hinge" axis="0 1 0" .../>

3. 将 left_arm_roll_joint 的轴从 Z(0,0,1) 改为 X(1,0,0):
   <joint name="left_arm_roll_joint" type="hinge" axis="1 0 0" .../>

4. 将 right_arm_roll_joint 的轴从 Z(0,0,1) 改为 X(1,0,0):
   <joint name="right_arm_roll_joint" type="hinge" axis="1 0 0" .../>

方案B: 保持现有结构但调整正负号
─────────────────────────────────────────────────
如果轴的定义是有意为之（例如匹配真实硬件），
只需要确保文档说明清楚即可。

方案C: 简化结构 - 移除shoulder层 (像FreeCAD一样)
─────────────────────────────────────────────────
将 arm_mesh 直接挂在 body 下，只用一个关节控制。
但这会改变物理行为。

下一步行动建议:
─────────────
1. 先确认哪个是"正确"的标准 (硬件? FreeCAD? 其他参考?)
2. 根据标准统一 Mujoco 的定义
3. 更新文档说明关节映射关系
""")

if __name__ == "__main__":
    pass