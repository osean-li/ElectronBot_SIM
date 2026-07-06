# 从 FreeCAD 导出 ElectronBot 零件并生成 MuJoCo MJCF

## 背景

原始 `left_arm.stl` / `right_arm.stl` 是将手臂零件和身体侧外壳（`Part__Feature035`/`036`）合并导出的，导致 MuJoCo 中控制手臂时身体外壳也跟着动。

目标：**身体侧外壳属于 body（不动），只有手臂零件随 Pitch/Roll 旋转。**

---

## 1. FreeCAD 零件分组

从 `electronbot_joints.FCMacro` 获取正确的零件分组：

```python
# 身体组（绕 Z 轴旋转腰部）
BODY_PARTS = [
    "Part__Feature034",  # cx=-1 cz=-9  身体中心
    "Part__Feature035",  # cx=8  cz=-5  身体右侧（肩安装座，容纳电子部件）
    "Part__Feature036",  # cx=-13 cz=-5  身体左侧
]

# 左臂组（绕 Y 轴 Pitch，绕 X 轴 Roll）
LEFT_ARM_PARTS = [
    "Part__Feature042",  # 左手
    "Part__Feature045",  # 左手镜
    "Part__Feature046",  # 臂件
    "Part__Feature047",  # 臂件
    "Part__Feature048",  # 臂件
    "Part__Feature049",  # 臂件
]

# 右臂组（绕 Y 轴 Pitch，绕 X 轴 Roll）
RIGHT_ARM_PARTS = [
    "Part__Feature027",  # 齿轮
    "Part__Feature028",  # 小齿轮
    "Part__Feature029",  # 右手
    "Part__Feature030",  # 臂件
    "Part__Feature031",  # 臂件
    "Part__Feature032",  # 臂件
    "Part__Feature033",  # 臂件
]
```

---

## 2. 用 FreeCAD 命令行导出 STL

```bash
QT_QPA_PLATFORM=offscreen DISPLAY= \
  /path/to/FreeCAD_1.1.1-Linux-x86_64-py311.AppImage \
  --appimage-extract-and-run -c "
import FreeCAD, Part, Mesh, MeshPart, os

doc = FreeCAD.open('cadelectron.FCStd')
OUT = 'assets/meshes'

# 导出身体
BODY = ['Part__Feature034','Part__Feature035','Part__Feature036']
shapes = [doc.getObject(p).Shape for p in BODY if doc.getObject(p)]
body = Part.Compound(shapes)
mesh = doc.addObject('Mesh::Feature', 'tmp')
mesh.Mesh = MeshPart.meshFromShape(Shape=body, LinearDeflection=0.5, AngularDeflection=0.5, Relative=False)
mesh.Mesh.write(os.path.join(OUT, 'body_fc.stl'))
doc.removeObject(mesh.Name)

# 导出左臂
LEFT = ['Part__Feature042','Part__Feature045','Part__Feature046','Part__Feature047','Part__Feature048','Part__Feature049']
shapes = [doc.getObject(p).Shape for p in LEFT if doc.getObject(p)]
larm = Part.Compound(shapes)
mesh = doc.addObject('Mesh::Feature', 'tmp')
mesh.Mesh = MeshPart.meshFromShape(Shape=larm, LinearDeflection=0.5, AngularDeflection=0.5, Relative=False)
mesh.Mesh.write(os.path.join(OUT, 'left_arm_fc.stl'))
doc.removeObject(mesh.Name)

# 导出右臂（同理）
RIGHT = ['Part__Feature027','Part__Feature028','Part__Feature029','Part__Feature030','Part__Feature031','Part__Feature032','Part__Feature033']
shapes = [doc.getObject(p).Shape for p in RIGHT if doc.getObject(p)]
rarm = Part.Compound(shapes)
mesh = doc.addObject('Mesh::Feature', 'tmp')
mesh.Mesh = MeshPart.meshFromShape(Shape=rarm, LinearDeflection=0.5, AngularDeflection=0.5, Relative=False)
mesh.Mesh.write(os.path.join(OUT, 'right_arm_fc.stl'))
doc.removeObject(mesh.Name)

FreeCAD.closeDocument(doc.Name)
"
```

导出结果（包围盒，单位 mm）：

| 文件 | X 范围 | 说明 |
|---|---|---|
| `body_fc.stl` | [-27.9, 27.9] | 含身体侧外壳 |
| `left_arm_fc.stl` | [-34.9, -1.5] | 纯左臂零件 |
| `right_arm_fc.stl` | [1.5, 34.9] | 纯右臂零件 |

---

## 3. 关键问题：MuJoCo 编译时自动中心化 mesh

MuJoCo 加载 inline mesh 时会**将 mesh 质心移到 body 原点**。

例如 `left_arm_fc.stl` 原始 X=[-34.9, -1.5]，质心 X=-18.2。编译后顶点变为 [-16.7, +16.7]（中心在 0）。

### 处理方式

1. **arm body pos** 放在 FreeCAD pivot（`X=±18mm` 即 `±0.018`）
2. **arm_geom pos** 偏移让臂壳完全伸出 body 外

```python
# 计算 geom 偏移量
body_hw = (body_mesh_width) / 2  # body 半宽
arm_hw  = (arm_mesh_width) / 2   # arm 半宽（编译后）

# 左臂需要整体左移：arm右边 ≤ body左边
left_geom_offset  = -(body_hw - abs(left_body_pos) + arm_hw) / 1000
# 右臂需要整体右移：arm左边 ≥ body右边
right_geom_offset = +(body_hw - abs(right_body_pos) + arm_hw) / 1000
```

---

## 4. 最终 MJCF 结构

```
base_link
 └─ body [J1: body_joint, Z轴 ±90°, kp=80]
     ├─ head [J2: head_joint, Y轴 ±15°, kp=40]
     ├─ left_arm pos="-0.018 0 0.065" [FreeCAD pivot]
     │   ├─ [J3: left_pitch_joint, Y轴 ±90°, kp=60]
     │   ├─ [J4: left_roll_joint,  X轴 ±45°, kp=30]
     │   ├─ left_arm_geom pos="-0.025 0 0" (向外伸出)
     │   └─ left_hand → hand_geom
     └─ right_arm pos="0.018 0 0.065"
         ├─ [J5: right_pitch_joint, Y轴 ±90°, kp=60]
         ├─ [J6: right_roll_joint,  X轴 ±45°, kp=30]
         ├─ right_arm_geom pos="0.025 0 0"
         └─ right_hand → hand_geom
```

### Actuator 参数（弧度制）

| 关节 | 舵机 | kp | kv | 范围 |
|---|---|---|---|---|
| body_joint | SG90 | 80 | 20 | ±90° |
| head_joint | 2g | 40 | 10 | ±15° |
| left/right_pitch | 2g | 60 | 15 | ±90° |
| left/right_roll | 2g | 30 | 8 | ±45° |

---

## 5. 生产脚本

最终生成脚本 `gen_full_arm.py`：

```python
#!/usr/bin/env python3
import trimesh, os

MESH_DIR = 'assets/meshes'
OUT_XML = 'assets/mjcf/electronbot_full_arm.xml'

# 加载 STL
meshes = {}
for name, fname in [('base_link','base_link.stl'),('body','body_fc.stl'),
                     ('head','head.stl'),('left_arm','left_arm_fc.stl'),
                     ('right_arm','right_arm_fc.stl')]:
    meshes[name] = trimesh.load(os.path.join(MESH_DIR, fname))

# 计算 arm 偏移
body_hw = (meshes['body'].bounds[1][0] - meshes['body'].bounds[0][0]) / 2
arm_hw  = (meshes['left_arm'].bounds[1][0] - meshes['left_arm'].bounds[0][0]) / 2

left_body_pos = -0.018   # FreeCAD LEFT_ARM pivot
left_geom_off = -(body_hw - abs(left_body_pos*1000) + arm_hw - 1) / 1000.0
right_body_pos = 0.018
right_geom_off = (body_hw - abs(right_body_pos*1000) + arm_hw - 1) / 1000.0

# 生成 inline mesh vertex/face 并组装 MJCF XML ...
```

---

## 6. 运行

```bash
cd ElectronBot_SIM
python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml
```

Control 面板拖动 `act_left_pitch` / `act_left_roll` 等即可控制对应关节。

---

## 7. 经验教训

1. **FreeCAD 的分组信息是关键**：BODY_PARTS 包含了身体侧外壳（Part__Feature035/036），这些不应该被 arm 带动
2. **MuJoCo inline mesh 自动中心化**：质心会移到 body 原点，需用 `geom pos` 补偿偏移
3. **kp 值必须配合弧度制**：kp=0.5-2.0 太低无法驱动，kp=30-80 是弧度制下合理范围
4. **Stable 参数**：`integrator="RK4" iterations="50"` + 适当 damping 避免数值爆炸
