# electronBot FreeCAD 关节控制指南

## 文件说明

| 文件 | 格式 | 大小 | 说明 |
|------|------|------|------|
| `ElectronBot.step` | STEP | 30.5 MB | 通用 CAD 交换格式，可用 FreeCAD/Fusion 360/Blender 打开 |
| `cadelectron.FCStd` | FreeCAD | 8.6 MB | FreeCAD 原生格式，保留 Assembly 结构（Assembly001~007） |
| `electronbot_joints.FCMacro` | FreeCAD 宏 | - | 关节控制宏，拖动滑块控制 6 个关节旋转 |

## 快速开始（3 步让关节动起来）

### 1. 打开模型

双击 `cadelectron.FCStd`，FreeCAD 加载需几十秒。

### 2. 运行关节控制宏

```
FreeCAD 菜单 → 宏(M) → 宏(C)...
→ 选中 electronbot_joints → 执行
```

### 3. 拖动滑块

弹出控制面板，6 个滑块分别控制：

| 滑块 | 旋转轴 | 范围 | 效果 |
|------|--------|------|------|
| 腰部旋转 | Z 轴 | -90° ~ 90° | 身体左右旋转 |
| 头部俯仰 | Y 轴 | -30° ~ 30° | 头部上下点头 |
| 左臂 Pitch | Y 轴 | -90° ~ 90° | 左臂上下摆动 |
| 左臂 Roll | X 轴 | -45° ~ 45° | 左臂前后扭转 |
| 右臂 Pitch | Y 轴 | -90° ~ 90° | 右臂上下摆动 |
| 右臂 Roll | X 轴 | -45° ~ 45° | 右臂前后扭转 |

---

## 技术原理

### 模型结构

STEP 文件导入 FreeCAD 后，所有零件嵌套在一个 `App::Part` 对象 `_X2_52A05DE54EF6_X0_` 中，包含 24 个子零件：

```
_X2_52A05DE54EF6_X0_ (App::Part, 24 子对象)
├── Part__Feature027    齿轮 (16 teeth)      → 右臂组
├── Part__Feature028    齿轮 (12 teeth)      → 右臂组
├── Part__Feature029    右手                 → 右臂组
├── Part__Feature030    右臂零件             → 右臂组
├── Part__Feature031    右臂零件             → 右臂组
├── Part__Feature032    右臂零件             → 右臂组
├── Part__Feature033    右臂零件             → 右臂组
├── Part__Feature034    身体中心 (vol=16545) → 身体组
├── Part__Feature035    身体右侧 (vol=18741) → 身体组
├── Part__Feature036    身体左侧 (vol=18652) → 身体组
├── Part__Feature037    前脸 (vol=2428)      → 头部组
├── Part__Feature038    头顶 (vol=9195)      → 头部组
├── Part__Feature039    头部主壳 (vol=14318)  → 头部组
├── Part__Feature040    小件                 → 头部组
├── Part__Feature041    小件                 → 头部组
├── Part__Feature042    左手                 → 左臂组
├── Part__Feature043    底座 (vol=21440)     → 固定
├── Part__Feature044    底座底 (vol=8198)    → 固定
├── Part__Feature045    左手镜               → 左臂组
├── Part__Feature046    左臂零件             → 左臂组
├── Part__Feature047    左臂零件             → 左臂组
├── Part__Feature048    左臂零件             → 左臂组
├── Part__Feature049    左臂零件             → 左臂组
└── Origin046           原点
```

### 关节分组与旋转中心

| 组 | 零件数 | 旋转中心 (cx, cy, cz) | 说明 |
|----|--------|----------------------|------|
| 身体 (body) | 3 | 约 (0, 0, -6) | 绕腰部 Z 轴旋转 |
| 头部 (head) | 5 | 约 (3, 0, 25) | 绕颈部 Y 轴俯仰 |
| 左臂 (left_arm) | 6 | 约 (-17, 0, 0) | 绕肩部 Y/X 轴运动 |
| 右臂 (right_arm) | 7 | 约 (17, 0, 0) | 绕肩部 Y/X 轴运动 |
| 底座 (base) | 2 | - | 固定不动 |

### 关键踩坑

1. **不能旋转 App::Part 容器** — 修改容器的 Placement 不会带动子零件
2. **必须直接操作子零件 Placement** — `child.Placement = Placement(...)` 
3. **绕非原点的旋转** — 需要用 `rel = pos - pivot` → 旋转 → `+ pivot` 公式
4. **复合旋转顺序** — Z → Y → X 的顺序构建 `Rotation` 对象

---

## 其他可行方案

| 方案 | 软件 | 难度 | 效果 |
|------|------|------|------|
| 本方案 | FreeCAD + 宏 | 已实现 | 6 轴控制 |
| Fusion 360 | Autodesk | 简单 | 图形化关节设置 |
| Blender + Armature | Blender | 中等 | 动画渲染 |
| Assembly4 工作台 | FreeCAD | 复杂 | 完整装配约束 |

---

## 文件位置

```
electronbot-docs/docs/cad/
├── ElectronBot.step          # 原始 STEP 文件
├── cadelectron.FCStd         # FreeCAD 原生文件
├── freecad-joint-control.md  # 本文档
└── ../electronbot_joints.FCMacro  # 关节控制宏（项目根目录）
```
