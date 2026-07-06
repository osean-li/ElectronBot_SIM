# electronBot 机器人结构解读

## 概述

electronBot 是一个桌面级小型人形机器人，共 **6 个自由度（6-DOF）**，由 6 个舵机驱动（一舵机一轴）。

在 FreeCAD 中打开 `cadelectron.FCStd` 可看到完整装配结构：左侧为零件树（组件列表），右侧为 3D 模型。透过半透明外壳可观察到内部舵机、齿轮、舵机臂、PCB 等关键部件的布局。关节控制面板可实时调节各关节角度，观察运动效果。

```
文件: ElectronBot.step (30.5 MB)
格式: STEP AP214 (ISO 10303)
零件数: 24 个独立零件 + 舵机组件 + PCB
```

---

## 自由度分布

electronBot 共有 **6 个旋转关节（revolute/hinge）**，每个关节由一个舵机独立驱动，构成 6-DOF 串联运动链。

### 运动学结构总览

```
                            头部 [J2]
                           │  Z↑
                    ○──────┤  ◎──→ Y (俯仰, -30°~+30°)
                   / 颈部   │
                  /         │
     左臂 [J3][J4]   身体 [J1]   右臂 [J5][J6]
    ┌──────────┐  ┌──────────┐  ┌──────────┐
    │ ◎→Y Pitch│  │ 腰部      │  │ Pitch Y←◎ │
    │  (-90~90)│  │ ↺ Z旋转  │  │  (-90~90) │
    │          │  │(-90~90) │  │          │
    │ ◎→X Roll │  └────┬─────┘  │ Roll X←◎ │
    │  (-45~45)│       │        │  (-45~45) │
    └──────────┘       │        └──────────┘
                       │
                  底座 (固定于世界)
```

> 坐标系：Z 轴向上，Y 轴向前，X 轴向右。原点位于底座底面中心。所有关节均为旋转副（revolute joint），无平动自由度。

---

### 关节详细规格

| 编号 | 关节名称 | 类型 | 旋转轴 | 范围 (deg) | 范围 (rad) | 舵机 | 物理位置 | 传动方式 |
|------|----------|------|--------|------------|------------|------|----------|----------|
| J1 | 腰部旋转 (waist_yaw) | revolute | Z 轴: `(0,0,1)` | -90 ~ +90 | -1.57 ~ +1.57 | SG90 9G | 底座与身体之间 | 舵机直驱，舵盘固定于底座，输出轴带动身体旋转 |
| J2 | 头部俯仰 (neck_pitch) | revolute | Y 轴: `(0,1,0)` | -30 ~ +30 | -0.52 ~ +0.52 | 2g 微型 | 身体顶部与头部之间 | 舵机固定于身体，舵机臂连杆驱动头部绕颈部 Y 轴旋转 |
| J3 | 左臂 Pitch (l_shoulder_pitch) | revolute | Y 轴: `(0,1,0)` | -90 ~ +90 | -1.57 ~ +1.57 | 2g 微型 | 身体左侧与左上臂之间 | 舵机固定于身体左侧，直接驱动上臂上下摆动 |
| J4 | 左臂 Roll (l_elbow_roll) | revolute | X 轴: `(1,0,0)` | -45 ~ +45 | -0.79 ~ +0.79 | 2g 微型 | 左上臂与前臂之间 | 舵机置于上臂内部，通过舵机臂 + 连杆驱动前臂扭转 |
| J5 | 右臂 Pitch (r_shoulder_pitch) | revolute | Y 轴: `(0,1,0)` | -90 ~ +90 | -1.57 ~ +1.57 | 2g 微型 | 身体右侧与右上臂之间 | 与 J3 对称，舵机固定于身体右侧 |
| J6 | 右臂 Roll (r_elbow_roll) | revolute | X 轴: `(1,0,0)` | -45 ~ +45 | -0.79 ~ +0.79 | 4.3g 超微型 | 右上臂与前臂之间 | 舵机置于上臂内部，**12T+16T 齿轮组**将舵机旋转转化为前臂扭转（T 型推杆机构） |

---

### 各关节运动范围说明

| 关节 | 0° (归零位) | + 方向 | - 方向 | 限位方式 |
|------|------------|--------|--------|----------|
| J1 腰部 | 身体正面朝前 | 身体向右转（顺时针俯视） | 身体向左转（逆时针俯视） | 机械结构 + 舵机行程 |
| J2 头部 | 头部竖直，面朝前方 | 头向下点（点头） | 头向上仰（抬头） | 颈部壳体限位，防止过仰撞到身体 |
| J3 左臂 Pitch | 手臂自然下垂（贴身体） | 手臂向前上方抬起 | 手臂向后摆动（受身体阻挡，范围小） | 身体壳体阻挡后摆方向 |
| J4 左臂 Roll | 手掌掌心朝内（自然位） | 前臂向外扭转（掌心朝前） | 前臂向内扭转（掌心朝后） | 舵机行程 + 前臂壳体限位 |
| J5 右臂 Pitch | 手臂自然下垂 | 手臂向前上方抬起 | 手臂向后摆动 | 与 J3 对称 |
| J6 右臂 Roll | 手掌掌心朝内 | 前臂向外扭转 | 前臂向内扭转 | 与 J4 对称 |

> 注："归零位"定义为机器人站立、面朝前的自然姿态。J3/J5 手臂 Pitch：下方空间大（下垂→前抬），后方空间小（身体阻挡）。实际前向抬起可达约 90°，后向约 -10°~-20°，表格范围为舵机理论最大值。

---

### 各关节涉及的移动零件

| 关节 | 随动的 CAD 零件 |
|------|----------------|
| J1 腰部 | body 组全部 3 件 (Part__Feature034/035/036) + head 组全部 5 件 + 双臂全部零件 |
| J2 头部 | head 组全部 5 件 (Part__Feature037/038/039/040/041) |
| J3 左臂 Pitch | left_arm 组全部 6 件 (Part__Feature042/045/046/047/048/049) |
| J4 左臂 Roll | left_arm 组全部 6 件（在 FreeCAD 宏中 arm sub-group 绕 X 轴整体旋转）；物理上为前臂部分 |
| J5 右臂 Pitch | right_arm 组全部 7 件 (Part__Feature027/028/029/030/031/032/033) |
| J6 右臂 Roll | right_arm 组全部 7 件；物理上为前臂 + 手掌部分 |

> 关节遵循串联运动链规则：父关节的运动会影响所有子关节的空间位置。例如 J1 腰部旋转时，头部和双臂一起随身体转动。

---

### DH 参数表（用于正运动学）

采用 **Modified DH 约定**（Craig 版本），坐标系 Z 轴与关节旋转轴对齐：

| i | 关节 | αᵢ₋₁ (rad) | aᵢ₋₁ (m) | dᵢ (m) | θᵢ (rad) | θᵢ 零位偏移 |
|---|------|-------------|-----------|--------|-----------|-------------|
| 1 | J1 腰部 | 0 | 0 | 0.038 | θ₁ | 0 (身体朝前) |
| 2 | J2 头部 | -π/2 | 0 | 0 | θ₂ | 0 (头竖直) |
| 3 | J3 左臂 Pitch | π/2 | 0.018 | 0 | θ₃ | 0 (臂下垂) |
| 4 | J4 左臂 Roll | -π/2 | 0 | 0.015 | θ₄ | 0 (掌心朝内) |
| 5 | J5 右臂 Pitch | π/2 | -0.018 | 0 | θ₅ | 0 (臂下垂) |
| 6 | J6 右臂 Roll | -π/2 | 0 | 0.015 | θ₆ | 0 (掌心朝内) |

> **说明**：
> - 为建模方便，将左臂 (J3→J4) 和右臂 (J5→J6) 放在同一 DH 表中。实际物理上是两条独立分支，在 MuJoCo 中应分开建模。
> - `a₁` = 0.038m 为底座底面到腰部旋转中心的高度；`d₃`/`d₅` = 0 为肩宽偏移已在 a₂/a₄ 中体现。
> - `d₄`/`d₆` = 0.015m 为上臂长度（肩到肘的距离）。
> - 末端执行器（手掌）位置需额外添加固定偏置。
>
> **MuJoCo 中不需要 DH 参数**，直接用相对 body 位置 + joint axis 定义（见本文 MuJoCo 章节）。DH 表供 ROS/Matlab 等传统机器人工具箱使用。

---

## 坐标定义

```
        Z (上)
        │
        │    Y (前)
        │   ╱
        │  ╱
        │ ╱
        └────────── X (右)
       原点在底座中心
```

| 轴 | 方向 | 对应关节 |
|----|------|----------|
| X | 左右（正=右） | 手臂 Roll |
| Y | 前后（正=前） | 头部/手臂 Pitch |
| Z | 上下（正=上） | 腰部旋转 |

---

## 坐标轴在视图中的方向

在 FreeCAD 3D 视图的右下角，可见一个彩色坐标轴指示器（X=红、Y=绿、Z=蓝）：

- **Z 轴竖直向上**：蓝色箭头指向屏幕上方，与机器人站立方向一致
- **Y 轴水平向前**：绿色箭头指向屏幕右下方（约 45° 斜角），代表机器人正面朝向
- **X 轴水平向右**：红色箭头指向屏幕右侧，正值为右侧，负值为左侧

原点（O）位于底座中心位置。从侧视图可直观看到：左臂 Pitch 抬起时，整个手臂组绕 Y 轴逆时针旋转；右臂 Roll 扭转时，手臂组绕 X 轴旋转。

---

## 零件树（24 个主要结构件）

```
electronBot
│
├── 底座 (Base) ─── 固定
│   ├── Part__Feature043  底座主体 (vol=21440 mm³)
│   └── Part__Feature044  底座底板 (vol=8198 mm³)
│
├── 身体 (Body) ─── J1 腰部 Z轴旋转
│   ├── Part__Feature034  身体中心 (vol=16545 mm³)
│   ├── Part__Feature035  身体右侧 (vol=18741 mm³)
│   └── Part__Feature036  身体左侧 (vol=18652 mm³)
│
├── 头部 (Head) ─── J2 颈部 Y轴俯仰
│   ├── Part__Feature039  头部主壳 (vol=14318 mm³)
│   ├── Part__Feature038  头壳顶部 (vol=9195 mm³)
│   ├── Part__Feature037  前面板/LCD区域 (vol=2428 mm³)
│   ├── Part__Feature040  小配件 (vol=136 mm³)
│   └── Part__Feature041  小配件 (vol=23 mm³)
│
├── 右臂 (Right Arm) ─── J5 Pitch + J6 Roll
│   ├── Part__Feature033  前臂件 (vol=1283 mm³)
│   ├── Part__Feature032  前臂件 (vol=1022 mm³)
│   ├── Part__Feature031  连接件 (vol=201 mm³)
│   ├── Part__Feature030  关节件 (vol=53 mm³)
│   ├── Part__Feature029  右手掌 (vol=264 mm³)
│   ├── Part__Feature027  齿轮 16T (vol=456 mm³)
│   └── Part__Feature028  齿轮 12T (vol=382 mm³)
│
└── 左臂 (Left Arm) ─── J3 Pitch + J4 Roll
    ├── Part__Feature049  前臂件 (vol=1283 mm³)
    ├── Part__Feature048  前臂件 (vol=1022 mm³)
    ├── Part__Feature047  连接件 (vol=201 mm³)
    ├── Part__Feature046  关节件 (vol=53 mm³)
    ├── Part__Feature045  左手镜面 (vol=264 mm³)
    └── Part__Feature042  左手掌 (vol=264 mm³)
```

---

## 零件树层次（FreeCAD 视图）

在 FreeCAD 中打开 `cadelectron.FCStd` 后，左侧模型树以 `App::Part` 容器 `_X2_52A05DE54EF6_X0_` 为根节点，包含 24 个子零件对象。零件按功能分为 5 组：

| 组 | 零件数量 | 说明 |
|----|----------|------|
| 底座 (Base) | 2 | 固定不动，提供支撑基础 |
| 身体 (Body) | 3 | 含腰部 Z 轴旋转关节 |
| 头部 (Head) | 5 | 含颈部 Y 轴俯仰关节 |
| 右臂 (Right Arm) | 7 | 含 Pitch + Roll 双轴 |
| 左臂 (Left Arm) | 6 | 含 Pitch + Roll 双轴（左臂无独立齿轮，使用镜像件）|

> 所有零件以 `Part__Feature` + 三位数字编号命名，编号在 STEP 导入时自动生成。右臂的齿轮（Part__Feature027/028）与舵机臂配合形成 T 型推杆传动机构。

---

## 运动学关系

### 父子层级

```
          底座 (固定)
            │
       身体 (被底座约束, 可绕Z旋转)
       ┌───┼───┐
     头部  左臂  右臂
```

- **底座** → 固定参考系，不运动
- **身体** → 子级于底座，绕 Z 轴旋转（腰部扭动）
- **头部** → 子级于身体，绕 Y 轴旋转（点头）
- **左右臂** → 子级于身体，可绕 Y 轴（Pitch）+ X 轴（Roll）

### 舵机驱动关系（BOM 确认）

```
SG90 9G (大舵机) ×1
  └── J1 腰部旋转 (Z轴)

2g 微型舵机 ×4
  ├── J2 头部俯仰 (Y轴)
  ├── J3 左臂 Pitch (Y轴)
  ├── J4 左臂 Roll (X轴)
  └── J5 右臂 Pitch (Y轴)

4.3g 超微型舵机 ×1
  └── J6 右臂 Roll (X轴)
```

> 6 个舵机各司其职，一轴一舵机。手臂 Pitch/Roll 是两个独立舵机，非联动。

---

## 总尺寸

| 维度 | 范围 | 尺寸 |
|------|------|------|
| 宽度 (X) | 左臂最左 ~ 右臂最右 | ≈ 52 mm |
| 深度 (Y) | 底座最前 ~ 背后 | ≈ 32 mm |
| 高度 (Z) | 底座 ~ 头顶 | ≈ 80 mm |

---

## 3D 视图中的内部结构

从侧面剖视（或使用透明线框模式）可观察到以下关键内部部件：

| 部件 | 位置 | 说明 |
|------|------|------|
| **腰部舵机 (SG90)** | 身体中心偏下 | 水平安装，驱动腰部 Z 轴旋转 |
| **头部舵机 (2g)** | 头颈部后方 | 驱动头部 Y 轴俯仰 |
| **手臂 Pitch 舵机 (2g)** | 肩膀内侧 | 左右各一个，驱动手臂上下摆动 |
| **手臂 Roll 舵机 (2g/4.3g)** | 上臂内部 | 驱动前臂绕 X 轴扭转；右臂使用 4.3g 超微型舵机 |
| **齿轮组 (12T + 16T)** | 右臂上臂处 | T 型推杆传动机构，将舵机旋转转化为前臂扭转 |
| **舵机臂 (白色/橙色)** | 各舵机输出轴 | 连接舵机与连杆，传递力矩 |
| **PCB 板** | 身体/头部内部 | 主控电路板，带有连接器 |
| **Type-C 接口** | 底座后方 | 供电与烧录接口 |
| **3D 打印结构件** | 各关节处 | 提供舵机安装座、轴承位、限位结构 |

> 在 FreeCAD 中可通过 `视图(V) → 显示模式 → 线框` 或调整零件透明度来观察内部结构。右臂的齿轮组（12T + 16T）与舵机臂配合，将舵机输出的旋转运动转化为手臂 Roll 方向的扭转。

---

## 舵机清单

| 舵机 | 数量 | 用途 |
|------|------|------|
| SG90 9G 经典舵机 180° | 1 | 腰部旋转 |
| 2g 微型舵机 | 4 | 头部俯仰 + 左臂Pitch + 左臂Roll + 右臂Pitch |
| 4.3g 超微型舵机 | 1 | 右臂 Roll |

---

## 关节运动可视化

在 FreeCAD 中运行 `electronbot_joints.FCMacro` 可打开关节控制器，拖动滑块实时观察各关节运动。以下是一个典型的姿态示例：

| 关节 | 旋转轴 | 角度 | 运动效果 |
|------|--------|------|----------|
| 腰部旋转 | Z 轴 | 0° | 身体保持正面朝向 |
| 头部俯仰 | Y 轴 | 0° | 头部保持水平 |
| 左臂 Pitch | Y 轴 | +39.4° | 左臂**向上抬起**（绕 Y 轴逆时针旋转） |
| 左臂 Roll | X 轴 | 0° | 前臂无扭转 |
| 右臂 Pitch | Y 轴 | 0° | 右臂保持自然下垂 |
| 右臂 Roll | X 轴 | -12.7° | 右臂**轻微内旋**（绕 X 轴顺时针扭转） |

> 从侧面观察可清晰看到：左臂 Pitch 抬起时，**整个手臂组（7 个零件）作为一个刚体**绕肩部 Y 轴旋转；右臂 Roll 扭转时，手臂组绕 X 轴旋转，前臂部分因齿轮传动产生相对扭转。关节控制器中每个滑块控制对应组零件的整体旋转，旋转中心通过包围盒中心计算得出。当角度归零时，所有零件恢复到初始装配位置。更精准的验证方法请参考 [freecad-joint-control.md](freecad-joint-control.md)。

## 相关文件

```
electronbot-docs/docs/cad/
├── ElectronBot.step               ← 当前文件 (STEP 通用格式)
├── cadelectron.FCStd              ← FreeCAD 原生格式 (含 Assembly)
├── freecad-joint-control.md       ← FreeCAD 关节控制操作指南
├── robot-structure.md             ← 本文档 (机器人结构解读)
└── ../../electronbot_joints.FCMacro  ← 关节控制宏
```

---

## MuJoCo MJCF 建模参考

> 本节提供所有必要数据，可直接用于构建 MJCF 格式的 MuJoCo 仿真模型。
> 坐标系与 CAD 保持一致（Z 轴向上），使用 `<compiler angle="degree"/>`，重力设为 `(0 0 -9.81)`。

### 运动学树

```
world
└── base (固定底座)
    └── [J1] waist_z (hinge, Z轴)
        └── body (身体)
            ├── [J2] neck_y (hinge, Y轴)
            │   └── head (头部)
            ├── [J3] left_shoulder_y (hinge, Y轴)
            │   └── left_upper_arm (左上臂)
            │       └── [J4] left_elbow_x (hinge, X轴)
            │           └── left_forearm (左前臂+手掌)
            └── [J5] right_shoulder_y (hinge, Y轴)
                └── right_upper_arm (右上臂)
                    └── [J6] right_elbow_x (hinge, X轴)
                        └── right_forearm (右前臂+手掌)
```

### 关节规格

关节位置均为 **相对于父刚体的本地坐标**（单位：米）。

| 关节 | 编号 | 类型 | 轴线 | 范围 (deg) | 父刚体 | 位置 pos (x, y, z) m | 说明 |
|------|------|------|------|------------|--------|----------------------|------|
| waist_z | J1 | hinge | (0, 0, 1) | -90 ~ 90 | base | (0, 0, 0.032) | 腰部旋转，位于底座中心上方 32mm |
| neck_y | J2 | hinge | (0, 1, 0) | -30 ~ 30 | body | (0, 0, 0.031) | 头部俯仰，位于腰部上方 31mm |
| left_shoulder_y | J3 | hinge | (0, 1, 0) | -90 ~ 90 | body | (-0.018, 0, 0.006) | 左臂上下摆动 |
| left_elbow_x | J4 | hinge | (1, 0, 0) | -45 ~ 45 | left_upper_arm | (0, 0, -0.015) | 左前臂扭转 |
| right_shoulder_y | J5 | hinge | (0, 1, 0) | -90 ~ 90 | body | (0.018, 0, 0.006) | 右臂上下摆动 |
| right_elbow_x | J6 | hinge | (1, 0, 0) | -45 ~ 45 | right_upper_arm | (0, 0, -0.015) | 右前臂扭转 |

> **数据来源说明**：关节位置从 FreeCAD 宏的 `pivots` 包围盒中心计算得出：
> - waist 中心 ≈ (0, 0, -0.006) → 相对 base 中心偏移 (0, 0, 0.032)
> - neck 中心 ≈ (0.003, 0, 0.025) → 相对 body center 偏移 (0.005, 0, 0.031)
> - 左肩 ≈ (-0.019, 0, 0.000) → 相对 body center 偏移 (-0.018, 0, 0.006)
> - 右肩 ≈ (0.018, 0, 0.000) → 相对 body center 偏移 (0.018, 0, 0.006)
>
> 注：CAD 中各零件组 `cy`（Y 坐标）值未从宏输出中提取，统一设为 0 轴中心，实际可能有 ±2mm 偏移，可在 MJCF 中微调。

---

### 刚体几何尺寸

所有单位为米，基于 CAD 零件包围盒估算。在 MJCF 中可用 `<geom type="box">` 近似，或用 `<mesh>` 引用导出的 `.stl` 文件。

| 刚体 | 近似包围盒 (x, y, z) m | 碰撞体类型 | 说明 |
|------|------------------------|------------|------|
| base | (0.052, 0.032, 0.015) | box | 底座整体，宽 52mm × 深 32mm × 高 15mm |
| body | (0.052, 0.028, 0.040) | box | 身体主体，上部略窄 |
| head | (0.028, 0.022, 0.025) | box | 球形头部，可用直径 0.028m 球体近似 |
| left_upper_arm | (0.008, 0.006, 0.016) | box | 左上臂筒体 |
| left_forearm | (0.007, 0.005, 0.018) | box | 左前臂 + 手掌 |
| right_upper_arm | (0.008, 0.006, 0.016) | box | 右上臂筒体 |
| right_forearm | (0.007, 0.005, 0.018) | box | 右前臂 + 手掌 |

> **几何简化建议**：头部建议用 `sphere` (r≈0.014) 偏移到合适位置；身体用 2 个 box 叠加（上身窄 + 下身宽）；手臂用 capsule 或分段 box 组合更美观。

---

### 质量 & 惯性

| 刚体 | 质量 (kg) | 惯性 diaginertia | 组成 |
|------|-----------|------------------|------|
| base | 0.030 | 1e-5 / 1e-5 / 2e-5 | 3D 打印底座 (29.6 cm³ 尼龙) |
| body | 0.085 | 2e-5 / 2e-5 / 3e-5 | 3D 打印身体 (54 cm³) + SG90 (9g) + ESP32 + PCB (~23g) |
| head | 0.035 | 5e-6 / 5e-6 / 5e-6 | 3D 打印头部 (26 cm³) + 2g 舵机 + LCD (~5g) |
| left_upper_arm | 0.008 | 3e-7 / 2e-7 / 4e-7 | 3D 打印上部 + 2g 舵机 (Pitch) |
| left_forearm | 0.005 | 2e-7 / 1e-7 / 3e-7 | 3D 打印下部 + 2g 舵机 (Roll) + 手掌 |
| right_upper_arm | 0.009 | 3e-7 / 2e-7 / 4e-7 | 3D 打印上部 + 2g 舵机 (Pitch) + 齿轮组 |
| right_forearm | 0.006 | 2e-7 / 1e-7 / 3e-7 | 3D 打印下部 + 4.3g 舵机 (Roll) + 手掌 |

> **惯性计算方式**：基于 box 近似，`I = m/12 * (边长²)`。实际零件为中空壳体，真实惯性比实体 box 公式偏大 1.5-2 倍。
> **总质量** ≈ 0.18 kg，不含电池约 0.17 kg。若仿真中惯性导致不稳定，可将 diaginertia 值 ×1.5。

---

### 执行器配置

MJCF 中用 `<position>` actuator：

```xml
<!-- 腰部 SG90，力矩较大 -->
<position name="wa"      joint="waist_z"           ctrlrange="-1.57 1.57"   kp="2.0"/>
<!-- 头部 2g 舵机 -->
<position name="ne"      joint="neck_y"            ctrlrange="-0.52 0.52"   kp="1.0"/>
<!-- 左臂 2g 舵机 -->
<position name="ls"      joint="left_shoulder_y"   ctrlrange="-1.57 1.57"   kp="1.0"/>
<position name="le"      joint="left_elbow_x"      ctrlrange="-0.79 0.79"   kp="0.5"/>
<!-- 右臂 -->
<position name="rs"      joint="right_shoulder_y"  ctrlrange="-1.57 1.57"   kp="1.0"/>
<position name="re"      joint="right_elbow_x"     ctrlrange="-0.79 0.79"   kp="0.5"/>
```

> `ctrlrange` 为弧度制（°×π/180）。SG90 力矩约 0.18 N·m，2g 舵机约 0.02 N·m，4.3g 约 0.05 N·m。若需限力，可加 `forcerange`。

---

### 完整 MJCF 模板

以下为可直接使用的骨架模板，使用 box/sphere 几何体近似：

```xml
<mujoco model="electronbot">
  <compiler angle="degree" autolimits="true"/>

  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <!-- ===== 底座 (固定) ===== -->
    <body name="base" pos="0 0 0">
      <geom name="base_geom" type="box" size="0.026 0.016 0.0075" rgba="0.3 0.3 0.3 1"/>
      <inertial pos="0 0 0" mass="0.030" diaginertia="1e-5 1e-5 2e-5"/>

      <!-- J1: 腰部 Z轴旋转 -->
      <joint name="waist_z" type="hinge" axis="0 0 1" pos="0 0 0.032"
             range="-90 90" limited="true"/>

      <!-- ===== 身体 ===== -->
      <body name="body" pos="0 0 0.032">
        <geom name="body_geom" type="box" size="0.026 0.014 0.020" pos="0 0 0" rgba="0.5 0.5 0.5 1"/>
        <inertial pos="0 0 0" mass="0.085" diaginertia="2e-5 2e-5 3e-5"/>

        <!-- J2: 头部 Y轴俯仰 -->
        <joint name="neck_y" type="hinge" axis="0 1 0" pos="0 0 0.031"
               range="-30 30" limited="true"/>

        <!-- ===== 头部 ===== -->
        <body name="head" pos="0 0 0.031">
          <geom name="head_geom" type="sphere" size="0.014" pos="0 0 0.012" rgba="0.7 0.7 0.7 1"/>
          <inertial pos="0 0 0.012" mass="0.035" diaginertia="5e-6 5e-6 5e-6"/>
        </body>

        <!-- J3: 左肩 Y轴 Pitch -->
        <joint name="left_shoulder_y" type="hinge" axis="0 1 0" pos="-0.018 0 0.006"
               range="-90 90" limited="true"/>

        <!-- ===== 左上臂 ===== -->
        <body name="left_upper_arm" pos="-0.018 0 0.006">
          <geom name="l_uparm_geom" type="box" size="0.004 0.003 0.008" pos="0 0 0" rgba="0.4 0.6 0.8 1"/>
          <inertial pos="0 0 0" mass="0.008" diaginertia="3e-7 2e-7 4e-7"/>

          <!-- J4: 左肘 X轴 Roll -->
          <joint name="left_elbow_x" type="hinge" axis="1 0 0" pos="0 0 -0.015"
                 range="-45 45" limited="true"/>

          <!-- ===== 左前臂 ===== -->
          <body name="left_forearm" pos="0 0 -0.015">
            <geom name="l_farm_geom" type="box" size="0.0035 0.0025 0.009" pos="0 0 0" rgba="0.3 0.5 0.7 1"/>
            <inertial pos="0 0 0" mass="0.005" diaginertia="2e-7 1e-7 3e-7"/>
          </body>
        </body>

        <!-- J5: 右肩 Y轴 Pitch -->
        <joint name="right_shoulder_y" type="hinge" axis="0 1 0" pos="0.018 0 0.006"
               range="-90 90" limited="true"/>

        <!-- ===== 右上臂 ===== -->
        <body name="right_upper_arm" pos="0.018 0 0.006">
          <geom name="r_uparm_geom" type="box" size="0.004 0.003 0.008" pos="0 0 0" rgba="0.4 0.6 0.8 1"/>
          <inertial pos="0 0 0" mass="0.009" diaginertia="3e-7 2e-7 4e-7"/>

          <!-- J6: 右肘 X轴 Roll -->
          <joint name="right_elbow_x" type="hinge" axis="1 0 0" pos="0 0 -0.015"
                 range="-45 45" limited="true"/>

          <!-- ===== 右前臂 ===== -->
          <body name="right_forearm" pos="0 0 -0.015">
            <geom name="r_farm_geom" type="box" size="0.0035 0.0025 0.009" pos="0 0 0" rgba="0.3 0.5 0.7 1"/>
            <inertial pos="0 0 0" mass="0.006" diaginertia="2e-7 1e-7 3e-7"/>
          </body>
        </body>

      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="wa"      joint="waist_z"           ctrlrange="-1.57 1.57"   kp="2.0"/>
    <position name="ne"      joint="neck_y"            ctrlrange="-0.52 0.52"   kp="1.0"/>
    <position name="ls"      joint="left_shoulder_y"   ctrlrange="-1.57 1.57"   kp="1.0"/>
    <position name="le"      joint="left_elbow_x"      ctrlrange="-0.79 0.79"   kp="0.5"/>
    <position name="rs"      joint="right_shoulder_y"  ctrlrange="-1.57 1.57"   kp="1.0"/>
    <position name="re"      joint="right_elbow_x"     ctrlrange="-0.79 0.79"   kp="0.5"/>
  </actuator>
</mujoco>
```

> 此模板使用 box/sphere 近似几何体，适合快速搭建和调试。后续可：
> 1. 将碰撞体替换为 `<mesh>` 引用从 `ElectronBot.step` 导出的 `.stl` 文件
> 2. 在 `<equality>` 中添加关节柔顺控制
> 3. 添加场地 `<geom type="plane"/>` 用于着地仿真
> 4. 使用 `<site>` 定义末端执行器位置（手掌、头部），用于 IK 或抓取任务
