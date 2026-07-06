# Phase 1：CAD → MJCF 建模

> **目标**：从 ElectronBot.step 原始 CAD 模型出发，提取零件几何体、质量/惯量、关节参数，生成 MuJoCo 可用的 MJCF 格式物理模型。
>
> **输入**：`assets/cad/ElectronBot.step`（30.5MB，24 个零件）
>
> **输出**：
> - `assets/mjcf/electronbot_full_arm.xml`——inline mesh 版，CAD 真实外形
>
> **文档版本**: v2.0  
> **最后更新**: 2026-07-06  
> **变更类型**: 根据实际实现重写——5 组 STL 合并导出（按 FreeCAD 运动组），非 24 零件独立导出

---

## 前置说明：CAD、STL、XML 都是什么

### 先看最终产物长什么样

当你成功运行下面的命令时：

```bash
cd ElectronBot_SIM
python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml
```

MuJoCo viewer 中出现的那个机器人模型，就是一个 **XML 文件** 描述的。这个过程的本质是：

```
CAD（图纸）  →  STL（模型）  →  XML（仿真描述）
```

### 三种文件分别是什么

| 文件格式 | 是什么 | 管什么 | 怎么来的 | 能直接看吗 |
|----------|--------|--------|----------|-----------|
| **CAD (.step / .FCStd)** | 工程师画的**原始设计图纸**，包含 24 个零件的精确尺寸、装配关系 | 告诉你有哪些零件、它们长什么样、怎么组装在一起 | 从 稚晖君 ElectronBot 开源项目获取 | 需要用 FreeCAD / SolidWorks 等 CAD 软件打开 |
| **STL (.stl)** | **3D 模型的表面网格**，只保存三角形顶点坐标，没有颜色/材质/装配信息 | 保存机器人每个部分的**外形**，给 MuJoCo 做碰撞和外观显示用 | 从 CAD 软件导出 | 大部分 3D 查看器都能打开 |
| **XML (.xml, MJCF)** | **MuJoCo 的模型描述文件**，纯文本格式，描述机器人有哪些刚体、关节、执行器、传感器 | 定义机器人**怎么动**：关节轴方向、旋转范围、舵机参数、物理属性 | 由 Python 脚本把 STL 数据内嵌到 XML 模板中 | 任何文本编辑器都能打开 |

### 一句话理解

```
CAD 是"设计图" → 导出零件的三维外形得到 STL → 把 STL 的外形数据 + 手工编写的关节参数填入 XML → MuJoCo 加载 XML 就能仿真
```

### 转换过程全景

```
┌───────────────────────────────────────────────────────────────────────┐
│                    ElectronBot.STEP (CAD 原始文件, 30MB)                │
│                    24 个零件, 来自稚晖君开源项目                          │
└──────────────────────┬────────────────────────────────────────────────┘
                       │
                       ▼  用 FreeCAD 打开
┌───────────────────────────────────────────────────────────────────────┐
│                FreeCAD 中确认零件分组 (关键步骤)                          │
│                                                                         │
│  通过 electronbot_joints.FCMacro 宏确定哪个零件属于哪个运动组:             │
│                                                                         │
│  底座组 (不动)     身体组 (绕Z转)     头部组 (绕Y俯仰)   左臂组   右臂组    │
│  ┌──────────┐    ┌──────────────┐   ┌──────────────┐  ┌────┐  ┌────┐  │
│  │ Part_043 │    │ Part_034     │   │ Part_037~041 │  │6件 │  │7件 │  │
│  │ Part_044 │    │ Part_035 ←──┼┐  │ (5个零件)     │  │    │  │    │  │
│  │          │    │ Part_036 ←──┼┤  │              │  │    │  │    │  │
│  └──────────┘    └──────────────┘│  └──────────────┘  └────┘  └────┘  │
│              ⚠️ 关键: 035/036是身体 |                                    │
│              外壳, 不属于手臂!   |                                       │
└──────────────────────────────────┼─────────────────────────────────────┘
                                   │
                                   ▼  按运动组合并导出 STL
┌───────────────────────────────────────────────────────────────────────┐
│               assets/meshes/ 目录 (5 个 STL 文件)                       │
│                                                                         │
│  base_link.stl    body.stl     head.stl    left_arm.stl  right_arm.stl │
│  (底座2零件合并)  (身体3零件)   (头部5零件)  (左臂6零件)   (右臂7零件)   │
│  ~1.1MB           ~364KB       ~713KB      ~850KB        ~317KB       │
└──────────────────────┬────────────────────────────────────────────────┘
                       │
                       ▼  运行 generate_inline_mesh.py
┌───────────────────────────────────────────────────────────────────────┐
│  python scripts/generate_inline_mesh.py \                              │
│      --input assets/meshes \                                           │
│      --output assets/mjcf/electronbot_full_arm.xml                     │
│                                                                         │
│  脚本做的事 (重点):                                                      │
│  1. 读取 5 个 STL 的三角形顶点数据                                        │
│  2. 把顶点坐标转为字符串, 内嵌到 XML 的 <mesh> 标签里                      │
│  3. 同时生成 <body> <joint> <actuator> 等仿真描述                        │
│                                                                         │
│  最终产物是一个 XML 文件, 内部包含了:                                     │
│  ├── 5 组 STL 的几何数据 (内联 mesh, ~1.7MB 都在一个文件里)                 │
│  ├── 7 个刚体的层次结构 (base→body→head/left_arm/right_arm)             │
│  ├── 6 个关节的定义 (轴方向、旋转范围)                                    │
│  ├── 6 个执行器的参数 (舵机型号对应的 kp/kv)                              │
│  └── 传感器定义                                                         │
└──────────────────────┬────────────────────────────────────────────────┘
                       │
                       ▼  python3 -m mujoco.viewer ...
┌───────────────────────────────────────────────────────────────────────┐
│                MuJoCo viewer 中加载 XML 后:                              │
│                                                                         │
│  1. 解析 <mesh> → 重建 3D 模型外形                                        │
│  2. 解析 <body> → 建立刚体父子关系                                         │
│  3. 解析 <joint> → 确定每个关节怎么转                                      │
│  4. 解析 <actuator> → 可以拖动滑块控制舵机                                  │
│  5. 开始仿真 → 物理引擎驱动机器人运动                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 你不需要关心哪些

- **不需要手动编辑 XML**：`electronbot_full_arm.xml` 有 1.7MB，是脚本自动生成的。你要改的是上游（CAD 零件分组或 STL 网格），然后重新跑脚本
- **不需要理解 CAD 软件**：零件分组已经在 `assets/cad/electronbot_joints.FCMacro` 中确认，直接使用即可
- **不需要手动处理 STL**：STL 是中间产物，由 CAD 导出后，脚本自动处理

你只需要知道：
- **修改机器人外形** → 改 CAD 图纸，重新导出 STL
- **修改关节参数** → 改 `generate_inline_mesh.py` 脚本中的模板
- **修改舵机 kp/kv** → 改 `update_actuator.py` 脚本中的参数
- **验证效果** → `python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml`

---

### 为什么选择 inline mesh 而不是外部 STL 引用

**一句结论：外部 STL 可以加载，但有路径和单位两个坑要填，inline mesh 自动避开了。**

#### 两种方案对比

```xml
<!-- 方式 A：外部 STL 引用（能行，但要填坑） -->
<compiler meshscale="0.001"/>            <!-- 坑1：STL 毫米→米缩放 -->
<mesh name="body" file="../meshes/body.stl"/>  <!-- 坑2：路径要写对 -->

<!-- 方式 B：inline mesh（当前方案，一步到位） -->
<mesh name="body" vertex="0.0012 0.0035 ..." face="0 1 2 ..."/> 
<!-- 缩放和路径都已在脚本中处理，XML 自带数据 -->
```

#### 三个核心区别

| 区别 | 外部 STL 引用 | inline mesh（当前方案） |
|------|-------------|---------------------|
| **路径** | MuJoCo 去 XML 同级目录找 `.stl` 文件，路径不对就加载空壳 | 顶点数据内嵌在 XML 中，**没有外部文件依赖** |
| **单位** | STL 是毫米 (mm)，MuJoCo 当米 (m) 解析 → 模型放大 1000 倍，惯性爆炸 | 脚本提前做 `vertices * 0.001` mm→m 缩放，数值正确 |
| **自动中心化** | 外部 STL **没有**自动中心化 → 手臂 `geom pos` 偏移量要重新手算 | MuJoCo 编译 inline mesh 时自动将几何中心移到 body 原点，`geom pos` 偏移可脚本自动计算 |

#### 为什么 inline 对 ElectronBot 更合适

1. **单文件可移植**：`electronbot_full_arm.xml` 自包含所有几何数据（~1.7MB），无需带着 5 个 STL 一起复制
2. **自动处理单位**：解决了已知的 `mm → m` 大坑
3. **自动处理位置偏移**：inline mesh 的自动中心化行为让脚本可以精确计算手臂 `geom pos` 偏移量
4. **外部 STL 的唯一优势**——"改外形不用重新生成 XML"——在实践中几乎用不到，外形定了就是定了

#### 如果你非要用外部 STL

两件事必须做对：

```xml
<!-- 1. 加 meshscale -->
<compiler angle="radian" autolimits="true" meshscale="0.001"/>

<!-- 2. 路径要指向正确位置 -->
<asset>
  <mesh name="body" file="../meshes/body.stl"/>
</asset>
```

但手臂的 `geom pos` 偏移量因为没有自动中心化，需要重新手动计算，不建议走这条路。

---

## 目录
- [1. 实际实现概述](#1-实际实现概述)
- [2. 从设计文档到实现：关键修正](#2-从设计文档到实现关键修正)
- [3. 最终模型结构](#3-最终模型结构)
- [4. 实现步骤](#4-实现步骤)
- [5. 验证方法](#5-验证方法)
- [6. 工程经验教训](#6-工程经验教训)

---

## 1. 实际实现概述

### 1.1 最终文件结构

```
assets/mjcf/
└── electronbot_full_arm.xml     ← 唯一输出: inline mesh 版，CAD 真实外形

assets/meshes/
├── base_link.stl                 ← 底座（2 个 CAD 零件合并导出）
├── body.stl                      ← 身体（3 个 CAD 零件合并导出，含侧外壳）
├── head.stl                      ← 头部（5 个 CAD 零件合并导出）
├── left_arm.stl                  ← 左臂（6 个 CAD 零件，纯手臂，不含侧外壳）
└── right_arm.stl                 ← 右臂（7 个 CAD 零件，纯手臂，不含侧外壳）
```

### 1.2 可视化验证结果

```bash
$ python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml
```

在 viewer 中可观察：
- 完整机器人模型，CAD 真实外形（inline mesh）
- 6 个关节可通过 actuator slider 独立拖动
- 腰部（body_joint）：绕 Z 轴旋转 ±90°
- 头部（head_joint）：绕 Y 轴俯仰 ±15°
- 左臂 Pitch（left_pitch_joint）：绕 Y 轴 ±90°
- 左臂 Roll（left_roll_joint）：绕 X 轴 ±45°
- 右臂 Pitch（right_pitch_joint）：绕 Y 轴 ±90°
- 右臂 Roll（right_roll_joint）：绕 X 轴 ±45°
- 模型无穿透，仿真稳定（RK4 积分器）

---

## 2. 从设计文档到实现：关键修正

### 2.1 原始设计文档 vs 实际实现

| 项目 | 原始设计文档（v1.2） | 实际实现（v2.0） |
|------|---------------------|-----------------|
| 输出文件名 | `electronbot_mesh.xml` | `electronbot_full_arm.xml` |
| STL 导出方式 | 24 个 CAD 零件分别导出 | 按运动组合并导出为 5 组 |
| body 命名 | `base/torso/head/left_arm/left_hand/right_arm/right_hand` | `base_link/body/head/left_arm/left_hand/right_arm/right_hand` |
| 关节命名 | `joint_body/joint_head/joint_lp/joint_lr/joint_rp/joint_rr` | `body_joint/head_joint/left_pitch_joint/left_roll_joint/right_pitch_joint/right_roll_joint` |
| 手臂运动轴 | 左臂 Pitch(0,1,0), Roll(1,0,0) | **左臂 Pitch(0,1,0), Roll(1,0,0)**（与文档一致） |
| 积分器 | implicitfast | **RK4**（稳定性问题导致） |
| actuator kp | 30-80（弧度制） | 30-80，与舵机规格匹配 |
| 场景文件 | `scene_mesh.xml` + `scene_tabletop.xml` | **已删除**（不需要） |
| 角色 | 几何基元版 + inline mesh 版两种 | **统一 inline mesh 版** |

### 2.2 关键 Bug 修复

**根本问题**：原始 CAD 导出的 `left_arm.stl` / `right_arm.stl` 错误地将身体侧外壳零件（`Part__Feature035`/`036`）合并到了手臂 STL 中。

**后果**：在 MuJoCo 中控制手臂 Pitch/Roll 时，身体侧外壳也跟着手臂一起运动。

**解决方案**：
1. 通过 FreeCAD 宏 `electronbot_joints.FCMacro` 确认正确的 CAD 零件分组
2. 按 5 个运动组重新合并导出 STL：
   - 底座组：`Part__Feature043` + `Part__Feature044`
   - 身体组：`Part__Feature034` + `Part__Feature035` + `Part__Feature036`（侧外壳属于身体，不动）
   - 头部组：5 个零件
   - 左臂组：6 个零件（纯手臂，不含侧外壳）
   - 右臂组：7 个零件（纯手臂，不含侧外壳）

### 2.3 最终验证通过的命令

```bash
cd ElectronBot_SIM
python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml
```

---

## 3. 最终模型结构

### 3.1 运动学树

```
world
└── base_link (固定)
    └── body [body_joint, Z轴 hinge, ±90°]
        ├── head [head_joint, Y轴 hinge, ±15°]
        ├── left_arm [left_pitch_joint, Y轴 hinge, ±90°]
        │             [left_roll_joint,  X轴 hinge, ±45°]
        │   └── left_hand (末端 box)
        └── right_arm [right_pitch_joint, Y轴 hinge, ±90°]
                       [right_roll_joint,  X轴 hinge, ±45°]
            └── right_hand (末端 box)
```

**与原始设计文档的区别**：
- 左/右臂直接挂在 body 下（无 `left_shoulder`/`right_shoulder` 中间 body）
- 每臂 2 个关节（Pitch + Roll），共 6 自由度
- 关节命名采用 `_pitch_joint` / `_roll_joint` 后缀

### 3.2 完整 MJCF 结构

```xml
<mujoco model="electronbot">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="RK4" iterations="50" cone="elliptic"/>
  <default>
    <joint damping="4.0" armature="0.1" frictionloss="0.5"/>
    <geom contype="0" conaffinity="0" condim="3" friction="0.8 0.3 0.1" density="0.1"/>
  </default>

  <asset>
    <!-- 5 个内联 mesh：base_link, body, head, left_arm, right_arm -->
    <mesh name="base_link" vertex="..." face="..."/>
    <mesh name="body"      vertex="..." face="..."/>
    <mesh name="head"      vertex="..." face="..."/>
    <mesh name="left_arm"  vertex="..." face="..."/>
    <mesh name="right_arm" vertex="..." face="..."/>
  </asset>

  <worldbody>
    <body name="base_link" pos="0 0 0.015">
      <geom name="base_geom" type="mesh" mesh="base_link" mass="0.045"/>

      <body name="body" pos="0 0 0.03">
        <joint name="body_joint" type="hinge" axis="0 0 1"
               range="-1.5708 1.5708" limited="true"/>
        <geom name="body_geom" type="mesh" mesh="body" mass="0.060"/>

        <!-- 头部 -->
        <body name="head" pos="0 0 0.07">
          <joint name="head_joint" type="hinge" axis="0 1 0"
                 range="-0.2618 0.2618" limited="true"/>
          <geom name="head_geom" type="mesh" mesh="head" mass="0.030"/>
        </body>

        <!-- 左臂（纯 LEFT_ARM_PARTS，不含身体外壳） -->
        <body name="left_arm" pos="-0.0180 0 0.065">
          <joint name="left_pitch_joint" type="hinge" axis="0 1 0"
                 range="-1.5708 1.5708" limited="true"/>
          <joint name="left_roll_joint" type="hinge" axis="1 0 0"
                 range="-0.7854 0.7854" limited="true"/>
          <geom name="left_arm_geom" type="mesh" mesh="left_arm"
                pos="-0.0256 0 0" mass="0.005"/>
          <body name="left_hand" pos="0 0.03 0">
            <geom name="left_hand_geom" type="box"
                  size="0.006 0.006 0.010" mass="0.003"/>
          </body>
        </body>

        <!-- 右臂（纯 RIGHT_ARM_PARTS，不含身体外壳） -->
        <body name="right_arm" pos="0.0180 0 0.065">
          <joint name="right_pitch_joint" type="hinge" axis="0 1 0"
                 range="-1.5708 1.5708" limited="true"/>
          <joint name="right_roll_joint" type="hinge" axis="1 0 0"
                 range="-0.7854 0.7854" limited="true"/>
          <geom name="right_arm_geom" type="mesh" mesh="right_arm"
                pos="0.0256 0 0" mass="0.005"/>
          <body name="right_hand" pos="0 0.03 0">
            <geom name="right_hand_geom" type="box"
                  size="0.006 0.006 0.010" mass="0.003"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="act_body"        joint="body_joint"
              ctrlrange="-1.5708 1.5708" kp="80"  kv="20"/>
    <position name="act_head"        joint="head_joint"
              ctrlrange="-0.2618 0.2618" kp="40"  kv="10"/>
    <position name="act_left_pitch"  joint="left_pitch_joint"
              ctrlrange="-1.5708 1.5708" kp="60"  kv="15"/>
    <position name="act_left_roll"   joint="left_roll_joint"
              ctrlrange="-0.7854 0.7854" kp="30"  kv="8"/>
    <position name="act_right_pitch" joint="right_pitch_joint"
              ctrlrange="-1.5708 1.5708" kp="60"  kv="15"/>
    <position name="act_right_roll"  joint="right_roll_joint"
              ctrlrange="-0.7854 0.7854" kp="30"  kv="8"/>
  </actuator>

  <sensor>
    <jointpos name="jpos_body" joint="body_joint"/>
    <jointpos name="jpos_head" joint="head_joint"/>
    <jointpos name="jpos_left_pitch" joint="left_pitch_joint"/>
    <jointpos name="jpos_left_roll" joint="left_roll_joint"/>
    <jointpos name="jpos_right_pitch" joint="right_pitch_joint"/>
    <jointpos name="jpos_right_roll" joint="right_roll_joint"/>
  </sensor>

  <keyframe>
    <key name="home" qpos="0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
```

### 3.3 执行器参数（弧度制）

| 关节 | 舵机 | kp | kv | ctrlrange (rad) | 对应角度 |
|------|------|:--:|:--:|:----------------:|:--------:|
| body_joint | SG90 | 80 | 20 | ±1.5708 | ±90° |
| head_joint | 2g | 40 | 10 | ±0.2618 | ±15° |
| left_pitch_joint | 2g | 60 | 15 | ±1.5708 | ±90° |
| left_roll_joint | 2g | 30 | 8 | ±0.7854 | ±45° |
| right_pitch_joint | 2g | 60 | 15 | ±1.5708 | ±90° |
| right_roll_joint | 2g | 30 | 8 | ±0.7854 | ±45° |

### 3.4 STL 合并对应关系

5 个 STL 文件的零件合并规则：

| STL 文件 | 包含的 CAD 零件 (Part__Feature) | 件数 |
|----------|--------------------------------|:---:|
| `base_link.stl` | 043（底座主体）+ 044（底座底板） | 2 |
| `body.stl` | 034（身体中心）+ 035（身体右侧外壳）+ 036（身体左侧外壳） | 3 |
| `head.stl` | 037（前脸/LCD）+ 038（头顶）+ 039（头部主壳）+ 040 + 041 | 5 |
| `left_arm.stl` | 042（左手）+ 045（左手镜）+ 046~049（臂件） | 6 |
| `right_arm.stl` | 027（齿轮）+ 028（小齿轮）+ 029（右手）+ 030~033（臂件） | 7 |

---

## 4. 实现步骤

### Step 1：确认 FreeCAD 零件分组

运行 `assets/cad/electronbot_joints.FCMacro` 获取正确的运动组：

```python
# 身体组（3 个零件，绕 Z 轴旋转腰部）
BODY_PARTS = [
    "Part__Feature034",  # 身体中心
    "Part__Feature035",  # 身体右侧（肩安装座，容纳电子部件）
    "Part__Feature036",  # 身体左侧
]

# 左臂组（6 个零件，绕 Y 轴 Pitch，绕 X 轴 Roll）
LEFT_ARM_PARTS = [
    "Part__Feature042",  # 左手
    "Part__Feature045",  # 左手镜
    "Part__Feature046",  # 臂件
    "Part__Feature047",  # 臂件
    "Part__Feature048",  # 臂件
    "Part__Feature049",  # 臂件
]

# 右臂组（7 个零件，绕 Y 轴 Pitch，绕 X 轴 Roll）
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

### Step 2：将 STL 按运动组合并导出

使用 FreeCAD 命令行或 GUI，将每组零件合并导出为单个 STL：

```bash
# 每组零件在 FreeCAD 中选中后合并 → 导出 STL
# 导出 5 个文件到 assets/meshes/：
#   base_link.stl, body.stl, head.stl, left_arm.stl, right_arm.stl
```

### Step 3：生成 inline mesh XML

```bash
python scripts/generate_inline_mesh.py \
    --input assets/meshes \
    --output assets/mjcf/electronbot_full_arm.xml
```

脚本核心逻辑：
1. 扫描 `assets/meshes/` 目录下的所有 `.stl` 文件
2. 按文件名分类：`base_link` → base, `body` → torso, `head` → head, `left_arm` → left_arm, `right_arm` → right_arm
3. 每组内多个 STL 合并为一个 inline mesh（顶点合并）
4. 保持 STL 原始单位（mm），不缩放
5. 输出 MJCF XML，内嵌 vertex/face 字符串

### Step 4：稳定性调优

**问题**：s 初始模型使用默认积分器导致仿真数值爆炸。

**解决方案**：
- 积分器：`implicitfast` → `RK4`（4 阶龙格-库塔，更稳定）
- 迭代次数：增加 `iterations="50"`
- 关节阻尼：`damping="4.0"`，`armature="0.1"`
- 执行器参数：按舵机规格设置（kp=30-80，kv=8-20）

### Step 5：最终调整 actuator 参数

```bash
python update_actuator.py
```

该脚本按舵机规格重新写入 `electronbot_full_arm.xml` 的 actuator 段：
- SG90 腰部舵机：kp=80, kv=20
- 2g 微型舵机（头部/双臂 Pitch）：kp=60, kv=15 (头部 kp=40)
- 2g 微型舵机（手臂 Roll）：kp=30, kv=8

### Step 6：验证

```bash
python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml
```

---

## 5. 验证方法

### 5.1 手动可视化验证

```bash
cd ElectronBot_SIM

# 启动 viewer，观察模型
python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml

# Control 面板拖动 actuator slider 测试各关节
# act_body / act_head / act_left_pitch / act_left_roll / act_right_pitch / act_right_roll
```

### 5.2 验证标准

- 6 个关节均可独立拖动，不卡死
- 身体绕 Z 轴旋转 ±90° 时，头部和双臂随动
- 手臂 Pitch/Y 轴运动时，仅手臂零件运动，身体侧外壳不动
- 仿真无 NaN 崩溃，稳态无抖动
- 模型比例与真机照片一致

### 5.3 模型结构快速检查

```bash
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_full_arm.xml'); print(m.njnt,'joints,',m.nu,'actuators')"
```

---

## 6. 工程经验教训

### 6.1 FreeCAD 分组是瓶颈

原始设计文档假设 24 个 CAD 零件可分别导出后独立处理。实际上：
- **运动组才是 MuJoCo 中的刚体单元**，同一组的零件在仿真中不可拆分
- 按运动组合并导出反而更简单（5 个文件 vs 24 个文件）
- 正确分组信息在 `assets/cad/electronbot_joints.FCMacro` 中定义，需运行宏才能提取

### 6.2 STL 导出时的零件污染

原始 STL 导出时身体侧外壳被错误合并到手臂文件中，必须重新导出修正。

**教训**：从 CAD 导出 STL 前，必须仔细确认每个零件属于哪个运动组，不能仅仅按名称或位置猜测。

### 6.3 MuJoCo mesh 自动中心化

MuJoCo 加载 inline mesh 时会自动将 mesh 质心移到 body 原点。这会导致位置偏移，必须用 `geom pos` 补偿：

```python
# 计算 arm geom 偏移量
body_hw = body_mesh_width / 2
arm_hw  = arm_mesh_width / 2
left_geom_offset = -(body_hw + arm_hw)
right_geom_offset = +(body_hw + arm_hw)
```

### 6.4 mm 单位对稳定性的影响

STL 为 mm 单位时惯性被放大 10¹² 倍，需用高增益控制器适配：
- `damping` 从 0.01 提到 4.0
- `armature` 从 0.001 提到 0.1
- 积分器改为 RK4

### 6.5 场景文件是不必要的

原始设计文档规划了 `scene_mesh.xml` 和 `scene_tabletop.xml` 两个场景文件。实际中：
- `electronbot_full_arm.xml` 自带光照和相机，可直接在 viewer 中观察
- 桌面场景对仿真无实际帮助，增大了维护负担
- **删繁就简，只保留核心模型文件**

---

## 附录：文件清单

### A. 核心文件

| 文件 | 描述 | 生成方式 |
|------|------|----------|
| `assets/mjcf/electronbot_full_arm.xml` | inline mesh 版 MJCF（~1.7MB） | `scripts/generate_inline_mesh.py` |
| `assets/meshes/base_link.stl` | 底座合并 STL | FreeCAD 按组导出 |
| `assets/meshes/body.stl` | 身体合并 STL | FreeCAD 按组导出 |
| `assets/meshes/head.stl` | 头部合并 STL | FreeCAD 按组导出 |
| `assets/meshes/left_arm.stl` | 左臂合并 STL（纯手臂） | FreeCAD 按组导出 |
| `assets/meshes/right_arm.stl` | 右臂合并 STL（纯手臂） | FreeCAD 按组导出 |

### B. CAD 源文件（`assets/cad/`）

| 文件 | 描述 |
|------|------|
| `ElectronBot.step` | 原始 CAD 模型（STEP 格式，24 零件） |
| `cadelectron.FCStd` | FreeCAD 原生格式（含装配结构） |
| `electronbot_joints.FCMacro` | 关节控制宏（提取零件分组信息） |
| `electronbot_assembly.FCMacro` | 装配宏 |
| `robot-structure.md` | 机器人结构解读文档 |
| `freecad-joint-control.md` | FreeCAD 关节控制操作指南 |

### C. 生成脚本

| 脚本 | 描述 |
|------|------|
| `scripts/generate_inline_mesh.py` | STL → inline mesh XML 生成器 |
| `update_actuator.py` | 写入按舵机规格匹配的 actuator kp/kv 参数 |

### D. 验证命令

```bash
# 最终验证
cd ElectronBot_SIM
python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml
```

---

## 变更记录

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|----------|------|
| v1.0 | 2026-07-03 | 初始版本（24 零件独立导出方案） | — |
| v1.1 | 2026-07-04 | 补充软件工程规范章节 | 架构师 |
| v1.2 | 2026-07-04 | 移除几何基元版，统一 inline mesh 版 | — |
| **v2.0** | **2026-07-06** | **根据实际实现重写——5 组 STL 合并导出，修正关节命名、结构、生成流程** | — |
