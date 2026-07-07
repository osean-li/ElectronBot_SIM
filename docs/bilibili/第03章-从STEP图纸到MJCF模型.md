# 第三章 从 STEP 图纸到 MJCF 模型

> **核心问题**：如何将包含 24 个零件的 ElectronBot CAD 原始图纸转换成 MuJoCo 可加载的仿真模型？

## 3.1 章节目标

本章从概要设计文档 Layer 1 出发，目标为：
1. 在 FreeCAD 中打开原始 STEP 文件并确认零件分组
2. 按运动组合并导出 5 组 STL
3. 编写脚本生成 inline mesh 格式的 MJCF 文件
4. 调参确保仿真稳定
5. 将模型与场景文件拆分

## 3.2 三类文件的定位

在开始操作前，需明确三种文件格式在流程中的角色：

| 文件格式 | 角色 | 内容 | 工具 |
|----------|------|------|------|
| CAD (.step / .FCStd) | 原始设计图纸 | 24 个零件的精确几何与装配关系 | FreeCAD / SolidWorks |
| STL (.stl) | 3D 表面网格 | 三角形顶点坐标，仅描述外形 | 从 CAD 导出 |
| MJCF (.xml) | 仿真描述文件 | 刚体层次、关节定义、执行器参数 | 脚本生成 |

三段转换关系：

```
CAD（设计意图）→ STL（外形）→ MJCF（运动描述）
```

## 3.3 零件分组与运动组确认

### 3.3.1 FreeCAD 中打开 STEP

```bash
/path/to/FreeCAD.AppImage assets/cad/ElectronBot.step
```

打开后可观察到 24 个零件，命名格式为 `Part__Feature` 后接序号，从名称难以判断物理归属。须通过专用宏文件确认分组。

### 3.3.2 运行宏确认分组

宏文件位于 `assets/cad/electronbot_joints.FCMacro`，在 FreeCAD 中运行后输出以下分组信息：

**底座组**（固定，不参与运动）：
```python
BASE_PARTS = [
    "Part__Feature043",  # 底座主体
    "Part__Feature044",  # 底座底板
]
```

**身体组**（绕 Z 轴旋转）：
```python
BODY_PARTS = [
    "Part__Feature034",  # 身体中心
    "Part__Feature035",  # 身体右侧外壳
    "Part__Feature036",  # 身体左侧外壳
]
```

**特别注意**：零件 `Part__Feature035` 和 `Part__Feature036` 属于身体组，**不属于**手臂。原始导出因未使用宏确认分组，曾导致这两个零件被错误地合并到手臂 STL 中。

**左臂组**（绕 Y 轴 Pitch、绕 X 轴 Roll，6 个零件）：
```python
LEFT_ARM_PARTS = [
    "Part__Feature042",  # 左手
    "Part__Feature045",  # 左手镜
    "Part__Feature046", "Part__Feature047",
    "Part__Feature048", "Part__Feature049",
]
```

**右臂组**（7 个零件）：
```python
RIGHT_ARM_PARTS = [
    "Part__Feature027",  # 齿轮
    "Part__Feature028",  # 小齿轮
    "Part__Feature029",  # 右手
    "Part__Feature030", "Part__Feature031",
    "Part__Feature032", "Part__Feature033",
]
```

**头部组**（5 个零件）：
```python
HEAD_PARTS = [
    "Part__Feature037", "Part__Feature038", "Part__Feature039",
    "Part__Feature040", "Part__Feature041",
]
```

### 3.3.3 我的分组实现过程

实际进行分组时，我按以下步骤操作：

1. **打开 FreeCAD 加载 STEP**：执行 `/path/to/FreeCAD.AppImage assets/cad/ElectronBot.step`。等待约 10 秒，FreeCAD 界面显示 24 个零件的装配树。
2. **寻找宏文件位置**：切换到 `assets/cad/` 目录，发现除了 STEP 文件外还有 `electronbot_joints.FCMacro`。这个宏是上次配合测试时留下的，我直接拿来用。
3. **在 FreeCAD 中运行宏**：菜单 Tools → Run Macro → 选择 `electronbot_joints.FCMacro`。宏运行后弹出分组信息，逐条复制到笔记中。
4. **逐个零件验证**：为了确认宏的分组正确，我逐组在 FreeCAD 中手动隐藏零件来观察：
   - 先隐藏 BODY_PARTS 中的 035 和 036，发现机器人的身体两侧外壳消失了，确认它们确实是身体的一部分。
   - 再隐藏 LEFT_ARM_PARTS 的 042~049，左臂全部消失，身体完好。
5. **记录分组**：将分组信息整理成 Python 列表，为后续导出脚本做准备。

### 3.3.4 零件污染问题

原始导出中的错误：身体侧外壳（`Part__Feature035` / `036`）被错误合并到手臂 STL。后果为 MuJoCo 中控制手臂时，身体外壳跟着手臂一起运动。排查此问题耗时较长，因为零件编号不包含结构信息，只能逐个隐藏定位。

**教训**：从 CAD 导出 STL 前，必须运行宏确认零件分组，不能凭名称或位置猜测。

#### 我的排查过程

发现这个问题是在初次加载生成的 XML 时。在 Viewer 中拖动 `act_left_pitch`，左臂抬起，但身体左侧外壳也跟着抬起来了。我最初以为是关节定义错误，排查了一上午。

排查步骤：
1. **检查关节轴方向**：怀疑 joint axis 设错了，左右臂轴反了——结果 XML 中 axis 定义正确。
2. **检查 body 父子关系**：怀疑 left_arm 挂在了错误的位置——结果父子关系正确。
3. **检查 STL 文件**：打开 `left_arm_fc.stl` 查看，发现它包含了身体外壳零件 `Part__Feature035`。
4. **回到 FreeCAD 验证**：逐个隐藏零件，观察哪个"不应该动"。最终确认 035 和 036 属于身体。
5. **重新导出**：按正确的分组重新运行导出脚本。

## 3.4 FreeCAD 命令行导出 STL

### 3.4.1 导出脚本

确认分组后，编写导出脚本 `export_groups.sh`。该脚本通过 FreeCAD 的命令行模式运行，使用 `QT_QPA_PLATFORM=offscreen` 在无显示器环境中执行：

```bash
QT_QPA_PLATFORM=offscreen DISPLAY= \
  /path/to/FreeCAD_1.1.1-Linux-x86_64-py311.AppImage \
  --appimage-extract-and-run -c "
import FreeCAD, Part, Mesh, MeshPart, os

doc = FreeCAD.open('assets/cad/cadelectron.FCStd')
OUT = 'assets/meshes'

# 身体组（3 个零件合并）
BODY = ['Part__Feature034','Part__Feature035','Part__Feature036']
shapes = [doc.getObject(p).Shape for p in BODY if doc.getObject(p)]
compound = Part.Compound(shapes)
mesh = doc.addObject('Mesh::Feature', 'tmp')
mesh.Mesh = MeshPart.meshFromShape(
    Shape=compound, LinearDeflection=0.5,
    AngularDeflection=0.5, Relative=False)
mesh.Mesh.write(os.path.join(OUT, 'body_fc.stl'))
doc.removeObject(mesh.Name)

# 左臂组（6 个零件合并）
LEFT = ['Part__Feature042','Part__Feature045',
        'Part__Feature046','Part__Feature047',
        'Part__Feature048','Part__Feature049']
shapes = [doc.getObject(p).Shape for p in LEFT if doc.getObject(p)]
compound = Part.Compound(shapes)
mesh = doc.addObject('Mesh::Feature', 'tmp')
mesh.Mesh = MeshPart.meshFromShape(
    Shape=compound, LinearDeflection=0.5,
    AngularDeflection=0.5, Relative=False)
mesh.Mesh.write(os.path.join(OUT, 'left_arm_fc.stl'))
doc.removeObject(mesh.Name)

# 右臂、底座、头部组同理（代码略）
FreeCAD.closeDocument(doc.Name)
"
```

### 3.4.2 脚本参数说明

- `LinearDeflection=0.5`：控制 STL 三角形边长不超过 0.5mm，保证网格精度
- `QT_QPA_PLATFORM=offscreen`：关闭 Qt 图形界面，允许在无显示器的环境中运行
- `Part.Compound(shapes)`：将同组的多个零件合并为一个复合体，再导出为单个 STL

### 3.4.3 我的导出实现过程

编写和执行导出脚本的过程如下：

1. **创建脚本文件**：新建 `export_groups.sh`，将宏输出的分组信息写入脚本。
2. **逐组编写导出代码**：先写身体组导出作为模板，测试通过后再复制为左臂、右臂、底座、头部。
3. **第一次运行报错**：
   ```bash
   QStandardPaths: wrong permissions on runtime directory
   ```
   原因是 FreeCAD 的 Qt 界面需要运行时目录权限。在命令前加上 `QT_QPA_PLATFORM=offscreen DISPLAY=` 解决。
4. **验证输出**：导出完成后执行 `ls -la assets/meshes/*.stl` 确认 5 个文件都在。用 `trimesh` 快速检查每个文件是否可加载：
   ```bash
   python3 -c "import trimesh; m=trimesh.load('assets/meshes/body_fc.stl'); print(m.bounds)"
   ```
5. **确认包围盒数据**：记录每个文件的包围盒范围，为后续偏移计算准备数据。

### 3.4.4 导出结果

| STL 文件 | 包含零件 | 件数 | 包围盒 X 范围 (mm) |
|----------|----------|:---:|:------------------:|
| `body_fc.stl` | 身体中心 + 两侧外壳 | 3 | [-27.9, 27.9] |
| `left_arm_fc.stl` | 左手 + 5 臂件 | 6 | [-34.9, -1.5] |
| `right_arm_fc.stl` | 右手 + 齿轮 + 5 臂件 | 7 | [1.5, 34.9] |

## 3.5 Inline Mesh 方案选择

MJCF 支持两种加载 3D 几何的方式：

**方式 A：外部 STL 引用**
```xml
<compiler meshscale="0.001"/>
<mesh name="body" file="../meshes/body.stl"/>
```

**方式 B：Inline Mesh（本项目选用）**
```xml
<mesh name="body" vertex="0.0012 0.0035 ..." face="0 1 2 ..."/>
```

选择方式 B 的原因有三：

1. **单位问题**。STL 默认单位为 mm，MuJoCo 内部使用 m。外部引用需加 `meshscale="0.001"` 缩放；Inline 方案由脚本在生成时完成 mm→m 转换。
2. **路径问题**。外部 STL 路径相对于 XML 文件位置，迁移环境时易出错。Inline 方案单文件自包含。
3. **自动中心化**。MuJoCo 编译 Inline Mesh 时自动将 mesh 质心移到 body 原点，使得 `geom pos` 偏移可通过脚本精确计算。外部 STL 无此特性，需手动测量。

## 3.6 MuJoCo 自动中心化与偏移计算

### 3.6.1 自动中心化的影响

检查导出的 STL 包围盒发现：`left_arm_fc.stl` 的质心在 X=-18.2mm，但 MuJoCo 编译后会被平移到 X=0。而 FreeCAD 装配中左臂的旋转 pivot 在 X=-18mm，因此需要额外补偿。

### 3.6.2 Geom 偏移量计算

```python
body_hw = 27.9   # 身体半宽 (mm)
arm_hw  = 16.7   # 手臂半宽 (mm)
left_body_pos = -0.018  # FreeCAD pivot 位置 (m)

# 偏移量公式：手臂需向外伸出，避免插入身体
left_geom_off = -(body_hw - abs(left_body_pos * 1000) + arm_hw - 1) / 1000.0
#              = -(27.9 - 18.0 + 16.7 - 1) / 1000
#              = -0.0256 (m)
```

结果：左臂 `geom pos` 需额外偏移 -0.0256m，右臂偏移 +0.0256m。

## 3.7 生成 inline mesh XML

### 3.7.1 生成脚本核心逻辑

`gen_full_arm.py` 的核心流程：

1. 使用 `trimesh` 库加载 5 个 STL 文件
2. 提取每个 mesh 的顶点与面数据，转换为字符串
3. 计算手臂偏移量
4. 组装 MJCF XML 模板并写入文件

### 3.7.2 最终模型结构

```xml
<mujoco model="electronbot">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="RK4" iterations="50"/>
  
  <worldbody>
    <body name="base_link" pos="0 0 47.0">
      <geom type="mesh" mesh="base_link" mass="0.045"/>
      <body name="body" pos="0 0 0.03">
        <joint name="body_joint" type="hinge" axis="0 0 1"
               range="-1.5708 1.5708"/>
        <geom type="mesh" mesh="body" mass="0.060"/>
        
        <body name="head" pos="0 0 0.07">
          <joint name="head_joint" type="hinge" axis="0 1 0"
                 range="-0.2618 0.2618"/>
          <geom type="mesh" mesh="head" mass="0.030"/>
        </body>
        
        <body name="left_arm" pos="-0.018 0 0.065">
          <joint name="left_pitch_joint" type="hinge" axis="0 1 0"
                 range="-1.5708 1.5708"/>
          <joint name="left_roll_joint" type="hinge" axis="1 0 0"
                 range="-0.7854 0.7854"/>
          <geom type="mesh" mesh="left_arm" pos="-0.0256 0 0" mass="0.005"/>
        </body>
        
        <!-- 右臂结构同左臂，pos 方向相反 -->
      </body>
    </body>
  </worldbody>
</mujoco>
```

### 3.7.3 Actuator 参数

| 关节 | 舵机 | kp | kv | 控制范围 (rad) | 对应角度 |
|------|------|:--:|:--:|:--------------:|:--------:|
| body_joint | SG90 | 80 | 20 | ±1.57 | ±90° |
| head_joint | 2g | 40 | 10 | ±0.26 | ±15° |
| left/right_pitch | 2g | 60 | 15 | ±1.57 | ±90° |
| left/right_roll | 2g | 30 | 8 | ±0.79 | ±45° |

kp 值根据舵机规格确定：SG90 扭矩 1.5kg·cm，kp 可用到 80；2g 舵机扭矩仅 0.2kg·cm，kp 过高会导致震荡。

## 3.8 仿真稳定性调优

### 3.8.1 积分器选择

首次加载 XML 并拖动 actuator 时，仿真出现 NaN 崩溃。排查发现根因为 STL 的 mm 单位导致惯性参数放大约 10¹² 倍，默认积分器无法收敛。

四种积分器的对比测试结果：

| 积分器 | 稳定性 | 备注 |
|--------|:------:|------|
| implicitfast（默认） | ❌ 爆炸 | 近似隐式欧拉，对极端条件不稳定 |
| Euler | ❌ 爆炸 | 一阶显式，最不稳定 |
| implicit | ❌ 爆炸 | 全隐式仍不稳定 |
| **RK4** | **✅ 稳定** | 4 阶龙格-库塔，精度最高，代价为 4 倍计算量 |

### 3.8.2 我的稳定性调试过程

生成 XML 后加载 viewer 遇到的 NaN 爆炸是我花费时间最长的一个问题。调试过程如下：

1. **第一次加载**：执行 `python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_full_arm.xml`，模型显示正常。
2. **拉动 actuator slider**：轻轻拖动 act_body，机器人立刻飞走，屏幕数值变成 NaN。仿真崩溃。
3. **排查接触参数**：怀疑是碰撞检测导致。调了 `<geom condim>`、`friction`、`margin`——没用。
4. **排查网格质量**：用 trimesh 检查 STL 是否有反转法线、重复顶点——网格质量正常。
5. **排查质量设置**：把 geom mass 从 0.005 调到 0.5——没用。
6. **怀疑阻尼不足**：把 damping 从 0.01 调到 1.0——还是崩。
7. **尝试换积分器**：MuJoCo 的 `<option integrator>` 支持 4 种。逐一测试：
   - `implicitfast` → 崩
   - `Euler` → 崩
   - `implicit` → 崩
   - `RK4` → **没崩！**
8. **确认根因**：同时把 damping 调到 4.0、armature 调到 0.1，模型完全稳定。
9. **记录日志**：在开发笔记中记下"STL mm 单位 → 惯性放大 10¹² 倍 → implicitfast 扛不住 → 换 RK4 + 提 damping"。

### 3.8.3 参数调优

除积分器外，还调整了以下参数：

```xml
<option timestep="0.002" integrator="RK4" iterations="50" cone="elliptic"/>
<default>
  <joint damping="4.0" armature="0.1" frictionloss="0.5"/>
</default>
```

- `damping`：0.01 → 4.0，抑制高频振动
- `armature`：0.001 → 0.1，增加关节惯性
- `iterations`：100 → 50，降低计算量

## 3.9 场景拆分

将原始单文件拆分为两个文件：

**`electronbot.xml`**：仅含机器人本体定义（`<mujoco>` ~ `</mujoco>` 中除场景外的全部内容），可独立加载。

**`electronbot_scene.xml`**：通过 `<include>` 引用机器人，独立管理场景元素：

```xml
<mujoco model="electronbot scene">
  <include file="electronbot.xml"/>
  
  <asset>
    <texture type="2d" name="groundplane" builtin="checker"
             rgb1="0.50 0.60 0.72" rgb2="0.06 0.13 0.22"
             markrgb="0.9 0.9 0.9" width="300" height="300"/>
  </asset>
  
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
```

### 3.9.1 场景拆分中的常见问题

| 问题 | 根因 | 解决 |
|------|------|------|
| XML 文件被写空 | 编辑器对大文件 `replace_in_file` 异常 | 改用 Python 脚本；`git checkout` 恢复 |
| 棋盘格地面不可见 | 模型尺寸畸变（70m 高）致纹理对比度不足 | 增大 rgb1/rgb2 差值；texrepeat 调至 0.05 |
| 零件断开 | 缩放 mesh 后 body 位置未同步 | 放弃缩放，仅调相机距离至 280m |
| 机器人悬空 47m | body 原点与 mesh 底部偏移 | `base_link pos z = 47.0` |
| EGL 渲染失败 | 无桌面环境且未设置环境变量 | 脚本开头加 `os.environ.setdefault("MUJOCO_GL","egl")` |
| 渲染分辨率超限 | 未设 offscreen framebuffer | scene.xml 加 `<global offwidth="960" offheight="720"/>` |

### 3.9.2 我的场景拆分过程

场景拆分是为了方便后面切换不同场景（桌面试、地面试、无场景纯模型试）。我的操作过程：

1. **从 `electronbot_full_arm.xml` 中提取模型本体**：复制原来文件，删除 `<asset>` 中的场景纹理和 `<worldbody>` 中的地面几何体，只保留机器人 body 树，保存为 `electronbot.xml`。
2. **创建 `electronbot_scene.xml`**：新建文件，通过 `<include file="electronbot.xml"/>` 引用模型，参考 `scene.xml` 的棋盘格设置添加场景元素。
3. **测试加载**：执行 `python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_scene.xml`——模型加载成功，棋盘格地面显示为纯色，完全不对。
4. **调试棋盘格问题**：翻开发笔记，对照之前踩过的坑：
   - 先确认模型尺寸：用 trimesh 测底座最低点在 z=-47m，机器人高 70m。
   - 棋盘格 `texrepeat` 从 1.0 改到 0.05，每格增大到 20m。
   - 亮暗格颜色从接近的 `0.2/0.1` 改为差别大的 `0.50/0.06`。
   - 重新加载，棋盘格显示正常。
5. **修正底座位置**：`base_link pos z` 从 `0.015` 改为 `47.0`，机器人落回地面。
6. **修正相机距离**：`cam.distance = 280.0` 保证全貌可见。
7. **记录 6 个问题**：将整个过程遇到的 6 个问题整理成章节表格，方便后续查询。

## 3.10 本章小结

本章从 STEP 原始 CAD 出发，经过零件分组、STL 合并导出、Inline Mesh XML 生成、稳定性调优及场景拆分，最终得到完整的 MJCF 模型文件。

与设计文档的主要差异：

| 项目 | 设计文档 | 实际实现 |
|------|---------|---------|
| STL 导出策略 | 24 零件分别导出 | 按 5 运动组合并导出 |
| 积分器 | implicitfast | RK4 |
| 场景文件 | 2 个独立场景 | 统一为 1 个 scene.xml |
| 模型尺寸 | 未指定 | mm→m 未缩放，模型高 70m，相机距 280m |
