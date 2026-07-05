# Phase 1：CAD → MJCF 建模

> **目标**：从 ElectronBot.step 原始 CAD 模型出发，提取零件几何体、质量/惯量、关节参数，生成 MuJoCo 可用的 MJCF 格式物理模型。
>
> **输入**：`xiaozhi-electronbot-docs/docs/cad/ElectronBot.step`（30.5MB，24 个零件）
>
> **输出**：
> - `assets/mjcf/electronbot_mesh.xml`——inline mesh 版（~500 fps，CAD 真实外形）
>
> **文档版本**: v1.2  
> **最后更新**: 2026-07-04  
> **变更类型**: 移除几何基元版，统一使用 inline mesh 版

---

## 1. 预期效果

### 1.1 文件结构

```
assets/mjcf/
├── electronbot_mesh.xml     ← inline mesh 版: CAD STL 真实外形 (~500 fps)
├── scene_mesh.xml           ← 场景外壳 include electronbot_mesh.xml
└── scene_tabletop.xml       ← 桌面场景 (含桌面碰撞体)
```

| 文件 | 用途 | fps | 文件大小 |
|------|------|:---:|------|
| electronbot_mesh.xml | 仿真+展示、真机效果预览 | ~500 | ~12MB 单文件 |

```
$ python scripts/validate_model.py
✅  electronbot.xml 加载成功
✅  24 个 body 定义完成
✅  6 个 joint 定义完成 (type=hinge, range 正确)
✅  6 个 actuator 定义完成 (gear 映射比正确)
✅  碰撞几何体生成完毕 (最大面数 < 200)
✅  惯性矩阵计算完毕 (总质量 ≈ 95g)
✅  MuJoCo viewer 中可拖拽视角观察
```

### 1.2 可视化验证标准

在 MuJoCo viewer 中打开 `electronbot_mesh.xml`，应看到：

```
- 完整机器人模型，近似白色 PLA 材质外观
- 6 个关节可以独立拖动（通过 viewer 的 actuator slider）
- 拖动头部 joint → 头部绕 Y 轴俯仰（±30°）
- 拖动身体 joint → 腰部绕 Z 轴旋转（±90°）
- 拖动右臂 pitch joint → 右臂举起/放下（±90°）
- 模型不会自我穿透（碰撞体正确）
- 零件位置、比例与真机照片一致
```

---

## 2. 建模参数——从代码推算的精确数值

### 2.1 24 个零件分组

从 FreeCAD 装配结构分析，24 个零件分为 5 个运动组：

| 组 | 零件数 | MuJoCo body 名 | 父 body | 运动类型 |
|----|:---:|------|------|------|
| 底座 (base) | 2 | `base` | world | 固定不动 |
| 身体 (body) | 3 | `torso` | base | 绕 Z 轴旋转 |
| 头部 (head) | 5 | `head` | torso | 绕 Y 轴俯仰 |
| 左臂 (left_arm) | 6 | `left_arm` | torso | Pitch+Roll |
| 右臂 (right_arm) | 7 | `right_arm` | torso | Pitch+Roll |

### 2.2 各零件体积（从 FreeCAD 宏注释中提取）

```
Part__Feature043 (底座)    : vol ≈ 21440 mm³
Part__Feature044 (底座底)   : vol ≈ 8198  mm³
Part__Feature034 (身体中心)  : vol ≈ 16545 mm³
Part__Feature035 (身体右侧)  : vol ≈ 18741 mm³
Part__Feature036 (身体左侧)  : vol ≈ 18652 mm³
Part__Feature037 (前脸)     : vol ≈ 2428  mm³
Part__Feature038 (头顶)     : vol ≈ 9195  mm³
Part__Feature039 (头部主壳)  : vol ≈ 14318 mm³
```

> 其余零件体积未标注，用 CAD 软件自动计算。

**质量计算**：PLA 密度 1.24 g/cm³

```
底座组总质量 ≈ (21440 + 8198) / 1000 * 1.24 ≈ 36.8g
身体组总质量 ≈ (16545 + 18741 + 18652) / 1000 * 1.24 ≈ 66.9g
头部组总质量 ≈ (2428 + 9195 + 14318 + 其他2件) / 1000 * 1.24 ≈ 32.0g
双臂组总质量 ≈ 每个臂约 8-12g
总质量估计 ≈ 95-110g（不含舵机、PCB、电池等电子件）
```

> 仿真中需加上舵机、PCB、电池的附加质量。SG90 约 9g，2g 舵机约 2g，电池约 18g，PCB 约 10g，
> 总附加质量约 60g。**完整机器人总质量约 160g。**

### 2.3 关节参数——关键数据

#### 2.3.1 旋转中心坐标（从 FreeCAD 宏提取）

| 关节 | MuJoCo body | 旋转中心 (x, y, z) mm | 旋转轴 |
|------|-------------|----------------------|--------|
| body | torso | ≈ (0, 0, -6) | Z |
| head | head | ≈ (3, 0, 25) | Y |
| left_arm | left_arm | ≈ (-17, 0, 0) | Y / X |
| right_arm | right_arm | ≈ (17, 0, 0) | Y / X |

#### 2.3.2 舵机→机械关节映射比（从固件安全范围 ← → CAD 机械范围推算）

| 关节 | 固件舵机安全范围 | 中心 | CAD 机械范围 | **映射比** | 方向 |
|------|:---:|:---:|:---:|:---:|:---:|
| HEAD | 75° ~ 105°（30°） | 90° | ±30°（60°） | **2.0** | 正向 |
| BODY | 30° ~ 150°（120°） | 90° | ±90°（180°） | **1.5** | 正向 |
| RIGHT_PITCH | 0° ~ 180°（180°） | 180→0 | ±90°（180°） | **1.0** | **反向** |
| LEFT_PITCH | 0° ~ 180°（180°） | 0→180 | ±90°（180°） | **1.0** | 正向 |
| RIGHT_ROLL | 100° ~ 180°（80°） | 140 | ±45°（90°） | **1.125** | **反向** |
| LEFT_ROLL | 0° ~ 80°（80°） | 40 | ±45°（90°） | **1.125** | 正向 |

> 映射比 = CAD 机械范围 / 固件舵机范围  
> 反向表示：舵机角度增大 → 机械关节角度减小（右手坐标系约定下）

**舵机初始位置 → MuJoCo 初始关节角度转换：**

```python
# 真机舵机初始值（来自 movements.h）
servo_initial = [180, 180, 0, 0, 90, 90]  # RP, RR, LP, LR, BODY, HEAD

# 中心值
center   = [180, 140, 0, 40, 90, 90]
ratio    = [1.0, 1.125, 1.0, 1.125, 1.5, 2.0]
direction = [-1, -1, 1, 1, 1, 1]  # -1=反向

# 计算 MuJoCo 初始关节角度：
# offset = (servo_angle - center) * ratio * direction
# RP:  (180-180) * 1.0 * -1 = 0°
# RR:  (180-140) * 1.125 * -1 = -45°  ← 右臂 Roll 在 MuJoCo 中初始 -45°
# LP:  (0-0) * 1.0 * 1 = 0°
# LR:  (0-40) * 1.125 * 1 = -45°     ← 左臂 Roll 初始 -45°
# BODY:(90-90) * 1.5 * 1 = 0°
# HEAD:(90-90) * 2.0 * 1 = 0°
```

> 注意：RR/LR 的舵机初始值 180/0 不等于机械中心 140/40，所以初始状态手臂 Roll 并非居中。
> 这是预定义的"休息姿态"——手臂自然下垂时 Roll 处于极限位置。

---

## 3. 实现步骤

### Step 1：CAD 零件导出到 STL → 生成 inline mesh XML

> ⚠️ **关键教训**：STL 文件使用毫米单位，MuJoCo 按米解析会导致惯性放大 10¹² 倍，关节卡死。
> **正确做法**：保持 STL 原始 mm 单位不变，用控制器参数适配大惯量。

```bash
# 1. FreeCAD 导出 STL
# 打开 cadelectron.FCStd → 每个零件 → 导出 → STL Mesh → assets/meshes/

# 2. 生成 inline mesh XML（核心创新）
python scripts/generate_inline_mesh.py

# 脚本核心逻辑: 从 STL 读取顶点/面，编码为字符串嵌入 MJCF
# 输出: assets/mjcf/electronbot_mesh.xml (不依赖文件系统，零外部文件)
```

**generate_inline_mesh.py 核心代码参考：**

```python
import trimesh, struct, base64, xml.etree.ElementTree as ET

def stl_to_inline_mesh(stl_path: str) -> tuple[str, str]:
    """STL → MuJoCo inline mesh 的 vertex/face 字符串"""
    mesh = trimesh.load(stl_path)
    # 顶点: 二进制编码后 base64
    verts = base64.b64encode(mesh.vertices.astype('<f4').tobytes()).decode()
    # 面: uint32 编码后 base64
    faces = base64.b64encode(mesh.faces.astype('<i4').tobytes()).decode()
    return verts, faces
```

**Inline mesh XML 示例：**

```xml
<asset>
  <mesh name="base_link"
        vertex="AAAAQAAAAMEAAADC..."   <!-- 二进制顶点数据 base64 -->
        face="AAAAAQAAAAIAAAAD..."/>   <!-- 二进制面数据 base64 -->
</asset>
```

**优势**：不依赖文件系统、零外部文件、单文件可移植（~2MB）。

### 🔴 关键教训：mm 单位控制器适配

STL 为 mm 单位时惯性被放大 10¹² 倍。**不要缩放 STL**（会导致 MuJoCo 自动重算顶点中心、位置错乱），用控制器参数适配：

```xml
<!-- 几何基元版 (m 单位, 正常惯性) -->
<joint damping="0.01" armature="0.001"/>
<position kp="50" forcerange="-0.15 0.15"/>

<!-- inline mesh 版 (mm 单位, 大惯性) -->
<joint damping="0.5" armature="0.01"/>          <!-- 防过阻尼 -->
<position kp="500" kv="50" forcerange="-100 100"/>  <!-- 高增益 + 大力矩 -->
```

相机参数同步适配：

```python
cam.lookat[:] = [0, 0, 50]   # mm 单位高度中心
cam.distance = 200            # 适配 mm 级大模型
```

### Step 2：计算惯性参数

对每个 STL 文件，用脚本计算体积和惯性矩阵：

```python
# scripts/calc_inertia.py
import trimesh
import numpy as np

PLA_DENSITY = 1.24e-6  # g/mm³ → kg/mm³ for MuJoCo

meshes = {
    "base_top": "assets/meshes/base_top.stl",
    # ... 全部 24 个零件
}

for name, path in meshes.items():
    mesh = trimesh.load(path)
    volume = mesh.volume  # mm³
    mass = volume * PLA_DENSITY  # kg
    inertia = mesh.moment_inertia * PLA_DENSITY  # 相对于质心
    com = mesh.center_mass
    
    print(f"{name}: mass={mass*1000:.1f}g, com={com}")
    # 输出格式可直接填入 MJCF
```

### Step 3：生成 MJCF XML（inline mesh 版）

文件：`assets/mjcf/electronbot_mesh.xml`——由 `generate_inline_mesh.py` 自动生成。

核心结构（mm 单位，高增益控制器适配大惯量）：

```xml
<mujoco model="electronbot">
  <compiler angle="degree" />

  <default>
    <geom rgba="0.9 0.9 0.9 1" />
    <joint limited="true" damping="0.01" armature="0.001" />
    <position ctrllimited="true" />
  </default>

  <!-- ===== 底座 (固定) ===== -->
  <body name="base" pos="0 0 0">
    <!-- 视觉 mesh (由 generate_inline_mesh.py 自动生成) -->
    <geom type="cylinder" size="0.026 0.008" mass="0.0266" />  <!-- 底座顶 -->
    <geom type="cylinder" size="0.026 0.006" pos="0 0 -0.006" mass="0.0102" />  <!-- 底座底 -->
    <!-- 附加质量：电子件 60g -->
    <geom type="box" size="0.02 0.02 0.005" pos="0 0 -0.01" 
          mass="0.060" rgba="0.2 0.2 0.2 0.3" />
    
    <!-- ===== 身体 (绕 Z 旋转 ±90°) ===== -->
    <body name="torso" pos="0 0 0.006">
      <joint name="joint_body" type="hinge" axis="0 0 1" 
             range="-90 90" pos="0 0 -0.006" />
      <geom type="box" size="0.026 0.018 0.030" mass="0.0669" />
      <geom type="box" size="0.012 0.012 0.015" pos="0 0 -0.005"
            mass="0.009" rgba="0.1 0.3 0.8 0.3" />  <!-- SG90 舵机 -->

      <!-- ===== 头部 (绕 Y 俯仰 ±30°) ===== -->
      <body name="head" pos="0.003 0 0.025">
        <joint name="joint_head" type="hinge" axis="0 1 0"
               range="-30 30" pos="-0.003 0 -0.025" />
        <geom type="sphere" size="0.018" mass="0.032" />
      </body>

      <!-- ===== 左臂 ===== -->
      <body name="left_arm" pos="-0.017 0 0">
        <joint name="joint_lp" type="hinge" axis="0 1 0"
               range="-90 90" pos="0.017 0 0" />
        <geom type="capsule" size="0.004 0.018" mass="0.005" />
        <body name="left_hand" pos="0 0 -0.018">
          <joint name="joint_lr" type="hinge" axis="1 0 0" range="-45 45" />
          <geom type="sphere" size="0.008" mass="0.003" />
        </body>
      </body>

      <!-- ===== 右臂 (对称) ===== -->
      <body name="right_arm" pos="0.017 0 0">
        <joint name="joint_rp" type="hinge" axis="0 1 0"
               range="-90 90" pos="-0.017 0 0" />
        <geom type="capsule" size="0.004 0.018" mass="0.005" />
        <body name="right_hand" pos="0 0 -0.018">
          <joint name="joint_rr" type="hinge" axis="1 0 0" range="-45 45" />
          <geom type="sphere" size="0.008" mass="0.003" />
        </body>
      </body>
    </body>
  </body>

  <!-- ===== 执行器 ===== -->
  <actuator>
    <position name="act_body" joint="joint_body" 
              ctrlrange="-90 90" gear="1.5" kp="60" forcerange="-0.15 0.15" />
    <position name="act_head" joint="joint_head" 
              ctrlrange="-30 30" gear="2.0" kp="40" forcerange="-0.03 0.03" />
    <position name="act_rp" joint="joint_rp" 
              ctrlrange="-90 90" gear="1.0" kp="50" forcerange="-0.03 0.03" />
    <position name="act_rr" joint="joint_rr" 
              ctrlrange="-45 45" gear="1.125" kp="30" forcerange="-0.05 0.05" />
    <position name="act_lp" joint="joint_lp" 
              ctrlrange="-90 90" gear="1.0" kp="50" forcerange="-0.03 0.03" />
    <position name="act_lr" joint="joint_lr" 
              ctrlrange="-45 45" gear="1.125" kp="30" forcerange="-0.03 0.03" />
  </actuator>

  <keyframe>
    <key name="home" qpos="0 0 0 -45 0 -45" />
  </keyframe>
</mujoco>
```

### Step 4：碰撞几何体简化

对每个显示几何体创建简化碰撞版本（MuJoCo 要求碰撞几何体为凸形状）：

```xml
<!-- 每个 body 中有视觉几何体和碰撞几何体 -->
<body name="torso" pos="0 0 0.006">
  <joint name="joint_body" type="hinge" axis="0 0 1" range="-90 90" pos="0 0 -0.006" />
  
  <!-- 视觉（高精度网格） -->
  <geom mesh="torso_center" group="1" mass="0.0205" />
  
  <!-- 碰撞（简化凸包） -->
  <geom type="cylinder" size="0.016 0.030" group="3" 
        rgba="0 0 0 0" mass="0" />
  <geom type="box" size="0.026 0.018 0.020" pos="0 0 -0.005" 
        group="3" rgba="0 0 0 0" mass="0" />
</body>
```

### Step 5：创建桌面场景

文件：`assets/mjcf/scene_tabletop.xml`

```xml
<mujoco model="electronbot_scene_tabletop">
  <include file="electronbot_mesh.xml" />

  <worldbody>
    <!-- 桌面 -->
    <geom name="table" type="box" size="200 200 10" pos="0 0 -50"
          rgba="0.6 0.4 0.2 1" />

    <!-- 光照 -->
    <light name="top" pos="0 0 200" />
    <light name="side" pos="200 200 100" />

    <!-- 摄像头（模拟 GC9A01 视角，mm 单位） -->
    <camera name="head_cam" pos="3 150 200" xyaxes="-1 0 0 0 0 1"
            fovy="60" resolution="240 240" />
  </worldbody>
</mujoco>
```

---

## 4. 验证方法

### 4.1 自动化验证脚本

```python
# scripts/validate_model.py

import mujoco
import numpy as np

def validate_model(model_path: str):
    """加载并验证 MJCF 模型"""
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    errors = []
    
    # 1. 检查 body 数量
    expected_bodies = {"base", "torso", "head", "left_arm", "left_hand", 
                       "right_arm", "right_hand"}
    actual_bodies = {model.body(i).name for i in range(model.nbody)}
    for b in expected_bodies:
        if b not in actual_bodies:
            errors.append(f"缺少 body: {b}")
    
    # 2. 检查 joint 数量和类型
    expected_joints = {
        "joint_body": ("hinge", -90, 90),
        "joint_head": ("hinge", -30, 30),
        "joint_lp":   ("hinge", -90, 90),
        "joint_lr":   ("hinge", -45, 45),
        "joint_rp":   ("hinge", -90, 90),
        "joint_rr":   ("hinge", -45, 45),
    }
    for j in range(model.njnt):
        name = model.joint(j).name
        if name in expected_joints:
            jtype, jmin, jmax = expected_joints[name]
            actual_range = model.jnt_range[j]
            if abs(actual_range[0] - jmin) > 1 or abs(actual_range[1] - jmax) > 1:
                errors.append(f"joint {name} range 不对: {actual_range}")
    
    # 3. 检查 actuator gear 映射比
    expected_gears = {
        "act_head": 2.0,    "act_body": 1.5,
        "act_rp":   1.0,    "act_lp":   1.0,
        "act_rr":   1.125,  "act_lr":   1.125,
    }
    for i in range(model.nu):
        name = model.actuator(i).name
        if name in expected_gears:
            actual_gear = model.actuator_gear[i][0]
            expected = expected_gears[name]
            if abs(actual_gear - expected) > 0.01:
                errors.append(f"actuator {name} gear={actual_gear}, 期望={expected}")
    
    # 4. 检查碰撞体（确保无大的未简化的 mesh 碰撞体）
    for g in range(model.ngeom):
        geom_type = model.geom_type[g]
        is_collision = model.geom_group[g] == 3 or model.geom_contype[g] > 0
        if is_collision and geom_type == 7:  # type 7 = mesh
            errors.append(f"碰撞体 {model.geom(g).name} 使用原始 mesh，应简化为凸体")
    
    # 5. 模拟 100 步，无崩溃
    try:
        for _ in range(100):
            mujoco.mj_step(model, data)
    except Exception as e:
        errors.append(f"仿真崩溃: {e}")
    
    # 6. 出口
    if errors:
        print("❌ 验证失败：")
        for e in errors:
            print(f"   - {e}")
        return False
    else:
        print("✅ 模型验证全部通过！")
        print(f"   nbody={model.nbody}  njoint={model.njnt}  ngeom={model.ngeom}")
        print(f"   nactuator={model.nu}  总质量={model.body_mass.sum()*1000:.1f}g")
        return True

if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "assets/mjcf/scene_tabletop.xml"
    validate_model(path)
```

### 4.2 手动可视化验证

```bash
# inline mesh 版 (CAD 真实外形)
MUJOCO_GL=egl python -m mujoco.viewer --mjcf=assets/mjcf/electronbot_mesh.xml

# 桌面场景
MUJOCO_GL=egl python -m mujoco.viewer --mjcf=assets/mjcf/scene_tabletop.xml

# SSH / 无 X11 环境 → EGL 无头渲染到 PNG
MUJOCO_GL=egl python3 -c "
import mujoco, cv2
m = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh.xml')
d = mujoco.MjData(m); r = mujoco.Renderer(m, 480, 480)
mujoco.mj_forward(m, d); r.update_scene(d)
cv2.imwrite('snapshot.png', cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
"
```

### 4.3 与真机对照

用真机拍照（前/侧/后三个角度），与 MuJoCo viewer 截图逐帧对比零件位置和比例。

---

## 5. 调试命令速查

```bash
# 模型结构检查
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_mesh.xml'); print(m.njnt,'joints,',m.nu,'actuators')"

# inline mesh 版 viewer
MUJOCO_GL=egl python -m mujoco.viewer --mjcf=assets/mjcf/electronbot_mesh.xml

# mesh 版 (展示)
MUJOCO_GL=egl python -m mujoco.viewer --mjcf=assets/mjcf/scene_mesh.xml

# CAD → STL 导出
python scripts/export_cad_meshes.py

# STL → inline mesh XML
python scripts/generate_inline_mesh.py

# 场景 mesh 版生成
sed 's/electronbot.xml/electronbot_mesh.xml/' assets/mjcf/scene.xml > assets/mjcf/scene_mesh.xml
```

## 6. 交付物清单

| 文件 | 描述 | 验证标准 |
|------|------|----------|
| `assets/meshes/*.stl` | 24个简化网格 | 每个 < 500KB |
| `assets/mjcf/electronbot_mesh.xml` | inline mesh 版 MJCF | 零外部文件, ~12MB, ~500 fps, validate_model.py 通过 |
| `assets/mjcf/scene_mesh.xml` | 场景 include mesh 版 | MuJoCo viewer 可打开 |
| `assets/mjcf/scene_tabletop.xml` | 桌面场景 (含桌面碰撞体) | MuJoCo viewer 可打开 |
| `scripts/generate_inline_mesh.py` | STL→inline mesh 生成器 | 输出合法 MJCF vertex/face |
| `scripts/calc_inertia.py` | 惯性计算脚本 | 输出正确的 mass/inertia |
| `scripts/validate_model.py` | 模型验证脚本 | 全部检查通过 |

---

## 7. 接口设计

### 7.1 模块对外接口

本模块（CAD → MJCF 建模）对外暴露三个核心脚本，均位于 `scripts/` 目录下，可作为命令行工具独立调用，也可被 Python 代码 import 复用。

#### 7.1.1 `generate_inline_mesh.py`

```python
def stl_to_inline_mesh(stl_path: str) -> tuple[str, str]:
    """
    将单个 STL 文件转换为 MuJoCo inline mesh 所需的 base64 编码字符串。

    参数:
        stl_path: STL 文件绝对或相对路径（单位: mm，不进行缩放）

    返回:
        tuple[str, str]:
            - vertex: 顶点数据 base64 字符串（float32 little-endian）
            - face:   面数据 base64 字符串（int32 little-endian）

    异常:
        FileNotFoundError: STL 文件不存在
        ValueError:        STL 解析失败（格式损坏或为空）
    """
```

命令行入口：`python scripts/generate_inline_mesh.py [--input assets/meshes/] [--output assets/mjcf/electronbot_mesh.xml]`

#### 7.1.2 `calc_inertia.py`

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class InertiaResult:
    """单个零件的惯性计算结果"""
    name: str                   # 零件名（与 STL 文件名对应）
    mass: float                 # 质量 (kg)
    com: np.ndarray             # 质心坐标 (3,) mm
    inertia_matrix: np.ndarray  # 相对质心的惯性张量 (3, 3) kg·mm²


def calculate_inertia(stl_path: str, density: float = 1.24e-6) -> InertiaResult:
    """
    基于 STL 几何体与均质密度假设计算质量、质心、惯性张量。

    参数:
        stl_path: STL 文件路径（单位: mm）
        density:  密度 (kg/mm³)，默认 PLA_DENSITY=1.24e-6（即 1.24 g/cm³）

    返回:
        InertiaResult: 含 mass/com/inertia_matrix 的数据类

    异常:
        FileNotFoundError: STL 文件不存在
        ZeroDivisionError: 退化几何体（体积为 0）导致惯性张量不可计算
    """
```

命令行入口：`python scripts/calc_inertia.py [--density 1.24e-6]`，输出 markdown 表格供直接粘贴到 MJCF。

#### 7.1.3 `validate_model.py`

```python
def validate_model(model_path: str) -> bool:
    """
    加载并验证 MJCF 模型的结构完整性、关节范围、actuator gear 映射、
    碰撞几何体合理性，并执行 100 步仿真 smoke test。

    参数:
        model_path: MJCF XML 文件路径（可为 scene.xml 或单文件）

    返回:
        bool: True 表示全部检查通过，False 表示存在错误（错误列表打印到 stdout）

    异常:
        mujoco.FatalError: XML 解析失败（语法错误/引用缺失）会直接抛出
    """
```

命令行入口：`python scripts/validate_model.py [assets/mjcf/scene_tabletop.xml]`

### 7.2 输入输出契约

| 接口 | 输入格式 | 输出格式 | 异常条件 |
|------|----------|----------|----------|
| `stl_to_inline_mesh` | STL 文件路径（ASCII 或 binary STL，mm 单位） | `(vertex_b64, face_b64)` 字符串元组 | 文件不存在；STL 损坏；顶点数为 0 |
| `calculate_inertia` | STL 路径 + 密度（kg/mm³） | `InertiaResult`（mass, com, inertia_matrix） | 文件不存在；零体积退化几何体；非流形 mesh |
| `validate_model` | MJCF XML 路径 | `bool`（详细错误打印到 stdout） | XML 语法错误；body/joint 缺失；引用文件不存在 |
| `generate_inline_mesh.py` (CLI) | `--input` meshes 目录 | `--output` electronbot_mesh.xml | 输入目录为空；24 个 STL 缺失 |
| `calc_inertia.py` (CLI) | `--density` 参数 | stdout markdown 表格 | 单个 STL 失败时跳过并记录，不中断批量流程 |

---

## 8. 数据模型

### 8.1 核心数据结构

#### 8.1.1 STL mesh 数据结构（trimesh.Trimesh 视图）

```python
import numpy as np

# trimesh.Trimesh 的关键字段
class TrimeshView:
    vertices: np.ndarray        # shape (N, 3), dtype float32, 单位 mm
    faces:    np.ndarray        # shape (M, 3), dtype int32, 顶点索引
    volume:   float             # 体积 mm³（仅对水密网格有效）
    center_mass: np.ndarray     # shape (3,) 质心 mm
    moment_inertia: np.ndarray  # shape (3, 3) 惯性张量 mm⁵（需乘密度得 kg·mm²）
```

#### 8.1.2 InertiaResult dataclass

```python
from dataclasses import dataclass

@dataclass
class InertiaResult:
    name: str
    mass: float                   # kg
    com: np.ndarray               # (3,) mm
    inertia_matrix: np.ndarray    # (3, 3) kg·mm²，关于质心
```

#### 8.1.3 MJCF inline mesh 格式

```xml
<mesh name="base_link"
      vertex="AAAAQAAAAMEAAADC..."   <!-- float32 little-endian base64 -->
      face="AAAAAQAAAAIAAAAD..."/>   <!-- int32 little-endian base64 -->
```

编码约定：
- 顶点：`base64(vertices.astype('<f4').tobytes())`
- 面：`base64(faces.astype('<i4').tobytes())`
- 单位：与 STL 一致（mm），不进行单位换算

#### 8.1.4 零件清单数据结构

```python
# scripts/calc_inertia.py 内部用
meshes: dict[str, str] = {
    "base_top":     "assets/meshes/base_top.stl",
    "torso_center": "assets/meshes/torso_center.stl",
    # ... 共 24 项
}
```

### 8.2 数据流

```
            ┌──────────────────┐
            │ ElectronBot.step │  (原始 CAD, 30.5MB)
            └────────┬─────────┘
                     │ FreeCAD 手动导出
                     ▼
        ┌──────────────────────────┐
        │ assets/meshes/*.stl × 24 │  (mm 单位, 每个小于 500KB)
        └─────┬────────────────────┘
              │
              ├──────────────────────────────────────┐
              ▼                                      ▼
   ┌─────────────────────────┐         ┌──────────────────────────┐
   │ generate_inline_mesh.py │         │ calc_inertia.py          │
   │ stl → base64 vertex/face│         │ stl → InertiaResult      │
   └───────────┬─────────────┘         └──────────┬───────────────┘
               │                                  │
               ▼                                  ▼
   ┌─────────────────────────┐         ┌──────────────────────────┐
   │ electronbot_mesh.xml    │◀────────│ mass/com/inertia 填入    │
   │ (inline mesh 版, ~12MB) │         │ body 的 <inertial> 标签   │
   └───────────┬─────────────┘         └──────────────────────────┘
               │
               ▼
   ┌─────────────────────────┐
   │ scene_tabletop.xml      │  (桌面场景)
   └───────────┬─────────────┘
               │
               ▼
   ┌─────────────────────────┐
   │ validate_model.py       │  → bool (通过/失败)
   │ body/joint/actuator/    │
   │ 碰撞体/100 步仿真       │
   └─────────────────────────┘
```

---

## 9. 错误处理与恢复

### 9.1 错误分类

| 错误类型 | 触发条件 | 处理策略 | 用户感知 |
|----------|----------|----------|----------|
| STL 文件加载失败 | 文件不存在、路径权限不足 | `FileNotFoundError` 抛出，CLI 退出码 2 | 控制台红色错误提示 + 路径 |
| STL 格式错误 | 非 STL 格式、ASCII/Binary 混乱、顶点数为 0 | `trimesh.load` 抛 `ValueError`，捕获后跳过该零件 | 警告日志，批量流程继续 |
| 退化几何体 | 体积为 0（薄壳、自相交、非流形） | `ZeroDivisionError` 捕获，惯性矩阵置为单位矩阵 ×1e-6 | 警告日志，标注"惯性使用占位值" |
| 惯性矩阵负定 | 数值误差导致主惯性矩为负 | `np.linalg.eigvalsh` 检测后取绝对值并 clamp | 警告日志，模型仍可加载 |
| MJCF XML 验证失败 | body/joint/actuator 缺失或范围错误 | `validate_model` 收集全部错误后一次性打印 | 控制台列出所有错误项 |
| mm 单位惯性放大 10¹² 倍 | STL 为 mm，未适配控制器参数 | 不缩放 STL，改用高 kp/kv/forcerange（见 §3 关键教训） | 训练版正常，mesh 版关节不卡死 |
| 碰撞体使用原始 mesh | 凸包简化遗漏，group=3 仍为 mesh | `validate_model` 报错并指明 geom 名 | 控制台提示需替换为凸体 |
| actuator gear 不匹配 | gear 与映射比表不一致 | `validate_model` 打印期望值 vs 实际值 | 控制台列出差异 |
| 仿真 100 步崩溃 | NaN、关节发散、穿透 | 捕获 `Exception`，记录堆栈 | 控制台输出崩溃原因 |

### 9.2 异常恢复流程

#### 9.2.1 STL 批量处理中的单文件失败

```
[calc_inertia.py 批量循环]
  for name, path in meshes.items():
      try:
          result = calculate_inertia(path, density)
          results.append(result)
      except (FileNotFoundError, ValueError, ZeroDivisionError) as e:
          logger.warning(f"跳过 {name}: {e}")
          # 用占位惯性，避免阻塞下游 MJCF 生成
          results.append(InertiaResult(
              name=name, mass=1e-6,
              com=np.zeros(3),
              inertia_matrix=np.eye(3) * 1e-9
          ))
  # 全部失败超过 50% → 退出码 3，提示用户检查 meshes 目录
```

#### 9.2.2 mm 单位惯性放大问题（已知坑）

现象：STL 为 mm 单位，MuJoCo 按米解析导致惯性放大 10¹² 倍，关节卡死不动。
恢复流程：
1. **不要缩放 STL**（会导致 MuJoCo 自动重算顶点中心、位置错乱）
2. 保持 STL 原 mm 单位，调整控制器参数适配大惯量：
   - `damping` 从 0.01 提到 0.5（防过阻尼）
   - `armature` 从 0.001 提到 0.01
   - `position kp` 从 50 提到 500
   - `forcerange` 从 ±0.15 提到 ±100
3. 相机参数同步适配：`cam.lookat=[0,0,50]`, `cam.distance=200`
4. 验证：`validate_model.py` 通过 + viewer 中关节可拖动

#### 9.2.3 validate_model 失败后的修复路径

```
validate_model 报错
   │
   ├─ body 缺失         → 检查 MJCF <body> 标签拼写、父嵌套关系
   ├─ joint range 错误  → 对照 §2.3.2 映射比表，修正 range 属性
   ├─ actuator gear 错误 → 对照 [1.0, 1.125, 1.0, 1.125, 1.5, 2.0] 修正
   ├─ 碰撞体为原始 mesh → 替换为 type="cylinder"/"box"/"capsule" 凸体
   └─ 仿真崩溃          → 检查 inertia_matrix 是否含 NaN、damping 是否过小
```

---

## 10. 配置管理

### 10.1 配置参数表

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `PLA_DENSITY` | float | 1.24e-6 | — | PLA 密度，单位 kg/mm³（即 1.24 g/cm³） |
| `EXTRA_MASS_KG` | float | 0.060 | 0 ~ 0.2 | 附加质量（舵机+PCB+电池），加在 base body |
| `MAX_STL_SIZE_KB` | int | 500 | 100 ~ 2000 | 单个 STL 文件大小上限 |
| `MAX_COLLISION_FACES` | int | 200 | 50 ~ 1000 | 碰撞几何体最大面数 |
| `SIM_STEPS_SMOKE` | int | 100 | 10 ~ 1000 | validate_model 中的 smoke test 步数 |
| `joint_body_range` | tuple | (-90, 90) | 度 | 腰部绕 Z 轴旋转范围 |
| `joint_head_range` | tuple | (-30, 30) | 度 | 头部绕 Y 轴俯仰范围 |
| `joint_lp_range` | tuple | (-90, 90) | 度 | 左臂 pitch 范围 |
| `joint_rp_range` | tuple | (-90, 90) | 度 | 右臂 pitch 范围 |
| `joint_lr_range` | tuple | (-45, 45) | 度 | 左臂 roll 范围 |
| `joint_rr_range` | tuple | (-45, 45) | 度 | 右臂 roll 范围 |
| `gear_head` | float | 2.0 | — | HEAD 关节映射比 |
| `gear_body` | float | 1.5 | — | BODY 关节映射比 |
| `gear_rp` | float | 1.0 | — | RIGHT_PITCH 映射比 |
| `gear_lp` | float | 1.0 | — | LEFT_PITCH 映射比 |
| `gear_rr` | float | 1.125 | — | RIGHT_ROLL 映射比 |
| `gear_lr` | float | 1.125 | — | LEFT_ROLL 映射比 |
| `kp_mesh` | float | 500 | 100 ~ 1000 | inline mesh 版 position kp（适配大惯量） |

> **映射比表**（与固件安全范围对应，顺序 `[RP, RR, LP, LR, BODY, HEAD]`）：`[1.0, 1.125, 1.0, 1.125, 1.5, 2.0]`

### 10.2 环境变量

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `MUJOCO_GL` | 未设置（自动） | 渲染后端：`egl`（无头 GPU）/ `osmesa`（纯 CPU）/ `glfw`（X11） |
| `ELECTRONBOT_MESH_DIR` | `assets/meshes` | STL 输入目录 |
| `ELECTRONBOT_MJCF_DIR` | `assets/mjcf` | MJCF 输出目录 |
| `ELECTRONBOT_DENSITY` | `1.24e-6` | 覆盖 PLA_DENSITY（用于材质切换实验） |
| `FREECAD_PATH` | — | FreeCAD 可执行路径，用于自动化 STL 导出（可选） |

---

## 11. 日志与可观测性

### 11.1 日志规范

日志级别：`DEBUG` / `INFO` / `WARNING` / `ERROR`，默认 `INFO`。
格式：`[%(asctime)s][%(levelname)s][%(name)s] %(message)s`

关键事件：

| 阶段 | 级别 | 事件 | 字段 |
|------|------|------|------|
| STL 加载 | INFO | `mesh loaded` | 文件名、顶点数、面数、体积(mm³)、质量(g) |
| STL 加载 | WARNING | `degenerate mesh` | 文件名、体积=0、原因 |
| 惯性计算 | INFO | `inertia computed` | name、mass(g)、com(mm)、主惯性矩 |
| Inline mesh 生成 | INFO | `inline mesh encoded` | name、vertex_b64_len、face_b64_len |
| XML 验证 | INFO | `validate pass` | nbody、njoint、nu、总质量(g) |
| XML 验证 | ERROR | `validate fail` | 错误项列表 |
| 仿真 smoke test | INFO | `smoke test ok` | 100 步耗时(ms) |
| 性能基准 | INFO | `fps benchmark` | 版本（mesh）、fps |

示例日志：
```
[2026-07-04T10:23:11][INFO][calc_inertia] mesh loaded file=torso_center.stl vertices=1240 faces=2480 volume=16545.2 mass=20.5
[2026-07-04T10:23:11][INFO][calc_inertia] inertia computed name=torso_center mass=20.5 com=[0.1, 0.0, 30.2] principal=[0.12, 0.15, 0.20]
[2026-07-04T10:23:12][INFO][validate_model] validate pass nbody=7 njoint=6 nu=7 total_mass=160.3
[2026-07-04T10:23:12][INFO][validate_model] smoke test ok steps=100 elapsed_ms=12.4
[2026-07-04T10:23:18][INFO][benchmark] fps benchmark version=mesh fps=487
```

### 11.2 关键指标

| 指标名 | 类型 | 采集方式 | 告警阈值 |
|--------|------|----------|----------|
| `fps_primitive` | gauge | benchmark 脚本 | < 1500 告警 |
| `fps_mesh` | gauge | benchmark 脚本 | < 300 告警 |
| `total_mass_g` | gauge | validate_model 输出 | < 140 或 > 180 告警 |
| `mesh_file_size_kb` | gauge | 文件系统 stat | > 600 告警 |
| `validate_pass` | bool(0/1) | validate_model 返回值 | =0 告警 |
| `smoke_test_crash` | bool(0/1) | validate_model 异常捕获 | =1 告警 |
| `stl_load_failures` | counter | calc_inertia 批量统计 | > 3 告警 |

---

## 12. 风险评估

### 12.1 技术风险

| 风险项 | 概率 | 影响 | 缓解措施 |
|--------|:---:|:---:|----------|
| STL 简化过度导致碰撞检测不准确 | 中 | 中 | 保留关键外形特征，凸包 + cylinder 组合；validate_model 检查面数 < 200 |
| 惯性参数与真机偏差（PLA 密度变化、打印填充率） | 高 | 中 | 暴露 `ELECTRONBOT_DENSITY` 环境变量；RL 训练用域随机化 ±10% 密度 |
| inline mesh 文件过大（~2MB）影响加载速度 | 中 | 低 | 单文件 ~2MB 可接受；如需优化可改用外部 .stl 引用 |
| mm 单位惯性放大导致关节卡死 | 高（已知） | 高 | 不缩放 STL，控制器参数适配（见 §3 关键教训） |
| 碰撞几何体非凸导致 MuJoCo 报错 | 中 | 高 | validate_model 强制检查 group=3 不为 mesh 类型 |
| 非流形 STL 导致 trimesh 计算失败 | 低 | 中 | 批量流程捕获异常并跳过，用占位惯性 |
| MuJoCo 版本升级导致 API 不兼容 | 低 | 中 | 锁定 mujoco==3.x；CI 跑 validate_model |

### 12.2 依赖风险

| 外部依赖 | 版本 | 风险 | 应对 |
|----------|------|------|------|
| `trimesh` | >=4.0 | STL 解析行为变化 | 单元测试覆盖 24 个标准 STL |
| `mujoco` (Python bindings) | 3.x | API 升级破坏 `MjModel.from_xml_path` | 锁版本，CI 验证 |
| FreeCAD | 0.21+ | 宏 API 变化影响 STL 导出 | 手动导出为主，自动化导出为辅 |
| `numpy` | >=1.24 | dtype 默认变化（float64 vs float32） | 显式 `.astype('<f4')` |
| ElectronBot.step 原始 CAD | 30.5MB | 上游 CAD 修订导致零件数变化 | 重新导出 STL 并跑 calc_inertia |

---

## 13. 变更记录

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|----------|------|
| v1.0 | 2026-07-03 | 初始版本 | — |
| v1.1 | 2026-07-04 | 补充软件工程规范章节 | 架构师 |
| v1.2 | 2026-07-04 | 移除几何基元版，统一使用 inline mesh 版；修复 actuator 排序匹配 DOF 顺序 | — |
