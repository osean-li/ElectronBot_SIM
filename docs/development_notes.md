# 开发笔记

> 开发过程中遇到的问题和解决方案

---

## 1. `mujoco.viewer` vs `test_env.py` 的关系

| | `mujoco.viewer` | `test_env.py --test all` |
|---|---|---|
| 做什么 | 打开 3D 窗口，拖滑块手动控制关节 | 自动运行 7 项测试 |
| 用到什么 | **仅 XML 文件** | robot.py / env.py / servo_sim.py 全部代码 |
| 验证什么 | 模型几何、关节轴向 | 角度映射 / DCE PID / I2C / ExtraData / 固件循环 |

viewer 看"外壳"，test_env 测"内脏"。viewer 通过不代表代码对，test_env 失败不代表 XML 错。

---

## 2. scene.xml / electronbot.xml / electronbot_mesh.xml 关系

```
assets/
├── scene.xml           → 场景外壳 (地面/光照) include electronbot.xml
│   └── electronbot.xml   → 本体: 圆柱/方块几何基元 (训练用, ~2000 fps)
│
├── scene_mesh.xml      → 场景外壳 include electronbot_mesh.xml  
│   └── electronbot_mesh.xml → 本体: CAD STL 网格 (展示用, ~500fps)
```

三个都需要保留。`scene_mesh.xml` 由 `sed 's/electronbot.xml/electronbot_mesh.xml/' scene.xml > scene_mesh.xml` 生成。

---

## 3. 无头服务器 (SSH/无 X11) 渲染

```bash
# mujoco.viewer 需要图形窗口, SSH 环境不可用
# 解决方案: EGL 无头渲染到 PNG
MUJOCO_GL=egl python3 -c "
import mujoco, cv2
m = mujoco.MjModel.from_xml_path('assets/scene.xml')
d = mujoco.MjData(m); r = mujoco.Renderer(m, 480, 480)
mujoco.mj_forward(m, d); r.update_scene(d)
cv2.imwrite('snapshot.png', cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
"
```

---

## 4. JOINT_PARAMS 与 MuJoCo joint 顺序映射

两者顺序不同，导致 DCE 控制时需转换：

```
JOINT_PARAMS (固件): [head, l_roll, l_pitch, r_roll, r_pitch, body]
MuJoCo joints:       [body, head, l_pitch, l_roll, r_pitch, r_roll]

映射: firmware_idx → mujoco_idx = [1, 3, 2, 5, 4, 0]
```

在 `robot.py` 中通过 `_fw_to_mujoco` 数组处理，`env.py` 中通过 `M2F` 数组处理。

**踩坑**: 初始代码用 `p[3]` 取 model_min，但 `p[3]` 是 mech_max。索引修正为 `p[4]`/`p[5]`。

---

## 5. DCE 扭矩符号约定

DCE 公式: `error = current_mech - setpoint_mech`, `output = Kp × error`

- **non-inverted** (body, l_pitch, l_roll): 模型↑=机械↑ → MuJoCo motor 方向 = **−DCE**
- **inverted** (head, r_pitch, r_roll): 模型↑=机械↓ → MuJoCo motor 方向 = **+DCE**

在 `servo_control_step` 中按 `j.inverted` 翻转符号。

**踩坑**: DCE 工作在机械角度空间，但 MuJoCo qpos 存的是模型角。需要在 `servo_control_step` 中先 `model_angle_to_mech()` 转换再给舵机。

---

## 6. 固件模式 vs 基础模式

| | 基础模式 | 固件模式 |
|---|---|---|
| 控制方式 | MuJoCo position actuator | 200Hz DCE PID |
| 用途 | 快速原型 | Sim2Real 前训练 |

**踩坑**: 初始 MJCF 中 position actuator kp=150 无 forcerange，与 DCE motor actuator 互相打架（position 75Nm vs DCE 0.15Nm）。修复：position kp→5, forcerange→±0.01。

---

## 7. qfrc_applied 残留 Bug

测试 7（抗扰动）施加 `qfrc_applied = -0.5Nm` 后没有清零，导致后续测试（test 1/2）也受影响。

修复：
- `robot.py:reset()` 中加 `self.data.qfrc_applied[:] = 0`
- `test_env.py` 中扰动后立即清零

---

## 8. CAD mesh 加载 (完整解决过程)

### 历程

| 尝试 | 结果 |
|------|------|
| STL 文件 + meshdir 相对路径 | ❌ `mesh 'base_link.stl' not found` |
| 绝对路径 meshdir | ❌ 同上 |
| 改用 OBJ 格式 | ❌ 同上 |
| trimesh 生成简单立方体 | ❌ 也不行 |
| `libassimp-dev` 软链接到 mujoco 目录 | ❌ 不生效 |
| `LD_LIBRARY_PATH` 设置 | ❌ 不生效 |
| 升级 mujoco 3.2.7 | ❌ 同样错误 |
| 降级 mujoco 2.3.7 | ❌ 同样错误 |
| 源码编译 mujoco | ✅ 编译成功但 `Illegal instruction` 崩溃 (CPU 不兼容) |
| MSH (MuJoCo native) 格式 | ❌ header 错误 |
| VFS 资源注册 | ❌ 无 MjVFS API (2.x 没有) |
| **`from_xml_string(assets=)`** | ❌ assets 仍走 filesystem |
| **`<mesh vertex="..." face="..."/>` inline 数据** | ✅ **成功！** |

### 最终方案: Inline mesh (推荐)

`scripts/generate_inline_mesh.py` 从 STL 读取顶点/面数据, 编码为字符串嵌入 MJCF:

```xml
<asset>
  <mesh name="base_link" 
        vertex="0.123 0.456 0.789 ..." 
        face="0 1 2 1 3 2 ..."/>
</asset>
```

**优势**:
- 不依赖文件系统
- 0 个外部文件
- 1.95MB 单文件可移植

**限制**:
- 几何基元 (圆柱/方块) 训练速度更快 (~2000 fps)
- Inline mesh 训练速度慢 (~500 fps), 仅适合展示/Demo

### 命令对照

```bash
# 训练用 (几何基元, 快速)
MUJOCO_GL=egl python -m mujoco.viewer --mjcf=simulation/electronbot_mujoco/electronbot_mujoco/assets/scene.xml

# 展示用 (CAD 真实外形, 美观)
MUJOCO_GL=egl python -m mujoco.viewer --mjcf=simulation/electronbot_mujoco/electronbot_mujoco/assets/electronbot_inline.xml
```

---

## 9. setup_env.sh pyyaml 冲突

pip upgrade 时系统的 ROS2 `launch-ros` 被 Python 发现，报告 `pyyaml required but not installed`。

修复：在 upgrade pip 之前先 `pip install pyyaml -q`，把依赖装进 venv 避免冲突。

---

## 10. Emoji 情绪动画 MP4 用法

`4.CAD-Model/Emoji/` 中 6 种情绪视频（不屑/愤怒/惊恐/难过/兴奋/静态）可用于 RL 情绪策略：

```
emoji_pose_extractor.py  ← 手工标注关键帧姿势
         ↓
  emoji_poses.json       ← [body,head,l_pitch,l_roll,r_pitch,r_roll] (°)
         ↓
emotional_reward.py      ← 新加 5 种 imitate 模式
```

训练：`python train_ppo.py --task wave --emotion excited` ↔ 模仿兴奋挥手动作

---

## 11. 角度映射入口位置

| 函数 | 作用 | 对应固件源码 |
|------|------|-------------|
| `model_angle_to_mech(idx, deg)` | 模型角→机械角 | `robot.cpp:127-152` |
| `mech_angle_to_model(idx, deg)` | 机械角→模型角 | `robot.cpp:225-261` |

ExtraData 使用**模型角度**编解码 (`protocol.py`)，不是机械角。

---

## 12. 调试命令速查

```bash
# 角度映射
python simulation/electronbot_mujoco/scripts/test_env.py --test mapping

# I2C 协议
python simulation/electronbot_mujoco/scripts/test_env.py --test i2c

# ExtraData 编解码
python simulation/electronbot_mujoco/scripts/test_env.py --test protoco

# 舵机 DCE 独立测试
python simulation/electronbot_mujoco/scripts/test_env.py --test servo

# 固件模式 + MuJoCo 动力学
python simulation/electronbot_mujoco/scripts/test_env.py --test firmware

# MJCF 结构检查
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('assets/scene.xml'); print(m.njnt,'joints,',m.nu,'actuators')"

# CAD → STL 导出
python simulation/electronbot_mujoco/scripts/export_cad_meshes.py
```

---

## 13. 待办

- [ ] Test 2 (固件模式) DCE 跟踪精度调优 — System ID 校准 PID/damping
- [ ] Test 7 (抗扰动) DCE 恢复能力
- [ ] `sudo apt install libassimp-dev` → 启用 CAD mesh 渲染
- [ ] MJCF 几何比例微调 (head 略大)
- [ ] ROS2 bridge 实测
- [ ] CAD SHOULDER 零件聚类优化 (left_shoulder/right_shoulder 未能从 step 中分离)
