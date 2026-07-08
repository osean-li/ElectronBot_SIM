# ElectronBot_SIM 项目长期记忆

## 用户偏好
- **命令自动执行**: 所有命令自动执行不弹确认框，调用 `execute_command` 时强制 `requires_approval=False`
- **系统环境**: Ubuntu 22.04，sudo 密码为 `1`，需要时使用 `echo 1 | sudo -S` 自动提权

## ElectronBot 渲染关键经验（2026-07-07 更新）

### 渲染后端
- **必须用 `MUJOCO_GL=egl`**，OSMesa 在此环境输出纯灰度(R==G==B)
- **模型单位是米(m)**，机器人约 70mm 高 24mm 宽 (使用 m 为单位的 mm 级数值)
- **FreeCAD STL mesh 坐标错误**(真实米级), 已用 box/cylinder/capsule 重写 electronbot.xml
- **scene_tabletop.xml 桌面 z=0**（不能是45.5否则机器人陷在桌下）
- MJCF 命名: joint_body/head/lp/lr/rr/joint_rr_pitch, act_rp/rr/lp/lr/body/head

### 渲染 Bug 修复（2026-07-07）
**问题**: 渲染显示一个灰色大立方体占满画面  
**根因**: `scene_tabletop.xml` `<map znear="0.0001" zfar="0.5"/>` 中 znear=0.1mm 导致浮点精度崩溃，相机超过 5cm 距离就全黑。同时 headlight 被设为全零。  
**修复**:
1. `scene_tabletop.xml`: `znear=0.001` `zfar=100.0`, headlight `ambient="0.3 0.3 0.35" diffuse="0.6 0.6 0.65"`
2. `env.py` `_render_rgb`: `cam.distance=0.25`, `azimuth=145`, `elevation=-25`, `lookat=[0,0,0.04]`

### 像素分析指标
| 配置 | gray% | std | dark% | uniq | 结论 |
|------|-------|-----|-------|------|------|
| 修复前 4cm | 99% | 62 | 41% | 246 | 灰色方块 |
| 修复前 25cm | 100% | 0 | 100% | 1 | 纯黑 |
| 修复后 25cm | 11% | 57 | 9% | 2823 | ✅ 正常 |

## 开发笔记自动记录规则 🔴 必须遵守
- **触发条件**：每次解决一个技术问题、修复一个 Bug、或完成一个功能模块后，**必须自动**在 `docs/notes/` 下创建开发笔记
- **文件命名**：`dev-note-YYYY-MM-DD-简短描述.md`（如 `dev-note-2026-07-07-fix-arm-explosion.md`）
- **格式模板**：参考 `docs/notes/TEMPLATE_开发笔记.md`，包含：背景、根因、方案、影响范围、经验总结
- **截图**: 若涉及可视化效果变化（如渲染、界面、模型外观），使用 MuJoCo EGL 渲染保存前后对比截图到 `docs/notes/screenshots/`
- **截图命令**：
  ```python
  import os; os.environ.setdefault("MUJOCO_GL", "egl")
  import mujoco
  model = mujoco.MjModel.from_xml_path('assets/mjcf/electronbot_scene.xml')
  renderer = mujoco.Renderer(model, 720, 960)
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  renderer.update_scene(data)
  import imageio; imageio.imwrite('docs/notes/screenshots/xxxx.png', renderer.render())
  ```
- 已有笔记：`dev-notes-freecad-mujoco-arm.md`, `场景切换指南.md`
