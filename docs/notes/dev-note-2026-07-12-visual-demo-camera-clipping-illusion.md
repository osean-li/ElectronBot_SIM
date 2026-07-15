# dev-note-2026-07-12-visual-demo-camera-clipping-illusion

> **日期**: 2026-07-12
> **标签**: `mujoco` `渲染` `视觉错觉` `穿模` `camera` `调试流程`

## 问题

`python3 src/electronbot_sim/visual_demo.py` 运行时，机器人腹部出现"穿模"（手臂几何体似乎穿透身体外壳），但
`python3 demos/01-CAD-to-MJCF_Demo/01_manual_control.py` 显示同一模型却无此问题。

## 排查过程

### 1. 对比状态
编写诊断脚本比较两种初始化方式（manual_control 不设 ctrl vs visual_demo 设 ctrl=0）：
```
qpos 完全一致 → max diff = 0.0
ctrl 完全一致 → max diff = 0.0
```

### 2. 对比渲染
使用 `MUJOCO_GL=osmesa` 无头渲染两边的首帧像素：
```
Diff: max=0.0 mean=0.0000 ✅ 像素级一致
```

### 3. 多角度排查
用 `azimuth=[135, -45, 45, 90, 180, 225, 270, 315]` 渲染 8 张图：
- **azimuth=135（manual_control 默认）**：相机在机器人**背面**，从背后看手臂贴在身体曲线后方，产生穿入腹部的**视觉错觉**
- **azimuth=90**：正面视角，左臂、右臂、胸前按钮、头顶圆环**完全独立**，毫无穿模

### 4. 根源
`manual_control.py` 和 `visual_demo.py` 初始代码和渲染结果**完全相同**。用户认为 manual_control "没问题"是因为在 GUI 中**手动旋转了视角**到正面，而 visual_demo 的程序化相机停在背面默认值 `azimuth=135`。

## 修复

```python
# visual_demo.py
viewer.cam.azimuth = 90   # 正面视角
```

## 关键教训

1. **"穿模"不一定是几何/物理问题**——从某些角度观察机械模型时，关节附近的多 mesh 重叠会自然产生视觉错觉
2. **先量化再比较**——用 `osmesa` 无头渲染 + `np.abs(diff).max()` 验证客观一致性比人工看图可靠得多
3. **永远先确认相机参数**——当用户说"A脚本正常、B脚本异常"但模型相同时，第一个排查项就应该是相机角度
4. **SSH 远程调试流水线**：
   ```
   本地改代码 → scp → Linux → MUJOCO_GL=osmesa → 渲染 PNG → scp 回本地对比
   ```

## 涉及文件

- `src/electronbot_sim/visual_demo.py`：改用 raw MuJoCo（不含 gymnasium），azimuth=90
- `src/electronbot_sim/env.py`：修复 model_file 默认值 + 关节/执行器命名映射
- `scripts/build_electronbot_xml.py`（新增）：输出推荐相机参数
