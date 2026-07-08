# 开发笔记：修复渲染灰色方块 + 暗画面问题

**日期**: 2026-07-07  
**作者**: AI 辅助诊断  
**标签**: 渲染, bugfix, MuJoCo, clipping

---

## 背景

用户报告 ElectronBot 模拟渲染显示一个巨大的灰色立方体占满画面（如图 `output.2e6b00cfdb.gif`），看不到机器人、桌子或其他物体。用 `env.py` 默认 camera 参数渲染时 gray=99%, unique_colors=246（几乎全是灰色）。

## 根因分析

通过编写自动化诊断脚本，测试了 8 种 camera 距离（4cm ~ 50cm）和 3 种场景配置。发现：

### 根因 1: znear 过小导致投影矩阵崩溃
- `scene_tabletop.xml` 视觉配置：`<map znear="0.0001" zfar="0.5"/>`
- `znear=0.0001m = 0.1mm` 极小值导致浮点精度问题
- 相机距离超过 **约 5cm** 时，OpenGL 投影矩阵退化，所有像素输出纯黑
- 4cm 是唯一能渲染的距离，但视角太近，只能看到桌面材质的一个灰色部分

### 根因 2: headlight 被清零
- `scene_tabletop.xml` `<headlight ambient="0 0 0" diffuse="0 0 0" specular="0 0 0"/>`
- 当头灯全为零时，场景仅靠 3 盏定向光源照明，亮度不足
- 只有 4cm 极近距离下光照明亮够用

### 对比数据

| 配置 | 距离 | gray% | std | dark% | uniq |
|------|------|-------|-----|-------|------|
| 修复前 | 4cm | 99% | 62 | 41% | 246 |
| 修复前 | 25cm | 100% | 0 | 100% | 1 |
| 修复后 | 25cm | 11% | 57 | 9% | 2823 |

## 修复方案

### 1. `assets/mjcf/scene_tabletop.xml`

**Before**:
```xml
<visual>
    <headlight ambient="0 0 0" diffuse="0 0 0" specular="0 0 0"/>
    <map znear="0.0001" zfar="0.5"/>
</visual>
```

**After**:
```xml
<visual>
    <headlight ambient="0.3 0.3 0.35" diffuse="0.6 0.6 0.65" specular="0.1 0.1 0.1"/>
    <map znear="0.001" zfar="100.0"/>
</visual>
```

### 2. `src/electronbot_sim/env.py` `_render_rgb`

**Before**:
```python
cam.lookat[:] = [0, 0, 0.055]
cam.distance = 0.04      # 4cm — 太近！
cam.azimuth = 180
cam.elevation = -35
```

**After**:
```python
cam.lookat[:] = [0, 0, 0.04]   # 机器人中心
cam.distance = 0.25            # 25cm — 可看到完整桌面
cam.azimuth = 145              # 四分之三角
cam.elevation = -25            # 俯视
```

## 影响范围

- `scene_tabletop.xml`: 所有使用该场景的渲染都会受益
- `env.py _render_rgb`: `render_mode="rgb_array"` 时的渲染输出
- 之前的"极近距离 + 极弱头灯"认为是正确配置的假设被推翻

## 经验总结

1. MuJoCo 的 `znear` 不应设太小，0.001m (1mm) 已经足够近，0.0001m 会导致精度问题
2. 自动化像素分析（gray_ratio, dark_ratio, std, unique_colors）是验证渲染质量的有效手段
3. 当渲染出现大面积纯色时，先检查 clipping planes（znear/zfar）而非光照
4. OSMesa 在此环境输出纯灰度，EGL 正常 — 这是一个已知的渲染后端问题

## 截图

- 修复前: `/tmp/electronbot_compare/01_before_4cm.png`
- 修复后: `/tmp/electronbot_compare/03_after_25cm_FIXED.png`
- 对比 GIF: `/tmp/electronbot_compare/before_after_compare.gif`
- 演示 GIF: `/tmp/electronbot_fixed_gif/electronbot_fixed_demo.gif`
