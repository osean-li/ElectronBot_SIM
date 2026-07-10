# dev-note: 修复 ElectronBot MJCF 模型 + 举手姿态正确显示

## 背景

启动 `python -m electronbot_sim.mcp_server` 后，机器人默认状态和举手状态显示都有问题：
1. 默认状态下手在身体内部（看不到）
2. hand_action 举手后，只有左臂举起来，右臂没动

## 根本原因

### 1. mesh 装配错位

原 `assets/mjcf/electronbot.xml` 里的 `body` / `head` / `arm` 位置是**视觉估算**的硬编码值（来自 build script 注释 "geom offset 将 mesh 向内/向下移动, 使手臂从身体两侧自然伸出"），但这些值跟实际 STL mesh 尺寸不匹配，导致：
- arm mesh 跟 body mesh 重叠，手藏在身体内部
- 头部稍微偏离身体中心

**修复**：重写 `scripts/rebuild_electronbot_xml.py`，按 STL mesh 的**包围盒尺寸**和**关节中心位置**自动计算 body 位置：

| body | 关键位置（米） | 说明 |
|------|---------------|------|
| base_link | (0, 0, 0.062) | base 底面接触地面 (z=0)，base 顶面 z=0.062 |
| body | (0, 0, 0) | waist joint 在 base 顶面 |
| head | (0, 0, 0.044) | neck joint 在 body 顶面 |
| left_arm | (-0.017, 0, 0.044) | shoulder 在 body 顶面左侧 |
| right_arm | (0.017, 0, 0.044) | shoulder 在 body 顶面右侧 |
| left_hand | (-0.030, 0, 0) | 局部 x 方向 30mm |
| right_hand | (0.030, 0, 0) | 局部 x 方向 30mm |

### 2. arm mesh 的 left/right 命名是反的

`left_arm.stl` 实际几何范围是 `x=[0, 34.9]`（向 +x 延伸），适合用作**右臂 mesh**。
`right_arm.stl` 实际几何范围是 `x=[-34.9, 0]`（向 -x 延伸），适合用作**左臂 mesh**。

这是因为原始 STL 是从 FreeCAD 按"臂延伸方向"导出的，跟"身体在哪一侧"无关。

**修复**：在 `rebuild_electronbot_xml.py` 里 mesh 互换：
```python
MESH_FILES = {
    "left_arm": "right_arm",   # 左臂 body 用 right_arm.stl (x 向 -x 延伸)
    "right_arm": "left_arm",   # 右臂 body 用 left_arm.stl (x 向 +x 延伸)
}
```

### 3. 关节 origin 必须对齐到 mesh 的肩部

每个 mesh 的 STL 坐标原点在 FreeCAD 里是"模型中心"，跟关节位置不一致。需要**平移 mesh 让关节中心落在 mesh 局部原点**：

```python
JOINT_OFFSETS_MM = {
    "base_link": np.array([0.0, 0.0, 15.0]),    # 底座顶部 (waist Z 轴)
    "body":      np.array([0.0, 0.0, -4.0]),    # 身体底部 (waist Z 轴)
    "head":      np.array([0.0, 0.0, 17.0]),    # 头部底部 (neck Y 轴)
    "left_arm":  np.array([0.0, 0.0, 17.8]),   # 肩部 (pitch Y 轴)
    "right_arm": np.array([0.0, 0.0, 17.8]),   # 肩部 (pitch Y 轴)
}
```

### 4. 右手 pitch 关节 axis 反了

MJCF 里所有 pitch joint 都用 `axis="0 1 0"` (Y 轴正方向)。但**左手 hand 在 -x 方向，绕 +Y 转 90° 让 hand 从 -x 转到 -z（向上）；右手 hand 在 +x 方向，绕 +Y 转 90° 让 hand 从 +x 转到 +z（向下）**。

这导致 RP=+90 让**右臂向下**（而不是向上），跟"举手"语义相反。

**修复**：把 right_pitch_joint 的 axis 改成 `axis="0 -1 0"`，让 RP=+90 让右臂**向上**。这样跟左臂视觉对称：
- LP=+90: 左臂向上 ✓
- RP=+90: 右臂向上 ✓

## 验证

### Home 姿态
- 机器人站立，头在顶部
- 双手在身体两侧略微下垂（roll=-45° 让前臂向前下方）
- 底座在地面

### 举手姿态 (hand_action action=1, hand=3)
- 双手举到头部两侧上方 ✓
- 完整 MCP 协议工作正常：
  ```json
  {"status": "ok", "action": 1, "hand": 3, "times": 6}
  ```

### 放手姿态 (hand_action action=2, hand=3)
- 双手垂到身体两侧下方 ✓

## 修改文件

- `assets/mjcf/electronbot.xml` — 重新生成
- `scripts/rebuild_electronbot_xml.py` — 新增生成脚本

## 注意事项

1. **没有改 env.py**。舵机/关节转换常量 `SERVO_CENTER/RATIO/DIRECTION/HOME` 保持原值（这些是固件规格）。
2. **没有改 mcp_bridge.py**。hand_action 实现保持原值（用 `servo=0 (RP) / 180 (LP)` 作为"上举"目标，配合修复后的 MJCF 模型后视觉上正确）。
3. **mm → m 缩放在 build script 里完成** (×0.001)，MJCF 全部用米制。
