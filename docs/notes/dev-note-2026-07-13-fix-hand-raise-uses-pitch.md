# dev-note-2026-07-13-fix-hand-raise-uses-pitch

> **日期**: 2026-07-13
> **标签**: `mcp` `动作系统` `hand_action` `举手` `运动学` `FK`

## 问题

用户反馈 `hand_action` 的「举手」(action=1) 动作不对，附图指出正确的举手姿态应是手臂明显向上抬起。

## 根因

旧实现把「举手」定义为 **roll 关节 → 0**（RR/LR，索引 1/3）。
但本模型的手臂 mesh 沿 **±X 横向**伸出，手 body 在 +Y（前方）：

- **roll 关节 = 绕 X 轴**（手臂自身长轴）→ 只让手臂"自转"，几乎不改变手臂高度。
- **pitch 关节 = 绕 Y 轴** → 才能把横向手臂抬起/放下。

FK 实测 (`scripts/fk_probe_raise.py`，追踪 `left/right_arm_geom` 世界 Z，base≈47m)：

| 姿态 | 右臂 geom 相对 base 高度 |
|------|--------------------------|
| HOME (roll=-45) | −7.0 m |
| 旧举手 (roll→0) | −4.1 m（仅抬 ~3m，几乎看不出） |
| **pitch=+90** | **+16.6 m（明显举起）** |
| pitch=−90 | −16.4 m（放下） |

结论：抬臂必须用 pitch，旧 roll 方案抬升量微乎其微。

## 修复

`src/electronbot_sim/mcp_bridge.py` `_hand_action` 改为 pitch 驱动：

- **举手 (1)**：右臂 `RP=+raise_mag`、左臂 `LP=-raise_mag`（镜像），roll 回正 0。
  `raise_mag = min(90, amount*3)`，`amount` 默认 30 → 满幅 90°。
- **放手 (2)**：回 home（pitch=0, roll=-45）。
- **挥手 (3)**：先抬到高位，再 pitch 小幅上下摆动。
- **拍打 (4)**：高位与低位间快速上下拍动。

`hand` 参数（1左/2右/3双）逻辑保留，抽出 `_set_arm()` 辅助函数统一写入。

## 验证

- FK 确认 pitch=+90 双臂升至 base 上方 16m。
- 渲染差分（home→举手，camera_distance=80）：9.0 万像素变化，跨 row 34~450，上半部占多数 → 大幅可见的上抬运动。
- GIF：`MUJOCO_GL=egl PYTHONPATH=src python3 scripts/gen_raise_gif.py --action 1 --hand 3 --out logs/RAISE_fixed.gif`

## 涉及文件

- `src/electronbot_sim/mcp_bridge.py`：`_hand_action` 重写为 pitch 驱动
- `scripts/fk_probe_raise.py`（新增）：FK 探针，定量判定抬臂关节
- `scripts/gen_raise_gif.py`（新增）：直接驱动 bridge 生成动作 GIF，快速迭代

## 关键教训

- 判断"哪个关节做哪个动作"时，先用 FK 追踪 **可见 mesh geom 的世界坐标**，不要凭直觉。手 body 在 +Y 方向，用它判断抬臂会失效（pitch 绕 Y 不动 Y 向点）。
- 本项目模型为 mm-当-m 的巨型未缩放尺度（extent≈177m），但抬臂运动学结论不受尺度影响。
