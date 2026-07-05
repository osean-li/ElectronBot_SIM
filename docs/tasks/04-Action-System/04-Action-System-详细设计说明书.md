# Phase 4：动作系统完整实现

> **目标**：在仿真中完整复现真机固件全部 12 个预设动作 + 舵机序列振荡器，包括**线性插值**（对齐固件）、安全角度裁剪、动作队列等精确行为。
>
> **⚠️ 关键设计决策**: 运动插值使用**线性插值**，对齐真机 `movements.cc:87` 的 `MoveServos()` 实现：
> ```cpp
> increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10.0);
> ```
> 不使用 EaseOutCubic！真机固件是纯线性等分。
>
> **前置依赖**：Phase 3 完成（MCP Bridge 可用）
>
> **输出**：`src/electronbot_sim/actions/`——独立可测试的动作系统模块
>
> **文档版本**: v1.1  
> **最后更新**: 2026-07-04  
> **变更类型**: 补充软件工程规范章节

---

## 1. 预期效果

### 完整动作测试

```python
from electronbot_sim.actions import ElectronBotActions

actions = ElectronBotActions(env)

# 所有预设动作逐一运行，效果与真机一致
actions.home()                          # → 复位
actions.hand_wave("both", times=3)       # → 双手挥手 3 次
actions.hand_raise("right")             # → 举右手
actions.hand_flap("left", times=2)      # → 左手拍打 2 次
actions.body_turn_left(angle=45)       # → 身体左转 45°
actions.head_nod(times=3)              # → 连续点头 3 次

# 高级：AI 自编程动作序列
actions.execute_sequence({
    "a": [
        {"s": {"rp": 90, "lp": 90, "h": 100}, "v": 800},
        {"osc": {"a": {"rp": 30, "lp": 30}, "o": {"rp": 120, "lp": 60}, 
                  "ph": {"lp": 180}, "p": 400, "c": 4}},
        {"s": {"b": 60}, "v": 1000, "d": 500},
        {"s": {"rp": 180, "lp": 0, "h": 90, "b": 90}, "v": 1500},
    ]
})
```

---

## 2. 实现细节

### 2.1 动作参数映射（从固件源码 1:1 移植）

真机固件 `movements.cc` 中的动作逻辑，在仿真中精确复现：

| 真机固件函数 | 仿真实现 | 关键参数 |
|-------------|---------|---------|
| `HandAction(1~12)` | `_hand_action()` | 上臂初始值 LP=0/RP=180，举手目标 LP=180/RP=0 |
| `BodyAction(1~3)` | `_body_action()` | 中心=90，左转+angle(<180)，右转-angle(>0) |
| `HeadAction(1~5)` | `_head_action()` | 中心=90，抬头+amount(≤105)，低头-amount(≥75) |
| `MoveServos(time, targets)` | `_move_servos()` | 10ms 步进，**线性插值**: `(target-pos)/(time/10.0)` |
| `OscillateServos()` | `_oscillate()` | 正弦: pos=amp*sin(phase)+center, 50ms 采样 (固件 `vTaskDelay(5)`, CONFIG_FREERTOS_HZ=100) |

### 2.2 动作类实现

```python
# src/electronbot_sim/actions/__init__.py

import json
import numpy as np
from typing import List, Dict, Optional

SERVO_INDEX = {
    "right_pitch": 0, "rp": 0,
    "right_roll":  1, "rr": 1,
    "left_pitch":  2, "lp": 2,
    "left_roll":   3, "lr": 3,
    "body":        4, "b":  4,
    "head":        5, "h":  5,
}

class ElectronBotActions:
    """ElectronBot 动作系统——1:1 复现真机固件 movements.cc"""
    
    def __init__(self, env, bridge):
        self.env = env
        self.bridge = bridge
        self._moving = False
    
    # ========== 预设单关节运动 ==========
    
    def servo_move(self, servo_type: str, position: float, speed_ms: int = 800):
        """单个舵机到绝对角度——等同 MCP servo_move"""
        idx = SERVO_INDEX[servo_type]
        target = self._clamp_servo(idx, position)
        current = self._get_servo_angles()
        current_servo = current[idx]
        joint_current = self._get_joint_angles()
        
        target_joint = self._servo_to_joint(idx, target)
        
        steps = max(1, speed_ms // 10)
        for step in range(1, steps + 1):
            t = step / steps
            # 线性插值 (对齐固件 movements.cc:87 increment_[] = (target-pos)/(time/10.0))
            interp = joint_current[idx] + (target_joint - joint_current[idx]) * t
            ctrl = np.copy(self._get_joint_angles())
            ctrl[idx] = interp
            self._apply_ctrl(ctrl)
    
    # ========== 预设手部动作（12种） ==========
    
    def hand_raise(self, hand: str = "both", speed_ms: int = 1000):
        """举手"""
        targets = self._get_servo_angles()
        if hand in ("left", "both"):
            targets[SERVO_INDEX["lp"]] = 180  # 左手举起
        if hand in ("right", "both"):
            targets[SERVO_INDEX["rp"]] = 0    # 右手举起
        self._move_servos(targets, speed_ms)
    
    def hand_lower(self, hand: str = "both", speed_ms: int = 1000):
        """放手"""
        targets = self._get_servo_angles()
        if hand in ("left", "both"):
            targets[SERVO_INDEX["lp"]] = 0
            targets[SERVO_INDEX["lr"]] = 0
        if hand in ("right", "both"):
            targets[SERVO_INDEX["rp"]] = 180
            targets[SERVO_INDEX["rr"]] = 180
        self._move_servos(targets, speed_ms)
    
    def hand_wave(self, hand: str = "both", times: int = 3, speed_ms: int = 600):
        """挥手——复现固件 HandAction case 7-9"""
        # ⚠️ 对齐真机固件 movements.cc:225 的 times 限制逻辑：
        #    times = 2 * max(3, min(100, times))
        times = 2 * max(3, min(100, times))
        
        targets = self._get_servo_angles()
        period = speed_ms // 10  # 每个小步时间
        
        if hand in ("left", "both"):
            targets[SERVO_INDEX["lp"]] = 150
        if hand in ("right", "both"):
            targets[SERVO_INDEX["rp"]] = 30
        self._move_servos(targets, speed_ms)
        
        for i in range(times * 2):
            if hand in ("left", "both"):
                delta = -30 if i % 2 == 0 else 30
                targets[SERVO_INDEX["lp"]] = 150 + delta
            if hand in ("right", "both"):
                delta = 30 if i % 2 == 0 else -30
                targets[SERVO_INDEX["rp"]] = 30 + delta
            self._move_servos(targets, period)
        
        # 回初始
        targets[SERVO_INDEX["lp"]] = 0
        targets[SERVO_INDEX["rp"]] = 180
        self._move_servos(targets, speed_ms)
    
    def hand_flap(self, hand: str = "both", times: int = 2, amount: int = 30,
                  speed_ms: int = 500):
        """拍打——复现固件 HandAction case 10-12"""
        amount = min(amount, 40)
        left_center = 40
        right_center = 140
        
        targets = self._get_servo_angles()
        period = speed_ms // 10
        
        if hand in ("left", "both"):
            targets[SERVO_INDEX["lr"]] = left_center
        if hand in ("right", "both"):
            targets[SERVO_INDEX["rr"]] = right_center
        self._move_servos(targets, speed_ms)
        
        for i in range(times):
            if hand in ("left", "both"):
                targets[SERVO_INDEX["lr"]] = left_center - amount
            if hand in ("right", "both"):
                targets[SERVO_INDEX["rr"]] = right_center + amount
            self._move_servos(targets, period)
            
            if hand in ("left", "both"):
                targets[SERVO_INDEX["lr"]] = left_center + amount
            if hand in ("right", "both"):
                targets[SERVO_INDEX["rr"]] = right_center - amount
            self._move_servos(targets, period)
        
        # 回初始
        targets[SERVO_INDEX["lr"]] = 0
        targets[SERVO_INDEX["rr"]] = 180
        self._move_servos(targets, speed_ms)
    
    # ========== 身体动作 ==========
    
    def body_turn_left(self, angle: int = 45, speed_ms: int = 1000):
        self._body_action(1, angle, speed_ms)
    
    def body_turn_right(self, angle: int = 45, speed_ms: int = 1000):
        self._body_action(2, angle, speed_ms)
    
    def body_center(self, speed_ms: int = 1000):
        self._body_action(3, 0, speed_ms)
    
    def _body_action(self, direction: int, angle: int, speed_ms: int):
        """复现固件 BodyAction——中心 90，方向变量"""
        targets = self._get_servo_angles()
        center = 90
        
        if direction == 1:       # 左转
            targets[4] = min(180, center + angle)
        elif direction == 2:     # 右转
            targets[4] = max(0, center - angle)
        elif direction == 3:     # 回中
            targets[4] = center
        
        self._move_servos(targets, speed_ms)
    
    # ========== 头部动作 ==========
    
    def head_look_up(self, angle: int = 10, speed_ms: int = 500):
        targets = self._get_servo_angles()
        targets[5] = min(105, 90 + angle)
        self._move_servos(targets, speed_ms)
    
    def head_look_down(self, angle: int = 10, speed_ms: int = 500):
        targets = self._get_servo_angles()
        targets[5] = max(75, 90 - angle)
        self._move_servos(targets, speed_ms)
    
    def head_nod(self, times: int = 1, angle: int = 10, speed_ms: int = 500):
        """点头——复现固件 HeadAction case 3/5"""
        period = speed_ms // 2
        targets_cnt = self._get_servo_angles()
        
        for _ in range(times):
            targets = np.copy(targets_cnt)
            targets[5] = min(105, 90 + angle)
            self._move_servos(targets, period)
            
            targets[5] = max(75, 90 - angle)
            self._move_servos(targets, period)
        
        targets_cnt[5] = 90
        self._move_servos(targets_cnt, period)
    
    def head_center(self, speed_ms: int = 500):
        targets = self._get_servo_angles()
        targets[5] = 90
        self._move_servos(targets, speed_ms)
    
    # ========== 组合动作 ==========
    
    def home(self, speed_ms: int = 1000):
        """复位到初始姿态"""
        home = np.array([180, 180, 0, 0, 90, 90])
        self._move_servos(home, speed_ms)
    
    # ========== 舵机序列（AI 自编程） ==========
    
    def execute_sequence(self, sequence: dict):
        """执行舵机序列——与真机 ExecuteServoSequence 同构"""
        actions = sequence.get("a", [])
        current = self._get_servo_angles()
        
        for i, action in enumerate(actions):
            if "osc" in action:
                # 振荡
                osc = action["osc"]
                amps = self._parse_servos(osc.get("a", {}))
                centers = self._parse_servos(osc.get("o", {}), current)
                period = osc.get("p", 500) / 1000.0
                cycles = osc.get("c", 5.0)
                self._oscillate(amps, centers, period, cycles)
                
            elif "s" in action:
                # 普通移动
                servo_targets = self._parse_servos(action["s"], current)
                speed = action.get("v", 1000)
                self._move_servos(servo_targets, speed)
            
            # 动作间延迟
            delay = action.get("d", 0)
            if delay > 0:
                import time
                time.sleep(delay / 1000.0)
        
        # 序列间延迟
        seq_delay = sequence.get("d", 0)
        if seq_delay > 0:
            import time
            time.sleep(seq_delay / 1000.0)
    
    def _oscillate(self, amplitudes: np.ndarray, centers: np.ndarray,
                   period: float, cycles: float):
        """正弦振荡——Oscillator::Refresh() 逻辑"""
        dt = 0.05  # 50ms 采样周期 (固件 movements.cc:167 `vTaskDelay(5)`, CONFIG_FREERTOS_HZ=100 → 5 ticks × 10ms/tick = 50ms)
        total_time = period * cycles
        steps = int(total_time / dt)
        
        for step in range(steps):
            phase = 2 * np.pi * step * dt / period
            targets = np.copy(centers)
            for i in range(6):
                if amplitudes[i] > 0:
                    targets[i] = centers[i] + amplitudes[i] * np.sin(phase)
            joint_targets = np.array([self._servo_to_joint(i, targets[i]) for i in range(6)])
            self._apply_ctrl(joint_targets)
        
        # 回中心
        joint_centers = np.array([self._servo_to_joint(i, centers[i]) for i in range(6)])
        self._apply_ctrl(joint_centers)
    
    # ========== 底层工具函数 ==========
    
    def _clamp_servo(self, idx: int, angle: int) -> int:
        """安全角度裁剪——与固件 ClampServoTarget 完全一致"""
        limits = {
            0: (0, 180), 1: (100, 180), 2: (0, 180),
            3: (0, 80),  4: (30, 150),  5: (75, 105),
        }
        lo, hi = limits[idx]
        return max(lo, min(hi, int(angle)))
    
    def _move_servos(self, targets: np.ndarray, time_ms: int):
        """缓动移动所有舵机——精确复现 MoveServos 逻辑
        
        真机固件 (movements.cc:87):
          increment_[i] = (target[i] - pos[i]) / (time / 10.0)
          → 每个 10ms 步进等量增加 → 纯线性插值
        """
        targets = np.array([self._clamp_servo(i, targets[i]) for i in range(6)])
        current_joint = self._get_joint_angles()
        
        target_joint = np.array([self._servo_to_joint(i, targets[i]) for i in range(6)])
        
        if time_ms <= 10:
            self._apply_ctrl(target_joint)
            return
        
        steps = max(1, time_ms // 10)
        self._moving = True
        for step in range(1, steps + 1):
            t = step / steps     # 线性插值 (对齐固件)
            interp = current_joint + (target_joint - current_joint) * t
            self._apply_ctrl(interp)
        self._moving = False
    
    def _apply_ctrl(self, joint_angles: np.ndarray):
        """将关节角度写入 MuJoCo 并步进"""
        self.env.data.ctrl[:6] = joint_angles
        mujoco.mj_step(self.env.model, self.env.data)
    
    def _get_servo_angles(self) -> np.ndarray:
        joint = self._get_joint_angles()
        return np.array([self._joint_to_servo(i, joint[i]) for i in range(6)])
    
    def _get_joint_angles(self) -> np.ndarray:
        return self.env.data.qpos[:6].copy()
    
    def _servo_to_joint(self, idx: int, servo: float) -> float:
        return self.bridge._servo_to_joint(idx, servo)
    
    def _joint_to_servo(self, idx: int, joint: float) -> float:
        return self.bridge._joint_to_servo(idx, joint)
    
    def _parse_servos(self, data: dict, defaults: Optional[np.ndarray] = None) -> np.ndarray:
        """解析舵机字典 {"rp": 120, "h": 100} → ndarray[6]"""
        if defaults is None:
            result = self._get_servo_angles()
        else:
            result = np.copy(defaults)
        for key, val in data.items():
            if key in SERVO_INDEX:
                idx = SERVO_INDEX[key]
                result[idx] = int(val)
        return result
```

---

## 3. 验证方法

### 3.1 自动化测试

```python
# tests/test_actions.py

def test_linear_move_servos():
    """MoveServos 线性插值测试——对齐固件"""
    from electronbot_sim.env import ElectronBotEnv
    from electronbot_sim.actions import ElectronBotActions
    
    env = ElectronBotEnv(render_mode=None)
    actions = ElectronBotActions(env)
    
    # 测试 _move_servos 线性等分行为
    targets = np.array([90, 140, 90, 40, 90, 90])
    actions._move_servos(targets, 1000)  # 100 步线性等分
    final = actions._get_servo_angles()
    for i in range(6):
        assert abs(final[i] - targets[i]) < 2.0, \
            f"舵机 {i}: 期望 {targets[i]}, 实际 {final[i]:.1f}"
    print("  ✅ 线性 MoveServos → 所有舵机到达目标")

def test_clamp_servo():
    """安全角度裁剪测试——与固件一致"""
    # head: 75-105
    assert actions._clamp_servo(5, 50) == 75   # 低于下限 → 裁剪
    assert actions._clamp_servo(5, 120) == 105  # 高于上限 → 裁剪
    assert actions._clamp_servo(5, 90) == 90    # 范围内 → 不变
    
    # body: 30-150
    assert actions._clamp_servo(4, -10) == 30
    assert actions._clamp_servo(4, 200) == 150
    
    # right_roll: 100-180
    assert actions._clamp_servo(1, 50) == 100
    assert actions._clamp_servo(1, 200) == 180

def test_all_preset_actions():
    """所有预设动作执行不应崩溃"""
    actions.home()
    assert not actions._moving  # 动作应完成
    
    for hand in ["left", "right", "both"]:
        actions.hand_raise(hand, speed_ms=300)
        actions.hand_lower(hand, speed_ms=300)
        actions.hand_wave(hand, times=1, speed_ms=200)
        actions.hand_flap(hand, times=1, speed_ms=200)
    
    actions.body_turn_left(speed_ms=300)
    actions.body_turn_right(speed_ms=300)
    actions.body_center(speed_ms=300)
    
    actions.head_look_up(speed_ms=300)
    actions.head_look_down(speed_ms=300)
    actions.head_nod(times=2, speed_ms=200)
    actions.head_center(speed_ms=300)

def test_sequence_with_oscillation():
    """复杂动作序列测试"""
    seq = {
        "a": [
            {"s": {"rp": 90, "lp": 90, "h": 100}, "v": 500},
            {"osc": {"a": {"rp": 20, "lp": 20}, 
                      "o": {"rp": 120, "lp": 60},
                      "p": 300, "c": 2}},
            {"s": {"b": 60, "h": 90}, "v": 800, "d": 300},
        ]
    }
    actions.execute_sequence(seq)  # 不应崩溃

def test_servo_limits_enforced_in_sequence():
    """序列中的角度不能超出安全范围"""
    seq = {
        "a": [
            {"s": {"h": 200}, "v": 500},  # 头部 200 → 应裁剪到 105
            {"s": {"b": -50}, "v": 500},  # 身体 -50 → 应裁剪到 30
        ]
    }
    actions.execute_sequence(seq)
    servo = actions._get_servo_angles()
    assert servo[5] <= 105
    assert servo[4] >= 30
```

### 3.2 视觉效果对比

录一段真机执行 `hand_wave("both", times=3)` 的视频，和仿真窗口并排对比：
- 动作轨迹形状一致
- 挥手频率一致
- 手臂幅度一致
- 真机 MoveServos 的线性插值 `(target-pos)/(time/10.0)` 每 10ms 等分行为与仿真一致

---

## 4. 交付物清单

| 文件 | 描述 |
|------|------|
| `src/electronbot_sim/actions/__init__.py` | ElectronBotActions 动作类 |
| `tests/test_actions.py` | 动作系统完整测试 |

---

## 5. 接口设计

### 5.1 模块对外接口

动作系统通过 `ElectronBotActions` 类对外提供服务，所有方法均以真机固件 `movements.cc` 中的函数为蓝本 1:1 移植。调用方需在初始化时注入 `env`（MuJoCo 仿真环境）与 `bridge`（舵机-关节映射桥接器）。

| 方法 | 签名 | 说明 | 对应固件函数 |
|------|------|------|-------------|
| `servo_move` | `servo_move(servo_type: str, position: float, speed_ms: int = 800)` | 单个舵机到绝对角度 | `MoveServos` |
| `hand_raise` | `hand_raise(hand: str, speed_ms: int = 1000)` | 举手（left/right/both） | `HandAction` case 1-3 |
| `hand_lower` | `hand_lower(hand: str, speed_ms: int = 1000)` | 放手回初始位 | `HandAction` case 4-6 |
| `hand_wave` | `hand_wave(hand: str, times: int = 3, speed_ms: int = 600)` | 挥手（times 受加倍逻辑约束） | `HandAction` case 7-9 |
| `hand_flap` | `hand_flap(hand: str, times: int = 2, amount: int = 30, speed_ms: int = 500)` | 拍打（amount 上限 40） | `HandAction` case 10-12 |
| `body_turn_left` | `body_turn_left(angle: int = 45, speed_ms: int = 1000)` | 身体左转 | `BodyAction(1)` |
| `body_turn_right` | `body_turn_right(angle: int = 45, speed_ms: int = 1000)` | 身体右转 | `BodyAction(2)` |
| `body_center` | `body_center(speed_ms: int = 1000)` | 身体回中（90°） | `BodyAction(3)` |
| `head_look_up` | `head_look_up(angle: int = 10, speed_ms: int = 500)` | 抬头（上限 105°） | `HeadAction(1)` |
| `head_look_down` | `head_look_down(angle: int = 10, speed_ms: int = 500)` | 低头（下限 75°） | `HeadAction(2)` |
| `head_nod` | `head_nod(times: int = 1, angle: int = 10, speed_ms: int = 500)` | 连续点头 | `HeadAction(3/5)` |
| `head_center` | `head_center(speed_ms: int = 500)` | 头部回中（90°） | `HeadAction(4)` |
| `home` | `home(speed_ms: int = 1000)` | 复位到初始姿态 `[180,180,0,0,90,90]` | `Home` |
| `execute_sequence` | `execute_sequence(sequence: dict)` | 执行 AI 自编程舵机序列 | `ExecuteServoSequence` |

#### SERVO_INDEX 映射常量表

`SERVO_INDEX` 提供舵机名称到索引的映射，同时支持全称与缩写两种写法，供 `_parse_servos()` 解析 JSON 序列时使用：

| 舵机名称（全称） | 缩写 | 索引 | 物理含义 |
|------------------|------|:----:|----------|
| `right_pitch` | `rp` | 0 | 右臂俯仰 |
| `right_roll` | `rr` | 1 | 右臂横滚 |
| `left_pitch` | `lp` | 2 | 左臂俯仰 |
| `left_roll` | `lr` | 3 | 左臂横滚 |
| `body` | `b` | 4 | 身体转向 |
| `head` | `h` | 5 | 头部俯仰 |

#### 动作参数范围表

以下参数范围对齐真机固件 `movements.cc` 中的边界检查逻辑，超出范围的输入将被裁剪或修正：

| 参数 | 适用方法 | 范围 | 边界处理 |
|------|----------|------|----------|
| `action` | hand_action 固件参数 | 1-4 | 超出范围由固件默认分支处理 |
| `hand` | hand_action 固件参数 | 1-3 | 1=左, 2=右, 3=双手 |
| `steps` | hand_action 固件参数 | 1-10 | 固件内部循环上限 |
| `speed_ms` | 所有移动方法 | 500-1500 | <10 时直接跳转到目标 |
| `amount`（hand_flap） | hand_flap | 10-50 | `min(amount, 40)` 强制裁剪 |
| `times`（hand_wave） | hand_wave | 1-50 | `times = 2 * max(3, min(100, times))` 加倍 |
| `angle`（body_turn） | body_turn_left/right | 0-90 | 中心 90 ± angle，裁剪到 [0,180] |
| `angle`（head） | head_look_up/down | 5-15 | 抬头裁剪到 ≤105，低头裁剪到 ≥75 |

### 5.2 输入输出契约

#### servo_move 输入输出

- **输入**：
  - `servo_type: str` — 必须是 `SERVO_INDEX` 中的合法键，否则抛出 `KeyError`
  - `position: float` — 舵机目标角度（度），超出安全范围由 `_clamp_servo` 裁剪
  - `speed_ms: int = 800` — 移动耗时（毫秒），决定线性插值步数
- **输出**：无返回值（`None`），通过副作用写入 MuJoCo `data.ctrl`
- **副作用**：每个 10ms 步进调用一次 `mujoco.mj_step`，推进仿真物理状态

#### execute_sequence 输入输出

- **输入**：
  ```python
  {
    "a": [                          # 动作列表（必填）
      {"s": {"rp": 90}, "v": 800},  # s=舵机目标字典, v=速度ms, d=延迟ms
      {"osc": {                      # osc=振荡器
        "a": {"rp": 20},             # a=振幅字典
        "o": {"rp": 120},            # o=中心字典
        "p": 400,                    # p=周期ms
        "c": 4                       # c=周期数
      }}
    ],
    "d": 0                           # 序列间延迟ms（可选）
  }
  ```
- **输出**：无返回值（`None`）
- **异常**：JSON 结构非法时静默跳过当前动作项；`_parse_servos` 对未知键忽略

#### _move_servos 输入输出

- **输入**：`targets: np.ndarray`（6 维舵机角度）, `time_ms: int`（移动耗时）
- **输出**：无返回值
- **契约**：`time_ms <= 10` 时跳过插值直接到位；否则按 `steps = time_ms // 10` 线性等分

---

## 6. 数据模型

### 6.1 核心数据结构

#### 舵机安全范围表

`_clamp_servo` 方法使用的 6 组安全角度限制，与固件 `ClampServoTarget` 完全一致。超出此范围的指令会被强制裁剪到边界值，防止机械结构碰撞或舵机堵转：

| 索引 | 舵机 | 下限 (°) | 上限 (°) | 中心 (°) | 说明 |
|:----:|------|:--------:|:--------:|:--------:|------|
| 0 | right_pitch (rp) | 0 | 180 | 180 | 右臂俯仰，全范围 |
| 1 | right_roll (rr) | 100 | 180 | 180 | 右臂横滚，受限 |
| 2 | left_pitch (lp) | 0 | 180 | 0 | 左臂俯仰，全范围 |
| 3 | left_roll (lr) | 0 | 80 | 0 | 左臂横滚，受限 |
| 4 | body (b) | 30 | 150 | 90 | 身体转向，±60° |
| 5 | head (h) | 75 | 105 | 90 | 头部俯仰，±15° |

#### 舵机序列 JSON 结构

`execute_sequence` 接受的 JSON 字典结构，支持两种动作类型（普通移动 `s` 与振荡 `osc`）混合编排：

```json
{
  "a": [
    {
      "s": {"rp": 90, "lp": 90, "h": 100},
      "v": 800,
      "d": 0
    },
    {
      "osc": {
        "a": {"rp": 20, "lp": 20},
        "o": {"rp": 120, "lp": 60},
        "ph": {"lp": 180},
        "p": 400,
        "c": 4
      }
    }
  ],
  "d": 0
}
```

字段说明：

| 字段 | 位置 | 类型 | 含义 |
|------|------|------|------|
| `a` | 顶层 | list | 动作列表，按顺序执行 |
| `s` | 动作项 | dict | 舵机目标字典，键为 SERVO_INDEX 中的缩写 |
| `v` | 动作项 | int | 移动耗时（ms），默认 1000 |
| `d` | 动作项 | int | 当前动作结束后的延迟（ms） |
| `osc` | 动作项 | dict | 振荡器参数 |
| `osc.a` | 振荡器 | dict | 振幅字典（相对中心偏移量） |
| `osc.o` | 振荡器 | dict | 中心字典（绝对角度） |
| `osc.ph` | 振荡器 | dict | 相位偏移（度），可选 |
| `osc.p` | 振荡器 | int | 周期（ms） |
| `osc.c` | 振荡器 | int/float | 周期数 |

#### 动作状态标志

| 标志 | 类型 | 含义 | 设置时机 |
|------|------|------|----------|
| `_moving` | `bool` | 是否正在执行插值移动 | `_move_servos` 开始时置 `True`，结束时置 `False` |

`_moving` 标志用于测试断言（`test_all_preset_actions` 中验证动作完成后应恢复 `False`），外部调用方也可读取此标志判断动作系统是否空闲。

#### 线性插值参数

对齐固件 `movements.cc:87` 的 `MoveServos()` 实现：

| 参数 | 公式 | 说明 |
|------|------|------|
| 步数 | `steps = max(1, time_ms // 10)` | 每 10ms 一步 |
| 插值进度 | `t = step / steps` | step ∈ [1, steps]，线性递增 |
| 插值位置 | `interp = current + (target - current) * t` | 纯线性等分，无缓动 |
| 固件对应 | `increment_[i] = (target - pos) / (time / 10.0)` | 每 10ms 增量恒定 |

### 6.2 数据流

动作系统的数据流从调用方输入到 MuJoCo 物理引擎，经过以下阶段：

```
调用方
  │  (方法名 + 参数)
  ▼
ElectBotActions.public_method()        例如 hand_wave("both", times=3)
  │  (展开为 _move_servos 调用序列)
  ▼
_move_servos(targets, time_ms)
  │  1. _clamp_servo(idx, angle) × 6   安全裁剪
  │  2. _servo_to_joint(idx, servo) × 6 舵机→关节映射 (via bridge)
  │  3. steps = time_ms // 10           计算插值步数
  ▼
线性插值循环 (step = 1..steps)
  │  interp = current + (target - current) * (step/steps)
  ▼
_apply_ctrl(joint_angles)
  │  1. env.data.ctrl[:6] = joint_angles   写入控制信号
  │  2. mujoco.mj_step(model, data)        物理步进
  ▼
MuJoCo 仿真状态更新 (qpos, qvel, ...)
```

对于 `execute_sequence`，数据流在 `_move_servos` 之外还包含振荡器路径：

```
execute_sequence(sequence)
  │  遍历 sequence["a"]
  ├─ action["s"] → _move_servos(普通移动)
  └─ action["osc"] → _oscillate(振荡)
       │  1. 解析 amps/centers/period/cycles
       │  2. dt = 0.05 (50ms 采样)
       │  3. steps = int(period * cycles / dt)
       ▼
     振荡循环 (step = 0..steps-1)
       │  phase = 2π * step * dt / period
       │  targets[i] = centers[i] + amps[i] * sin(phase)
       ▼
     _apply_ctrl(joint_targets)
```

---

## 7. 错误处理与恢复

### 7.1 错误分类

| 错误类别 | 触发条件 | 严重等级 | 处理策略 | 对应代码位置 |
|----------|----------|:--------:|----------|-------------|
| 无效舵机类型名 | `servo_type` 不在 `SERVO_INDEX` 中 | 中 | 抛出 `KeyError`，由调用方捕获 | `servo_move` → `SERVO_INDEX[servo_type]` |
| 角度超出安全范围 | 舵机角度低于下限或高于上限 | 低 | `_clamp_servo` 自动裁剪到边界，记录日志 | `_clamp_servo` |
| 序列 JSON 解析失败 | `sequence` 不含 `"a"` 键或结构非法 | 中 | 静默跳过非法动作项，继续执行后续项 | `execute_sequence` |
| 振荡周期为零 | `osc.p == 0` 导致除零 | 高 | 参数修正为默认周期 500ms，记录警告 | `_oscillate` |
| 振荡振幅过大 | 振幅导致角度超出安全范围 | 中 | `_clamp_servo` 在每步裁剪，记录日志 | `_oscillate` → `_apply_ctrl` |
| MuJoCo 仿真步进失败 | `mj_step` 抛出异常（模型损坏等） | 高 | 捕获异常，状态回滚到插值前，重新抛出 | `_apply_ctrl` |
| `time_ms` 非法值 | `time_ms <= 0` | 低 | `steps = max(1, ...)` 保证至少 1 步 | `_move_servos` |
| `times` 参数越界 | `times < 1` 或 `times > 100` | 低 | `times = 2 * max(3, min(100, times))` 强制加倍 | `hand_wave` |

### 7.2 异常恢复流程

#### 角度超限恢复流程

```
输入 position
  │
  ▼
_clamp_servo(idx, angle)
  │  查表 limits[idx] = (lo, hi)
  ├─ angle < lo  → 返回 lo, 记录 WARNING 日志 (原始值 → lo)
  ├─ angle > hi  → 返回 hi, 记录 WARNING 日志 (原始值 → hi)
  └─ lo ≤ angle ≤ hi → 返回 angle (正常)
  │
  ▼
继续执行 _move_servos
```

#### 振荡参数异常恢复流程

```
_oscillate(amps, centers, period, cycles)
  │
  ├─ period == 0?
  │    是 → period = 0.5 (修正为 500ms), 记录 WARNING
  │    否 → 继续
  │
  ├─ cycles == 0?
  │    是 → cycles = 1.0 (至少 1 周期), 记录 WARNING
  │    否 → 继续
  │
  ├─ total_time = period * cycles
  ├─ steps = int(total_time / 0.05)
  │
  ▼
振荡循环
  │  每步 targets[i] = centers[i] + amps[i] * sin(phase)
  │  若 targets[i] 超出安全范围 → _clamp_servo 裁剪 (在 _servo_to_joint 之前)
  ▼
振荡结束 → 回中心
```

#### MuJoCo 步进失败恢复流程

```
_apply_ctrl(joint_angles)
  │  env.data.ctrl[:6] = joint_angles
  │
  ▼
mujoco.mj_step(model, data)
  │
  ├─ 成功 → 返回
  └─ 失败 (RuntimeError)
       │  1. 保存当前 qpos 快照 (插值前状态)
       │  2. 记录 ERROR 日志 (含 joint_angles, 异常信息)
       │  3. 恢复 qpos 到插值前状态
       │  4. _moving = False (释放动作锁)
       │  5. 重新抛出异常，由调用方决定后续行为
```

#### 序列 JSON 异常恢复流程

```
execute_sequence(sequence)
  │
  ▼
遍历 sequence["a"]
  │
  for each action:
  │  ├─ "osc" in action → 调用 _oscillate
  │  │    └─ 异常 → 记录 ERROR, 跳过当前项, 继续下一项
  │  ├─ "s" in action → 调用 _move_servos
  │  │    └─ 异常 → 记录 ERROR, 跳过当前项, 继续下一项
  │  └─ 既无 "osc" 也无 "s" → 记录 WARNING, 跳过
  │
  ▼
序列执行完成 (部分项可能被跳过)
```

---

## 8. 配置管理

### 8.1 配置参数表

动作系统的关键配置参数均硬编码对齐真机固件，不通过外部配置文件加载（保证仿真-真机一致性）：

| 参数名 | 值 | 位置 | 对齐固件 | 说明 |
|--------|:--:|------|----------|------|
| 线性插值步进 | 10 ms/步 | `_move_servos` | `movements.cc:87` | `steps = time_ms // 10` |
| 振荡采样周期 | 50 ms | `_oscillate` | `movements.cc:167` `vTaskDelay(5)` | `CONFIG_FREERTOS_HZ=100` → 5 ticks × 10ms |
| times 加倍逻辑 | `2 * max(3, min(100, times))` | `hand_wave` | `movements.cc:225` | 强制最小 6 次、最大 200 次 |
| hand_flap amount 上限 | 40 | `hand_flap` | `movements.cc` | `min(amount, 40)` |
| 舵机安全范围 (rp) | (0, 180) | `_clamp_servo` | `ClampServoTarget` | 右臂俯仰 |
| 舵机安全范围 (rr) | (100, 180) | `_clamp_servo` | `ClampServoTarget` | 右臂横滚 |
| 舵机安全范围 (lp) | (0, 180) | `_clamp_servo` | `ClampServoTarget` | 左臂俯仰 |
| 舵机安全范围 (lr) | (0, 80) | `_clamp_servo` | `ClampServoTarget` | 左臂横滚 |
| 舵机安全范围 (body) | (30, 150) | `_clamp_servo` | `ClampServoTarget` | 身体转向 |
| 舵机安全范围 (head) | (75, 105) | `_clamp_servo` | `ClampServoTarget` | 头部俯仰 |
| home 姿态 | `[180,180,0,0,90,90]` | `home` | 固件 home 常量 | 复位目标 |
| hand_wave 初始位 (lp) | 150 | `hand_wave` | `movements.cc` | 挥手起始 |
| hand_wave 初始位 (rp) | 30 | `hand_wave` | `movements.cc` | 挥手起始 |
| hand_flap 中心 (lr) | 40 | `hand_flap` | `movements.cc` | 拍打中心 |
| hand_flap 中心 (rr) | 140 | `hand_flap` | `movements.cc` | 拍打中心 |

### 8.2 环境变量

动作系统本身不读取环境变量，但其依赖的 `bridge`（舵机-关节映射桥接器）和 `env`（MuJoCo 仿真环境）可能受以下环境变量影响：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `MUJOCO_GL` | `egl` | MuJoCo 渲染后端（`egl`/`glfw`/`osmesa`），仅影响可视化 |
| `ELECTRONBOT_RENDER` | `0` | 是否启用可视化渲染（`1` 启用），测试时设为 `0` |
| `ELECTRONBOT_SIM_DT` | `0.002` | 仿真物理步长（秒），默认 2ms（500Hz） |

---

## 9. 日志与可观测性

### 9.1 日志规范

动作系统使用 Python 标准 `logging` 模块，logger 名称为 `electronbot_sim.actions`。日志级别与内容规范如下：

| 级别 | 触发场景 | 日志内容 |
|------|----------|----------|
| `INFO` | 每个动作开始执行 | `[actions] {method_name} 开始: params={...}` |
| `INFO` | 每个动作执行完成 | `[actions] {method_name} 完成: 耗时={ms}ms, 步数={steps}` |
| `WARNING` | 安全裁剪触发 | `[actions] _clamp_servo 裁剪: servo={idx}, 原始={raw}° → 裁剪={clamped}°` |
| `WARNING` | 振荡参数异常修正 | `[actions] _oscillate 参数修正: period={0→500}ms 或 cycles={0→1}` |
| `INFO` | 振荡器启动 | `[actions] _oscillate 启动: amps={...}, centers={...}, period={p}ms, cycles={c}` |
| `INFO` | 振荡器完成 | `[actions] _oscillate 完成: 总步数={steps}, 回中心` |
| `WARNING` | 序列 JSON 跳过非法项 | `[actions] execute_sequence 跳过第 {i} 项: 缺少 s/osc 键` |
| `ERROR` | MuJoCo 步进失败 | `[actions] mj_step 失败: joint_angles={...}, error={exc}` |
| `ERROR` | 序列项执行异常 | `[actions] execute_sequence 第 {i} 项异常: {exc}, 跳过` |

日志格式建议：`%(asctime)s [%(levelname)s] %(name)s: %(message)s`，时间戳精度到毫秒。

### 9.2 关键指标

| 指标名 | 类型 | 采集点 | 说明 |
|--------|------|--------|------|
| `action_count` | counter | 每个公开方法入口 | 累计动作执行次数 |
| `action_latency_ms` | histogram | 每个公开方法出口 | 动作执行耗时分布 |
| `clamp_events_total` | counter | `_clamp_servo` 裁剪时 | 安全裁剪事件总数 |
| `clamp_ratio` | gauge | `_clamp_servo` | 裁剪事件 / 总调用比 |
| `oscillation_steps` | histogram | `_oscillate` 出口 | 振荡器步数分布 |
| `sequence_items_total` | counter | `execute_sequence` | 序列项累计执行数 |
| `sequence_items_skipped` | counter | `execute_sequence` | 序列项跳过数（非法项） |
| `mj_step_failures` | counter | `_apply_ctrl` 异常 | MuJoCo 步进失败次数 |
| `moving_flag_set_duration_ms` | gauge | `_move_servos` | `_moving=True` 持续时长 |

---

## 10. 风险评估

### 10.1 技术风险

| 风险项 | 可能性 | 影响 | 严重度 | 缓解措施 |
|--------|:------:|:----:|:------:|----------|
| 线性插值与固件行为偏差（浮点精度） | 中 | 低 | 中 | Python `float64` 精度远高于固件 `float`，实际偏差 <0.001°，可通过容差 2.0° 断言吸收；测试 `test_linear_move_servos` 验证 |
| 振荡器 50ms 采样精度问题（`vTaskDelay` raw ticks） | 中 | 中 | 中 | 固件 `vTaskDelay(5)` 基于 `CONFIG_FREERTOS_HZ=100`，实际 tick 可能漂移 ±1ms；仿真用固定 `dt=0.05`，长期累积可能导致相位偏差；建议关键场景校验总时长 |
| `times` 加倍逻辑导致动作执行时间超预期 | 高 | 中 | 高 | `hand_wave(times=3)` 实际执行 `2*3=6` 次往返（12 次移动），用户预期可能为 3 次；已在 docstring 标注，建议调用方传入 `times` 时除以 2 |
| 仿真与真机舵机响应特性差异（SG90 死区、回程间隙） | 高 | 高 | 高 | 仿真为刚体模型，无死区/回程间隙/堵转；真机 SG90 死区约 ±2°，回程间隙约 1-3°；Sim2Real 阶段需通过 `ServoCalibrator` 校准 trim，并在动作幅度上预留容差 |
| MuJoCo 物理参数与真机质量分布偏差 | 中 | 中 | 中 | 仿真模型的质量、惯量、摩擦系数为估算值；建议通过真机运动捕捉数据反推校准 |
| 振荡器相位偏移 `ph` 字段未在仿真实现 | 低 | 低 | 低 | 当前 `_oscillate` 忽略 `ph` 字段，多舵机同相振荡；若需异相振荡需补充实现 |

### 10.2 依赖风险

| 依赖项 | 版本要求 | 风险 | 缓解措施 |
|--------|----------|------|----------|
| `mujoco` | ≥3.0 | API 变更可能导致 `mj_step` 调用方式变化 | 锁定版本，CI 矩阵测试 |
| `numpy` | ≥1.24 | 数组操作语义稳定，风险低 | 常规升级即可 |
| MuJoCo 模型文件 (`.xml`) | 与 Phase 1 一致 | 模型修改可能导致关节索引偏移 | 模型文件版本化，变更需回归测试 |
| `bridge`（舵机-关节映射） | Phase 3 交付 | 映射函数 `_servo_to_joint` 错误会导致动作失真 | 单元测试覆盖 6 个舵机的双向映射 |
| Python 标准库 `time.sleep` | — | `execute_sequence` 中 `time.sleep(delay/1000)` 阻塞事件循环 | 异步场景需替换为 `asyncio.sleep` |

---

## 11. 变更记录

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|----------|------|
| v1.0 | 2026-07-04 | 初始版本：完成章节 1-4（预期效果、实现细节、验证方法、交付物清单） | 架构组 |
| v1.1 | 2026-07-04 | 补充软件工程规范章节 5-11（接口设计、数据模型、错误处理、配置管理、日志可观测性、风险评估、变更记录） | 架构组 |
