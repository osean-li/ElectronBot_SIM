# ElectronBot 真实硬件开发参考手册

> **目的**: 为 ElectronBot_SIM 仿真提供 1:1 真实参数依据  
> **来源**: ElectronBot 固件 (robot.h/cpp)、ServoDrive 源代码、SDK LowLevel、原理图  
> **最后更新**: 2026-07-01

---

## 目录

1. [机械结构与关节参数](#1-机械结构与关节参数)
2. [角度映射算法](#2-角度映射算法)
3. [舵机控制子系统](#3-舵机控制子系统)
4. [I2C 通信协议](#4-i2c-通信协议)
5. [USB CDC 通信协议](#5-usb-cdc-通信协议)
6. [图像传输参数](#6-图像传输参数)
7. [PID 控制参数](#7-pid-控制参数)
8. [硬件规格](#8-硬件规格)
9. [传感器参数](#9-传感器参数)
10. [仿真适配建议](#10-仿真适配建议)

---

## 1. 机械结构与关节参数

### 1.1 关节完整定义 (来源: robot.h:27-127)

ElectronBot 共有 **6 个自由度 (DOF)**，由 6 个独立舵机驱动：

| 索引 | 关节名 | I2C ID | 机械角Min(°) | 机械角Max(°) | 模型角Min(°) | 模型角Max(°) | 反转 | 功能描述 |
|:---:|--------|:------:|:------------:|:------------:|:------------:|:------------:|:---:|----------|
| j1 | Head (头部) | 2 | 70 | 95 | -15 | +15 | ✅ | 头部俯仰 |
| j2 | L.ArmRoll (左臂Roll) | 4 | -9 | 3 | 0 | 30 | ❌ | 左臂旋转 |
| j3 | L.ArmPitch (左臂俯仰) | 6 | -16 | 117 | -20 | 180 | ❌ | 左臂抬举 |
| j4 | R.ArmRoll (右臂Roll) | 8 | 133 | 141 | 0 | 30 | ✅ | 右臂旋转 |
| j5 | R.ArmPitch (右臂俯仰) | 10 | 15 | 150 | -20 | 180 | ✅ | 右臂抬举 |
| j6 | Body (腰部) | 12 | 0 | 180 | -90 | +90 | ❌ | 腰部旋转 |

> **关键发现**:  
> - j1 (Head) 机械角范围极小 (70°~95°, 仅 25° 行程), 说明是**微型连杆机构**不做直接旋转  
> - 所有关节**零位**均为模型角度 0°  
> - Inverted 表示模型角→机械角的**映射方向翻转**

### 1.2 运动学链 (来源: Unity Prefab)

```
基座 (base_fixed)
  └── body (腰部旋转, j6)
        ├── head (头部俯仰, j1)
        ├── left_shoulder
        │     ├── roll (左臂Roll, j2)
        │     └── pitch (左臂俯仰, j3) → 末端
        └── right_shoulder
              ├── roll (右臂Roll, j4)
              └── pitch (右臂俯仰, j5) → 末端
```

### 1.3 机械行程与模型角度对照

```
关节  | 机械行程 (度) | 模型行程 (度) | 传动关系
------|-------------|-------------|----------
Head  | 70→95  (Δ25°) | -15→+15 (Δ30°) | 连杆放大, 反转
L.Roll| -9→3  (Δ12°) | 0→30  (Δ30°) | 2.5x 放大
L.Pitch|-16→117(Δ133°)| -20→180(Δ200°)| ~1.5x 放大
R.Roll| 133→141(Δ8°) | 0→30  (Δ30°) | 3.75x 放大, 反转
R.Pitch|15→150(Δ135°)| -20→180(Δ200°)| ~1.48x 放大, 反转
Body  | 0→180  (Δ180°)| -90→90 (Δ180°)| 1:1 直接
```

> **仿真注意**: Body 是 1:1 直驱，双臂 Pitch 有 ~1.5x 机构放大

### 1.4 连杆长度估算

基于 Unity Prefab Transform 层级和视觉外观估算（CAD STEP 精确值待 FreeCAD 解析后确认）：

| 连杆 | 估长 (mm) | 说明 |
|------|----------|------|
| base_height | ~30 | 底座高度 |
| body_radius | ~40 | 机身半径 |
| shoulder_to_elbow | ~60 | 肩→肘 |
| elbow_to_wrist | ~50 | 肘→腕 |
| head_height | ~25 | 头部高度 |

---

## 2. 角度映射算法

### 2.1 模型角 → 机械角 (写操作, robot.cpp:127-152, 241-261)

**正向 (inverted=false):**
```
sAngle = (angle - modelAngelMin) / (modelAngelMax - modelAngelMin)
       × (angleMax - angleMin) + angleMin
```

**反向 (inverted=true):**
```
sAngle = (angle - modelAngelMin) / (modelAngelMax - modelAngelMin)
       × (angleMin - angleMax) + angleMax
```

### 2.2 机械角 → 模型角 (读操作, robot.cpp:225-261)

**正向 (inverted=false):**
```
jAngle = (angle - angleMin) / (angleMax - angleMin)
       × (modelAngelMax - modelAngelMin) + modelAngelMin
```

**反向 (inverted=true):**
```
jAngle = (angleMax - angle) / (angleMax - angleMin)
       × (modelAngelMax - modelAngelMin) + modelAngelMin
```

### 2.3 具体关节映射示例

以 **j1 (Head, inverted=true)** 为例：

```python
# 写入: 模型角 0° → 机械角
sAngle = (0 - (-15)) / (15 - (-15)) * (70 - 95) + 95
       = 15/30 * (-25) + 95 = 82.5°

# 读取: 机械角 82.5° → 模型角
jAngle = (95 - 82.5) / (95 - 70) * (15 - (-15)) + (-15)
       = 12.5/25 * 30 - 15 = 0°
```

以 **j6 (Body, inverted=false)** 为例：

```python
# 写入: 模型角 45° → 机械角
sAngle = (45 - (-90)) / (90 - (-90)) * (180 - 0) + 0
       = 135/180 * 180 = 135°

# 读取: 机械角 135° → 模型角
jAngle = (135 - 0) / (180 - 0) * (90 - (-90)) + (-90)
       = 135/180 * 180 - 90 = 45°
```

> **仿真注意**: 所有控制算法和运动学解算**始终使用模型角度**。机械角度仅用于校验舵机硬件范围。

---

## 3. 舵机控制子系统

### 3.1 硬件规格

| 参数 | 值 |
|------|-----|
| **MCU** | STM32F042P6 (Cortex-M0, 32K Flash, 6K SRAM) |
| **电机驱动** | FM116B (H桥) |
| **角度传感器** | 电位器 + 12-bit ADC (STM32F042 内置) |
| **控制频率** | **200 Hz** (TIM14 定时器中断) |
| **PWM 频率/分辨率** | TIM3, 最大占空比 1000 (0~1000) |
| **I2C 角色** | 从机 (Slave), 7-bit 地址 |
| **工厂 I2C 地址** | 12 (出厂默认, 偶数) |

### 3.2 ADC 角度标定

```
angle = mechanicalAngleMin
      + (mechanicalAngleMax - mechanicalAngleMin)
      × (adcVal - adcValAtAngleMin)
      / (adcValAtAngleMax - adcValAtAngleMin)
```

| 参数 | 出厂默认值 |
|------|-----------|
| adcValAtAngleMin | 250 |
| adcValAtAngleMax | 3000 |
| 角度分辨率 | 180° / (3000-250) = **0.0655°/LSB** |
| LSB/度 | ~15.3 |

### 3.3 舵机出厂配置 (configurations.h)

```c
BoardConfig_t {
    .configStatus      = CONFIG_OK,
    .nodeId            = 12,         // 7-bit I2C 地址
    .initPos           = 90.0,       // 初始机械角度
    .toqueLimit        = 0.5,        // 力矩限制 50%
    .velocityLimit     = 0,          // 无速度限制
    .adcValAtAngleMin  = 250,        // ADC 最小值
    .adcValAtAngleMax  = 3000,       // ADC 最大值
    .mechanicalAngleMin= 0,          // 机械角下限
    .mechanicalAngleMax= 180,        // 机械角上限
    .dceKp             = 10.0,       // 比例增益
    .dceKv             = 0.0,        // 速度积分增益
    .dceKi             = 0.0,        // 位置积分增益
    .dceKd             = 50.0,       // 微分增益
    .enableMotorOnBoot = false,      // 上电不使能
}
```

> **注意**: 以上为舵机驱动板的**出厂默认值**。PC Host 在连接后会通过 I2C 重新写入各关节 PID 和力矩限制。
> PID 配置会**写入 Flash 持久化**（自动 Commit）。

### 3.4 DCE 控制算法 (motor.cpp:5-26)

```
DCE_INTEGRAL_LIMIT = 500.0

errorPos    = inputPos - setPointPos
errorVel    = inputVel - setPointVel
deltaPos    = errorPos - lastError

integralPos += errorPos     (clamped to ±500)
integralVel += errorVel     (clamped to ±500)

output = Kp × errorPos
       + Ki × integralPos
       + Kv × integralVel
       + Kd × deltaPos

output = clamp(output, -limitTorque, +limitTorque)
limitTorque = torquePercent × 1000
```

> **物理含义**: `output` 直接映射为 PWM 占空比 (0~1000)。当 `torquePercent=0.5` 时, 最大 PWM = 500。

### 3.5 PWM 方向控制

```c
if (_pwm >= 0):
    CH1=0, CH2=min(_pwm, 1000)    // 正转
else:
    CH1=min(-_pwm, 1000), CH2=0   // 反转
```

---

## 4. I2C 通信协议

### 4.1 帧格式

| 方向 | 字节数 | 说明 |
|------|:-----:|------|
| Host → Servo | 5 字节 | i2cTxData[0..4] |
| Servo → Host | 5 字节 | i2cRxData[0..4] |
| 广播地址 | 0x00 | 7-bit |

### 4.2 完整命令表 (main.cpp:95-208)

| 命令码 | 功能 | 发送数据 | 返回数据 | 持久化 |
|:-----:|------|---------|---------|:---:|
| `0x01` | Set angle | `[0x01] + float(4B)` | `[0x01] + float(angle 4B)` | ❌ |
| `0x02` | Set velocity | `[0x02] + float(4B)` | `[0x02] + float(velocity 4B)` | ❌ |
| `0x03` | Set torque | `[0x03] + float(4B)` | `[0x03] + float(angle 4B)` | ❌ |
| `0x11` | Get angle | `[0x11]` | `[0x11] + float(angle 4B)` | — |
| `0x12` | Get velocity | `[0x12]` | `[0x12] + float(velocity 4B)` | — |
| `0x21` | Set ID | `[0x21] + uint8(1B)` | `[0x21] + float(angle 4B)` | ✅ |
| `0x22` | Set Kp | `[0x22] + float(4B)` | `[0x22] + float(angle 4B)` | ✅ |
| `0x23` | Set Ki | `[0x23] + float(4B)` | `[0x23] + float(angle 4B)` | ✅ |
| `0x24` | Set Kv | `[0x24] + float(4B)` | `[0x24] + float(angle 4B)` | ✅ |
| `0x25` | Set Kd | `[0x25] + float(4B)` | `[0x25] + float(angle 4B)` | ✅ |
| `0x26` | Set torque limit | `[0x26] + float(4B)` | `[0x26] + float(angle 4B)` | ✅ |
| `0x27` | Set init pos | `[0x27] + float(4B)` | `[0x27] + float(angle 4B)` | ✅ |
| `0xFF` | Enable/Disable | `[0xFF] + uint8(1B)` | 原样回显 | ❌ |

> **注意**: `float` 为 IEEE 754 单精度，**_little-endian 字节序_**。

### 4.3 各关节调优后的 PID (来自 robot.h 注释)

```c
joint[1] Head:    Kp=30,   Ki=0.4, Kv=0, Kd=200, TorqueLimit=0.5
joint[2] L.Roll:  Kp=50,   Ki=0.8, Kv=0, Kd=600, TorqueLimit=1.0
joint[3] L.Pitch: Kp=50,   Ki=0.8, Kv=0, Kd=300, TorqueLimit=0.5
joint[4] R.Roll:  Kp=50,   Ki=0.8, Kv=0, Kd=600, TorqueLimit=1.0
joint[5] R.Pitch: Kp=50,   Ki=0.8, Kv=0, Kd=300, TorqueLimit=0.5
joint[6] Body:    Kp=150,  Ki=0.8, Kv=0, Kd=300, TorqueLimit=0.5
```

> **关键观察**: 
> - Body (腰部) 的 Kp=150 远大于其他关节，说明需要更大力矩对抗惯量
> - 双臂 Roll 的 Kd=600 最大，用于抑制末端振动
> - Kv 始终为 0（未启用速度前馈）

---

## 5. USB CDC 通信协议

### 5.1 基本参数

| 参数 | 值 |
|------|-----|
| USB VID | `0x1001` |
| USB PID | `0x8023` |
| 接口类型 | USB CDC Bulk |
| EP1 IN | 接收 (Host→Device) |
| EP1 OUT | 发送 (Device→Host) |
| 超时 | 100 ms |

### 5.2 ExtraData 32 字节精确布局

这是 **PC ↔ MCU 双向传递**的核心数据结构，每 4 轮传输中各发送 1 次。

#### PC → MCU (角度写入, electron_low_level.cpp:157-176)

```
Byte 0:        enable_flag       (uint8_t, 0=禁用 1=启用)
Bytes 1–4:     joint_angles[0]   (float LE, j1)
Bytes 5–8:     joint_angles[1]   (float LE, j2)
Bytes 9–12:    joint_angles[2]   (float LE, j3)
Bytes 13–16:   joint_angles[3]   (float LE, j4)
Bytes 17–20:   joint_angles[4]   (float LE, j5)
Bytes 21–24:   joint_angles[5]   (float LE, j6)
Bytes 25–31:   reserved (未使用)
```

#### MCU → PC (角度读取, electron_low_level.cpp:179-185)

```
Byte 0:        enable_status     (uint8_t, 当前使能状态回读)
Bytes 1–4:     current_angles[0] (float LE, j1)
Bytes 5–8:     current_angles[1] (float LE, j2)
Bytes 9–12:    current_angles[2] (float LE, j3)
Bytes 13–16:   current_angles[3] (float LE, j4)
Bytes 17–20:   current_angles[4] (float LE, j5)
Bytes 21–24:   current_angles[5] (float LE, j6)
Bytes 25–31:   reserved
```

#### ExtraData 打包代码 (C++)

```cpp
// 写入
extraDataBufferTx[index][0] = _enable ? 1 : 0;
for (int j = 0; j < 6; j++)
    for (int i = 0; i < 4; i++) {
        auto* b = (unsigned char*) &jointAngleSetPoints[j];
        extraDataBufferTx[index][j * 4 + i + 1] = *(b + i);
    }

// 读取
for (int j = 0; j < 6; j++)
    _jointAngles[j] = *((float*)(extraDataBufferRx + 4 * j + 1));
```

#### ExtraData 打包代码 (Python 等效)

```python
import struct

def pack_extra_data(angles_rad, enable=True):
    """打包 6 个模型角度 → 32 字节 ExtraData"""
    buf = bytearray(32)
    buf[0] = 1 if enable else 0
    for j, angle in enumerate(angles_rad):
        struct.pack_into('<f', buf, 1 + j * 4, float(angle))
    return bytes(buf)

def unpack_extra_data(data):
    """解析 32 字节 → 6 个模型角度"""
    enable = data[0] != 0
    angles = [
        struct.unpack_from('<f', data, 1 + j * 4)[0]
        for j in range(6)
    ]
    return angles, enable
```

> **仿真关键**: 仿真中的 RobotInterface 必须实现相同的 pack/unpack 接口，确保 Sim2Real 切换时上层代码零修改。

---

## 6. 图像传输参数

### 6.1 传输格式

| 参数 | 值 |
|------|-----|
| 分辨率 | **240 × 240** (正方形) |
| 颜色格式 | RGB (3 通道), 逐行扫描 |
| 单帧大小 | 240 × 240 × 3 = **172,800 字节** |
| 转换前格式 | BGRA → RGB (OpenCV) |

### 6.2 分片传输协议 (SyncTask, electron_low_level.cpp:130-155)

每帧分 **4 轮**传输：

```
Round p=0,1,2,3:
  1. RECV: 32 字节 ExtraData (从 MCU 收)
  2. SEND: 84 × 512 = 43,008 字节 (图像块)
  3. Memcpy: 192 字节 (图像尾) + 32 字节 (ExtraData) → usbBuffer200
  4. SEND: 1 × 224 字节 (尾包: 192 + 32)
```

**每轮数据量**: 43,008 + 192 = **43,200 字节** (= 172,800 / 4)  
**总传输量**: 4 × (84×512 + 224) = **172,928 字节**  
**有效载荷**: 172,800 图像 + 128 ExtraData (4×32)

### 6.3 图像帧率

- 固件端: 由 PC Sync() 驱动, 非固定帧率
- PC 端: 每次 `SetImageSrc()` + `Sync()` 触发一次完整帧传输
- USB 全速 (12 Mbps): 理论最大 ~7 fps (172,928 B/frame)
- 实际可用: **3–5 fps** (受限于 MCU 处理能力)

> **仿真适配**: MuJoCo 仿真中 camera 输出 `240×240×3 RGB` uint8 数组，与实机格式完全一致。

---

## 7. PID 控制参数

### 7.1 舵机端 DCE 默认值 (出厂)

| 参数 | 值 |
|------|-----|
| Kp_default | 10.0 |
| Ki_default | 0.0 |
| Kv_default | 0.0 |
| Kd_default | 50.0 |
| TorqueLimit | 0.5 (50%) |
| IntegralLimit | ±500.0 |
| PWM_max | 1000 |

### 7.2 Host 写入的各关节调优 PID

| 关节 | Kp | Ki | Kd | TorqueLimit |
|------|:--:|:--:|:--:|:-----------:|
| Head (j1) | 30 | 0.4 | 200 | 0.5 |
| L.Roll (j2) | 50 | 0.8 | 600 | 1.0 |
| L.Pitch (j3) | 50 | 0.8 | 300 | 0.5 |
| R.Roll (j4) | 50 | 0.8 | 600 | 1.0 |
| R.Pitch (j5) | 50 | 0.8 | 300 | 0.5 |
| Body (j6) | 150 | 0.8 | 300 | 0.5 |

### 7.3 仿真中 PD 控制的等效参数

MuJoCo 使用 `position` actuator (kp/kv) 而非 DCE：

```python
# 等效于实机的 PID 在 MuJoCo 中的 actuator gain
mjcf_gain = {
    "j1_head":      {"kp": 30,  "kv": 5},    # Kd/Kp ≈ 6.7 → kv ≈ Kp × 0.15
    "j2_l_roll":    {"kp": 50,  "kv": 8},
    "j3_l_pitch":   {"kp": 50,  "kv": 8},
    "j4_r_roll":    {"kp": 50,  "kv": 8},
    "j5_r_pitch":   {"kp": 50,  "kv": 8},
    "j6_body":      {"kp": 150, "kv": 20},
}
```

> **注意**: MuJoCo 的 position actuator 增益是**归一化**的 (kp/actuator_gear), 需要根据传动比换算。上述值为参考起点, 需通过 System ID 校准。

---

## 8. 硬件规格

### 8.1 主控板

| 参数 | 值 |
|------|-----|
| MCU | **STM32F405RGT6** (Cortex-M4, 168MHz, 1MB Flash, 192KB SRAM) |
| USB | Full-Speed OTG (12 Mbps) |
| I2C | 主模式, 连接 6 路舵机 |
| SPI | 连接 240×240 LCD (ST7789) |
| RTOS | FreeRTOS |
| 编程口 | SWD |

### 8.2 舵机驱动板 (6 个)

| 参数 | 值 |
|------|-----|
| MCU | STM32F042P6 (Cortex-M0, 48MHz) |
| 电机驱动 | FM116B |
| 传感器 | 12-bit ADC 电位器 |
| 控制频率 | 200 Hz |

### 8.3 USB 摄像头 (外接)

| 参数 | 值 |
|------|-----|
| 接口 | USB 2.0 |
| 分辨率 | 最高 1080p (仿真中 resize 到 240×240) |
| 用途 | 视觉感知 (VLA 输入) |

---

## 9. 传感器参数

### 9.1 手势传感器

| 参数 | 值 |
|------|-----|
| 型号 | **PAJ7620U2** |
| 接口 | I2C |
| 功能 | 9 种手势识别 (上/下/左/右/前/后/顺时针/逆时针/挥手) |

### 9.2 IMU

| 参数 | 值 |
|------|-----|
| 型号 | **MPU6050** (6 轴) |
| 接口 | I2C |
| 输出 | 3 轴加速度 + 3 轴陀螺仪 |
| 加速度范围 | ±2/4/8/16g |
| 陀螺仪范围 | ±250/500/1000/2000°/s |

### 9.3 仿真传感器对应

| 实机传感器 | MuJoCo 仿真传感器 | 参数 |
|-----------|------------------|------|
| USB Camera 240×240 | `<camera>` RGB-D (D435 风格) | 240×240, fov=60° |
| MPU6050 Accel | `<accelerometer>` | noise=0.001 |
| MPU6050 Gyro | `<gyro>` | noise=0.001 |
| 电位器角度 | `<jointpos>` / `<actuatorpos>` | — |
| PAJ7620U2 手势 | 暂不仿真 (VLA 替代) | — |

---

## 10. 仿真适配建议

### 10.1 运动学链定义

仿真中模型角度顺序必须与实机一致：

```yaml
# 模型角度 (rad), 顺序固定:
joint_angles = [
    body_yaw,       # j6, 腰部 (±90°)
    head_pitch,     # j1, 头部 (±15°)
    left_arm_pitch, # j3, 左臂俯仰 (-20°~180°)
    left_arm_roll,  # j2, 左臂Roll (0~30°)
    right_arm_pitch,# j5, 右臂俯仰 (-20°~180°)
    right_arm_roll, # j4, 右臂Roll (0~30°)
]
```

### 10.2 MuJoCo actuator 配置对应

```xml
<!-- 每个关节对应一个 position actuator -->
<actuator>
    <!-- j6: Body (腰部, 直接驱动, 最大力矩) -->
    <position name="body" joint="body_joint"
              kp="150" kv="20"
              forcerange="-0.5 0.5"
              ctrlrange="-1.57 1.57"/>

    <!-- j1: Head (连杆放大, inverted) -->
    <position name="head" joint="head_joint"
              kp="30" kv="5"
              forcerange="-0.25 0.25"
              ctrlrange="-0.2618 0.2618"/>

    <!-- 其他关节类似... -->
</actuator>
```

### 10.3 Domain Randomization 校准

基于实机参数设计随机化范围：

| 参数 | 标称值 | 随机化范围 | 依据 |
|------|:------:|:----------:|------|
| joint damping | 5–20 N·s/m | ±30% | DCE Kd 离散范围 |
| actuator kp | 30–150 | ±25% | 各关节 Kp 差异 |
| friction | 0.05–0.15 | ±50% | 微型连杆摩擦 |
| armature | 0.001–0.01 | ±50% | 小电机转动惯量 |
| camera noise | σ=0.01 | ±50% | RGB sensor noise |
| payload mass | 0–50g | ±100% | 末端负载变化 |

### 10.4 阻抗控制参数基准

实机 DCE 控制可等效为阻抗控制：

```python
# Kp, Kd 从实机 PID 参数直接换算
impedance_Kp = {
    "head": 30,     # N·m/rad (等效刚度)
    "body": 150,
}
impedance_Kd = {
    "head": 5,      # N·m·s/rad (等效阻尼)
    "body": 20,
}
damping_ratio = 0.7  # 临界阻尼比
```

### 10.5 通信时序约束

| 约束 | 值 | 仿真影响 |
|------|-----|---------|
| 控制频率 | 200 Hz (舵机) | MuJoCo timestep ≤ 0.005s |
| USB 帧率 | 3–5 fps | 视觉控制 must ≤ 5Hz |
| I2C 延迟 | ~1ms/joint | 忽略 (仿真瞬时) |
| USB 延迟 | ~25ms/帧 | Sim2Real 需模拟 |

---

## 附录 A: 关键数值速查表

| 类别 | 参数 | 值 |
|------|------|-----|
| **DOF** | 关节数 | 6 |
| **I2C** | 帧长 | 5 字节 |
| **I2C** | 命令数 | 13 个 |
| **USB** | ExtraData | 32 字节 |
| **USB** | VID:PID | 0x1001:0x8023 |
| **图像** | 分辨率 | 240×240×3 (RGB) |
| **图像** | 单帧大小 | 172,800 字节 |
| **控制** | 舵机控制频率 | 200 Hz |
| **控制** | PWM 范围 | 0–1000 |
| **ADC** | 分辨率 | 12-bit (250–3000) |
| **ADC** | 角度分辨率 | 0.0655°/LSB |
| **传感器** | 手势 | PAJ7620U2 |
| **传感器** | IMU | MPU6050 |

---

## 附录 B: 与现有 ElectronBot_SIM 文件对照

| 实机源码 (行号) | 仿真文件 | 参数一致性 |
|---------|-------------|:--:|
| robot.h:27-127 (关节定义) | `utils.py:15-22 JOINT_PARAMS` | ✅ 1:1 |
| robot.h:143-152 (JointStatus_t) | `robot.py:46-52 JointStatus` | ✅ 1:1 |
| robot.cpp:127-152 (角度映射, 写入) | `utils.py:43-68 model_angle_to_mech()` | ✅ 公式一致 |
| robot.cpp:225-261 (角度映射, 读取) | `utils.py:71-98 mech_angle_to_model()` | ✅ 公式一致 |
| robot.cpp:241-261 (UpdateJointAngle) | `robot.py:267-296 _update_joint_angle()` | ✅ 流程一致 |
| motor.cpp:5-26 (DCE CalcDceOutput) | `servo_sim.py:180-212 calc_dce_output()` | ✅ 公式一致 |
| motor.cpp:29-47 (PWM SetPwm) | `servo_sim.py:229-255 step_200hz()` | ✅ PWM=0~1000 |
| configurations.h:20-36 (出厂值) | `servo_sim.py:28-43 ServoFactoryConfig` | ✅ 值一致 |
| main.cpp:88-208 (I2C命令) | `servo_sim.py:86-175 handle_i2c_command()` | ✅ 13条命令 |
| main.cpp:230-233 (ADC→角度) | `servo_sim.py:237-244` (注释说明) | ✅ 仿真用MuJoCo替代 |
| main.cpp:33-94 (主循环) | `robot.py:342-369 control_step()` | ✅ 流程一致 |
| electron_low_level.cpp:157-176 (ExtraData写) | `protocol.py:36-43 encode_extra_data()` | ✅ 布局一致 |
| electron_low_level.cpp:179-185 (ExtraData读) | `protocol.py:46-55 decode_extra_data()` | ✅ 布局一致 |
| electron_low_level.cpp:130-155 (图像传输) | `robot.py:MuJoCo Renderer 240×240` | ✅ 分辨率一致 |
| Unity Prefab (Transform层级) | `electronbot.xml` 运动学链 | ⚠️ 需CAD精确位置 |
