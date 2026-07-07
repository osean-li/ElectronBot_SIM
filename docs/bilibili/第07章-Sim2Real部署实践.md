# 第七章 Sim2Real 部署实践

> **核心问题**：如何将仿真中训练的策略经 MCP 协议零修改部署到真机 ESP32，并处理部署过程中的偏差？

## 7.1 章节目标

本章从概要设计文档的 Layer 7（应用评估与 Sim2Real 部署）出发，目标为：
1. 实现 `McpCloudBridge`，打通云端 API 通信链路
2. 完成第一次真机部署并记录出现的问题
3. 排查并修复速度参数截断、动作队列冲突、trim 不对称三个主要问题
4. 执行真机校准并对比仿真与真机的执行效果

## 7.2 真机硬件规格

本项目目标真机为搭载 xiaozhi-esp32 release v2.2.6 固件的 ElectronBot：

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | ESP32-S3-WROOM-N16R8 | 双核 240MHz，16MB Flash + 8MB PSRAM |
| 舵机 1 | SG90 9G | 身体旋转，扭矩 1.5kg·cm |
| 舵机 2-5 | 2g 微型舵机 | 双臂 Pitch ×2 + Roll ×2，扭矩 0.2kg·cm |
| 舵机 6 | 4.3g 舵机 | 头部俯仰，扭矩 0.5kg·cm |
| 屏幕 | GC9A01 240×240 | 圆形 LCD |
| 通信 | MQTT/WebSocket | 作为客户端连接小智云端 |
| 摄像头 | **无** | 不支持视觉 VLA |
| 编码器 | **无** | 全部为开环 PWM，无位置反馈 |

## 7.3 仿真 vs 真机能力对照

| 维度 | 仿真 | 真机 (release v2.2.6) |
|------|:---:|:---------------------:|
| 预设动作（8 种） | ✅ | ✅ |
| 精确关节角度控制 | ✅ servo_move | ❌ 需固件 OTA |
| 动作序列执行 | ✅ servo_sequences | ❌ 需固件 OTA |
| 关节位置反馈 | ✅ 精确 qpos | ❌ 无编码器 |
| 关节速度反馈 | ✅ 精确 qvel | ❌ 无编码器 |
| 末端位置 | ✅ 精确 xpos | ❌ 无编码器 |
| 摄像头输入 | ✅ MuJoCo 渲染 | ❌ 无硬件 |
| 控制延迟 | <1ms（进程内） | 200-500ms（云端转发） |

## 7.4 模式 A：云端 API 透传

### 7.4.1 McpCloudBridge 实现

```python
class McpCloudBridge:
    def __init__(self, api_url, device_id):
        self.api_url = api_url
        self.device_id = device_id
    
    async def call(self, tool_name, arguments):
        async with httpx.AsyncClient() as client:
            resp = await client.post(
                f"{self.api_url}/devices/{self.device_id}/tools",
                json={"name": tool_name, "arguments": arguments}
            )
            return resp.json()
    
    async def list_tools(self):
        resp = await client.get(
            f"{self.api_url}/devices/{self.device_id}/tools")
        return resp.json()["tools"]
```

### 7.4.2 部署命令

```bash
python -m src.electronbot_sim2real.deploy_cloud \
    --policy checkpoints/bc_wave.pt \
    --device-id eb-001 \
    --api-url https://api.xiaozhi.cn/v1
```

### 7.4.3 通信链路

```
Python 策略 → HTTPS(~100ms) → 小智云端 → MQTT(~100-400ms) → ESP32
```

适用于 VLA 语音控制、预设动作序列等对延迟不敏感的场景。PPO 等 50Hz 闭环策略不可通过此路径部署（步长 20ms < 最小延迟 200ms）。

## 7.5 部署问题排查

第一次部署中发现了三个主要问题。

### 问题 1：速度参数截断

**现象**：仿真中发 `steps=2` 挥 2 次，真机挥了 6 次。

**原因**：真机固件对 `steps` 参数有硬性限制：
```c
times = 2 * max(3, min(100, steps));
// steps=2 → max(3, min(100, 2)) = 3 → times = 6
```

**修复**：在仿真桥接器中添加相同的限制逻辑：
```python
def hand_action(self, action, hand, steps, speed, amount=30):
    steps = max(3, min(100, steps))  # 对齐真机固件
    # ... 后续执行逻辑
```

### 问题 2：动作队列冲突

**现象**：连续发送两个命令时，第二个命令被丢弃。

**原因**：真机固件使用 FreeRTOS 动作队列（`xQueueReceive`）逐帧执行。队列满时丢弃新命令。

**修复**：在仿真桥接器中模拟队列行为：命令间增加间隔、队列满时返回 `busy` 状态。

### 问题 3：Trim 不对称

**现象**：双手举手时左右臂高度不一致。

**原因**：3D 打印公差、舵机个体差异、装配误差的综合结果。

**修复**：执行校准流程，为每个舵机设定 trim 值。

## 7.6 真机校准

校准工具逐一提示各舵机对齐参考位置，根据用户观察输入偏差值：

```python
for servo_idx, name in enumerate(["rp","rr","lp","lr","b","h"]):
    await backend.call("self.electron.servo_move",
                       {"servo_type": name, "position": 90, "speed": 500})
    trim = float(input(f"{name} 偏差 (°): "))
    await backend.call("self.electron.set_trim",
                       {"servo": servo_idx, "trim": trim})
```

校准完成后通过 `get_trims` 验证：
```python
backend.call("self.electron.get_trims", {})
# → {"trims": [0, -3, 2, 0, -1, 0]}
```

Trim 值保存在 ESP32 的 NVS 中，断电不丢失，校准一次即可。

## 7.7 分层部署路线图

基于真机 `release v2.2.6` 的实际能力，规划分层部署路径：

```
L1 立即可部署（云端 API，无需固件修改）
├── VLA 语音控制
├── 预设动作执行
└── 状态查询与校准

L2 短期可部署（需固件 OTA 升级）
├── 添加 MCP servo_move 工具
├── 添加 MCP servo_sequence 工具
└── 降低 action task 优先级（修复音频卡顿）

L3 中期可部署（需 WebSocket 直连固件）
├── ONNX 推理部署（MLP 策略，ESP32-S3 可行）
├── 低延迟 MCP 直连（<10ms RTT）
└── 半闭环控制（指令值+时间戳估计）

L4 远期（需硬件升级）
├── 带编码器的智能舵机
├── 摄像头集成
└── 真闭环控制 + ACT 本地推理
```

## 7.8 已知硬件限制

以下限制不在仿真中建模（建模不精确会产生误导），以文档形式标注：

| 限制 | 影响 |
|------|------|
| 无编码器 | joint_vel/ee_positions 真机不可得，RL 策略盲操 |
| 无摄像头 | 不支持视觉 VLA，仅纯文本 VLA |
| 云端延迟 200-500ms | PPO@50Hz 策略不可通过云端部署 |
| 伺服死区 2-5° | <5° 微调真机不响应 |
| SG90 扭矩 1.5kg·cm | 策略可能超出实际扭矩 |
| 电池电压跌落 | 6 舵机同动时速度下降 |
| 动作无法中断 | stop 命令有 1-3s 延迟 |

## 7.9 全书总结

本书按项目实施顺序，从环境搭建到真机部署，完整记录了 ElectronBot_SIM 平台的构建过程。各章对应概要设计文档的各层：

| 章 | 对应架构层 | 核心产出 |
|:--:|:---------:|---------|
| 二 | L1 | MJCF 模型加载与验证 |
| 三 | L1 | STEP→MJCF 全流程与场景拆分 |
| 四 | L2+L3 | Gymnasium 环境 + 12 种动作工具 |
| 五 | L4 | MCP 协议 + 统一 Backend API |
| 六 | L5+L6 | 传感器 + BC/PPO 训练管线 |
| 七 | L7 | 云端部署 + 校准 + 问题排查 |

每一章末尾都记录了实际实现与概要设计文档的差异。这些差异表——而非漂亮的架构图——才是本书最有价值的贡献。它们说明了一个简单的工程事实：设计是对现实的近似，实现才是真实的。
