# ElectronBot_SIM — 全栈 AI 机器人仿真与 Sim2Real 平台 · 概要设计文档

> 版本：v2.2  
> 日期：2026-07-03  
> 基于：xiaozhi-esp32 release v2.2.6 + 稚晖君 ElectronBot 机械结构  
> 核心目标：仿真中开发的 AI 策略，通过 MCP 协议零修改部署到真机 ESP32 上

---

## 目录

1. [系统架构](#1-系统架构)
2. [模块划分](#2-模块划分)
3. [模块间接口](#3-模块间接口)
4. [技术选型](#4-技术选型)
5. [部署拓扑](#5-部署拓扑)
6. [MCP 统一接口](#6-mcp-统一接口)
7. [Sim2Real 全链路](#7-sim2real-全链路)
8. [真机对接](#8-真机对接)

---

## 1. 系统架构

### 1.1 7 层分层架构

平台采用 7 层分层架构，自底向上从硬件参考到应用评估：

```
┌─────────────────────────────────────────────────────────────────┐
│                    ElectronBot_SIM 7 层架构                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Layer 7: 应用与评估层                              ◄── 用户入口  │
│  ┌──────────┐ ┌──────────┐ ┌──────────────┐ ┌───────────────┐ │
│  │ Web UI   │ │ CLI 工具 │ │ Benchmark    │ │ Sim2Real      │ │
│  │ (Three.js│ │ (训练/   │ │ Suite        │ │ Bridge        │ │
│  │ 3D 可视) │ │  评估)   │ │ (7 任务×5指标)│ │ (云端API部署) │ │
│  └────┬─────┘ └────┬─────┘ └──────┬───────┘ └───────┬───────┘ │
│       └─────────────┴─────────────┴───────────────┴─────────┘ │
│                              │                                  │
│  Layer 6: 智能决策层                              ◄── AI 大脑   │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │  ┌───────────────┐  ┌───────────────┐  ┌──────────────┐ │ │
│  │  │ RL Training   │  │ Imitation     │  │ Text-VLA      │ │ │
│  │  │ (PPO/SAC/TD3) │  │ Learning      │  │ Planning      │ │ │
│  │  │               │  │ (BC/ACT/DP)   │  │ (LLM → MCP)  │ │ │
│  │  │ 64 并行环境    │  │ 示范→策略     │  │ 语音→MCP序列 │ │ │
│  │  └───────────────┘  └───────────────┘  └──────────────┘ │ │
│  │                                                           │ │
│  │  行为树引擎 (py_trees)                                     │ │
│  │  语音意图 → 任务分解 → 原子动作序列 → MCP 命令              │ │
│  └──────────────────────────────────────────────────────────┘ │
│                              │                                  │
│  Layer 5: 传感器与观测层                          ◄── 感知系统  │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐  │ │
│  │  │ Camera   │ │ Joint    │ │ Contact  │ │ Observation│  │ │
│  │  │ Sensor   │ │ Sensor   │ │ Sensor   │ │ Builder    │  │ │
│  │  │ RGB+D+Seg│ │ 位置+速度│ │ 接触力   │ │ 标准化字典  │  │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └────────────┘  │ │
│  └──────────────────────────────────────────────────────────┘ │
│                              │                                  │
│  Layer 4: MCP 协议层                              ◄── 统一接口  │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │  McpSimBridge (仿真端)          McpCloudBridge (真机端)    │ │
│  │  ┌────────────────────┐       ┌────────────────────┐    │ │
│  │  │ 12 ElectronBot 工具│       │ 云端小智 API 透传  │    │ │
│  │  │ tools/call 协议    │       │ type:"mcp" 封装    │    │ │
│  │  │ 舵机→关节转换      │       │ 8 个预设动作工具   │    │ │
│  │  └────────────────────┘       └────────────────────┘    │ │
│  │                                                           │ │
│  │  WebSocket Server (:8080)      WebSocket Client → 云端    │ │
│  │  (仿真调试用)                  (ESP32 连接云后台)          │ │
│  └──────────────────────────────────────────────────────────┘ │
│                              │                                  │
│  Layer 3: 动作系统层                              ◄── 运动控制  │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │  ElectronBotActions                                        │ │
│  │  ├── 预设动作: 举手/挥手/拍打/转头/点头/转身 (12种)        │ │
│  │  ├── 单舵机控制: servo_move (线性插值，对齐固件)            │ │
│  │  ├── 序列执行: servo_sequences (普通+振荡模式)             │ │
│  │  ├── 安全裁剪: ClampServoTarget (6 组硬限位)               │ │
│  │  └── 振荡器: OscillateServos (正弦插值, 50ms 采样)         │ │
│  └──────────────────────────────────────────────────────────┘ │
│                              │                                  │
│  Layer 2: 物理仿真引擎                            ◄── 物理核心  │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │  MuJoCo 3.x                                               │ │
│  │  ├── 刚体动力学 (7 body, 24 零件)                         │ │
│  │  ├── 6 铰链关节 (hinge, 含映射比 gear)                     │ │
│  │  ├── 6 位置执行器 (50Hz, kp 校准)                         │ │
│  │  ├── 接触力学 (碰撞检测, 接触力)                           │ │
│  │  ├── 摄像头渲染 (RGB/Depth/Segmentation) — 仿真专属        │ │
│  │  └── 域随机化 (阻尼/质量/摩擦)                             │ │
│  │                                                            │ │
│  │  渲染后端: OpenGL (本地) / EGL (headless 训练)              │ │
│  └──────────────────────────────────────────────────────────┘ │
│                              │                                  │
│  Layer 1: 模型描述层                              ◄── 建模基础  │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │  STEP (30.5MB) → FreeCAD 拆解 → URDF → MJCF (MuJoCo XML) │ │
│  │  ├── 24 个 3D 打印零件几何体                               │ │
│  │  ├── 惯性参数 (PLA 1.24g/cm³ + 电子件 60g)                │ │
│  │  ├── 6 关节旋转中心 (CAD 坐标)                             │ │
│  │  ├── 舵机→机械关节映射比 (1.0/1.125/1.5/2.0)              │ │
│  │  └── 简化碰撞几何体 (<200 面凸包)                          │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 层间数据流

```
Layer 1 (MJCF) ──mjModel──→ Layer 2 (MuJoCo)
                                  │
                          ctrl[]  │  qpos[], sensor[]
                                  ▼
Layer 2 ───────────────────→ Layer 3 (动作系统)
  物理状态                        │
                                  │ servo_move / hand_action 等
                                  ▼
Layer 3 ───────────────────→ Layer 4 (MCP Bridge)
  方法调用                        │
                                  │ JSON-RPC (tools/call)
                                  ▼
Layer 4 ───────────────────→ Layer 5 (传感器)
  MCP 响应                        │
                                  │ obs dict {joint_pos, image, ...}
                                  ▼
Layer 5 ───────────────────→ Layer 6 (AI 策略)
  观测字典                        │
                                  │ action (6D增量 / MCP命令)
                                  ▼
Layer 6 ───────────────────→ Layer 7 (评估+部署)
  策略推理
```

### 1.3 架构设计原则

1. **MCP 协议是唯一分界线**：AI 策略层只发送 MCP JSON-RPC 命令，完全不感知下面是仿真还是真机
2. **仿真内可观测更多，但 Sim2Real 策略只用真机可得数据**：提供 `obs_mode="full"` 和 `obs_mode="realistic"` 两种模式
3. **以真机固件为准**：仿真行为 1:1 对齐真实固件源码，不做"优化"或"美化"
4. **渐进式 AI 管线**：BC → RL → VLA，难度递增，仿真验证后 Sim2Real

---

## 2. 模块划分

### 2.1 模块总览

```
ElectronBot_SIM/
├── assets/                     # Layer 1 — 模型描述层
│   ├── cad/                    #   原始 CAD 模型 (STEP/FCStd)
│   ├── meshes/                 #   24 个简化 STL 网格
│   └── mjcf/                   #   MuJoCo 模型文件 + 场景文件
│
├── src/
│   ├── electronbot_sim/        # Layer 2+3+4 — 仿真核心 (无 AI 依赖)
│   │   ├── env.py              #   Gymnasium 环境 (MuJoCo 封装)
│   │   ├── mcp_bridge.py       #   Layer 4 — MCP 仿真桥接器
│   │   ├── mcp_server.py       #   Layer 4 — 仿真 WebSocket 服务器 (调试用)
│   │   ├── backend.py          #   Layer 4 — 统一 Backend API (sim↔cloud)
│   │   ├── actions/            #   Layer 3 — 动作系统
│   │   │   └── __init__.py     #     ElectronBotActions 类
│   │   ├── sensors/            #   Layer 5 — 传感器模块
│   │   │   ├── camera.py       #     CameraSensor (RGB+D+Seg, 仿真专属)
│   │   │   ├── joint.py        #     JointSensor
│   │   │   └── contact.py      #     ContactSensor
│   │   ├── observation.py      #   Layer 5 — 观测构建 (full / realistic)
│   │   └── domain_randomizer.py#   Layer 2 — 域随机化工具
│   │
│   ├── electronbot_ai/         # Layer 6 — AI 训练管线 (依赖 electronbot_sim)
│   │   ├── il/                 #   模仿学习
│   │   │   ├── collect_demos.py
│   │   │   ├── train_bc.py
│   │   │   └── train_act.py
│   │   ├── rl/                 #   强化学习
│   │   │   ├── train_ppo.py
│   │   │   ├── parallel_env.py
│   │   │   └── domain_randomization.py
│   │   ├── vla/                #   文本语言动作规划
│   │   │   └── llm_planner.py  #   LLM → MCP 序列 (纯文本/语音输入)
│   │   └── tasks/              #   任务定义
│   │       ├── base.py
│   │       ├── reach.py
│   │       ├── push.py
│   │       ├── pick_place.py
│   │       ├── stack.py
│   │       ├── follow.py
│   │       └── gesture.py
│   │
│   ├── electronbot_benchmark/  # Layer 7 — 评估系统
│   │   ├── suite.py
│   │   ├── run.py
│   │   └── report.py
│   │
│   └── electronbot_sim2real/   # Layer 7 — Sim2Real 部署
│       ├── deploy_cloud.py     #   模式A: 云端 API 透传 (release v2.2.6)
│       ├── deploy_onnx.py      #   模式B: ONNX 推理部署
│       ├── deploy_websocket.py #   模式C: WebSocket 直连 (需固件 OTA)
│       └── calibrate.py        #   真机校准工具
│
├── scripts/                    # 工具脚本
│   ├── calc_inertia.py
│   ├── cad_to_urdf.py
│   ├── validate_model.py
│   └── flash_firmware.sh
│
├── tests/
├── web/                        # Three.js 3D 可视化前端
├── docker/                     # 容器化
├── docs/
│   ├── tasks/                  #   8 阶段详细开发文档
│   └── 概要设计/               #   架构设计文档
├── checkpoints/
├── demos/
├── results/
└── pyproject.toml
```

### 2.2 模块职责矩阵

| 模块 | 层 | 职责 | 依赖 | 独立可测 |
|------|:---:|------|------|:---:|
| `assets/` | L1 | 3D 模型资产管理 | CAD 工具 | ✅ |
| `electronbot_sim.env` | L2 | Gymnasium RL 环境 | MuJoCo | ✅ |
| `electronbot_sim.actions` | L3 | 6 关节动作执行 | env (ctrl) | ✅ |
| `electronbot_sim.mcp_bridge` | L4 | MCP JSON-RPC 仿真工具 | actions | ✅ |
| `electronbot_sim.backend` | L4 | 统一 sim/cloud API | mcp_bridge | ✅ |
| `electronbot_sim.sensors` | L5 | 物理量传感器 | env (mjData) | ✅ |
| `electronbot_sim.observation` | L5 | 观测组装 | sensors | ✅ |
| `electronbot_ai.il` | L6 | 模仿学习训练 | env + actions | ✅ |
| `electronbot_ai.rl` | L6 | 强化学习训练 | env | ✅ |
| `electronbot_ai.vla` | L6 | 文本 LLM→MCP 规划 | mcp_bridge | ✅ |
| `electronbot_ai.tasks` | L6 | 奖励函数+终止条件 | env | ✅ |
| `electronbot_benchmark` | L7 | 标准化评估 | ai + env | ✅ |
| `electronbot_sim2real` | L7 | 策略部署到真机 | backend | ✅ |
| `web/` | L7 | Three.js 3D 可视化 | mcp_server(:8080) | ✅ |

### 2.3 模块依赖图

```
electronbot_benchmark ──→ electronbot_ai ──→ electronbot_sim ──→ MuJoCo
         │                       │                    │
         └───────→ electronbot_sim2real ←────────────┘
                          │
                  云端小智 API (HTTP/WS)
                          │
                   ESP32 真机 (MQTT/WS 客户端)
```

**依赖原则**：
- `electronbot_sim`：零 AI 依赖，可独立运行和测试
- `electronbot_ai`：只依赖 `electronbot_sim` 的环境和动作接口
- `electronbot_benchmark`：依赖 AI 模块 + 仿真模块
- `electronbot_sim2real`：依赖仿真模块的 MCP Bridge 接口 + 云端 API

---

## 3. 模块间接口

### 3.1 仿真环境接口 (Layer 2 → 外部)

```python
class ElectronBotEnv(gym.Env):
    """Gymnasium 标准 RL 环境"""
    
    # ── 动作空间 (Layer 2 → Layer 3/6) ──
    action_space: Box(low=-1.0, high=1.0, shape=(6,))
    # 6 关节增量控制，归一化到 [-1, 1]，内部映射到物理增量
    
    # ── 观测空间 (Layer 2 → Layer 5/6) ──
    # obs_mode="full" (仿真专属，研究用)
    observation_space_full = Dict({
        "joint_pos":   Box(low=-180, high=180, shape=(6,)),   # 关节角度 (°)
        "joint_vel":   Box(low=-np.inf, high=np.inf, shape=(6,)),  # 关节速度 (°/s)
        "ee_left_pos": Box(low=-np.inf, high=np.inf, shape=(3,)),   # 左末端位置 (m)
        "ee_right_pos":Box(low=-np.inf, high=np.inf, shape=(3,)),   # 右末端位置 (m)
        "image":       Box(0, 255, shape=(240,240,3), dtype=np.uint8),  # RGB (仿真专属)
        "depth":       Box(0, 255, shape=(240,240), dtype=np.uint8),     # 深度 (仿真专属)
    })
    
    # obs_mode="realistic" (Sim2Real 用，只含真机可获取数据)
    # 真机 SG90/2g/4.3g 舵机无编码器，不可得 joint_vel / ee_positions / camera
    observation_space_realistic = Dict({
        "commanded_joint_pos": Box(low=-180, high=180, shape=(6,)),  # 最后发出的角度指令
        "is_moving":           Box(low=0, high=1, shape=(1,)),       # 动作任务是否在执行
        "battery_voltage":     Box(low=3.0, high=4.2, shape=(1,)),   # 电池电压 (V)
        "battery_percent":     Box(low=0, high=100, shape=(1,)),     # 估算电量百分比
    })
    
    # ── 核心方法 ──
    def reset(self, *, seed=None, options=None) -> tuple[ObsType, dict]:
        """重置环境到初始状态，触发域随机化"""
    
    def step(self, action: np.ndarray) -> tuple[ObsType, float, bool, bool, dict]:
        """执行动作，返回 (obs, reward, terminated, truncated, info)"""
    
    def render(self) -> np.ndarray | None:
        """渲染当前帧 (仿真专属)"""
```

### 3.2 动作系统接口 (Layer 3 → Layer 4)

```python
class ElectronBotActions:
    """动作系统——1:1 对齐真机固件 movements.cc"""
    
    def __init__(self, env: ElectronBotEnv):
        self.env = env
    
    # ── 预设动作 (与真机 8 个 MCP 工具对应) ──
    def hand_action(self, action: int, hand: int, steps: int,
                    speed: int, amount: int = 30) -> dict:
        """
        action: 1=举手, 2=放手, 3=挥手, 4=拍打
        hand: 1=左手, 2=右手, 3=双手
        steps: 重复次数 (真机: times=2*max(3,min(100,steps)))
        speed: 动作速度 (ms), 越小越快
        amount: 动作幅度 (10-50), 仅举手使用
        → 返回 {"status": "ok"}
        """
    
    def body_turn(self, direction: int, speed: int,
                  angle: int, steps: int = 1) -> dict:
        """
        direction: 1=左转, 2=右转, 3=回中心
        angle: 转动角度 (0-90°)
        """
    
    def head_move(self, action: int, speed: int,
                  angle: int, steps: int = 1) -> dict:
        """
        action: 1=抬头, 2=低头, 3=点头, 4=回中心, 5=连续点头
        angle: 头部角度 (1-15°)
        """
    
    def stop(self) -> dict:
        """清空动作队列 + 复位"""
    
    def home(self) -> dict:
        """复位到初始姿态 [180,180,0,0,90,90] + trim"""
    
    # ── 舵机级控制 (仿真专属) ──
    def servo_move(self, servo_type: str, position: float,
                   speed: int = 1000) -> dict:
        """
        单舵机精确定位
        servo_type: rp/rr/lp/lr/b/h
        position: 目标角度 (°), 自动裁剪到安全范围
        interpolation: "linear" (对齐固件)
        """
    
    def servo_sequences(self, sequence: str) -> dict:
        """
        执行 AI 生成的动作序列
        sequence: JSON 字符串 {"a":[{"s":{...},"v":...}, ...]}
        """
    
    # ── 内部方法 ──
    def _move_servos(self, targets: np.ndarray, time_ms: int):
        """
        6 舵机联动缓动，插值方式: linear (对齐固件 MoveServos)
        
        真机固件实现 (movements.cc:87):
          increment_[i] = (target[i] - pos[i]) / (time / 10.0)
          每个 10ms 步进等量增加 → 纯线性插值
        """
    
    def _oscillate(self, amplitudes, centers, period, cycles):
        """
        正弦振荡模式 (对齐固件 OscillateServos)
        """
```

### 3.3 MCP Bridge 接口 (Layer 4 → 外部)

```python
class McpSimBridge:
    """
    仿真端 MCP JSON-RPC 桥接器
    实现与真机固件完全一致的 MCP 协议
    
    真机协议结构 (type:"mcp" 封装):
    {
      "type": "mcp",
      "payload": {
        "jsonrpc": "2.0",
        "method": "tools/call",              ← 固定值
        "params": {
          "name": "self.electron.hand_action", ← 工具名在这里
          "arguments": { "action": 3, ... }     ← 参数在这里
        },
        "id": 3
      }
    }
    """
    
    def __init__(self, actions: ElectronBotActions):
        self.actions = actions
        self._register_tools()
    
    def handle_request(self, request: dict) -> dict:
        """
        处理 JSON-RPC 请求，支持两种格式：
        
        1. 标准 MCP 格式 (与真机一致):
           {"method":"tools/call","params":{"name":"self.electron.xxx","arguments":{...}}}
        
        2. 扁平格式 (仿真内部调试):
           {"method":"self.electron.xxx","params":{...}}
        """
    
    # ── 注册的 12 个工具 ──
    # 真机对齐 (8个): hand_action, body_turn, head_move, stop,
    #                 get_status, set_trim, get_trims, battery.get_level
    # 仿真专属 (4个): servo_move, servo_sequences, home, get_ip
```

### 3.4 统一 Backend API (Layer 4 → Layer 6/7)

```python
class ElectronBotBackend:
    """
    统一后端 API——AI 策略通过此类访问机器人，
    不感知下面是仿真还是真机。
    """
    
    def __init__(self, mode: Literal["sim", "cloud"], **kwargs):
        """
        mode="sim":  连接仿真 MCP Bridge (本地 MuJoCo)
        mode="cloud": 连接云端小智 API (真机 ESP32)
        
        kwargs for cloud: api_url, device_id, api_key
        """
    
    def call(self, method: str, params: dict) -> dict:
        """
        调用 MCP 工具——仿真和真机完全相同的调用方式
        
        Example:
          # 仿真
          backend = ElectronBotBackend("sim")
          backend.call("self.electron.hand_action", {"action":3,"hand":3,"steps":2,"speed":600})
          
          # 真机 (通过云端 API)
          backend = ElectronBotBackend("cloud", api_url="https://api.xiaozhi.cn/v1",
                                       device_id="eb-001")
          backend.call("self.electron.hand_action", {"action":3,"hand":3,"steps":2,"speed":600})
        """
    
    async def call_async(self, method: str, params: dict) -> dict:
        """异步版本，用于高并发场景"""
```

### 3.5 云端 Sim2Real Bridge 接口 (Layer 7 → ESP32)

```python
class McpCloudBridge:
    """
    真机云端 MCP 桥接器
    ——通过小智云端 API 透传 MCP 命令到 ESP32 真机
    
    通信链路:
    Python → HTTPS → 小智云端后台 → MQTT/WebSocket → ESP32 真机
    """
    
    def __init__(self, api_url: str, device_id: str, api_key: str = None):
        self.api_url = api_url
        self.device_id = device_id
    
    async def call(self, tool_name: str, arguments: dict) -> dict:
        """
        通过云端 API 调用真机 MCP 工具
        → POST {api_url}/devices/{device_id}/tools
        → Body: {"name": tool_name, "arguments": arguments}
        → 返回: {"result": {...}} 或 {"error": {...}}
        """
    
    async def list_tools(self) -> list[dict]:
        """获取真机当前注册的工具列表"""
    
    async def get_device_status(self) -> dict:
        """获取设备连接状态"""
```

---

## 4. 技术选型

### 4.1 选型总览

| 层 | 选型 | 版本 | 选择理由 |
|----|------|:---:|------|
| **物理引擎** | **MuJoCo** | ≥ 3.2.0 | 免费开源、C 核心高性能、Python 原生绑定、Google 维护 |
| **RL 框架** | **Stable-Baselines3 → cleanRL → Isaac Lab** | — | 渐进式：入门→进阶→工业级 |
| **IL 框架** | **robomimic + ACT** | — | 学术界标准 HDF5 数据格式，ACT 官方实现 |
| **语言** | **Python 3.11** | ≥ 3.11 | MuJoCo binding 性能足够，AI 生态完整 |
| **3D 可视化** | **Three.js (Web)** | — | 浏览器零安装，跨平台 |
| **行为树** | **py_trees** | — | Python 原生，ROS2 集成成熟 |
| **推理引擎** | **ONNX Runtime** | — | 跨框架模型部署，CPU/GPU 通用 |
| **Async HTTP** | **httpx** | — | 云端 API 调用，支持 HTTP/2 |
| **容器化** | **Docker (GPU 训练)** | — | 环境一致性 |

### 4.2 备选方案评估

| 功能 | 推荐方案 | 备选方案 | 未选理由 |
|------|----------|----------|----------|
| 物理引擎 | MuJoCo | Isaac Sim | 需 NVIDIA GPU，闭源组件多 |
| | | PyBullet | 物理精度较低 |
| CAD→URDF | FreeCAD + yourdfpy | Blender + Phobos | Phobos 插件不稳定 |
| RL 训练 | Stable-Baselines3 | RLlib | 依赖重，调试复杂 |
| 大模型 | Qwen2.5 (本地/云端) | GPT-4V (API) | 本地推理零成本、可离线 |
| VLA 输入 | **纯文本/语音** (真机可用) | 视觉 VLA (仅仿真) | 真机无摄像头 |
| 可视化 | Three.js (Web) | RViz2 | 需安装 ROS2 环境 |

### 4.3 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 运动插值 | **线性插值** (非 EaseOutCubic) | 对齐真机 `MoveServos()` 行为 (movements.cc:87) |
| MCP 协议 | `tools/call` 两层嵌套 | 对齐小智 MCP 协议规范 |
| 真机通信 | **云端 API 透传** (非 WebSocket 直连) | release v2.2.6 ESP32 无 WebSocket Server |
| 观测模式 | `full` + `realistic` 双模式 | full 用于研究，realistic 用于 Sim2Real |
| VLA 模式 | **纯文本 VLA** (无摄像头) | 真机 ElectronBot 无摄像头硬件 |
| 动作实现 | `ElectronBotActions` 为单一来源 | 消除 MCP Bridge 与 Actions 的重复实现 |

### 4.4 开发环境

#### 实际开发机 (maple)

| 配置项 | 规格 |
|--------|------|
| **CPU** | Intel Core i9-11900 @ 2.50GHz (8核16线程, L3 16MB) |
| **RAM** | 64GB DDR4 |
| **GPU** | NVIDIA GeForce RTX 2060 12GB (TU106, 184W) |
| **CUDA** | 13.2 / Driver 595.71.05 |
| **存储** | 2× NVMe 465GB + 1× HDD 1.8TB |
| **OS** | Ubuntu 22.04.5 LTS (kernel 6.8) |

#### 最低要求

| 配置项 | 最低 |
|--------|------|
| **CPU** | 4 核 (i5/R5) |
| **RAM** | 16GB |
| **GPU** | 无 (CPU 训练也可) |
| **存储** | 50GB 空闲 |
| **OS** | Ubuntu 22.04 / Win11+WSL2 |

#### 环境搭建

```bash
# 一键环境搭建 (Ubuntu 22.04, CUDA 13.2)
conda create -n ebotsim python=3.11 -y && conda activate ebotsim

# 核心依赖
pip install mujoco gymnasium stable-baselines3 robomimic

# PyTorch with CUDA 13.2 (适用于 RTX 2060 / RTX 3060 / RTX 4090)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124

# 其他工具
pip install numpy scipy opencv-python yourdfpy websockets py_trees onnxruntime httpx
```

---

## 5. 部署拓扑

### 5.1 开发拓扑

```
┌─────────────────────────────────────────────────────────────────────┐
│                 开发工作站: maple@maple-B560-HD3                      │
│                 Ubuntu 22.04 | i9-11900 | RTX 2060 12GB | 64GB RAM   │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  Conda env: ebotsim (Python 3.11)                               │ │
│  │  ├── MuJoCo 3.x (物理仿真)                                       │ │
│  │  ├── Gymnasium (RL 环境)                                         │ │
│  │  ├── SB3 / robomimic (RL/IL 训练)                                │ │
│  │  ├── PyTorch + CUDA 13.2 (GPU 加速，RTX 2060 12GB)              │ │
│  │  ├── McpSimBridge (MCP 仿真桥接)                                  │ │
│  │  └── WebSocket Server :8080 (仿真调试)                            │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  ESP-IDF 5.x (固件编译，按需)                                    │ │
│  │  └── xiaozhi-esp32-2.2.6 → build/flash                          │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  LLM 推理 (本地 GPU)                                             │ │
│  │  └── Qwen2.5 (RTX 2060 12GB VRAM 可运行 7B 量化版)              │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  存储:                                                           │ │
│  │  ├── /dev/nvme0n1 (465GB): 数据集 + 模型权重                     │ │
│  │  └── /dev/sda (1.8TB): 大文件归档                                │ │
│  └────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.2 训练拓扑（本地 GPU）

```
┌─────────────────────────────────────────────────────────────────────┐
│  maple@maple-B560-HD3 — RTX 2060 12GB                               │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  PPO / SAC Trainer (PyTorch + CUDA 13.2)                      │   │
│  │  GPU: 策略更新 + 前向推理 (RTX 2060 12GB)                      │   │
│  └──────────┬───────────────────────────────────────────────────┘   │
│             │ rollout                                                │
│  ┌──────────▼───────────────────────────────────────────────────┐   │
│  │  16-32× 并行 MuJoCo 环境 (SubprocVecEnv)                       │   │
│  │  每环境独立进程，EGL headless 渲染                              │   │
│  │  域随机化: 每次 reset 注入物理参数噪声                          │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
│  并行度建议: RTX 2060 12GB 可运行 16-32 并行环境                    │
│  监控: TensorBoard (localhost:6006)                                  │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.3 Sim2Real 部署拓扑

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Sim2Real 部署拓扑                              │
│                                                                       │
│  模式 A: 云端 API 透传 (release v2.2.6, 当前可用)                    │
│  ⚠️ 延迟: 200-500ms RTT (HTTPS→云端→MQTT/WS→ESP32)                  │
│  ┌──────────────┐    HTTPS      ┌────────────────┐    MQTT/WS       │
│  │ Python 策略   │ ──────────→  │ 小智云端后台    │ ──────────→     │
│  │ (McpCloudBridge)│ ~100ms    │ (ASR→LLM→MCP) │  ~100-400ms    │
│  └──────────────┘              └────────────────┘                   │
│                                                         ▼            │
│                                              ┌──────────────────┐    │
│                                              │ ESP32-S3 真机    │    │
│                                              │ 8 个预设动作工具   │    │
│                                              └──────────────────┘    │
│  适用: VLA 语音控制、预设动作序列 (延迟可接受)                         │
│  ⚠️ PPO@50Hz 策略不可直接部署通过此路径 (步长20ms << 延迟200ms+)     │
│                                                                       │
│  模式 C: WebSocket 直连 (需固件 OTA 升级, 未来可用)                   │
│  ⚠️ 延迟: <10ms RTT (局域网直连), 但需固件增加 WS Server + servo_move │
│  ┌──────────────┐    ws://IP:8080/ws    ┌──────────────────────┐     │
│  │ Python 策略   │ ─────────────────→   │ ESP32-S3 (OTA 后)    │     │
│  │ (McpWsBridge) │                      │ 8+4 个 MCP 工具      │     │
│  └──────────────┘                      │ 含 servo_move/host   │     │
│                                         └──────────────────────┘     │
│  适用: 需要低延迟闭环的 RL/IL 策略 (PPO ONNX 本地推理)                 │
│                                                                       │
│  模式 D: ONNX 本地推理 (需固件 OTA + ESP32-S3 算力评估)               │
│  ┌──────────────┐                      ┌──────────────────────┐     │
│  │ 仿真训练策略   │ ──ONNX导出+OTA──→   │ ESP32-S3 真机        │     │
│  │ (PPO MLP)    │                      │ SPI Flash 加载 ONNX  │     │
│  └──────────────┘                      │ 本地推理@~10ms        │     │
│                                         └──────────────────────┘     │
│  适用: MLP 策略 (<500KB ONNX), ESP32-S3 可行; Transformer 不可行      │
│                                                                       │
│  能力对照:                                                             │
│  ┌──────────────┬──────────────────┬──────────────────────┐         │
│  │   能力        │  仿真             │  真机 (release v2.2.6)│         │
│  ├──────────────┼──────────────────┼──────────────────────┤         │
│  │ 预设动作 (8)  │  ✅ 与真机一致     │  ✅ 原生支持          │         │
│  │ servo_move   │  ✅ 线性插值       │  ❌ 需固件 OTA        │         │
│  │ servo_seq    │  ✅ 序列+振荡      │  ❌ 需固件 OTA        │         │
│  │ 摄像头       │  ✅ MuJoCo渲染     │  ❌ 无硬件            │         │
│  │ 关节反馈     │  ✅ 精确 qpos      │  ❌ 无编码器          │         │
│  │ WebSocket    │  ✅ :8080 调试     │  ❌ 需固件 OTA        │         │
│  │ 控制路径     │  本地进程内调用     │  云端 API 透传       │         │
│  └──────────────┴──────────────────┴──────────────────────┘         │
└──────────────────────────────────────────────────────────────────────┘
```

### 5.4 云端 API 通信流程

```
                    GET /devices/{id}/tools
  Python ─────────────────────────────────→ 小智云端
         ←─────────────────────────────────  {tools: [{name, schema},...]}
         
                    POST /devices/{id}/tools/call
                    {name: "self.electron.hand_action", arguments: {...}}
  Python ─────────────────────────────────→ 小智云端
                                           │
                    type:"mcp", type:"mcp" │ MQTT/WebSocket
                    payload:{              │
                      method:"tools/call", │
                      params:{             ▼
                        name:"...",     ESP32
                        arguments:{...}
                      }
                    }
         ←─────────────────────────────────  {content: [{type:"text", text:"..."}], isError: false}
```

---

## 6. MCP 统一接口

### 6.1 协议格式 (以真机固件为准)

MCP 消息封装在基础通信协议消息体中，遵循 JSON-RPC 2.0 规范：

```json
// 基础消息封装 (WebSocket / MQTT)
{
  "session_id": "...",
  "type": "mcp",
  "payload": {
    // ←─ 标准 JSON-RPC 2.0 ─→
    "jsonrpc": "2.0",
    "method": "tools/call",        // ← 固定为 "tools/call"
    "params": {
      "name": "self.electron.hand_action",   // ← 工具名在这里
      "arguments": {                         // ← 参数在这里
        "action": 3,
        "hand": 3,
        "steps": 2,
        "speed": 600
      }
    },
    "id": 3
  }
}

// 成功响应
{
  "type": "mcp",
  "payload": {
    "jsonrpc": "2.0",
    "id": 3,
    "result": {
      "content": [
        {"type": "text", "text": "true"}
      ],
      "isError": false
    }
  }
}
```

### 6.2 仿真内部兼容处理

仿真 MCP Bridge 同时支持两种格式：

```python
def handle_request(self, request: dict) -> dict:
    method = request.get("method", "")
    
    # 标准 MCP 格式 (tools/call 嵌套) —— 与真机一致
    if method == "tools/call":
        tool_name = request["params"]["name"]
        tool_args = request["params"].get("arguments", {})
        result = self._call_tool(tool_name, tool_args)
        return {
            "jsonrpc": "2.0", "id": request.get("id"),
            "result": {
                "content": [{"type": "text", "text": str(result)}],
                "isError": False
            }
        }
    
    # 扁平格式 —— 仿真内部调试/测试用
    tool_name = method
    tool_args = request.get("params", {})
    result = self._call_tool(tool_name, tool_args)
    return {
        "jsonrpc": "2.0", "id": request.get("id"),
        "result": result
    }
```

### 6.3 MCP 工具映射表

| 工具名 | 真机 v2.2.6 | 仿真 | 类型 | 说明 |
|--------|:---:|:---:|------|------|
| `self.electron.hand_action` | ✅ | ✅ | 预设 | 举手/放手/挥手/拍打 (4种×3手=12子动作) |
| `self.electron.body_turn` | ✅ | ✅ | 预设 | 左转/右转/回中心 |
| `self.electron.head_move` | ✅ | ✅ | 预设 | 抬头/低头/点头/回中心/连续点头 |
| `self.electron.stop` | ✅ | ✅ | 系统 | 清空队列+复位 |
| `self.electron.get_status` | ✅ | ✅ | 系统 | 返回 "moving" / "idle" |
| `self.electron.set_trim` | ✅ | ✅ | 校准 | 设置指定舵机偏移 (NVS 保存) |
| `self.electron.get_trims` | ✅ | ✅ | 校准 | 读取 6 舵机 trim 值 |
| `self.battery.get_level` | ✅ | ✅ | 系统 | 电量和充电状态 |
| `self.electron.servo_move` | ❌ | ✅ | @sim_only | 单舵机精确定位 (线性插值) |
| `self.electron.servo_sequences` | ❌ | ✅ | @sim_only | AI 生成的动作序列+振荡 |
| `self.electron.home` | ❌ | ✅ | @sim_only | 显式复位命令 |
| `self.electron.get_ip` | ❌ | ✅ | @sim_only | 设备 IP 查询 |

> `@sim_only` 标记的工具在真机 release v2.2.6 上不可用。其作用：
> - 验证更复杂的 AI 策略
> - 作为"能力预览"，等待固件 OTA 升级
> - 在仿真中进行更细粒度验证
>
> Sim2Real 降级策略：仿真中产出的序列优先转为 8 个预设动作组合；无法转换的标记为"仿真验证通过，等待固件升级"。

### 6.4 统一 Backend API

```python
# ── 仿真模式 ──
backend = ElectronBotBackend("sim")
result = backend.call("self.electron.hand_action", {
    "action": 3, "hand": 3, "steps": 2, "speed": 600
})

# ── 真机模式（通过云端 API）── 仅改模式参数
backend = ElectronBotBackend("cloud",
    api_url="https://api.xiaozhi.cn/v1",
    device_id="eb-001")
result = backend.call("self.electron.hand_action", {
    "action": 3, "hand": 3, "steps": 2, "speed": 600
})
# ^^^ 完全相同的调用方式！
```

---

## 7. Sim2Real 全链路

### 7.1 域随机化

| 随机化参数 | 范围 | 物理含义 |
|-----------|:---:|------|
| 关节摩擦 | ±50% | 3D 打印件公差 + 润滑差异 |
| 关节阻尼 | ±20% | 舵机内部齿轮阻尼 |
| 执行器增益 | ±10% | SG90/2g 舵机个体差异 |
| 零件质量 | ±15% | PLA 打印密度不一致 |
| 观测噪声 | 0-0.5° | 无编码器反馈的开环漂移 |

### 7.2 仿真精确度保证 (以固件源码为准)

| 措施 | 对标真机行为 | 固件源码位置 |
|------|-------------|-------------|
| **线性插值** `(target-pos)/(time/10.0)` 每 10ms 步进 | 复现 `MoveServos()` | movements.cc:87 |
| 6 组硬限位裁剪 `ClampServoTarget()` | 复现安全范围检查 | electron_bot_controller.cc |
| 舵机→关节映射比 (1.0/1.125/1.5/2.0) | 固件安全范围↔CAD机械范围 | config.h + CAD |
| 50Hz 控制频率 + 10ms 步进 | 对标 LEDC PWM + Otto 插值 | movements.cc:91-98 |
| 正弦振荡 `sin(phase)` + 50ms 采样 (固件 `vTaskDelay(5)`, 100Hz tick → 50ms) | 复现 `OscillateServos()` | movements.cc:147+ |
| trim 偏置 6 维向量 | 对标 `SetTrims()` | movements.cc:59-73 |
| 动作队列 `xQueueReceive` 逐帧执行 | 对标 ActionTask | electron_bot_controller.cc:71-101 |

### 7.3 舵机→机械关节映射表

| 关节 | 舵机安全范围 | 中心 | CAD 机械范围 | 映射比 | 方向 |
|------|:---:|:---:|:---:|:---:|:---:|
| HEAD | 75°~105° | 90° | ±30° | **2.0** | 正向 |
| BODY | 30°~150° | 90° | ±90° | **1.5** | 正向 |
| RIGHT_PITCH | 0°~180° | 180→0 | ±90° | **1.0** | 反向 |
| LEFT_PITCH | 0°~180° | 0→180 | ±90° | **1.0** | 正向 |
| RIGHT_ROLL | 100°~180° | 140 | ±45° | **1.125** | 反向 |
| LEFT_ROLL | 0°~80° | 40 | ±45° | **1.125** | 正向 |

### 7.4 Sim2Real 部署流程

```
Step 1: 仿真验证
  策略在 MuJoCo 中通过全部 Benchmark 任务

Step 2: 能力降级 (按需)
  如果策略使用了 servo_move/servo_sequences:
  → 降级为 8 个预设动作组合
  → 或标记"等待固件 OTA"

Step 3: 云端部署 (模式A)
  python -m electronbot_sim2real.deploy_cloud \
      --policy checkpoints/bc_wave.pt \
      --device-id eb-001 \
      --api-url https://api.xiaozhi.cn/v1

Step 4: 校准 (首次部署)
  python -m electronbot_sim2real.calibrate --device-id eb-001
  → trim 保存到 ESP32 NVS

Step 5: 效果验证
  → 录真机执行视频
  → 与 MuJoCo 仿真录制并排对比
  → 确认 8 个预设动作轨迹一致

Step 6: Benchmark (可选)
  python -m electronbot_benchmark.run --mode cloud --device-id eb-001
```

---

## 8. 真机对接

### 8.1 真机硬件规格

| 组件 | 型号 |
|------|------|
| 主控 | ESP32-S3-WROOM-N16R8 |
| 固件 | **xiaozhi-esp32 release v2.2.6** |
| 舵机 | SG90 9G ×1 + 2g ×4 + 4.3g ×1 |
| 屏幕 | GC9A01 240×240 圆形 LCD |
| 音频 | ICS-43434 麦克风 + MAX98357A + 2030-4R3W 喇叭 |
| 电池 | 103030 1000mAh / USB 供电 |
| **摄像头** | **无** (真机不支持视觉 VLA) |
| MCP 工具 | 8 个预设动作工具 |
| 通信 | 云端 MQTT/WebSocket (ESP32 作为客户端连接到小智云端后台) |
| 本地端口 | **无 WebSocket Server** (release v2.2.6 无 :8080 端点) |

### 8.2 关键差异：仿真 vs 真机

| 维度 | 仿真 | 真机 (release v2.2.6) |
|------|------|------|
| 控制精度 | 精确角度控制 (servo_move) | 仅预设动作 (hand/body/head) |
| 传感器反馈 | 关节角度/速度/接触力/RGB-D | **无编码器反馈** (开环) |
| 视觉输入 | MuJoCo 摄像头渲染 | **无摄像头** |
| 通信接口 | WebSocket :8080 (仿真调试) | 云端 API 透传 |
| 动作执行 | 仿真物理步进 | LEDC PWM → 物理舵机 |
| 域随机化 | 摩擦/阻尼/质量/噪声 | — |

### 8.3 仿真↔真机代码切换

```python
# 仿真
backend = ElectronBotBackend("sim")

# 真机 (云端 API) —— 仅改模式参数
backend = ElectronBotBackend("cloud",
    api_url="https://api.xiaozhi.cn/v1",
    device_id="eb-001")

# 以下代码完全不变
result = backend.call("self.electron.hand_action", {
    "action": 3, "hand": 3, "steps": 2, "speed": 600
})
```

### 8.4 Sim2Real 分层部署路径

基于固件 `release v2.2.6` 的实际能力，推荐分层部署：

```
┌─────────────────────────────────────────────────────────────────┐
│  L1 立即可部署 (云端 API)                                        │
│  ├── VLA 语音控制 (Qwen2.5 → 预设动作序列)  ← 最可行路径         │
│  ├── 预设动作执行 (hand_action/body_turn/head_move)              │
│  └── 状态查询与校准 (get_status/set_trim)                        │
├─────────────────────────────────────────────────────────────────┤
│  L2 短期可部署 (需固件 OTA)                                      │
│  ├── 添加 MCP servo_move 工具 (原始角度控制)                      │
│  ├── 添加 MCP servo_sequence 工具 (序列执行)                      │
│  └── 降低 action task 优先级 (修复运动时音频卡顿)                  │
├─────────────────────────────────────────────────────────────────┤
│  L3 中期可部署 (需 WebSocket 直连固件)                            │
│  ├── ONNX 推理部署 (MLP 策略, ESP32-S3 可行)                     │
│  ├── 低延迟 MCP 直连 (<10ms RTT)                                 │
│  └── 半闭环控制 (基于指令值+时间戳的开环估计)                       │
├─────────────────────────────────────────────────────────────────┤
│  L4 远期 (需硬件升级)                                            │
│  ├── 带编码器的智能舵机 (真闭环)                                   │
│  ├── 摄像头集成 (视觉反馈)                                        │
│  └── 真闭环控制 + ACT 本地推理                                    │
└─────────────────────────────────────────────────────────────────┘
```

### 8.5 已知硬件限制 (Sim2Real 关键约束)

以下硬件限制在仿真中未被完整建模，是 Sim2Real gap 的主要来源：

| 限制 | 影响 | 仿真状态 |
|------|------|:---:|
| **无编码器** — SG90/2g/4.3g 全部为开环 PWM | joint_vel/ee_positions 真机不可得，RL 策略本质"盲操" | `obs_mode="realistic"` 已排除 |
| **无摄像头** — xiaozhi 版无视觉传感器 | 不可用视觉 VLA，仅纯文本 VLA | 已标注 |
| **云端延迟** — 200-500ms RTT | PPO@50Hz 策略不可通过云端部署 | 拓扑图已标注 |
| **伺服死区** — 2-5° deadband | <5° 微调真机不响应 | 域随机化待加入 |
| **伺服扭矩限制** — SG90: 1.5kg·cm | 仿真策略可能超出实际扭矩 | MJCF 待加入 forcerange |
| **电池放电** — 6 舵机同时运动电压下降 | 运动速度/扭矩随电量变化 | 域随机化待加入 |
| **音频冲突** — action task P=23 抢占音频 | 运动时音频可能卡顿 | 固件侧优化 |
| **动作无法中断** — MoveServos 无 abort 机制 | stop 命令有 1-3s 延迟 | 仿真待模拟 |

---

## 附录 A: 6 关节参数速查

| 关节 | MCP 代号 | 索引 | GPIO | 安全范围 (°) | 初始 (°) | 映射比 | MuJoCo joint |
|------|----------|:---:|------|:---:|:---:|:---:|------|
| 右臂 Pitch | `rp` | 0 | GPIO 5 | 0-180 | 180 | 1.0 | `joint_rp` |
| 右臂 Roll | `rr` | 1 | GPIO 4 | 100-180 | 180 | 1.125 | `joint_rr` |
| 左臂 Pitch | `lp` | 2 | GPIO 7 | 0-180 | 0 | 1.0 | `joint_lp` |
| 左臂 Roll | `lr` | 3 | GPIO 15 | 0-80 | 0 | 1.125 | `joint_lr` |
| 身体 | `b` | 4 | GPIO 6 | 30-150 | 90 | 1.5 | `joint_body` |
| 头部 | `h` | 5 | GPIO 16 | 75-105 | 90 | 2.0 | `joint_head` |

## 附录 B: 安全角度裁剪

```python
servo_limits = {
    0: (0, 180),    # Right Pitch
    1: (100, 180),  # Right Roll
    2: (0, 180),    # Left Pitch
    3: (0, 80),     # Left Roll
    4: (30, 150),   # Body
    5: (75, 105),   # Head
}
```

## 附录 C: MCP 命令速查

```json
// ── 挥手 (8个预设动作, 真机+仿真共用) ──
{"type":"mcp","payload":{"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.hand_action","arguments":{"action":3,"hand":3,"steps":2,"speed":600}},"id":1}}

// ── 身体左转 ──
{"type":"mcp","payload":{"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.body_turn","arguments":{"direction":1,"speed":800,"angle":30}},"id":2}}

// ── 点头 ──
{"type":"mcp","payload":{"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.head_move","arguments":{"action":3,"steps":2,"speed":600,"angle":10}},"id":3}}

// ── 单舵机定位 (仿真专属) ──
// 仿真扁平格式: {"method":"self.electron.servo_move","params":{"servo_type":"rp","position":120,"speed":800}}
// 或标准格式: {"method":"tools/call","params":{"name":"self.electron.servo_move","arguments":{"servo_type":"rp","position":120,"speed":800}}}

// ── 获取状态 ──
{"type":"mcp","payload":{"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.get_status","arguments":{}},"id":4}}
```
