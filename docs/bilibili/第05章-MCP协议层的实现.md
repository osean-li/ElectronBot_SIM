# 第五章 MCP 协议层的实现

> **核心问题**：如何使 AI 策略通过标准 JSON 命令控制机器人，且不感知底层是仿真还是真机？

## 5.1 章节目标

本章从概要设计文档的 Layer 4（MCP 协议层）出发，目标为：
1. 实现 `McpSimBridge`——仿真端 JSON-RPC 解析器
2. 支持标准 MCP 格式与扁平调试格式双协议
3. 实现 `ElectronBotBackend`——统一后端 API，一行切换 sim/cloud 模式
4. 实现 WebSocket 调试服务器

## 5.2 MCP 协议格式

MCP（Model Context Protocol）基于 JSON-RPC 2.0 规范，定义 AI 与机器人之间的标准通信协议。

### 5.2.1 请求格式

```json
{
  "type": "mcp",
  "payload": {
    "jsonrpc": "2.0",
    "method": "tools/call",
    "params": {
      "name": "self.electron.hand_action",
      "arguments": {
        "action": 3,
        "hand": 3,
        "steps": 2,
        "speed": 600
      }
    },
    "id": 3
  }
}
```

各字段含义：
- `type: "mcp"`：消息类型标识，用于区分同一通信通道上的不同消息类型
- `method: "tools/call"`：固定值，表示工具调用
- `params.name`：实际工具名称，遵循 `self.electron.xxx` 命名约定
- `params.arguments`：工具的参数键值对

### 5.2.2 响应格式

```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "result": {
    "content": [
      {"type": "text", "text": "true"}
    ],
    "isError": false
  }
}
```

## 5.3 McpSimBridge 实现

### 5.3.1 工具注册

Bridge 采用注册表模式，不实现动作逻辑，仅做 JSON ↔ Python 方法调用的转换：

```python
class McpSimBridge:
    def __init__(self, actions):
        self.actions = actions
        self._tools = {}
        self._register_tools()
    
    def _register_tools(self):
        self._tools = {
            "self.electron.hand_action":    self.actions.hand_action,
            "self.electron.body_turn":      self.actions.body_turn,
            "self.electron.head_move":      self.actions.head_move,
            "self.electron.stop":           self.actions.stop,
            "self.electron.servo_move":     self.actions.servo_move,
            "self.electron.servo_sequences": self.actions.servo_sequences,
            "self.electron.home":           self.actions.home,
            "self.electron.get_status":     self._get_status,
            "self.electron.set_trim":       self._set_trim,
            "self.electron.get_trims":      self._get_trims,
            "self.battery.get_level":       self._get_battery_level,
            "self.electron.get_ip":         self._get_ip,
        }
```

### 5.3.2 双格式兼容

Bridge 同时支持两种输入格式：

**标准 MCP 格式**（与真机一致）：
```python
# method = "tools/call", 工具名在 params.name
if method == "tools/call":
    tool_name = request["params"]["name"]
    tool_args = request["params"].get("arguments", {})
```

**扁平格式**（仿真调试用，去一层嵌套）：
```python
# method 直接为工具名
else:
    tool_name = method
    tool_args = request.get("params", {})
```

## 5.4 统一 Backend API

### 5.4.1 ElectronBotBackend

为屏蔽底层差异，提供统一的顶层接口：

```python
class ElectronBotBackend:
    def __init__(self, mode="sim", **kwargs):
        if mode == "sim":
            env = ElectronBotEnv(render_mode="rgb_array")
            env.reset()
            actions = ElectronBotActions(env)
            self.bridge = McpSimBridge(actions)
        elif mode == "cloud":
            self.bridge = McpCloudBridge(
                kwargs["api_url"], kwargs["device_id"])
    
    def call(self, method, params):
        return self.bridge.handle_request({
            "method": "tools/call",
            "params": {"name": method, "arguments": params},
            "id": 1
        })
```

### 5.4.2 一行切换示例

```python
# 仿真模式
backend = ElectronBotBackend("sim")
backend.call("self.electron.hand_action", {"action": 3, "hand": 3, "steps": 2, "speed": 600})

# 真机模式——仅改构造参数
backend = ElectronBotBackend("cloud",
    api_url="https://api.xiaozhi.cn/v1",
    device_id="eb-001")
backend.call("self.electron.hand_action", {"action": 3, "hand": 3, "steps": 2, "speed": 600})
```

### 5.4.3 McpCloudBridge 实现

真机端通过云端 API 转发 MCP 命令到 ESP32：

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

## 5.5 WebSocket 调试服务器

为支持浏览器端调试，实现了 WebSocket 服务器：

```python
import asyncio, json, websockets

async def handle_client(websocket):
    async for message in websocket:
        request = json.loads(message)
        response = bridge.handle_request(request)
        await websocket.send(json.dumps(response))

async def main():
    async with websockets.serve(handle_client, "localhost", 8080):
        await asyncio.Future()

asyncio.run(main())
```

启动后可通过 WebSocket 客户端发送 MCP 命令直接控制仿真机器人。

## 5.6 MCP 工具清单

平台共注册 12 个 MCP 工具，分为两类：

**真机对齐（8 个）**——release v2.2.6 原生支持：

| 工具名 | 用途 |
|--------|------|
| `self.electron.hand_action` | 举手/放手/挥手/拍打 |
| `self.electron.body_turn` | 左转/右转/回中心 |
| `self.electron.head_move` | 抬头/低头/点头/回中心 |
| `self.electron.stop` | 停止并复位 |
| `self.electron.get_status` | 返回 "moving" / "idle" |
| `self.electron.set_trim` | 设置舵机偏移 |
| `self.electron.get_trims` | 读取 6 舵机 trim 值 |
| `self.battery.get_level` | 电池电量与充电状态 |

**仿真专属（4 个）**——真机 v2.2.6 暂不支持，待固件 OTA：

| 工具名 | 用途 |
|--------|------|
| `self.electron.servo_move` | 单舵机精确定位 |
| `self.electron.servo_sequences` | AI 生成动作序列 |
| `self.electron.home` | 显式复位 |
| `self.electron.get_ip` | 设备 IP 查询 |

Sim2Real 降级策略：仿真策略若使用了仿真专属工具，部署时优先降级为预设动作组合；无法降级则标记为"仿真验证通过，等待固件升级"。

## 5.7 通信链路与延迟

**仿真链路**（进程内调用，延迟 < 1ms）：
```
AI 策略 → ElectronBotBackend → McpSimBridge → ElectronBotActions → MuJoCo ctrl
```

**云端真机链路**（HTTPS + MQTT，延迟 200-500ms）：
```
AI 策略 → HTTPS(~100ms) → 小智云端 → MQTT(~100-400ms) → ESP32
```

三种通信模式的延迟对比：

| 模式 | 延迟 | 适用场景 |
|------|:----:|---------|
| 仿真（进程内） | <1ms | RL 训练、调试开发 |
| 云端 API | 200-500ms | VLA 语音控制、预设动作 |
| WebSocket 直连（OTA 后） | <10ms | 实时闭环控制 |

## 5.8 本章小结

本章完成了 MCP 协议层的实现。与设计文档的主要差异：

| 项目 | 设计文档 | 实际实现 |
|------|---------|---------|
| 工具数量 | 8 个真机 + 4 个仿真 | 同上，核心 6 个先实现，其余逐步添加 |
| 响应格式 | 标准 content/isError | 支持标准与扁平两种格式 |
| WebSocket | 文档中有规划 | 已实现 localhost:8080 |
| 异常处理 | 未详细定义 | 已实现 try/except → error 返回 |
