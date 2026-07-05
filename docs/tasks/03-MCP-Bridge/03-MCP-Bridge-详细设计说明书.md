# Phase 3：MCP Bridge 协议层

> **目标**：实现仿真端的 MCP JSON-RPC 服务器。仿真实现全部 8+4 个 MCP 工具，将其中 8 个与 release v2.2.6 真机对齐的工具通过云端 MQTT/WS 部署，4 个仿真专属工具用于高级 AI 验证。
>
> **前置依赖**：Phase 2 完成（env.py 可用）
>
> **真机对齐版本**：xiaozhi-esp32 **release v2.2.6**
>
> **输出**：`src/electronbot_sim/mcp_bridge.py`
>
> **文档版本**: v1.1  
> **最后更新**: 2026-07-04  
> **变更类型**: 补充软件工程规范章节

---

## 1. 预期效果

### 1.1 协议对齐说明

> ⚠️ **关键设计决策**: 真实 xiaozhi-esp32 固件的 MCP 协议使用 **`tools/call` 两层嵌套**结构。
> 仿真端需同时支持此标准格式和简化调试格式。

**真实固件 MCP 协议结构** (参照 `docs/mcp-protocol_zh.md`):
```json
// 外层封装
{"type":"mcp", "payload": {
  // 内层 JSON-RPC
  "jsonrpc":"2.0",
  "method":"tools/call",                    // ← 固定值，不是工具名
  "params":{
    "name":"self.electron.hand_action",     // ← 工具名在这里
    "arguments":{"action":3, "hand":3}      // ← 参数在这里
  },
  "id":3
}}

// 标准成功响应
{"type":"mcp", "payload": {
  "jsonrpc":"2.0", "id":3,
  "result":{
    "content":[{"type":"text", "text":"true"}],
    "isError":false
  }
}}
```

### 1.2 阶段完成后的状态

```
终端 A（仿真 WebSocket 服务器——调试用）：
$ python -m electronbot_sim.mcp_server
🔌 ElectronBot 仿真 MCP 服务器已启动 (调试模式)
   ws://localhost:8080/ws
   ───

终端 B（客户端——标准 MCP 格式，与真机云端通信一致）：
$ websocat ws://localhost:8080/ws

# 标准 tools/call 格式
{"type":"mcp","payload":{"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.hand_action","arguments":{"action":3,"hand":3,"steps":3,"speed":600}},"id":1}}
← {"type":"mcp","payload":{"jsonrpc":"2.0","id":1,"result":{"content":[{"type":"text","text":"{'status': 'ok'}"}],"isError":false}}}

# 仿真也支持简化扁平格式（仅仿真内部调试用）
{"jsonrpc":"2.0","method":"self.electron.get_status","params":{}}
← {"jsonrpc":"2.0","id":null,"result":{"content":[{"type":"text","text":"idle"}],"isError":false}}
```

### 1.2 统一 API

```python
# 仿真模式
from electronbot_sim import ElectronBotBackend
backend = ElectronBotBackend("sim")  # 连接仿真 MCP Bridge
backend.call("self.electron.home", {})

# 真机模式——一行改动
backend = ElectronBotBackend("real", host="192.168.1.100")  # 连接真机 WebSocket
backend.call("self.electron.home", {})  # 完全相同的调用
```

---

## 2. 架构设计

### 2.1 MCP Bridge 类

```python
# src/electronbot_sim/mcp_bridge.py

import json
import numpy as np
from typing import Any, Dict, Optional

class McpSimBridge:
    """
    仿真端 MCP 服务器
    ——与真机 xiaozhi-esp32 McpServer 实现同构的工具注册
    """
    
    def __init__(self, env: "ElectronBotEnv"):
        self.env = env
        
        # ── 舵机角度 → MuJoCo 关节角度转换 ──
        # 这些数值来自 Phase 1 的分析
        self._servo_centers = np.array([180, 140, 0, 40, 90, 90])    # 舵机中心
        self._servo_ratios = np.array([1.0, 1.125, 1.0, 1.125, 1.5, 2.0])  # 映射比
        self._servo_directions = np.array([-1, -1, 1, 1, 1, 1])      # 方向
        
        # ── 注册工具映射表 ──
        # 工具可用范围:
        #   真机可用 (8个): hand_action, body_turn, head_move, stop,
        #                   get_status, set_trim, get_trims, battery.get_level
        #   @sim_only (4个): servo_move, servo_sequences, home, get_ip
        #                    ⚠️ 这4个在真机 release v2.2.6 上不可用，需固件 OTA
        self._tool_registry = {
            "self.electron.servo_move":          self._servo_move,          # @sim_only
            "self.electron.servo_sequences":     self._servo_sequences,     # @sim_only
            "self.electron.hand_action":         self._hand_action,
            "self.electron.body_turn":           self._body_turn,
            "self.electron.head_move":           self._head_move,
            "self.electron.home":                self._home,                # @sim_only
            "self.electron.stop":                self._stop,
            "self.electron.get_status":          self._get_status,
            "self.electron.set_trim":            self._set_trim,
            "self.electron.get_trims":           self._get_trims,
            "self.battery.get_level":            self._battery_level,
            "self.electron.get_ip":              self._get_ip,              # @sim_only
        }
        
        # ── 动作队列中的状态 ──
        self._is_moving = False
        self._trims = np.zeros(6, dtype=int)
    
    # ========== 核心：舵机↔关节转换 ==========
    
    def _servo_to_joint(self, servo_index: int, servo_angle: float) -> float:
        """舵机角度 → MuJoCo 机械关节角度 (度)"""
        offset = servo_angle - self._servo_centers[servo_index]
        ratio = self._servo_ratios[servo_index]
        direction = self._servo_directions[servo_index]
        return offset * ratio * direction
    
    def _joint_to_servo(self, servo_index: int, joint_angle: float) -> float:
        """MuJoCo 机械关节角度 → 舵机角度 (度)"""
        ratio = self._servo_ratios[servo_index]
        direction = self._servo_directions[servo_index]
        ratio = ratio * direction if ratio != 0 else 1
        return joint_angle / ratio + self._servo_centers[servo_index]
    
    # ========== 工具实现 ==========
    
    def _servo_index_from_name(self, servo_type: str) -> int:
        """servo_type 字符串 → servo_index"""
        mapping = {
            "right_pitch": 0, "rp": 0,
            "right_roll":  1, "rr": 1,
            "left_pitch":  2, "lp": 2,
            "left_roll":   3, "lr": 3,
            "body":        4, "b":  4,
            "head":        5, "h":  5,
        }
        return mapping.get(servo_type.lower(), -1)
    
    def _servo_move(self, servo_type: str, position: float, speed: int = 1000,
                     **kwargs) -> Dict:
        """self.electron.servo_move 仿真实现"""
        idx = self._servo_index_from_name(servo_type)
        if idx < 0:
            return {"error": f"无效舵机类型: {servo_type}"}
        
        # 1. 舵机角度 → 机械关节角度
        joint_angle = self._servo_to_joint(idx, position)
        
        # 2. 获取当前所有关节角度
        current = self._get_joint_angles()
        
        # 3. 线性插值执行 (对齐固件 movements.cc:87 MoveServos)
        steps = max(1, speed // 10)
        for step in range(1, steps + 1):
            t = step / steps              # 线性插值, 非 EaseOutCubic
            target = np.copy(current)
            target[idx] = current[idx] + (joint_angle - current[idx]) * t
            self._step_sim(target)
        
        return {"status": "ok"}
    
    def _servo_sequences(self, sequence: str, **kwargs) -> Dict:
        """self.electron.servo_sequences 仿真实现"""
        seq = json.loads(sequence)
        actions = seq.get("a", [])
        
        for action in actions:
            if "osc" in action:
                # 振荡模式
                self._execute_oscillation(action["osc"])
            elif "s" in action:
                # 普通移动模式
                targets = self._parse_servo_targets(action["s"])
                speed = action.get("v", 1000)
                self._move_all_servos(targets, speed)
            
            # 动作间延迟
            delay = action.get("d", 0)
            if delay > 0:
                import time
                time.sleep(delay / 1000.0)
        
        return {"status": "ok"}
    
    def _execute_oscillation(self, osc_params: dict):
        """执行振荡动作——与固件 OscillateServos 行为一致"""
        amplitudes = self._parse_servo_values(osc_params.get("a", {}), default=0)
        centers = self._parse_servo_values(osc_params.get("o", {}), default=None)
        period = osc_params.get("p", 500) / 1000.0  # ms → s
        cycles = osc_params.get("c", 5)
        
        # 用当前角度填充未指定的 center
        current = self._get_servo_angles()
        for i in range(6):
            if centers[i] is None:
                centers[i] = current[i]
        
        # 正弦振荡
        dt = self.env.model.opt.timestep
        total_steps = int(period * cycles / dt)
        
        for step in range(total_steps):
            phase = 2 * np.pi * step * dt / period
            for i in range(6):
                target_servo = centers[i] + amplitudes[i] * np.sin(phase)
                joint_angle = self._servo_to_joint(i, target_servo)
                self.env.data.ctrl[i] = joint_angle
            self._step_sim_raw()
    
    def _hand_action(self, action: int, hand: int = 3, steps: int = 1, 
                     speed: int = 1000, amount: int = 30, **kwargs) -> Dict:
        """预设手部动作——复制真机固件 HandAction 的逻辑"""
        # 动作映射：action=1举手、2放手、3挥手、4拍打；hand=1左手、2右手、3双手
        # ⚠️ 对齐真机固件 movements.cc:225 的 times 限制逻辑：
        #    times = 2 * max(3, min(100, times))
        steps = 2 * max(3, min(100, steps))
        # 此处在仿真中复现 movements.cc 的 HandAction 轨迹
        
        # 简化实现：根据 action+hand 计算目标角度序列
        targets_list = self._generate_hand_targets(action, hand, steps, amount)
        for targets in targets_list:
            self._move_all_servos(targets, speed)
        
        return {"status": "ok"}
    
    def _body_turn(self, direction: int, steps: int = 1, speed: int = 1000, 
                   angle: int = 45, **kwargs) -> Dict:
        """身体转向"""
        current = self._get_servo_angles()
        center = self._servo_centers[4]  # body center = 90
        
        if direction == 1:      # 左转
            current[4] = min(180, center + angle)
        elif direction == 2:    # 右转
            current[4] = max(0, center - angle)
        elif direction == 3:    # 回中
            current[4] = center
        
        self._move_all_servos(current, speed)
        return {"status": "ok"}
    
    def _head_move(self, action: int, steps: int = 1, speed: int = 1000,
                   angle: int = 5, **kwargs) -> Dict:
        """头部运动"""
        current = self._get_servo_angles()
        center = self._servo_centers[5]  # head center = 90
        
        if action == 1:         # 抬头
            current[5] = min(105, center + angle)
        elif action == 2:       # 低头
            current[5] = max(75, center - angle)
        elif action == 3:       # 点头一次
            current[5] = center + angle
            self._move_all_servos(current, speed // 3)
            current[5] = center - angle
            self._move_all_servos(current, speed // 3)
            current[5] = center
            self._move_all_servos(current, speed // 3)
        elif action == 4:       # 回中
            current[5] = center
        
        self._move_all_servos(current, speed // 3)
        return {"status": "ok"}
    
    def _home(self, **kwargs) -> Dict:
        """复位"""
        home_servo = np.array([180, 180, 0, 0, 90, 90])
        self._move_all_servos(home_servo, 1000)
        return {"status": "ok"}
    
    def _stop(self, **kwargs) -> Dict:
        """停止 → 立即复位"""
        self._is_moving = False
        return self._home()
    
    def _get_status(self, **kwargs) -> Dict:
        return {"status": "moving" if self._is_moving else "idle"}
    
    def _set_trim(self, servo_type: str, trim_value: int, **kwargs) -> Dict:
        idx = self._servo_index_from_name(servo_type)
        if idx < 0:
            return {"error": f"无效舵机类型: {servo_type}"}
        self._trims[idx] = trim_value
        return {"status": "ok", "message": f"舵机 {servo_type} trim={trim_value}"}
    
    def _get_trims(self, **kwargs) -> Dict:
        return {"trims": self._trims.tolist()}
    
    def _battery_level(self, **kwargs) -> Dict:
        return {"level": 100, "charging": False}
    
    def _get_ip(self, **kwargs) -> Dict:
        return {"ip": "127.0.0.1", "connected": True}
    
    # ========== 内部辅助 ==========
    
    def _get_servo_angles(self) -> np.ndarray:
        """MuJoCo 关节角度 → 舵机角度"""
        joint_angles = self.env.data.qpos[:6].copy()
        servo = np.zeros(6)
        for i in range(6):
            servo[i] = self._joint_to_servo(i, joint_angles[i])
        return servo
    
    def _get_joint_angles(self) -> np.ndarray:
        return self.env.data.qpos[:6].copy()
    
    def _step_sim(self, joint_targets: np.ndarray):
        self.env.data.ctrl[:] = joint_targets
        self._step_sim_raw()
    
    def _step_sim_raw(self):
        mujoco.mj_step(self.env.model, self.env.data)
    
    def _move_all_servos(self, servo_targets: np.ndarray, time_ms: int):
        """将所有舵机缓动到目标位置（线性插值，对齐固件）"""
        current_servo = self._get_servo_angles()
        current_joint = self._get_joint_angles()
        
        # 舵机目标 → 关节目标
        joint_targets = np.zeros(6)
        for i in range(6):
            joint_targets[i] = self._servo_to_joint(i, servo_targets[i])
        
        # 线性插值 (对齐固件 movements.cc:87)
        steps = max(1, time_ms // 10)
        self._is_moving = True
        for step in range(1, steps + 1):
            t = step / steps
            # 线性插值 (对齐固件: increment = (target-pos)/(time/10.0))
            interpolated = current_joint + (joint_targets - current_joint) * t
            self._step_sim(interpolated)
        self._is_moving = False
    
    # ========== JSON-RPC 入口 ==========
    
    def handle_request(self, request: Dict) -> Dict:
        """处理 JSON-RPC 请求——兼容真实 MCP 协议和仿真简化格式
        
        真实 MCP 协议 (参照固件 mcp_server.cc + mcp-protocol_zh.md):
          {"method":"tools/call", "params":{"name":"工具名", "arguments":{参数}}}
          → 响应: {"result":{"content":[{type:"text", text:"..."}], "isError":false}}
        
        仿真简化格式 (内部调试):
          {"method":"self.electron.xxx", "params":{参数}}
          → 响应: {"result":{返回值}}
        """
        method = request.get("method", "")
        req_id = request.get("id")
        
        # ── 路径 1: 标准 MCP tools/call 格式 (与真机一致) ──
        if method == "tools/call":
            tool_name = request.get("params", {}).get("name", "")
            tool_args = request.get("params", {}).get("arguments", {})
            
            handler = self._tool_registry.get(tool_name)
            if handler is None:
                return self._error(req_id, -32601, f"Unknown tool: {tool_name}")
            
            try:
                result = handler(**tool_args)
                return self._success_mcp(req_id, str(result))
            except Exception as e:
                return self._error(req_id, -32603, str(e))
        
        # ── 路径 2: 扁平格式 (仿真内部调试/测试兼容) ──
        params = request.get("params", {})
        handler = self._tool_registry.get(method)
        if handler is None:
            return self._error(req_id, -32601, f"未知方法: {method}")
        
        try:
            result = handler(**params)
            return self._success_mcp(req_id, str(result))
        except Exception as e:
            return self._error(req_id, -32603, str(e))
    
    @staticmethod
    def _success_mcp(req_id, text: str) -> Dict:
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "content": [{"type": "text", "text": text}],
                "isError": False
            }
        }
    
    @staticmethod
    def _error(req_id, code: int, message: str) -> Dict:
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": code, "message": message}
        }
```

### 2.2 WebSocket 服务器 (仿真调试用)

```python
# src/electronbot_sim/mcp_server.py

import asyncio
import json
import websockets
from electronbot_sim.env import ElectronBotEnv
from electronbot_sim.mcp_bridge import McpSimBridge

class McpWebSocketServer:
    """仿真 WebSocket 服务器——用于本地调试，非真机通信接口
    
    ⚠️ 注意：真机 ESP32 (release v2.2.6) 没有 WebSocket Server。
    此服务器仅用于仿真环境下的本地调试和开发。
    真机通信请使用 ElectronBotBackend("cloud", ...) 通过云端 API 透传。
    """
    
    def __init__(self, host="localhost", port=8080):
        self.host = host
        self.port = port
        self.env = ElectronBotEnv(render_mode="human")
        self.bridge = McpSimBridge(self.env)
        self.clients = set()
    
    async def handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    # ── 标准 MCP 封装格式 ──
                    if data.get("type") == "mcp":
                        payload = data["payload"]
                        response_payload = self.bridge.handle_request(payload)
                        # 按原格式封装返回
                        await websocket.send(json.dumps({
                            "type": "mcp",
                            "payload": response_payload
                        }))
                    else:
                        # ── 扁平格式 (调试兼容) ──
                        response_payload = self.bridge.handle_request(data)
                        await websocket.send(json.dumps(response_payload))
                    
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        "jsonrpc": "2.0", "error": {"code": -32700, "message": "Parse error"}
                    }))
        finally:
            self.clients.remove(websocket)
    
    async def start(self):
        print(f"🔌 ElectronBot 仿真 MCP 服务器已启动 (调试模式)")
        print(f"   ws://{self.host}:{self.port}/ws")
        print(f"   ⚠️ 此服务器仅用于仿真调试，不用于真机连接")
        print(f"   真机部署请使用: ElectronBotBackend('cloud', ...)")
        print(f"   ───")
        async with websockets.serve(self.handler, self.host, self.port):
            await asyncio.Future()  # 永久运行

# 入口
if __name__ == "__main__":
    server = McpWebSocketServer()
    asyncio.run(server.start())
```

### 2.3 统一 Backend API

```python
# src/electronbot_sim/backend.py

import json
from typing import Literal

class ElectronBotBackend:
    """
    统一后端——AI 策略通过此类访问机器人，
    不感知下面是仿真还是真机。
    
    模式说明:
    - "sim":  本地 MuJoCo 仿真 (调试/训练用)
    - "cloud": 小智云端 API 透传 → ESP32 真机 (生产部署)
    
    ⚠️ release v2.2.6 真机无本地 WebSocket Server，
    所有真机通信必须通过云端小智 API 透传。
    """
    
    def __init__(self, mode: Literal["sim", "cloud"] = "sim", **kwargs):
        self.mode = mode
        
        if mode == "sim":
            from electronbot_sim.env import ElectronBotEnv
            from electronbot_sim.mcp_bridge import McpSimBridge
            self._env = ElectronBotEnv(render_mode=kwargs.get("render", "human"))
            self._bridge = McpSimBridge(self._env)
            
        elif mode == "cloud":
            # 通过小智云端 API 连接真机 ESP32
            self._api_url = kwargs.get("api_url", "https://api.xiaozhi.cn/v1")
            self._device_id = kwargs["device_id"]
            self._api_key = kwargs.get("api_key")
            
        else:
            raise ValueError(f"Unknown mode: {mode}")
    
    def call(self, method: str, params: dict) -> dict:
        """
        调用 MCP 工具——仿真和真机完全相同的调用方式
        
        Example:
          # 仿真
          backend = ElectronBotBackend("sim")
          backend.call("self.electron.hand_action", {"action":3,"hand":3,"steps":2,"speed":600})
          
          # 真机 (云端 API)
          backend = ElectronBotBackend("cloud", device_id="eb-001")
          backend.call("self.electron.hand_action", {"action":3,"hand":3,"steps":2,"speed":600})
        """
        if self.mode == "sim":
            # 仿真：直接调用 MCP Bridge
            return self._bridge.handle_request({
                "jsonrpc": "2.0",
                "method": "tools/call",
                "params": {"name": method, "arguments": params}
            })
        else:
            # 真机：通过云端 API 透传 (同步封装)
            import asyncio
            return asyncio.run(self._call_cloud(method, params))
    
    async def call_async(self, method: str, params: dict) -> dict:
        """异步调用真机"""
        if self.mode == "sim":
            return self.call(method, params)
        return await self._call_cloud(method, params)
    
    async def _call_cloud(self, tool_name: str, arguments: dict) -> dict:
        """通过小智云端 API 发送 MCP 命令到 ESP32"""
        import httpx
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self._api_url}/devices/{self._device_id}/tools/call",
                json={"name": tool_name, "arguments": arguments},
                headers={"Authorization": f"Bearer {self._api_key}"} if self._api_key else {},
                timeout=30.0
            )
            response.raise_for_status()
            return response.json()
```

---

## 3. 验证方法

### 3.1 自动化测试

```python
# tests/test_mcp_bridge.py

import json
from electronbot_sim.env import ElectronBotEnv
from electronbot_sim.mcp_bridge import McpSimBridge

def test_all_tools():
    env = ElectronBotEnv(render_mode=None)
    bridge = McpSimBridge(env)
    
    # 测试每个工具——使用标准 tools/call 格式 (与真机一致)
    tests = [
        ("self.electron.home", {}),
        ("self.electron.get_status", {}),
        ("self.electron.servo_move", {"servo_type": "rp", "position": 120, "speed": 500}),
        ("self.electron.servo_move", {"servo_type": "rr", "position": 160, "speed": 500}),
        ("self.electron.servo_move", {"servo_type": "lp", "position": 60, "speed": 500}),
        ("self.electron.servo_move", {"servo_type": "lr", "position": 20, "speed": 500}),
        ("self.electron.servo_move", {"servo_type": "b", "position": 60, "speed": 500}),
        ("self.electron.servo_move", {"servo_type": "h", "position": 100, "speed": 500}),
        ("self.electron.hand_action", {"action": 3, "hand": 3, "steps": 1, "speed": 300}),
        ("self.electron.body_turn", {"direction": 1, "speed": 500, "angle": 30}),
        ("self.electron.head_move", {"action": 3, "speed": 300, "angle": 5}),
        ("self.electron.get_ip", {}),
        ("self.battery.get_level", {}),
        ("self.electron.set_trim", {"servo_type": "rp", "trim_value": 5}),
        ("self.electron.get_trims", {}),
    ]
    
    for tool_name, args in tests:
        # 使用标准 tools/call 格式
        result = bridge.handle_request({
            "jsonrpc": "2.0",
            "method": "tools/call",        # ← 固定值
            "params": {
                "name": tool_name,          # ← 工具名
                "arguments": args           # ← 参数
            },
            "id": 1
        })
        assert "error" not in result, f"{tool_name} 失败: {result.get('error')}"
        # 验证 MCP 标准响应格式
        assert "result" in result
        assert "content" in result["result"]
        assert result["result"]["isError"] == False
        print(f"  ✅ {tool_name}")
    
    print(f"✅ 全部 {len(tests)} 个 MCP 工具测试通过 (tools/call 格式)")
    
def test_flat_format():
    """验证扁平格式兼容性"""
    env = ElectronBotEnv(render_mode=None)
    bridge = McpSimBridge(env)
    
    # 扁平格式 (仿真调试)
    result = bridge.handle_request({
        "jsonrpc": "2.0",
        "method": "self.electron.get_status",
        "params": {}
    })
    assert "error" not in result
    assert result["result"]["content"][0]["text"] in ("idle", "moving")
    print("  ✅ 扁平格式兼容性通过")

def test_servo_to_joint_conversion():
    """验证舵机↔关节转换正确性"""
    env = ElectronBotEnv(render_mode=None)
    bridge = McpSimBridge(env)
    
    # 测试 home 姿态下关节角度
    bridge._home()
    servo_angles = bridge._get_servo_angles()
    joint_angles = bridge._get_joint_angles()
    
    # home: servo=[180,180,0,0,90,90], joint=[0,-45,0,-45,0,0]
    expected_joint = [0, -45, 0, -45, 0, 0]
    for i in range(6):
        assert abs(joint_angles[i] - expected_joint[i]) < 1.0, \
            f"关节 {i}: 期望 {expected_joint[i]}°, 实际 {joint_angles[i]:.1f}°"
    
    print("✅ 舵机↔关节转换验证通过")

def test_servo_sequence():
    """验证 servo_sequences"""
    env = ElectronBotEnv(render_mode=None)
    bridge = McpSimBridge(env)
    
    seq = json.dumps({
        "a": [
            {"s": {"rp": 90, "lp": 90}, "v": 500},
            {"osc": {"a": {"rp": 20}, "o": {"rp": 120}, "p": 300, "c": 2}}
        ]
    })
    
    result = bridge._servo_sequences(sequence=seq)
    assert result["status"] == "ok"
    print("✅ servo_sequences 验证通过")
```

### 3.2 WebSocket 端到端测试

```python
# tests/test_websocket_e2e.py

import asyncio
import json
import websockets

async def test_ws_e2e():
    async with websockets.connect("ws://localhost:8080/ws") as ws:
        # 测试 tools/call 标准格式
        await ws.send(json.dumps({
            "type": "mcp",
            "payload": {
                "jsonrpc": "2.0",
                "method": "tools/call",
                "params": {"name": "self.electron.home", "arguments": {}},
                "id": 1
            }
        }))
        resp = json.loads(await ws.recv())
        assert resp["type"] == "mcp"
        assert resp["payload"]["result"]["isError"] == False
        print("  ✅ home via WebSocket (tools/call 格式)")
        
        # 测试 get_status
        await ws.send(json.dumps({
            "type": "mcp",
            "payload": {
                "jsonrpc": "2.0",
                "method": "tools/call",
                "params": {"name": "self.electron.get_status", "arguments": {}},
                "id": 2
            }
        }))
        resp = json.loads(await ws.recv())
        text = resp["payload"]["result"]["content"][0]["text"]
        assert text in ("moving", "idle")
        print("  ✅ get_status via WebSocket (tools/call 格式)")
    
    print("✅ WebSocket 端到端测试通过")
```

### 3.3 手动验证

```bash
# 启动仿真服务器
python -m electronbot_sim.mcp_server

# 另一个终端，用 websocat 测试
websocat ws://localhost:8080/ws

# 标准 tools/call 格式 (与真机云端通信一致)
> {"type":"mcp","payload":{"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.hand_action","arguments":{"action":3,"hand":3,"steps":2,"speed":600}},"id":1}}
< {"type":"mcp","payload":{"jsonrpc":"2.0","id":1,"result":{"content":[{"type":"text","text":"{'status': 'ok'}"}],"isError":false}}}

# 观察仿真窗口：机器人双手挥手
```

---

## 4. 交付物清单

| 文件 | 描述 |
|------|------|
| `src/electronbot_sim/mcp_bridge.py` | MCP Bridge 核心（工具注册+舵机↔关节转换） |
| `src/electronbot_sim/mcp_server.py` | WebSocket 服务器入口 |
| `src/electronbot_sim/backend.py` | 统一 Backend API（sim↔real 切换） |
| `tests/test_mcp_bridge.py` | MCP 工具单元测试 |
| `tests/test_websocket_e2e.py` | WebSocket 端到端测试 |

---

## 5. 接口设计

### 5.1 模块对外接口

MCP Bridge 层提供三类对外接口：JSON-RPC 请求处理入口、统一 Backend 调用接口、WebSocket 调试服务器。所有接口均以 JSON-RPC 2.0 为协议基础，并在外层包裹 MCP 协议封装。

#### 5.1.1 McpSimBridge 核心接口

```python
class McpSimBridge:
    def handle_request(self, request: Dict) -> Dict: ...
```

- **职责**：处理 JSON-RPC 2.0 请求，兼容两种调用格式
- **入参 `request`**：JSON-RPC 请求字典，必须包含 `method` 字段
- **返回值**：标准 JSON-RPC 响应字典，包含 `jsonrpc`、`id`、`result` 或 `error` 字段
- **格式兼容**：
  - **标准 `tools/call` 两层嵌套格式**（与真机云端通信一致）：
    ```json
    {"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.hand_action","arguments":{"action":3}},"id":1}
    ```
  - **扁平调试格式**（仿真内部调试用）：
    ```json
    {"jsonrpc":"2.0","method":"self.electron.get_status","params":{}}
    ```
- **副作用**：除查询类工具（`get_status`、`get_trims`、`battery.get_level`、`get_ip`）外，其余工具会驱动 MuJoCo 仿真步进，改变机器人关节状态。

#### 5.1.2 ElectronBotBackend 统一接口

```python
class ElectronBotBackend:
    def __init__(self, mode: Literal["sim", "cloud"] = "sim", **kwargs): ...
    def call(self, method: str, params: dict) -> dict: ...
    async def call_async(self, method: str, params: dict) -> dict: ...
```

- **职责**：屏蔽 sim/cloud 差异，AI 策略层通过同一接口访问仿真或真机
- **`mode="sim"`**：本地 MuJoCo 仿真，直接调用 `McpSimBridge.handle_request`
- **`mode="cloud"`**：通过小智云端 API 透传至 ESP32 真机，调用 `https://api.xiaozhi.cn/v1/devices/{device_id}/tools/call`
- **`call` 同步语义**：sim 模式即时返回；cloud 模式内部使用 `asyncio.run` 包装异步 HTTP 调用
- **`call_async` 异步语义**：适用于高并发场景，cloud 模式下复用 `httpx.AsyncClient`

#### 5.1.3 McpWebSocketServer 调试接口

```python
class McpWebSocketServer:
    def __init__(self, host="localhost", port=8080): ...
    async def handler(self, websocket): ...
    async def start(self): ...
```

- **职责**：本地 WebSocket 调试服务器，非真机通信接口
- **监听地址**：`ws://localhost:8080/ws`
- **消息格式**：
  - MCP 封装格式：`{"type":"mcp","payload":{<JSON-RPC>}}`
  - 扁平格式：`{<JSON-RPC>}`
- **安全提示**：仅用于本地调试，不提供任何认证机制，禁止暴露到公网

### 5.2 输入输出契约

#### 5.2.1 12 个 MCP 工具签名表

| 工具名 | 参数 | 返回值 | sim_only | 说明 |
|--------|------|--------|:---:|------|
| `self.electron.servo_move` | `servo_type: str`, `position: float`, `speed: int=1000` | `{"status":"ok"}` 或 `{"error":...}` | ✅ | 单舵机移动 |
| `self.electron.servo_sequences` | `sequence: str` (JSON) | `{"status":"ok"}` | ✅ | 多舵机序列动作（含振荡） |
| `self.electron.hand_action` | `action: int`, `hand: int=3`, `steps: int=1`, `speed: int=1000`, `amount: int=30` | `{"status":"ok"}` | ❌ | 预设手部动作（1举手/2放手/3挥手/4拍打） |
| `self.electron.body_turn` | `direction: int`, `steps: int=1`, `speed: int=1000`, `angle: int=45` | `{"status":"ok"}` | ❌ | 身体转向（1左/2右/3回中） |
| `self.electron.head_move` | `action: int`, `steps: int=1`, `speed: int=1000`, `angle: int=5` | `{"status":"ok"}` | ❌ | 头部运动（1抬/2低/3点头/4回中） |
| `self.electron.home` | 无 | `{"status":"ok"}` | ✅ | 复位到 home 姿态 |
| `self.electron.stop` | 无 | `{"status":"ok"}` | ❌ | 紧急停止并复位 |
| `self.electron.get_status` | 无 | `{"status":"idle"}` 或 `{"status":"moving"}` | ❌ | 查询运动状态 |
| `self.electron.set_trim` | `servo_type: str`, `trim_value: int` | `{"status":"ok","message":...}` | ❌ | 设置舵机微调 |
| `self.electron.get_trims` | 无 | `{"trims":[int×6]}` | ❌ | 查询全部微调值 |
| `self.battery.get_level` | 无 | `{"level":100,"charging":false}` | ❌ | 电池电量（仿真固定 100%） |
| `self.electron.get_ip` | 无 | `{"ip":"127.0.0.1","connected":true}` | ✅ | 查询 IP 地址 |

> **工具可用性矩阵**: 8 个真机可用工具（`hand_action`、`body_turn`、`head_move`、`stop`、`get_status`、`set_trim`、`get_trims`、`battery.get_level`）与 release v2.2.6 真机协议对齐，可云端部署；4 个 `@sim_only` 工具（`servo_move`、`servo_sequences`、`home`、`get_ip`）在真机 release v2.2.6 上不可用，需固件 OTA 升级。

#### 5.2.2 servo_type 参数取值表

| 完整名 | 简写 | servo_index | 安全范围 |
|--------|------|:---:|----------|
| `right_pitch` | `rp` | 0 | 0-180 |
| `right_roll` | `rr` | 1 | 100-180 |
| `left_pitch` | `lp` | 2 | 0-180 |
| `left_roll` | `lr` | 3 | 0-80 |
| `body` | `b` | 4 | 30-150 |
| `head` | `h` | 5 | 75-105 |

#### 5.2.3 JSON-RPC 响应契约

- **成功响应**：
  ```json
  {"jsonrpc":"2.0","id":<req_id>,"result":{"content":[{"type":"text","text":"<str>"}],"isError":false}}
  ```
- **错误响应**：
  ```json
  {"jsonrpc":"2.0","id":<req_id>,"error":{"code":<int>,"message":"<str>"}}
  ```
- **`id` 字段**：请求中的 `id` 原样回传；扁平格式未提供时返回 `null`
- **`text` 字段**：始终为字符串（内部使用 `str(result)` 序列化）

---

## 6. 数据模型

### 6.1 核心数据结构

#### 6.1.1 JSON-RPC 2.0 消息结构

```python
# 请求
{
    "jsonrpc": "2.0",          # 固定值
    "method": str,             # "tools/call" 或工具名
    "params": dict,            # 参数字典
    "id": int | str | None     # 请求标识，用于关联响应
}

# 成功响应
{
    "jsonrpc": "2.0",
    "id": int | str | None,
    "result": {
        "content": [{"type": "text", "text": str}],
        "isError": False
    }
}

# 错误响应
{
    "jsonrpc": "2.0",
    "id": int | str | None,
    "error": {"code": int, "message": str}
}
```

#### 6.1.2 MCP 封装格式

外层封装用于 WebSocket 传输，区分 MCP 协议消息与其他类型消息：

```python
# 请求封装
{"type": "mcp", "payload": {<JSON-RPC 请求>}}

# 响应封装
{"type": "mcp", "payload": {<JSON-RPC 响应>}}
```

#### 6.1.3 舵机↔关节转换参数

机器人有 6 个舵机，每个舵机有 3 个转换参数（中心、比例、方向）：

| servo_index | 名称 | center | ratio | direction | 说明 |
|:---:|------|:---:|:---:|:---:|------|
| 0 | right_pitch | 180 | 1.0 | -1 | 右臂俯仰 |
| 1 | right_roll | 140 | 1.125 | -1 | 右臂横滚 |
| 2 | left_pitch | 0 | 1.0 | 1 | 左臂俯仰 |
| 3 | left_roll | 40 | 1.125 | 1 | 左臂横滚 |
| 4 | body | 90 | 1.5 | 1 | 腰部旋转 |
| 5 | head | 90 | 2.0 | 1 | 头部俯仰 |

转换公式：
- `_servo_to_joint(idx, servo_angle) = (servo_angle - center[idx]) * ratio[idx] * direction[idx]`
- `_joint_to_servo(idx, joint_angle) = joint_angle / (ratio[idx] * direction[idx]) + center[idx]`

#### 6.1.4 工具注册表结构

```python
# _tool_registry: Dict[str, Callable]
{
    "self.electron.servo_move":          self._servo_move,          # @sim_only
    "self.electron.servo_sequences":     self._servo_sequences,     # @sim_only
    "self.electron.hand_action":         self._hand_action,
    "self.electron.body_turn":           self._body_turn,
    "self.electron.head_move":           self._head_move,
    "self.electron.home":                self._home,                # @sim_only
    "self.electron.stop":                self._stop,
    "self.electron.get_status":          self._get_status,
    "self.electron.set_trim":            self._set_trim,
    "self.electron.get_trims":           self._get_trims,
    "self.battery.get_level":            self._battery_level,
    "self.electron.get_ip":              self._get_ip,              # @sim_only
}
```

#### 6.1.5 内部运行时状态

```python
# 动作执行状态
_is_moving: bool                    # 是否有动作正在执行
_trims: np.ndarray, shape=(6,)      # 各舵机微调值（int）
```

### 6.2 数据流

```
[客户端] 
   │
   │  WebSocket 消息 (JSON 字符串)
   ▼
[McpWebSocketServer.handler]
   │
   │  解析 JSON → 识别 type=="mcp" 或扁平格式
   ▼
[McpSimBridge.handle_request]
   │
   ├── method == "tools/call" ?
   │     ├── 是: 从 params.name 取工具名, params.arguments 取参数
   │     └── 否: 将 method 作为工具名, params 作为参数
   │
   │  查 _tool_registry
   ▼
[工具处理函数 _xxx_tool(**args)]
   │
   ├── 舵机角度 → 关节角度 (_servo_to_joint)
   ├── MuJoCo 仿真步进 (mj_step)
   └── 关节角度 → 舵机角度 (_joint_to_servo) [查询类]
   │
   │  返回 dict
   ▼
[McpSimBridge._success_mcp / _error]
   │
   │  封装 JSON-RPC 响应
   ▼
[McpWebSocketServer.handler]
   │
   │  按 type=="mcp" 封装外层
   ▼
[客户端]
```

**ElectronBotBackend 调用流（sim 模式）**：
```
backend.call("self.electron.home", {})
   → 构造 {"jsonrpc":"2.0","method":"tools/call","params":{"name":"self.electron.home","arguments":{}},"id":None}
   → McpSimBridge.handle_request(request)
   → 返回 JSON-RPC 响应 dict
```

**ElectronBotBackend 调用流（cloud 模式）**：
```
backend.call("self.electron.home", {})
   → asyncio.run(_call_cloud("self.electron.home", {}))
   → POST https://api.xiaozhi.cn/v1/devices/{device_id}/tools/call
       body: {"name":"self.electron.home","arguments":{}}
       headers: Authorization: Bearer {api_key}
   → response.json()
```

---

## 7. 错误处理与恢复

### 7.1 错误分类

| 错误码 | 错误类型 | 触发场景 | 处理策略 | 影响范围 |
|:---:|------|------|------|------|
| -32700 | Parse error | 客户端发送的 JSON 字符串无法解析 | 返回错误响应，关闭当前消息处理 | 单条消息 |
| -32600 | Invalid Request | JSON-RPC 请求不符合规范（缺 method 字段等） | 返回错误响应 | 单条消息 |
| -32601 | Method not found | 工具名不在 `_tool_registry` 中，或 `tools/call` 的 `params.name` 未注册 | 返回错误响应，提示未知工具名 | 单条消息 |
| -32602 | Invalid params | 参数类型不匹配、`servo_type` 无法识别、参数缺失 | 返回错误响应，附带具体参数错误信息 | 单条消息 |
| -32603 | Internal error | 工具执行过程中抛出异常（MuJoCo 步进失败、数组越界等） | 捕获 `Exception`，返回错误响应，记录堆栈日志 | 单条消息，仿真状态可能不一致 |
| -1 | Timeout | 云端 API 调用超过 30s（sim 模式即时执行，不触发） | `httpx.TimeoutException` 上抛，由调用方重试或降级 | 单次调用 |
| -1 | Auth failure | 云端 API 返回 401/403，`api_key` 无效或权限不足 | `httpx.HTTPStatusError` 上抛，提示重新认证 | 阻断 cloud 模式所有调用 |
| -1 | Connection lost | WebSocket 连接断开（网络抖动、服务端重启） | 客户端检测到断开后自动重连，重连失败 3 次后告警 | 中断调试会话 |
| -1 | Sim state corruption | 仿真状态异常（关节角度超限、NaN 等） | 内部 `try/except` 捕获，返回 -32603，建议调用 `home` 复位 | 后续动作可能异常 |

### 7.2 异常恢复流程

#### 7.2.1 工具执行异常恢复

```python
# McpSimBridge.handle_request 中的异常处理
try:
    result = handler(**tool_args)
    return self._success_mcp(req_id, str(result))
except TypeError as e:
    # 参数类型/数量不匹配
    return self._error(req_id, -32602, f"参数错误: {e}")
except ValueError as e:
    # 参数值非法（如 servo_type 未知）
    return self._error(req_id, -32602, str(e))
except Exception as e:
    # 兜底：内部错误
    logger.exception("工具执行异常", extra={"tool": tool_name, "trace_id": req_id})
    return self._error(req_id, -32603, str(e))
```

#### 7.2.2 仿真状态恢复

当工具执行导致仿真状态异常（如关节角度 NaN、位置超限）时，按以下流程恢复：

1. 检测异常：工具执行后检查 `env.data.qpos` 是否包含 NaN 或 Inf
2. 立即停止：设置 `_is_moving = False`
3. 状态复位：调用 `_home()` 将所有舵机回到 home 姿态
4. 返回错误：向客户端返回 -32603 错误，提示已自动复位
5. 日志记录：记录异常前的最后 10 步关节角度，用于事后分析

#### 7.2.3 WebSocket 连接断开重连

```
客户端侧：
1. 检测到连接断开 → 等待 1s
2. 第 1 次重连 → 失败则等待 2s
3. 第 2 次重连 → 失败则等待 4s（指数退避）
4. 第 3 次重连 → 失败则记录错误日志，停止重连
5. 重连成功后，重新发送未完成的请求（基于 req_id 去重）

服务端侧：
1. `handler` 的 `finally` 块确保从 `self.clients` 移除断开的连接
2. 仿真环境状态保留，等待新连接接入
3. 不主动推送断连期间的状态变化
```

#### 7.2.4 云端 API 认证失败恢复

```
1. httpx 返回 401/403
2. ElectronBotBackend 抛出 HTTPStatusError
3. 调用方捕获后：
   a. 检查 api_key 是否过期 → 重新获取 token
   b. 检查 device_id 是否存在 → 提示用户核对设备列表
   c. 连续 3 次认证失败 → 禁用 cloud 模式，降级为 sim 模式并告警
```

#### 7.2.5 工具执行超时处理

| 模式 | 超时阈值 | 触发条件 | 处理 |
|------|---------|---------|------|
| sim | 即时（无超时） | MuJoCo 步进同步执行 | 不需要 |
| cloud | 30s | `httpx.AsyncClient` `timeout=30.0` | 抛出 `httpx.TimeoutException`，由调用方决定重试或降级 |

---

## 8. 配置管理

### 8.1 配置参数表

| 参数名 | 默认值 | 类型 | 说明 | 适用模式 |
|--------|--------|------|------|---------|
| `ws_host` | `localhost` | str | WebSocket 服务器监听地址 | sim |
| `ws_port` | `8080` | int | WebSocket 服务器监听端口（仅调试） | sim |
| `ws_path` | `/ws` | str | WebSocket 路径 | sim |
| `api_url` | `https://api.xiaozhi.cn/v1` | str | 小智云端 API 基础 URL | cloud |
| `api_timeout` | `30.0` | float | 云端 API 调用超时时间（秒） | cloud |
| `device_id` | 无（必填） | str | 真机设备 ID | cloud |
| `api_key` | `None` | str | 云端 API 认证 token，`None` 表示不携带 | cloud |
| `render_mode` | `human` | str | 仿真渲染模式：`human`/`rgb_array`/`None` | sim |
| `servo_centers` | `[180,140,0,40,90,90]` | list[float] | 舵机中心角度（6 维） | sim |
| `servo_ratios` | `[1.0,1.125,1.0,1.125,1.5,2.0]` | list[float] | 舵机→关节映射比（6 维） | sim |
| `servo_directions` | `[-1,-1,1,1,1,1]` | list[int] | 舵机方向（6 维，±1） | sim |
| `home_pose` | `[180,180,0,0,90,90]` | list[int] | home 姿态舵机角度（6 维） | sim |
| `sim_timestep` | MuJoCo 默认 | float | 仿真步长（秒），由模型 XML 决定 | sim |
| `max_reconnect` | `3` | int | WebSocket 最大重连次数 | sim 客户端 |
| `reconnect_backoff` | `[1,2,4]` | list[int] | 重连退避时间（秒） | sim 客户端 |

### 8.2 环境变量

| 变量名 | 用途 | 默认值 | 示例 |
|--------|------|--------|------|
| `ELECTRONBOT_SIM_WS_PORT` | 覆盖 WebSocket 调试端口 | `8080` | `8081` |
| `ELECTRONBOT_CLOUD_API_URL` | 覆盖云端 API URL | `https://api.xiaozhi.cn/v1` | `https://api.test.xiaozhi.cn/v1` |
| `ELECTRONBOT_CLOUD_API_KEY` | 云端 API 认证 token | 无 | `sk-xxxxx` |
| `ELECTRONBOT_CLOUD_DEVICE_ID` | 默认真机设备 ID | 无 | `eb-001` |
| `ELECTRONBOT_LOG_LEVEL` | 日志级别 | `INFO` | `DEBUG` |
| `ELECTRONBOT_SIM_RENDER` | 仿真渲染模式 | `human` | `rgb_array` |
| `ELECTRONBOT_BACKEND_MODE` | 默认 backend 模式 | `sim` | `cloud` |

**加载优先级**（从高到低）：
1. 构造函数显式参数：`ElectronBotBackend("cloud", api_url=..., api_key=...)`
2. 环境变量：`ELECTRONBOT_CLOUD_API_URL` 等
3. 代码内默认值

### 8.3 工具可用性矩阵

| 工具名 | sim 模式 | cloud 模式（release v2.2.6） | 备注 |
|--------|:---:|:---:|------|
| `self.electron.hand_action` | ✅ | ✅ | 真机可用 |
| `self.electron.body_turn` | ✅ | ✅ | 真机可用 |
| `self.electron.head_move` | ✅ | ✅ | 真机可用 |
| `self.electron.stop` | ✅ | ✅ | 真机可用 |
| `self.electron.get_status` | ✅ | ✅ | 真机可用 |
| `self.electron.set_trim` | ✅ | ✅ | 真机可用 |
| `self.electron.get_trims` | ✅ | ✅ | 真机可用 |
| `self.battery.get_level` | ✅ | ✅ | 真机可用 |
| `self.electron.servo_move` | ✅ | ❌ | @sim_only，需固件 OTA |
| `self.electron.servo_sequences` | ✅ | ❌ | @sim_only，需固件 OTA |
| `self.electron.home` | ✅ | ❌ | @sim_only，需固件 OTA |
| `self.electron.get_ip` | ✅ | ❌ | @sim_only，需固件 OTA |

**合计**：8 真机可用 + 4 sim_only = 12 工具

---

## 9. 日志与可观测性

### 9.1 日志规范

#### 9.1.1 日志格式

采用结构化日志（JSON Lines），便于后续聚合分析：

```json
{"ts":"2026-07-04T10:23:45.123Z","level":"INFO","module":"mcp_bridge","trace_id":3,"method":"tools/call","tool":"self.electron.hand_action","duration_ms":0.8,"status":"ok"}
```

#### 9.1.2 日志级别

| 级别 | 使用场景 |
|------|---------|
| `DEBUG` | 舵机↔关节转换中间值、MuJoCo 步进细节 |
| `INFO` | 工具调用入口/出口、WebSocket 连接建立/断开、配置加载 |
| `WARNING` | 未知工具名被调用（可能是 sim_only 工具误用）、参数接近安全边界 |
| `ERROR` | 工具执行异常、JSON 解析失败、云端 API 401/403/超时 |
| `CRITICAL` | 仿真状态 NaN/Inf、MuJoCo 步进失败、需要人工介入 |

#### 9.1.3 trace_id 传递

- **来源**：JSON-RPC 请求中的 `id` 字段
- **传递**：从 `handle_request` 入口注入，贯穿工具执行、日志记录、错误返回
- **扁平格式**：`id` 为 `null` 时，自动生成 `trace_id = "auto-" + uuid4()[:8]`
- **用途**：通过 `trace_id` 可串联单次请求的全部日志，便于问题定位

#### 9.1.4 关键日志字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `trace_id` | int/str | JSON-RPC 请求 ID |
| `method` | str | JSON-RPC method（`tools/call` 或工具名） |
| `tool` | str | 实际执行的工具名 |
| `args` | dict | 工具入参（脱敏后） |
| `duration_ms` | float | 工具执行耗时（毫秒） |
| `status` | str | `ok` / `error` |
| `error_code` | int | 错误码（仅 status=error 时） |
| `mode` | str | `sim` / `cloud` |
| `client` | str | WebSocket 客户端标识 |

### 9.2 关键指标

| 指标名 | 类型 | 说明 | 告警阈值 |
|--------|------|------|---------|
| `mcp_requests_total` | counter | MCP 请求总数（按工具名分维度） | - |
| `mcp_request_duration_ms` | histogram | 工具调用耗时分布 | sim p99 > 10ms；cloud p99 > 1000ms |
| `mcp_error_rate` | gauge | 错误率（按错误码分维度） | > 5% 持续 5 分钟 |
| `mcp_tool_not_found_total` | counter | 未知工具名调用次数 | > 0 即告警（可能是 sim_only 误用） |
| `ws_connections_active` | gauge | 当前 WebSocket 活跃连接数 | - |
| `ws_reconnect_total` | counter | WebSocket 重连次数 | 单客户端 > 3 次/小时 |
| `cloud_api_latency_ms` | histogram | 云端 API 调用延迟 | p99 > 2000ms |
| `cloud_auth_failures_total` | counter | 云端认证失败次数 | > 0 即告警 |
| `sim_state_corruption_total` | counter | 仿真状态异常次数 | > 0 即告警 |

**典型耗时基线**：

| 操作 | sim 模式 | cloud 模式 |
|------|---------|-----------|
| `get_status` / `get_trims` / `battery.get_level` / `get_ip` | < 0.1ms | 200-500ms |
| `servo_move` (speed=1000, 100 步插值) | 0.5-2ms | 200-500ms |
| `hand_action` (steps=3, 多次插值) | 2-5ms | 200-500ms |
| `home` (全舵机复位) | 3-8ms | 200-500ms |

---

## 10. 风险评估

### 10.1 技术风险

| 风险项 | 可能性 | 影响 | 风险等级 | 缓解措施 |
|--------|:---:|:---:|:---:|------|
| 仿真与真机协议一致性偏差 | 中 | 高 | **高** | 持续对照 `docs/mcp-protocol_zh.md` 与真机固件 `mcp_server.cc`；建立真机回归测试套件；每个 release 版本同步对齐 |
| 云端 API 变更导致通信中断 | 中 | 高 | **高** | 锁定 API 版本（`/v1`）；监控 API 变更公告；保持 `api_url` 可配置以快速切换备用域名 |
| 4 个 sim_only 工具误用到真机 | 高 | 中 | **高** | 在 `ElectronBotBackend.call` 中校验工具可用性矩阵；cloud 模式下拦截 sim_only 工具并返回明确错误；日志记录调用尝试 |
| WebSocket 调试服务器无认证风险 | 高 | 高 | **高** | 默认仅监听 `localhost`，禁止绑定 `0.0.0.0`；启动时打印安全警告；文档明确"仅调试用"；生产环境禁用 WebSocket 服务器 |
| 舵机转换参数与真机偏差 | 中 | 中 | **中** | 转换参数来源于 Phase 1 真机标定，定期用真机验证；参数可通过配置覆盖，便于现场调整 |
| MuJoCo 仿真步进阻塞 WebSocket | 中 | 中 | **中** | 长时间动作（如 `servo_sequences`）拆分为异步任务；客户端可主动断开取消 |
| JSON-RPC `id` 冲突 | 低 | 低 | **低** | 客户端建议使用递增整数；服务端不校验 id 唯一性，但日志按 id 关联 |

### 10.2 依赖风险

| 依赖项 | 版本要求 | 用途 | 风险 | 缓解措施 |
|--------|---------|------|------|---------|
| `websockets` | >=10.0 | WebSocket 服务器/客户端 | API 升级可能破坏兼容性 | 锁定版本范围；CI 覆盖端到端测试 |
| `httpx` | >=0.24 | 云端 API HTTP 客户端 | 异步行为变更 | 锁定版本；单元测试 mock HTTP |
| `numpy` | >=1.21 | 舵机角度数组运算 | 与 MuJoCo 版本耦合 | 随 MuJoCo 一同升级 |
| `mujoco` | >=2.3 | 仿真引擎 | 升级可能改变物理行为 | 锁定版本；回归测试验证关键动作 |
| 真机固件 | release v2.2.6 | 协议对齐基准 | 真机 OTA 升级会引入新工具或废弃旧工具 | 跟踪固件 release notes；建立协议版本映射表 |
| 小智云端 API | v1 | cloud 模式通信 | API 变更、限流、宕机 | 实现重试与降级；监控可用性 |
| Python | >=3.9 | 运行时 | 新版本可能移除依赖的特性 | CI 矩阵测试 3.9/3.10/3.11 |

### 10.3 风险监控与告警

- **协议一致性巡检**：每周自动运行真机回归测试套件，对比仿真与真机响应
- **sim_only 工具误用监控**：`mcp_tool_not_found_total` 指标在 cloud 模式下 > 0 即告警
- **WebSocket 安全审计**：定期检查服务器监听地址，禁止绑定公网 IP
- **云端 API 可用性监控**：`cloud_api_latency_ms` 与 `cloud_auth_failures_total` 联动告警

---

## 11. 变更记录

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|---------|------|
| v1.0 | 2026-07-04 | 初版：MCP Bridge 协议层详细设计，包含架构、验证方法、交付物清单 | 架构组 |
| v1.1 | 2026-07-04 | 补充软件工程规范章节：接口设计、数据模型、错误处理、配置管理、日志与可观测性、风险评估 | 架构组 |
