"""MCP Bridge 协议层 — 仿真端 JSON-RPC 桥接器.

对齐 docs/tasks/03-MCP-Bridge 详细设计说明书.
对齐真机: xiaozhi-esp32 release v2.2.6 的 mcp_server.cc + movements.cc

═══════════════════════════════════════════════════════════════════
  设计原则 (嵌入式固件工程师强制规范)
═══════════════════════════════════════════════════════════════════
  1. 协议格式与真机 1:1 对齐: tools/call 两层嵌套 JSON-RPC 2.0
  2. 舵机↔关节转换: 复用 env.py 的常量与函数 (单一数据源)
  3. 线性插值: 对齐 movements.cc:87 MoveServos(), 禁用缓动
  4. 安全裁剪: 对齐 ClampServoTarget(), 6 组硬限位
  5. 角度单位: Python 层用度数, 写 ctrl 前通过 env.apply_joint_targets_deg() 转弧度
  6. 12 个 MCP 工具: 8 个真机对齐 + 4 个 @sim_only

  MCP 协议结构 (与真机一致):
  {
    "type": "mcp",
    "payload": {
      "jsonrpc": "2.0",
      "method": "tools/call",
      "params": {"name": "self.electron.hand_action", "arguments": {...}},
      "id": 3
    }
  }
"""
from __future__ import annotations

import json
import logging
import time
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from .env import (
    SERVO_CENTER,
    SERVO_DIRECTION,
    SERVO_HOME,
    SERVO_LIMITS,
    SERVO_NAME_TO_INDEX,
    SERVO_RATIO,
    clamp_servo_target,
    joint_array_to_servo_array,
    joint_to_servo,
    servo_array_to_joint_array,
    servo_to_joint,
)

logger = logging.getLogger("electronbot_sim.mcp_bridge")


# ═══════════════════════════════════════════════════════════════════
#  JSON-RPC 错误码 (对齐 JSON-RPC 2.0 规范)
# ═══════════════════════════════════════════════════════════════════
class JsonRpcError:
    PARSE_ERROR = -32700      # 解析错误
    INVALID_REQUEST = -32600  # 无效请求
    METHOD_NOT_FOUND = -32601  # 方法未找到
    INVALID_PARAMS = -32602   # 无效参数
    INTERNAL_ERROR = -32603   # 内部错误


# ═══════════════════════════════════════════════════════════════════
#  MCP 工具注册表 (12 个, 对齐真机 release v2.2.6)
# ═══════════════════════════════════════════════════════════════════
# @sim_only 标记的工具在真机 release v2.2.6 上不可用
MCP_TOOLS: List[Dict[str, Any]] = [
    # ── 8 个真机对齐工具 ──
    {
        "name": "self.electron.hand_action",
        "description": "预设手部动作: 1=举手 2=放手 3=挥手 4=拍打",
        "sim_only": False,
        "input_schema": {
            "type": "object",
            "properties": {
                "action": {"type": "integer", "enum": [1, 2, 3, 4]},
                "hand": {"type": "integer", "enum": [1, 2, 3], "default": 3},
                "steps": {"type": "integer", "minimum": 1, "maximum": 100, "default": 1},
                "speed": {"type": "integer", "minimum": 100, "maximum": 2000, "default": 1000},
                "amount": {"type": "integer", "minimum": 10, "maximum": 50, "default": 30},
            },
            "required": ["action"],
        },
    },
    {
        "name": "self.electron.body_turn",
        "description": "身体转向: 1=左转 2=右转 3=回中心",
        "sim_only": False,
        "input_schema": {
            "type": "object",
            "properties": {
                "direction": {"type": "integer", "enum": [1, 2, 3]},
                "angle": {"type": "integer", "minimum": 0, "maximum": 90, "default": 45},
                "speed": {"type": "integer", "minimum": 100, "maximum": 2000, "default": 1000},
                "steps": {"type": "integer", "minimum": 1, "maximum": 10, "default": 1},
            },
            "required": ["direction"],
        },
    },
    {
        "name": "self.electron.head_move",
        "description": "头部动作: 1=抬头 2=低头 3=点头 4=回中心 5=连续点头",
        "sim_only": False,
        "input_schema": {
            "type": "object",
            "properties": {
                "action": {"type": "integer", "enum": [1, 2, 3, 4, 5]},
                "angle": {"type": "integer", "minimum": 1, "maximum": 15, "default": 5},
                "speed": {"type": "integer", "minimum": 100, "maximum": 2000, "default": 500},
                "steps": {"type": "integer", "minimum": 1, "maximum": 10, "default": 1},
            },
            "required": ["action"],
        },
    },
    {
        "name": "self.electron.stop",
        "description": "紧急停止并复位",
        "sim_only": False,
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "self.electron.get_status",
        "description": "查询运动状态, 返回 'idle' 或 'moving'",
        "sim_only": False,
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "self.electron.set_trim",
        "description": "设置舵机微调 (写入 NVS)",
        "sim_only": False,
        "input_schema": {
            "type": "object",
            "properties": {
                "servo_type": {"type": "string", "enum": list(SERVO_NAME_TO_INDEX.keys())},
                "trim_value": {"type": "number", "minimum": -30, "maximum": 30},
            },
            "required": ["servo_type", "trim_value"],
        },
    },
    {
        "name": "self.electron.get_trims",
        "description": "查询所有舵机微调值",
        "sim_only": False,
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "self.battery.get_level",
        "description": "查询电池电量和充电状态",
        "sim_only": False,
        "input_schema": {"type": "object", "properties": {}},
    },
    # ── 4 个 @sim_only 工具 (真机 release v2.2.6 不可用) ──
    {
        "name": "self.electron.servo_move",
        "description": "[@sim_only] 单舵机精确定位 (线性插值)",
        "sim_only": True,
        "input_schema": {
            "type": "object",
            "properties": {
                "servo_type": {"type": "string", "enum": list(SERVO_NAME_TO_INDEX.keys())},
                "position": {"type": "number"},
                "speed": {"type": "integer", "minimum": 10, "maximum": 3000, "default": 1000},
            },
            "required": ["servo_type", "position"],
        },
    },
    {
        "name": "self.electron.servo_sequences",
        "description": "[@sim_only] 执行 AI 生成的动作序列 (含振荡)",
        "sim_only": True,
        "input_schema": {
            "type": "object",
            "properties": {
                "sequence": {"type": "string", "description": "JSON 序列字符串"},
            },
            "required": ["sequence"],
        },
    },
    {
        "name": "self.electron.home",
        "description": "[@sim_only] 复位到初始姿态",
        "sim_only": True,
        "input_schema": {
            "type": "object",
            "properties": {
                "speed": {"type": "integer", "minimum": 100, "maximum": 3000, "default": 1000},
            },
        },
    },
    {
        "name": "self.electron.get_ip",
        "description": "[@sim_only] 查询设备 IP 地址",
        "sim_only": True,
        "input_schema": {"type": "object", "properties": {}},
    },
]

# 工具名 → 处理函数名的映射 (在 McpSimBridge._register_tools 中填充)
_TOOL_HANDLERS: Dict[str, str] = {
    "self.electron.hand_action": "_hand_action",
    "self.electron.body_turn": "_body_turn",
    "self.electron.head_move": "_head_move",
    "self.electron.stop": "_stop",
    "self.electron.get_status": "_get_status",
    "self.electron.set_trim": "_set_trim",
    "self.electron.get_trims": "_get_trims",
    "self.battery.get_level": "_battery_level",
    "self.electron.servo_move": "_servo_move",
    "self.electron.servo_sequences": "_servo_sequences",
    "self.electron.home": "_home",
    "self.electron.get_ip": "_get_ip",
}


class McpSimBridge:
    """仿真端 MCP JSON-RPC 桥接器.

    实现与真机固件完全一致的 MCP 协议, 12 个工具 (8 真机对齐 + 4 @sim_only).
    舵机↔关节转换复用 env.py 的常量 (单一数据源).
    线性插值对齐 movements.cc:87 MoveServos().

    使用示例:
        env = ElectronBotEnv()
        bridge = McpSimBridge(env)
        result = bridge.handle_request({
            "jsonrpc": "2.0",
            "method": "tools/call",
            "params": {"name": "self.electron.hand_action",
                       "arguments": {"action": 3, "hand": 3, "steps": 2, "speed": 600}},
            "id": 1
        })
    """

    def __init__(self, env: "ElectronBotEnv"):
        self.env = env
        # trim 偏置 (6 维), 对齐固件 SetTrims(), 单位: 舵机度数
        self._trims = np.zeros(6, dtype=np.float32)
        # 动作状态标志 (对齐固件 _moving)
        self._moving = False
        # 设备 IP (仿真虚构)
        self._device_ip = "127.0.0.1"
        # 工具处理函数字典
        self._handlers: Dict[str, Callable] = {}
        self._register_tools()
        logger.info("McpSimBridge 初始化完成, 注册 %d 个工具", len(self._handlers))

    # ================================================================
    #  工具注册
    # ================================================================
    def _register_tools(self) -> None:
        """注册 12 个 MCP 工具处理函数."""
        for tool_name, handler_name in _TOOL_HANDLERS.items():
            handler = getattr(self, handler_name, None)
            if handler is None:
                logger.error("工具处理函数缺失: %s", handler_name)
                continue
            self._handlers[tool_name] = handler

    def list_tools(self) -> List[Dict[str, Any]]:
        """返回工具列表 (对齐真机 list_tools 响应)."""
        return [
            {
                "name": t["name"],
                "description": t["description"],
                "inputSchema": t["input_schema"],
                "sim_only": t["sim_only"],
            }
            for t in MCP_TOOLS
        ]

    # ================================================================
    #  JSON-RPC 请求处理
    # ================================================================
    def handle_request(self, request: Dict) -> Dict:
        """处理 JSON-RPC 请求, 支持两种格式.

        格式 1 (标准 MCP, 与真机一致):
            {"method": "tools/call", "params": {"name": "self.electron.xxx", "arguments": {...}}}

        格式 2 (扁平, 仿真内部调试):
            {"method": "self.electron.xxx", "params": {...}}

        返回 JSON-RPC 2.0 响应:
            成功: {"jsonrpc": "2.0", "id": <id>, "result": {"content": [...], "isError": false}}
            失败: {"jsonrpc": "2.0", "id": <id>, "error": {"code": <int>, "message": <str>}}
        """
        if not isinstance(request, dict):
            return self._error_response(None, JsonRpcError.INVALID_REQUEST,
                                        "请求必须是 JSON 对象")

        req_id = request.get("id")
        method = request.get("method", "")

        try:
            if method == "tools/call":
                # 标准 MCP 格式
                params = request.get("params", {})
                tool_name = params.get("name", "")
                tool_args = params.get("arguments", {})
                if not tool_name:
                    return self._error_response(req_id, JsonRpcError.INVALID_PARAMS,
                                                "缺少工具名 (params.name)")
                result = self._call_tool(tool_name, tool_args)
                return self._success_response(req_id, result)
            elif method == "tools/list":
                # 工具列表查询
                return self._success_response(req_id, {"tools": self.list_tools()})
            elif method.startswith("self."):
                # 扁平格式 (仿真内部调试)
                tool_name = method
                tool_args = request.get("params", {})
                result = self._call_tool(tool_name, tool_args)
                # 扁平格式返回简化 result (不带 content 包装)
                return {"jsonrpc": "2.0", "id": req_id, "result": result}
            else:
                return self._error_response(req_id, JsonRpcError.METHOD_NOT_FOUND,
                                            f"未知方法: {method}")
        except Exception as e:
            logger.exception("工具调用异常: %s", e)
            return self._error_response(req_id, JsonRpcError.INTERNAL_ERROR,
                                        f"内部错误: {e}")

    def _call_tool(self, tool_name: str, args: Dict) -> Any:
        """调用工具处理函数, 返回结果."""
        handler = self._handlers.get(tool_name)
        if handler is None:
            raise ValueError(f"未注册的工具: {tool_name}")
        logger.debug("调用工具 %s, 参数: %s", tool_name, args)
        return handler(**args)

    def _success_response(self, req_id: Any, result: Any) -> Dict:
        """构造 JSON-RPC 成功响应 (对齐真机 result.content 格式)."""
        text = result if isinstance(result, str) else json.dumps(result, ensure_ascii=False)
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "content": [{"type": "text", "text": text}],
                "isError": False,
            },
        }

    def _error_response(self, req_id: Any, code: int, message: str) -> Dict:
        """构造 JSON-RPC 错误响应."""
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": code, "message": message},
        }

    # ================================================================
    #  舵机↔关节转换 (委托给 env.py, 单一数据源)
    # ================================================================
    def _servo_to_joint(self, servo_index: int, servo_angle: float) -> float:
        """舵机角度 → 机械关节角度 (度)."""
        return servo_to_joint(servo_index, servo_angle)

    def _joint_to_servo(self, servo_index: int, joint_angle: float) -> float:
        """机械关节角度 → 舵机角度 (度)."""
        return joint_to_servo(servo_index, joint_angle)

    def _servo_index_from_name(self, servo_type: str) -> int:
        """舵机名称 → 索引, 无效名称抛 ValueError."""
        idx = SERVO_NAME_TO_INDEX.get(servo_type.lower())
        if idx is None:
            raise ValueError(
                f"无效舵机类型: {servo_type}, 可选: {list(SERVO_NAME_TO_INDEX.keys())}"
            )
        return idx

    # ================================================================
    #  底层运动控制 (对齐固件 movements.cc)
    # ================================================================
    def _move_all_servos(self, servo_targets: np.ndarray, time_ms: int) -> None:
        """6 舵机联动缓动 (线性插值, 对齐 movements.cc:87 MoveServos).

        固件实现:
            increment_[i] = (target[i] - pos[i]) / (time / 10.0)
            每 10ms 步进等量增加 → 纯线性插值

        参数:
            servo_targets: (6,) 舵机目标角度 (度, 含 trim)
            time_ms: 总时长 (毫秒), 10ms 步进
        """
        servo_targets = np.asarray(servo_targets, dtype=np.float32).reshape(-1)
        if servo_targets.shape[0] != 6:
            logger.error("_move_all_servos: 维度不匹配 %s", servo_targets.shape)
            return

        # 安全裁剪 (对齐 ClampServoTarget)
        clamped = np.array([
            clamp_servo_target(i, servo_targets[i]) for i in range(6)
        ], dtype=np.float32)

        # 当前舵机角度 (由机械关节角度反推)
        current_joint = self.env._get_joint_angles_deg()
        current_servo = joint_array_to_servo_array(current_joint)

        # time_ms <= 10 → 直接到位, 不插值
        if time_ms <= 10:
            target_joint = servo_array_to_joint_array(clamped)
            self.env.apply_joint_targets_deg(target_joint)
            self.env.step_simulation()
            return

        # 线性插值 (对齐固件: steps = time / 10.0, 每步 increment 恒定)
        steps = max(1, time_ms // 10)
        self._moving = True
        self.env.set_moving_state(True)

        for step in range(1, steps + 1):
            t = step / steps  # 线性进度, 0→1
            interp_servo = current_servo + (clamped - current_servo) * t
            interp_joint = servo_array_to_joint_array(interp_servo)
            self.env.apply_joint_targets_deg(interp_joint)
            if not self.env.step_simulation():
                logger.warning("仿真状态非法, 中断插值")
                break

        self._moving = False
        self.env.set_moving_state(False)

    def _step_sim(self, joint_targets_deg: np.ndarray) -> None:
        """直接设置机械关节目标并推进一步仿真 (单步, 供 servo_move 等使用).

        参数:
            joint_targets_deg: (6,) 机械关节角度 (度)
        """
        self.env.apply_joint_targets_deg(joint_targets_deg)
        self.env.step_simulation()

    # ================================================================
    #  8 个真机对齐工具
    # ================================================================
    def _hand_action(self, action: int, hand: int = 3, steps: int = 1,
                     speed: int = 1000, amount: int = 30, **kwargs) -> Dict:
        """预设手部动作 (对齐固件 HandAction).

        action: 1=举手 2=放手 3=挥手 4=拍打
        hand:   1=左手 2=右手 3=双手
        steps:  重复次数 (固件: times = 2 * max(3, min(100, steps)))
        speed:  动作速度 (ms), 越小越快
        amount: 动作幅度 (10-50), 仅举手使用
        """
        if action not in (1, 2, 3, 4):
            return {"error": "action 必须为 1-4"}
        if hand not in (1, 2, 3):
            return {"error": "hand 必须为 1-3"}

        # 固件 times 限制 (对齐 movements.cc:225)
        times = 2 * max(3, min(100, int(steps)))

        # 当前舵机角度
        current_joint = self.env._get_joint_angles_deg()
        current_servo = joint_array_to_servo_array(current_joint)

        # 根据动作类型构造目标
        # 索引: 0=RP 1=RR 2=LP 3=LR 4=BODY 5=HEAD
        for _ in range(times):
            if action == 1:  # 举手
                targets = current_servo.copy()
                if hand in (1, 3):  # 左手
                    targets[2] = 180  # LP 上举
                if hand in (2, 3):  # 右手
                    targets[0] = 0    # RP 上举 (反向, 0=最高)
                self._move_all_servos(targets, speed)

            elif action == 2:  # 放手
                targets = current_servo.copy()
                if hand in (1, 3):
                    targets[2] = 0    # LP 下放
                if hand in (2, 3):
                    targets[0] = 180  # RP 下放 (反向, 180=最低)
                self._move_all_servos(targets, speed)

            elif action == 3:  # 挥手
                # 挥手: 抬起 → 摆动 → 放下
                # 起始位: 抬手
                raise_targets = current_servo.copy()
                if hand in (1, 3):
                    raise_targets[2] = 150  # LP 抬起位
                if hand in (2, 3):
                    raise_targets[0] = 30   # RP 抬起位
                self._move_all_servos(raise_targets, speed)

                # 摆动 (roll 来回)
                wave_left = raise_targets.copy()
                wave_right = raise_targets.copy()
                if hand in (1, 3):
                    wave_left[3] = 60   # LR 外摆
                    wave_right[3] = 20  # LR 内收
                if hand in (2, 3):
                    wave_left[1] = 120  # RR 外摆
                    wave_right[1] = 160  # RR 内收
                self._move_all_servos(wave_left, speed // 2)
                self._move_all_servos(wave_right, speed // 2)

            elif action == 4:  # 拍打
                amount = min(amount, 40)  # 固件上限
                targets = current_servo.copy()
                if hand in (1, 3):
                    # LP 上下拍打
                    targets[2] = 90 + amount
                if hand in (2, 3):
                    # RP 上下拍打 (反向)
                    targets[0] = 90 - amount
                self._move_all_servos(targets, speed)
                # 回到原位
                self._move_all_servos(current_servo, speed)

        return {"status": "ok", "action": action, "hand": hand, "times": times}

    def _body_turn(self, direction: int, angle: int = 45,
                   speed: int = 1000, steps: int = 1, **kwargs) -> Dict:
        """身体转向 (对齐固件 BodyAction).

        direction: 1=左转 2=右转 3=回中心
        angle: 转动角度 (0-90°)
        """
        if direction not in (1, 2, 3):
            return {"error": "direction 必须为 1-3"}
        angle = max(0, min(90, int(angle)))

        # 舵机 body 安全范围 [30, 150], 中心 90
        # 左转: 90 + angle (映射后关节角度增加, 但需裁剪到 150)
        # 右转: 90 - angle (裁剪到 30)
        # 回中: 90
        if direction == 1:
            target_servo_body = min(150, 90 + angle)
        elif direction == 2:
            target_servo_body = max(30, 90 - angle)
        else:
            target_servo_body = 90

        current_servo = joint_array_to_servo_array(self.env._get_joint_angles_deg())
        for _ in range(max(1, int(steps))):
            targets = current_servo.copy()
            targets[4] = target_servo_body
            self._move_all_servos(targets, speed)

        return {"status": "ok", "direction": direction, "angle": angle}

    def _head_move(self, action: int, angle: int = 5,
                   speed: int = 500, steps: int = 1, **kwargs) -> Dict:
        """头部动作 (对齐固件 HeadAction).

        action: 1=抬头 2=低头 3=点头 4=回中心 5=连续点头
        angle: 头部角度 (1-15°)
        """
        if action not in (1, 2, 3, 4, 5):
            return {"error": "action 必须为 1-5"}
        angle = max(1, min(15, int(angle)))

        # 舵机 head 安全范围 [75, 105], 中心 90
        # 抬头: 90 + angle (裁剪到 105)
        # 低头: 90 - angle (裁剪到 75)
        # 回中: 90
        # 点头: 抬起 → 低下 → 回中
        if action == 1:
            target_head = min(105, 90 + angle)
            current_servo = joint_array_to_servo_array(self.env._get_joint_angles_deg())
            targets = current_servo.copy()
            targets[5] = target_head
            self._move_all_servos(targets, speed)
        elif action == 2:
            target_head = max(75, 90 - angle)
            current_servo = joint_array_to_servo_array(self.env._get_joint_angles_deg())
            targets = current_servo.copy()
            targets[5] = target_head
            self._move_all_servos(targets, speed)
        elif action == 3:  # 点头
            self._head_move(1, angle, speed, steps)  # 抬头
            self._head_move(2, angle, speed, steps)  # 低头
            self._head_move(4, 0, speed, steps)      # 回中
        elif action == 4:  # 回中心
            current_servo = joint_array_to_servo_array(self.env._get_joint_angles_deg())
            targets = current_servo.copy()
            targets[5] = 90
            self._move_all_servos(targets, speed)
        elif action == 5:  # 连续点头
            for _ in range(max(1, int(steps))):
                self._head_move(3, angle, speed, 1)

        return {"status": "ok", "action": action, "angle": angle}

    def _stop(self, **kwargs) -> Dict:
        """紧急停止并复位 (对齐固件 Stop)."""
        self._moving = False
        self.env.set_moving_state(False)
        # 复位到 home
        self._home(speed=500)
        return {"status": "stopped"}

    def _get_status(self, **kwargs) -> Dict:
        """查询运动状态 (对齐固件 GetStatus)."""
        return {"status": "moving" if self._moving else "idle"}

    def _set_trim(self, servo_type: str, trim_value: float, **kwargs) -> Dict:
        """设置舵机微调 (对齐固件 SetTrim, 写入 NVS).

        仿真中保存在内存, 不持久化.
        """
        idx = self._servo_index_from_name(servo_type)
        trim_value = max(-30.0, min(30.0, float(trim_value)))
        self._trims[idx] = trim_value
        logger.info("设置 trim: %s[%d] = %.2f", servo_type, idx, trim_value)
        return {"status": "ok", "servo": servo_type, "trim": trim_value}

    def _get_trims(self, **kwargs) -> Dict:
        """查询所有舵机微调值 (对齐固件 GetTrims)."""
        names = ["rp", "rr", "lp", "lr", "body", "head"]
        return {
            "trims": {name: float(self._trims[i]) for i, name in enumerate(names)},
        }

    def _battery_level(self, **kwargs) -> Dict:
        """查询电池电量 (对齐固件 battery.get_level)."""
        info = self.env.get_battery_info()
        return {
            "level": int(info["percent"]),
            "voltage": info["voltage"],
            "is_charging": info["is_charging"],
        }

    # ================================================================
    #  4 个 @sim_only 工具
    # ================================================================
    def _servo_move(self, servo_type: str, position: float,
                    speed: int = 1000, **kwargs) -> Dict:
        """[@sim_only] 单舵机精确定位 (线性插值).

        对齐固件 MoveServos (但真机 release v2.2.6 未暴露此工具).
        """
        idx = self._servo_index_from_name(servo_type)
        # 安全裁剪
        clamped_pos = clamp_servo_target(idx, position)

        # 当前舵机角度
        current_servo = joint_array_to_servo_array(self.env._get_joint_angles_deg())
        # 仅移动指定舵机
        targets = current_servo.copy()
        targets[idx] = clamped_pos
        # 加 trim
        targets[idx] += self._trims[idx]
        self._move_all_servos(targets, speed)

        return {"status": "ok", "servo": servo_type, "position": clamped_pos}

    def _servo_sequences(self, sequence: str, **kwargs) -> Dict:
        """[@sim_only] 执行 AI 生成的动作序列 (含振荡).

        JSON 格式 (对齐 ExecuteServoSequence):
        {
          "a": [
            {"s": {"rp": 90, "lp": 90, "h": 100}, "v": 800, "d": 0},
            {"osc": {"a": {"rp": 20}, "o": {"rp": 120}, "p": 400, "c": 4}}
          ],
          "d": 0
        }
        """
        try:
            seq = json.loads(sequence) if isinstance(sequence, str) else sequence
        except json.JSONDecodeError as e:
            return {"error": f"序列 JSON 解析失败: {e}"}

        actions = seq.get("a", [])
        initial_delay = seq.get("d", 0)
        if initial_delay > 0:
            time.sleep(initial_delay / 1000.0)

        executed = 0
        for act in actions:
            if "s" in act:  # 普通动作
                servo_dict = act["s"]
                velocity = act.get("v", 1000)
                delay = act.get("d", 0)
                # 构造 6 维目标
                current_servo = joint_array_to_servo_array(self.env._get_joint_angles_deg())
                targets = current_servo.copy()
                for name, val in servo_dict.items():
                    if name in SERVO_NAME_TO_INDEX:
                        idx = SERVO_NAME_TO_INDEX[name]
                        targets[idx] = clamp_servo_target(idx, val)
                self._move_all_servos(targets, velocity)
                if delay > 0:
                    time.sleep(delay / 1000.0)
                executed += 1

            elif "osc" in act:  # 振荡器
                osc = act["osc"]
                amplitudes = osc.get("a", {})  # 振幅
                centers = osc.get("o", {})     # 中心
                period = osc.get("p", 400)     # 周期 ms
                cycles = osc.get("c", 1)       # 周期数
                self._oscillate(amplitudes, centers, period, cycles)
                executed += 1

        return {"status": "ok", "executed": executed}

    def _oscillate(self, amplitudes: Dict, centers: Dict,
                   period: int, cycles: int) -> None:
        """正弦振荡器 (对齐固件 OscillateServos).

        固件实现:
            vTaskDelay(5)  // 5 ticks @ 100Hz = 50ms 采样
            targets[i] = centers[i] + amplitudes[i] * sin(phase)
        """
        # 50ms 采样 (对齐固件 vTaskDelay(5) @ 100Hz)
        dt_sample = 0.05
        total_steps = int(period * cycles / 1000.0 / dt_sample)
        if total_steps <= 0:
            return

        self._moving = True
        self.env.set_moving_state(True)

        # 构造振幅/中心数组 (6 维)
        amp = np.zeros(6, dtype=np.float32)
        cen = joint_array_to_servo_array(self.env._get_joint_angles_deg())
        for name, val in amplitudes.items():
            if name in SERVO_NAME_TO_INDEX:
                amp[SERVO_NAME_TO_INDEX[name]] = val
        for name, val in centers.items():
            if name in SERVO_NAME_TO_INDEX:
                cen[SERVO_NAME_TO_INDEX[name]] = val

        for step in range(total_steps):
            phase = 2 * np.pi * step * dt_sample / (period / 1000.0)
            targets = cen + amp * np.sin(phase)
            # 安全裁剪
            targets = np.array([
                clamp_servo_target(i, targets[i]) for i in range(6)
            ], dtype=np.float32)
            target_joint = servo_array_to_joint_array(targets)
            self.env.apply_joint_targets_deg(target_joint)
            if not self.env.step_simulation():
                break
            time.sleep(dt_sample)  # 50ms 节流 (对齐固件 vTaskDelay)

        self._moving = False
        self.env.set_moving_state(False)

    def _home(self, speed: int = 1000, **kwargs) -> Dict:
        """[@sim_only] 复位到初始姿态.

        舵机 home: [180, 180, 0, 0, 90, 90] (对齐固件 servo_initial)
        """
        self._move_all_servos(SERVO_HOME.copy(), speed)
        return {"status": "ok", "position": SERVO_HOME.tolist()}

    def _get_ip(self, **kwargs) -> Dict:
        """[@sim_only] 查询设备 IP (仿真虚构)."""
        return {"ip": self._device_ip}

    # ================================================================
    #  状态查询 (供 Backend / 调试用)
    # ================================================================
    def get_servo_state(self) -> np.ndarray:
        """获取当前 6 舵机角度 (度)."""
        return joint_array_to_servo_array(self.env._get_joint_angles_deg())

    @property
    def is_moving(self) -> bool:
        """当前是否在执行动作."""
        return self._moving
