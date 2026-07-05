"""MCP Bridge 测试 — 对齐 Phase 3 §3.

测试:
  - 全部 12 个 MCP 工具 (tools/call 格式)
  - 扁平格式兼容性
  - 舵机↔关节转换正确性
  - servo_sequences 振荡器
"""
from __future__ import annotations

import json
import os

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "osmesa")


@pytest.fixture
def bridge():
    """创建 MCP Bridge fixture."""
    from electronbot_sim.env import ElectronBotEnv
    from electronbot_sim.mcp_bridge import McpSimBridge

    env = ElectronBotEnv(render_mode=None)
    br = McpSimBridge(env)
    env.reset()
    yield br
    env.close()


def _call(bridge, tool_name: str, args: dict) -> dict:
    """用标准 tools/call 格式调用工具."""
    return bridge.handle_request({
        "jsonrpc": "2.0",
        "method": "tools/call",
        "params": {"name": tool_name, "arguments": args},
        "id": 1,
    })


class TestAllTools:
    """全部 12 个 MCP 工具测试."""

    def test_home(self, bridge):
        result = _call(bridge, "self.electron.home", {})
        assert "error" not in result
        assert result["result"]["isError"] == False

    def test_get_status(self, bridge):
        result = _call(bridge, "self.electron.get_status", {})
        assert "error" not in result
        text = result["result"]["content"][0]["text"]
        assert "idle" in text or "moving" in text

    def test_servo_move_all_types(self, bridge):
        """6 种舵机类型都应可调用."""
        for servo_type, position in [("rp", 120), ("rr", 160), ("lp", 60),
                                      ("lr", 20), ("b", 60), ("h", 100)]:
            result = _call(bridge, "self.electron.servo_move",
                          {"servo_type": servo_type, "position": position, "speed": 300})
            assert "error" not in result, f"servo_move {servo_type} 失败"

    def test_hand_action(self, bridge):
        result = _call(bridge, "self.electron.hand_action",
                      {"action": 3, "hand": 3, "steps": 1, "speed": 300})
        assert "error" not in result

    def test_body_turn(self, bridge):
        result = _call(bridge, "self.electron.body_turn",
                      {"direction": 1, "speed": 300, "angle": 30})
        assert "error" not in result

    def test_head_move(self, bridge):
        result = _call(bridge, "self.electron.head_move",
                      {"action": 3, "speed": 300, "angle": 5})
        assert "error" not in result

    def test_stop(self, bridge):
        result = _call(bridge, "self.electron.stop", {})
        assert "error" not in result

    def test_set_get_trim(self, bridge):
        """set_trim + get_trims."""
        result = _call(bridge, "self.electron.set_trim",
                      {"servo_type": "rp", "trim_value": 5})
        assert "error" not in result

        result = _call(bridge, "self.electron.get_trims", {})
        assert "error" not in result

    def test_battery_level(self, bridge):
        result = _call(bridge, "self.battery.get_level", {})
        assert "error" not in result

    def test_get_ip(self, bridge):
        result = _call(bridge, "self.electron.get_ip", {})
        assert "error" not in result

    def test_servo_sequences(self, bridge):
        """servo_sequences 含振荡器."""
        seq = json.dumps({
            "a": [
                {"s": {"rp": 90, "lp": 90}, "v": 300},
                {"osc": {"a": {"rp": 20}, "o": {"rp": 120}, "p": 300, "c": 2}},
            ]
        })
        result = _call(bridge, "self.electron.servo_sequences",
                      {"sequence": seq})
        assert "error" not in result


class TestFlatFormat:
    """扁平格式兼容性测试."""

    def test_flat_get_status(self, bridge):
        """扁平格式应与 tools/call 格式兼容."""
        result = bridge.handle_request({
            "jsonrpc": "2.0",
            "method": "self.electron.get_status",
            "params": {},
        })
        assert "error" not in result
        assert result["result"]["isError"] == False


class TestServoJointConversion:
    """舵机↔关节转换测试."""

    def test_home_conversion(self, bridge):
        """home 姿态下舵机↔关节转换正确."""
        from electronbot_sim.env import SERVO_HOME, HOME_QPOS, servo_to_joint
        for i in range(6):
            joint = servo_to_joint(i, SERVO_HOME[i])
            assert abs(joint - HOME_QPOS[i]) < 0.5, \
                f"关节 {i}: 期望 {HOME_QPOS[i]}, 实际 {joint}"

    def test_roundtrip_conversion(self, bridge):
        """舵机→关节→舵机 往返转换应一致."""
        from electronbot_sim.env import servo_to_joint, joint_to_servo
        for i in range(6):
            for angle in [0, 45, 90, 135, 180]:
                joint = servo_to_joint(i, angle)
                back = joint_to_servo(i, joint)
                assert abs(back - angle) < 0.5, \
                    f"往返转换不一致: servo={angle} → joint={joint} → servo={back}"


class TestErrorHandling:
    """错误处理测试."""

    def test_unknown_tool(self, bridge):
        """未知工具应返回 -32601 错误."""
        result = _call(bridge, "self.electron.unknown_tool", {})
        assert "error" in result
        assert result["error"]["code"] == -32601

    def test_invalid_servo_type(self, bridge):
        """无效舵机类型应返回错误."""
        result = _call(bridge, "self.electron.servo_move",
                      {"servo_type": "invalid", "position": 90})
        assert "error" in result
