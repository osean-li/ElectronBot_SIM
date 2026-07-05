"""统一 Backend API — AI 策略访问机器人的唯一入口.

对齐 docs/tasks/03-MCP-Bridge 详细设计说明书 §3.4.
对齐 docs/概要设计/ElectronBot_SIM-概要设计文档.md §6.4.

═══════════════════════════════════════════════════════════════════
  设计目标
═══════════════════════════════════════════════════════════════════
  AI 策略层通过此类访问机器人, 完全不感知下面是仿真还是真机.
  切换 sim ↔ cloud 仅需改 mode 参数, 调用代码完全不变.

  使用示例:
      # 仿真
      backend = ElectronBotBackend("sim")
      backend.call("self.electron.hand_action", {"action":3,"hand":3,"steps":2,"speed":600})

      # 真机 (云端 API 透传, release v2.2.6)
      backend = ElectronBotBackend("cloud",
          api_url="https://api.xiaozhi.cn/v1", device_id="eb-001")
      backend.call("self.electron.hand_action", {"action":3,"hand":3,"steps":2,"speed":600})
      # ^^^ 完全相同的调用方式!
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Dict, Literal, Optional

logger = logging.getLogger("electronbot_sim.backend")


class ElectronBotBackend:
    """统一后端 API — 仿真/真机无缝切换.

    mode="sim":   连接仿真 MCP Bridge (本地 MuJoCo), 同步调用, <1ms 延迟
    mode="cloud": 连接云端小智 API (真机 ESP32), HTTP 调用, 200-500ms RTT

    参数 (sim 模式):
        env: ElectronBotEnv 实例 (可选, 若不提供则自动创建)

    参数 (cloud 模式):
        api_url:    云端 API 基地址, 如 "https://api.xiaozhi.cn/v1"
        device_id:  设备 ID, 如 "eb-001"
        api_key:    API 密钥 (Bearer token)
        timeout:    HTTP 超时秒数, 默认 30
    """

    def __init__(self, mode: Literal["sim", "cloud"] = "sim", **kwargs: Any):
        self.mode = mode
        self._sim_bridge = None
        self._cloud_bridge = None

        if mode == "sim":
            env = kwargs.get("env")
            if env is None:
                # 延迟创建 env, 避免导入时加载 MuJoCo
                from .env import ElectronBotEnv
                env = ElectronBotEnv(**{k: v for k, v in kwargs.items()
                                        if k in ("render_mode", "obs_mode",
                                                 "friction_range", "gain_range",
                                                 "mass_range", "servo_deadband",
                                                 "battery_voltage", "max_episode_steps")})
            from .mcp_bridge import McpSimBridge
            self._sim_bridge = McpSimBridge(env)
            logger.info("Backend[sim] 初始化完成")

        elif mode == "cloud":
            self._cloud_kwargs = {
                "api_url": kwargs.get("api_url", "https://api.xiaozhi.cn/v1"),
                "device_id": kwargs.get("device_id"),
                "api_key": kwargs.get("api_key"),
                "timeout": kwargs.get("timeout", 30.0),
            }
            if not self._cloud_kwargs["device_id"]:
                raise ValueError("cloud 模式必须提供 device_id")
            logger.info("Backend[cloud] 初始化完成, device_id=%s",
                        self._cloud_kwargs["device_id"])
        else:
            raise ValueError(f"未知 mode: {mode}, 必须为 'sim' 或 'cloud'")

    # ================================================================
    #  同步调用 (sim 模式主要使用)
    # ================================================================
    def call(self, method: str, params: Dict) -> Dict:
        """调用 MCP 工具 — 仿真和真机完全相同的调用方式.

        参数:
            method: 工具名, 如 "self.electron.hand_action"
            params: 工具参数字典

        返回:
            成功: {"result": <工具返回值>, "isError": false}
            失败: {"error": {"code": <int>, "message": <str>}}
        """
        if self.mode == "sim":
            return self._call_sim(method, params)
        else:
            # cloud 模式: 同步封装异步调用
            return asyncio.run(self._call_cloud(method, params))

    def _call_sim(self, method: str, params: Dict) -> Dict:
        """仿真模式调用 (同步, <1ms)."""
        request = {
            "jsonrpc": "2.0",
            "method": "tools/call",
            "params": {"name": method, "arguments": params},
            "id": 1,
        }
        response = self._sim_bridge.handle_request(request)
        if "error" in response:
            return {"error": response["error"]}
        # 标准 MCP 响应: result.content[0].text
        result = response.get("result", {})
        content = result.get("content", [])
        if content and isinstance(content, list):
            text = content[0].get("text", "")
            # 尝试解析为 JSON, 失败则返回原始文本
            try:
                import json
                parsed = json.loads(text)
                return {"result": parsed, "isError": result.get("isError", False)}
            except (json.JSONDecodeError, TypeError):
                return {"result": text, "isError": result.get("isError", False)}
        return {"result": result, "isError": False}

    async def _call_cloud(self, method: str, params: Dict) -> Dict:
        """云端模式调用 (异步, 200-500ms RTT)."""
        if self._cloud_bridge is None:
            # 延迟导入, 避免循环依赖
            from electronbot_sim2real.deploy_cloud import McpCloudBridge
            self._cloud_bridge = McpCloudBridge(**self._cloud_kwargs)
        try:
            result = await self._cloud_bridge.call(method, params)
            return {"result": result, "isError": False}
        except Exception as e:
            logger.error("云端调用失败: %s", e)
            return {"error": {"code": -32603, "message": str(e)}}

    # ================================================================
    #  异步调用 (cloud 模式推荐使用, 高并发场景)
    # ================================================================
    async def call_async(self, method: str, params: Dict) -> Dict:
        """异步调用 MCP 工具 (cloud 模式推荐, 避免阻塞事件循环)."""
        if self.mode == "sim":
            # sim 模式本质同步, 但提供 async 接口保持一致
            return self._call_sim(method, params)
        return await self._call_cloud(method, params)

    # ================================================================
    #  便捷方法
    # ================================================================
    def list_tools(self) -> list:
        """列出可用工具."""
        if self.mode == "sim":
            return self._sim_bridge.list_tools()
        # cloud 模式: 异步获取
        if self._cloud_bridge is None:
            from electronbot_sim2real.deploy_cloud import McpCloudBridge
            self._cloud_bridge = McpCloudBridge(**self._cloud_kwargs)
        return asyncio.run(self._cloud_bridge.list_tools())

    def get_status(self) -> Dict:
        """查询当前状态."""
        return self.call("self.electron.get_status", {})

    @property
    def sim_bridge(self):
        """暴露内部 sim bridge (仅供调试/Actions 使用, cloud 模式为 None)."""
        return self._sim_bridge

    @property
    def env(self):
        """暴露内部 env (仅 sim 模式可用)."""
        if self._sim_bridge is not None:
            return self._sim_bridge.env
        return None
