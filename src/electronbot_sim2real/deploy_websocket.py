"""McpWebSocketBridge — WebSocket 直连部署 (路径 C, 需固件 OTA).

对齐 docs/tasks/08-Sim2Real 详细设计说明书 §7.

═══════════════════════════════════════════════════════════════════
  ⚠️ 重要约束 (嵌入式固件工程师强制说明)
═══════════════════════════════════════════════════════════════════
  release v2.2.6 真机 ESP32 **不启动** WebSocket Server,
  本模块为【骨架占位】, 需要固件 OTA 升级后才能使用.

  OTA 升级路径 (参考 Otto Robot 已有的 WebSocket Server 实现):
    cd xiaozhi-esp32-2.2.6
    idf.py set-target esp32s3
    idf.py menuconfig  # Component config → xiaozhi → Board Type → Electron Bot
    # 添加 WebSocket Server + servo_move 工具
    idf.py build
    python -m electronbot_sim2real.ota_push \
        --firmware build/xiaozhi.bin --device-id eb-001

  OTA 完成后, ESP32 将监听 ws://<IP>:8080/ws, 延迟 <10ms RTT,
  支持实时闭环控制 (PPO@50Hz 策略可直接部署).

  消息格式 (与仿真 mcp_server.py 一致):
    {"type": "mcp", "payload": {
      "jsonrpc": "2.0", "method": "tools/call",
      "params": {"name": ..., "arguments": {...}}, "id": 1
    }}
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Dict, Optional

logger = logging.getLogger("electronbot_sim2real.deploy_websocket")


class McpWebSocketBridge:
    """WebSocket 直连 MCP 桥接器 (路径 C, 需固件 OTA).

    ⚠️ 本模块为骨架占位, 需固件 OTA 升级后才能使用.
    release v2.2.6 真机无 WebSocket Server.

    参数:
        host: ESP32 IP 地址
        port: WebSocket 端口, 默认 8080
    """

    def __init__(self, host: str, port: int = 8080):
        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}/ws"
        self._ws = None
        self._request_id = 0
        self._reconnect_count = 0
        self._max_reconnect = 5
        self._reconnect_interval = 2.0  # 秒

        logger.warning(
            "⚠️ McpWebSocketBridge 为骨架占位, 需固件 OTA 升级后才能使用. "
            "release v2.2.6 真机无 WebSocket Server."
        )
        logger.info("目标: ws://%s:%d (OTA 后可用)", host, port)

    async def connect(self) -> bool:
        """连接到 ESP32 WebSocket Server.

        ⚠️ 需固件 OTA 升级, release v2.2.6 不支持.

        返回: True 若连接成功
        """
        try:
            import websockets
        except ImportError as e:
            raise ImportError(
                "未安装 websockets 库, 请运行: pip install websockets"
            ) from e

        # 5 次重连 (2s 间隔)
        for attempt in range(self._max_reconnect):
            try:
                logger.info(
                    "连接 ESP32 WebSocket [%d/%d]: %s",
                    attempt + 1, self._max_reconnect, self.url,
                )
                self._ws = await websockets.connect(self.url)
                logger.info("WebSocket 连接成功: %s", self.url)
                return True
            except Exception as e:
                logger.warning(
                    "连接失败 [%d/%d]: %s",
                    attempt + 1, self._max_reconnect, e,
                )
                if attempt < self._max_reconnect - 1:
                    await asyncio.sleep(self._reconnect_interval)

        logger.error(
            "WebSocket 连接失败 %d 次, 请确认:\n"
            "  1. 固件已 OTA 升级 (release v2.2.6 不支持 WebSocket Server)\n"
            "  2. ESP32 IP 地址正确: %s\n"
            "  3. 端口 %d 已开放\n"
            "  4. 设备与主机在同一网络",
            self._max_reconnect, self.host, self.port,
        )
        return False

    async def call(self, tool_name: str, arguments: Dict,
                   timeout: float = 10.0) -> Dict:
        """通过 WebSocket 调用真机 MCP 工具.

        ⚠️ 需固件 OTA 升级, release v2.2.6 不支持.

        参数:
            tool_name: 工具名
            arguments: 工具参数
            timeout:   超时秒数 (实时控制场景 <10ms)

        返回: 工具执行结果
        """
        if self._ws is None:
            connected = await self.connect()
            if not connected:
                return {"error": {"code": "WS_CONNECT_FAILED",
                                  "message": "WebSocket 连接失败, 需固件 OTA"}}

        self._request_id += 1
        # 消息格式 (与仿真 mcp_server.py 一致)
        request = {
            "type": "mcp",
            "payload": {
                "jsonrpc": "2.0",
                "method": "tools/call",
                "params": {"name": tool_name, "arguments": arguments},
                "id": self._request_id,
            },
        }

        try:
            await self._ws.send(json.dumps(request))
            raw_response = await asyncio.wait_for(
                self._ws.recv(), timeout=timeout
            )
            response = json.loads(raw_response)
            # 解包 type:"mcp" 封装
            if isinstance(response, dict) and response.get("type") == "mcp":
                response = response.get("payload", response)
            return response
        except asyncio.TimeoutError:
            logger.error("WebSocket 调用超时 (%.1fs): %s", timeout, tool_name)
            return {"error": {"code": "WS_TIMEOUT",
                              "message": f"调用超时 {timeout}s"}}
        except Exception as e:
            logger.error("WebSocket 调用异常: %s", e)
            # 断线重连
            self._ws = None
            return {"error": {"code": "WS_ERROR", "message": str(e)}}

    async def close(self) -> None:
        """关闭 WebSocket 连接."""
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None
            logger.info("WebSocket 连接已关闭")

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


def check_ota_required() -> bool:
    """检查是否需要 OTA 升级.

    返回: True (release v2.2.6 始终需要 OTA 才能使用 WebSocket 直连)
    """
    logger.warning(
        "⚠️ WebSocket 直连 (路径 C) 需要固件 OTA 升级. "
        "release v2.2.6 真机无 WebSocket Server.\n"
        "请参考 Otto Robot 板型 (xiaozhi-esp32/main/boards/otto-robot/) "
        "添加 WebSocket Server + servo_move 工具后重新编译烧录."
    )
    return True
