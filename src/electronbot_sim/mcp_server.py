"""MCP WebSocket 调试服务器 — 仿真端调试用.

对齐 docs/tasks/03-MCP-Bridge 详细设计说明书 §5.
对齐 docs/概要设计/ElectronBot_SIM-概要设计文档.md §5.

═══════════════════════════════════════════════════════════════════
  重要约束 (嵌入式固件工程师强制规范)
═══════════════════════════════════════════════════════════════════
  1. 仅用于仿真调试, 真机 release v2.2.6 无 WebSocket Server
  2. 默认仅监听 localhost (127.0.0.1), 禁止绑定 0.0.0.0
  3. 真机通信必须走云端 API (见 electronbot_sim2real.deploy_cloud)

  消息封装格式 (与真机一致):
  {
    "type": "mcp",
    "payload": {
      "jsonrpc": "2.0",
      "method": "tools/call",
      "params": {"name": "self.electron.xxx", "arguments": {...}},
      "id": 3
    }
  }

  使用示例:
      env = ElectronBotEnv(render_mode="human")
      bridge = McpSimBridge(env)
      server = McpWebSocketServer(bridge, port=8080)
      asyncio.run(server.start())
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Optional

logger = logging.getLogger("electronbot_sim.mcp_server")


class McpWebSocketServer:
    """MCP WebSocket 调试服务器.

    监听 localhost:8080, 接收 JSON-RPC 消息, 转发给 McpSimBridge.
    仅用于仿真调试, 真机无此端点.

    参数:
        bridge: McpSimBridge 实例
        host:   监听地址, 默认 "localhost" (禁止 0.0.0.0)
        port:   监听端口, 默认 8080
    """

    def __init__(self, bridge, host: str = "localhost", port: int = 8080):
        self.bridge = bridge
        self.host = host
        self.port = port
        self._server = None
        self._connections = set()

        # 安全检查: 禁止绑定 0.0.0.0 (真机无此端点, 仅供调试)
        if host in ("0.0.0.0", "::"):
            logger.warning("禁止绑定 %s, 强制改为 localhost (调试服务器不应公开暴露)", host)
            self.host = "localhost"

    async def handler(self, websocket) -> None:
        """WebSocket 连接处理器."""
        self._connections.add(websocket)
        peer = websocket.remote_address if hasattr(websocket, "remote_address") else "unknown"
        logger.info("WebSocket 客户端连接: %s", peer)
        try:
            async for raw_msg in websocket:
                await self._handle_message(websocket, raw_msg)
        except Exception as e:
            logger.warning("WebSocket 连接异常: %s", e)
        finally:
            self._connections.discard(websocket)
            logger.info("WebSocket 客户端断开: %s", peer)

    async def _handle_message(self, websocket, raw_msg) -> None:
        """处理单条消息, 支持 type:"mcp" 封装与裸 JSON-RPC."""
        try:
            msg = json.loads(raw_msg)
        except json.JSONDecodeError as e:
            print(f"❌ JSON 解析失败: {e}")
            print(f"   原始数据: {raw_msg[:200]}")
            await websocket.send(json.dumps({
                "jsonrpc": "2.0", "id": None,
                "error": {"code": -32700, "message": f"JSON 解析失败: {e}"},
            }))
            return

        # 解包 type:"mcp" 封装 (与真机协议一致)
        wrapped = False
        if isinstance(msg, dict) and msg.get("type") == "mcp":
            payload = msg.get("payload", {})
            wrapped = True
        else:
            payload = msg

        # 打印接收到的请求
        method = payload.get("method", "?")
        tool_name = ""
        tool_args = {}
        if method == "tools/call":
            tool_name = payload.get("params", {}).get("name", "?")
            tool_args = payload.get("params", {}).get("arguments", {})
        elif method == "tools/list":
            tool_name = "[tools/list]"
        else:
            tool_name = method
            tool_args = payload.get("params", {})

        print(f"📥 收到请求: {tool_name}")
        if tool_args:
            print(f"   参数: {tool_args}")

        # 转发给 McpSimBridge
        response = self.bridge.handle_request(payload)

        # 打印响应摘要
        if "error" in response:
            print(f"   ❌ 错误: {response['error'].get('message', str(response['error']))}")
        else:
            result = response.get("result", {})
            content = result.get("content", [{}])
            text = content[0].get("text", "") if content else ""
            print(f"   ✅ 响应: {text[:120]}")

        # 如果是封装格式, 响应也封装
        if wrapped:
            response = {"type": "mcp", "payload": response}

        await websocket.send(json.dumps(response, ensure_ascii=False))

    async def _process_request(self, connection, request):
        """预处理连接请求 — 静默拒绝非 WebSocket 连接（如浏览器 HTTP 请求）."""
        if request.headers.get("Upgrade", "").lower() != "websocket":
            return connection.respond(426, "WebSocket connection required")
        return None  # 允许 WebSocket 连接继续

    async def start(self) -> None:
        """启动 WebSocket 服务器 (阻塞, 直到被取消)."""
        try:
            import websockets
        except ImportError as e:
            raise ImportError(
                "未安装 websockets 库, 请运行: pip install websockets"
            ) from e

        print(f"🔌 ElectronBot 仿真 MCP 服务器已启动 (调试模式)")
        print(f"   ws://{self.host}:{self.port}/ws")
        print(f"   ⚠️ 此服务器仅用于仿真调试，不用于真机连接")
        print(f"   真机部署请使用: ElectronBotBackend('cloud', ...)")
        print(f"   ───")
        logger.info("启动 MCP WebSocket 调试服务器: ws://%s:%d", self.host, self.port)

        self._server = await websockets.serve(
            self.handler, self.host, self.port,
            process_request=self._process_request,
        )
        await self._server.wait_closed()

    def start_sync(self) -> None:
        """同步启动 (阻塞主线程)."""
        asyncio.run(self.start())

    async def stop(self) -> None:
        """停止服务器."""
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        # 关闭所有客户端连接
        for ws in list(self._connections):
            try:
                await ws.close()
            except Exception:
                pass
        logger.info("MCP WebSocket 服务器已停止")


def run_server(host: str = "localhost", port: int = 8080,
               render_mode: Optional[str] = "human") -> None:
    """便捷启动函数.

    参数:
        host:        监听地址
        port:        监听端口
        render_mode: 渲染模式, "human" (默认) / "rgb_array" / None
    """
    import socket

    # ── 预检端口可用性, 避免创建 MuJoCo/GLFW 资源后再发现端口被占用 ──
    # (一旦创建 launch_passive viewer, 后台 GLFW 线程很难干净清理, 会段错误)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
    except OSError:
        print(f"❌ 端口 {port} 已被占用, 请先关闭占用该端口的进程:")
        print(f"   lsof -i :{port}")
        print(f"   或 kill $(lsof -t -i :{port})")
        return
    finally:
        sock.close()

    from .env import ElectronBotEnv
    from .mcp_bridge import McpSimBridge

    env = None
    try:
        # MCP 调试服务器: 使用项目主场景 electronbot_scene.xml (含机器人本体 + 地面 + 灯光)
        # env.py 默认的 scene_tabletop.xml 是桌面抓取任务场景, MCP 调试不需要桌面/物体
        env = ElectronBotEnv(render_mode=render_mode, model_file="electronbot_scene.xml")
        # 主动触发一次渲染, 确保 MuJoCo 窗口在 WebSocket 监听前弹出
        # (human 模式下创建 launch_passive viewer, 显示 home 姿态)
        env.render()
        bridge = McpSimBridge(env)
        server = McpWebSocketServer(bridge, host=host, port=port)
        asyncio.run(server.start())
    except OSError as e:
        if e.errno == 98:  # EADDRINUSE (预检后仍可能被抢占)
            print(f"❌ 端口 {port} 已被占用, 请先关闭占用该端口的进程:")
            print(f"   lsof -i :{port}")
            print(f"   或 kill $(lsof -t -i :{port})")
        else:
            print(f"❌ 系统错误: {e}")
    except KeyboardInterrupt:
        print("\n⏹ 服务器已停止 (Ctrl+C)")
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="ElectronBot MCP WebSocket 调试服务器")
    parser.add_argument("--host", default="localhost", help="监听地址")
    parser.add_argument("--port", type=int, default=8080, help="监听端口")
    parser.add_argument("--render", default="human", help="渲染模式: human (默认) / rgb_array")
    args = parser.parse_args()
    run_server(args.host, args.port, args.render)
