"""McpCloudBridge — 云端 API 透传部署 (路径 A, release v2.2.6 当前可用).

对齐 docs/tasks/08-Sim2Real 详细设计说明书 §3.
对齐 docs/概要设计/ElectronBot_SIM-概要设计文档.md §5.4.

═══════════════════════════════════════════════════════════════════
  通信拓扑 (关键事实, 嵌入式固件工程师强制约束)
═══════════════════════════════════════════════════════════════════
  ESP32 (WebSocket 客户端) ──MQTT/WS──> 小智云端后台 <──HTTPS API── Python

  - ESP32 主动连接云端后台 (WebSocket/MQTT 客户端角色)
  - ESP32 **不启动**任何本地 WebSocket 服务器
  - Python 端**无法**直接 ws://IP:8080 连接到 ESP32
  - 唯一控制路径: Python → 云端 API → 云端后台 → MQTT/WS → ESP32

  延迟约束 (核心约束):
  - 路径 A/B: 200-500ms RTT (HTTPS → 云端 → MQTT/WS → ESP32)
  - 有效闭环延迟: 400-1000ms (对应 20-50 仿真步)
  - 结论: 实时 RL 策略 (PPO@50Hz) 不可通过云端部署
          仅支持预设动作调用 (LLM/VLA 场景)

  HTTP 请求格式:
    POST {api_url}/devices/{device_id}/tools/call
    Headers: {"Authorization": "Bearer {api_key}"}
    Body: {"name": "self.electron.hand_action", "arguments": {...}}

  环境变量:
    XIAOZHI_API_URL    = https://api.xiaozhi.cn/v1
    XIAOZHI_API_KEY    = <必填>
    XIAOZHI_DEVICE_ID  = eb-001
"""
from __future__ import annotations

import asyncio
import logging
import os
from typing import Any, Dict, List, Optional

logger = logging.getLogger("electronbot_sim2real.deploy_cloud")

# ═══════════════════════════════════════════════════════════════════
#  常量 (对齐设计文档)
# ═══════════════════════════════════════════════════════════════════
DEFAULT_API_URL = "https://api.xiaozhi.cn/v1"
DEFAULT_TIMEOUT = 30.0  # 秒
MAX_RETRIES = 3         # 重试次数
RETRY_BASE_DELAY = 1.0  # 指数退避基数 (1s, 2s, 4s)

# release v2.2.6 真机可用工具 (8 个, 硬约束)
REAL_MACHINE_TOOLS = [
    "self.electron.hand_action",
    "self.electron.body_turn",
    "self.electron.head_move",
    "self.electron.stop",
    "self.electron.get_status",
    "self.electron.set_trim",
    "self.electron.get_trims",
    "self.battery.get_level",
]


class McpCloudBridge:
    """云端 API 透传 MCP 桥接器 (路径 A, release v2.2.6 当前可用).

    通过小智云端 API 透传 MCP 命令到 ESP32 真机.
    通信链路: Python → HTTPS → 小智云端后台 → MQTT/WS → ESP32 真机

    参数:
        api_url:    云端 API 基地址, 默认从 XIAOZHI_API_URL 环境变量读取
        device_id:  设备 ID, 如 "eb-001", 默认从 XIAOZHI_DEVICE_ID 读取
        api_key:    API 密钥, 默认从 XIAOZHI_API_KEY 读取
        timeout:    HTTP 超时秒数, 默认 30
    """

    def __init__(
        self,
        api_url: Optional[str] = None,
        device_id: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self.api_url = api_url or os.environ.get("XIAOZHI_API_URL", DEFAULT_API_URL)
        self.device_id = device_id or os.environ.get("XIAOZHI_DEVICE_ID")
        self.api_key = api_key or os.environ.get("XIAOZHI_API_KEY")
        self.timeout = timeout

        if not self.device_id:
            raise ValueError(
                "device_id 必填, 请传入 device_id 参数或设置 XIAOZHI_DEVICE_ID 环境变量"
            )

        self._client = None  # httpx.AsyncClient 延迟初始化
        logger.info(
            "McpCloudBridge 初始化: api_url=%s, device_id=%s, timeout=%.1fs",
            self.api_url, self.device_id, self.timeout,
        )

    async def _get_client(self):
        """延迟初始化 httpx.AsyncClient."""
        if self._client is None:
            try:
                import httpx
            except ImportError as e:
                raise ImportError(
                    "未安装 httpx 库, 请运行: pip install httpx"
                ) from e
            headers = {"Content-Type": "application/json"}
            if self.api_key:
                headers["Authorization"] = f"Bearer {self.api_key}"
            self._client = httpx.AsyncClient(
                base_url=self.api_url,
                headers=headers,
                timeout=self.timeout,
            )
        return self._client

    async def call(self, tool_name: str, arguments: Dict,
                   timeout: Optional[float] = None) -> Dict:
        """通过云端 API 调用真机 MCP 工具 (异步).

        参数:
            tool_name: 工具名, 如 "self.electron.hand_action"
            arguments: 工具参数字典
            timeout:   本次调用超时 (秒), 默认使用实例 timeout

        返回:
            成功: {"success": true, "result": ..., "execution_time_ms": ...}
            失败: {"error": {"code": "...", "message": "..."}}

        异常:
            TimeoutError: 3 次重试 + 指数退避后仍超时
            DeviceOfflineError: 设备离线 60s 超时
            RuntimeError: 401 Unauthorized 等
        """
        # 检查工具是否在真机可用列表中 (release v2.2.6 硬约束)
        if tool_name not in REAL_MACHINE_TOOLS:
            logger.warning(
                "工具 %s 不在真机 release v2.2.6 可用列表中, 调用可能失败",
                tool_name,
            )

        client = await self._get_client()
        url = f"/devices/{self.device_id}/tools/call"
        body = {"name": tool_name, "arguments": arguments}
        req_timeout = timeout or self.timeout

        # 3 次重试 + 指数退避 (1s, 2s, 4s)
        last_error = None
        for attempt in range(MAX_RETRIES):
            try:
                logger.debug(
                    "云端调用 [%d/%d] %s, args=%s",
                    attempt + 1, MAX_RETRIES, tool_name, arguments,
                )
                response = await client.post(
                    url, json=body, timeout=req_timeout
                )

                # 401 Unauthorized: 立即终止
                if response.status_code == 401:
                    raise RuntimeError(
                        "401 Unauthorized — 请检查 api_key (XIAOZHI_API_KEY)"
                    )

                # 5xx: 重试
                if response.status_code >= 500:
                    last_error = RuntimeError(
                        f"服务端错误 {response.status_code}: {response.text}"
                    )
                    logger.warning(
                        "云端调用 5xx (尝试 %d/%d): %s",
                        attempt + 1, MAX_RETRIES, last_error,
                    )
                    await asyncio.sleep(RETRY_BASE_DELAY * (2 ** attempt))
                    continue

                # 其他错误: 不重试
                if response.status_code != 200:
                    error_body = {}
                    try:
                        error_body = response.json()
                    except Exception:
                        error_body = {"raw": response.text}
                    return {
                        "error": {
                            "code": f"HTTP_{response.status_code}",
                            "message": str(error_body),
                        }
                    }

                # 成功
                result = response.json()
                logger.debug("云端调用成功: %s", tool_name)
                return result

            except (asyncio.TimeoutError, Exception) as e:
                # 超时或网络错误: 重试
                last_error = e
                logger.warning(
                    "云端调用异常 (尝试 %d/%d): %s",
                    attempt + 1, MAX_RETRIES, e,
                )
                if attempt < MAX_RETRIES - 1:
                    await asyncio.sleep(RETRY_BASE_DELAY * (2 ** attempt))

        # 重试耗尽
        raise TimeoutError(
            f"云端调用 {tool_name} 重试 {MAX_RETRIES} 次后仍失败: {last_error}"
        )

    async def list_tools(self) -> List[Dict]:
        """获取真机当前注册的工具列表.

        返回: [{"name": ..., "description": ..., "inputSchema": {...}}, ...]
        """
        client = await self._get_client()
        url = f"/devices/{self.device_id}/tools"
        try:
            response = await client.get(url, timeout=self.timeout)
            if response.status_code == 200:
                data = response.json()
                return data.get("tools", [])
            logger.warning("list_tools 失败: HTTP %d", response.status_code)
            return []
        except Exception as e:
            logger.error("list_tools 异常: %s", e)
            return []

    async def get_device_status(self) -> Dict:
        """获取设备连接状态.

        返回: {"online": bool, "version": str, "last_seen": str}
        """
        client = await self._get_client()
        url = f"/devices/{self.device_id}/status"
        try:
            response = await client.get(url, timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
            return {"online": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            logger.error("get_device_status 异常: %s", e)
            return {"online": False, "error": str(e)}

    def call_sync(self, tool_name: str, arguments: Dict) -> Dict:
        """同步封装 (内部使用 asyncio.run).

        注意: 在已有事件循环的环境中会失败, 此时请使用 call() 异步版本.
        """
        return asyncio.run(self.call(tool_name, arguments))

    async def close(self) -> None:
        """关闭 HTTP 客户端."""
        if self._client is not None:
            await self._client.aclose()
            self._client = None
            logger.info("McpCloudBridge 已关闭")

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


# ═══════════════════════════════════════════════════════════════════
#  自定义异常
# ═══════════════════════════════════════════════════════════════════
class DeviceOfflineError(Exception):
    """设备离线异常."""
    pass


class CloudApiError(Exception):
    """云端 API 调用异常."""
    pass
