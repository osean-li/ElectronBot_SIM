"""ServoCalibrator — 真机舵机云端校准工具.

对齐 docs/tasks/08-Sim2Real 详细设计说明书 §6.

═══════════════════════════════════════════════════════════════════
  校准流程
═══════════════════════════════════════════════════════════════════
  1. 通过云端 API 调用 set_trim 设置舵机微调
  2. trim 保存到 ESP32 NVS (断电重启保持)
  3. 交互式逐关节校准: W/S 微调, Enter 确认, Q 跳过

  ⚠️ 校准过程通过云端 API 透传, 延迟 200-500ms RTT, 交互较慢.
  """
from __future__ import annotations

import asyncio
import logging
from typing import Dict, List

from .deploy_cloud import McpCloudBridge

logger = logging.getLogger("electronbot_sim2real.calibrate")

# 6 舵机名称 (顺序与 SERVO_CENTER 一致)
SERVO_NAMES = ["right_pitch", "right_roll", "left_pitch", "left_roll", "body", "head"]
SERVO_SHORT = ["rp", "rr", "lp", "lr", "body", "head"]


class ServoCalibrator:
    """真机舵机云端校准工具.

    参数:
        bridge: McpCloudBridge 实例 (已配置 device_id)
    """

    def __init__(self, bridge: McpCloudBridge):
        self.bridge = bridge
        self._trims: Dict[str, float] = {}

    async def get_current_trims(self) -> Dict:
        """获取当前所有 trim 值."""
        try:
            result = await self.bridge.call("self.electron.get_trims", {})
            if "error" not in result:
                self._trims = result.get("trims", result.get("result", {}).get("trims", {}))
                return self._trims
            logger.error("获取 trim 失败: %s", result.get("error"))
        except Exception as e:
            logger.error("获取 trim 异常: %s", e)
        return {}

    async def set_trim(self, servo_name: str, trim_value: float) -> bool:
        """设置单个舵机 trim (写入 NVS).

        参数:
            servo_name:  舵机名称, 如 "rp" / "right_pitch"
            trim_value:  微调值 (-30 到 30)

        返回: True 若设置成功
        """
        try:
            result = await self.bridge.call("self.electron.set_trim", {
                "servo_type": servo_name,
                "trim_value": float(trim_value),
            })
            if "error" not in result:
                self._trims[servo_name] = trim_value
                logger.info("设置 trim: %s = %.2f", servo_name, trim_value)
                return True
            logger.error("设置 trim 失败: %s", result.get("error"))
        except Exception as e:
            logger.error("设置 trim 异常: %s", e)
        return False

    async def calibrate(self) -> Dict:
        """交互式逐关节校准.

        每个关节:
        1. 显示当前 trim 值
        2. W/S 微调 (+1/-1), A/D 大调 (+5/-5)
        3. Enter 确认, Q 跳过

        返回: {servo_name: trim_value}
        """
        print("=" * 60)
        print("ElectronBot 舵机云端校准工具")
        print("=" * 60)
        print("操作: W=+1, S=-1, A=+5, D=-5, Enter=确认, Q=跳过")
        print(f"⚠️ 通过云端 API 透传, 每次操作延迟 200-500ms")
        print()

        # 获取当前 trim
        await self.get_current_trims()

        for i, name in enumerate(SERVO_NAMES):
            short = SERVO_SHORT[i]
            current_trim = self._trims.get(short, self._trims.get(name, 0.0))
            print(f"\n--- 校准关节 {i+1}/6: {name} ({short}) ---")
            print(f"当前 trim: {current_trim:.2f}")

            trim = current_trim
            while True:
                try:
                    cmd = input("W/S/A/D/Enter/Q > ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break

                if cmd == "q":
                    print(f"跳过 {short}")
                    break
                elif cmd == "" or cmd == "enter":
                    await self.set_trim(short, trim)
                    break
                elif cmd == "w":
                    trim = min(30, trim + 1)
                    await self.set_trim(short, trim)
                elif cmd == "s":
                    trim = max(-30, trim - 1)
                    await self.set_trim(short, trim)
                elif cmd == "a":
                    trim = min(30, trim + 5)
                    await self.set_trim(short, trim)
                elif cmd == "d":
                    trim = max(-30, trim - 5)
                    await self.set_trim(short, trim)
                else:
                    print("未知命令, 可选: W/S/A/D/Enter/Q")

        print("\n校准完成!")
        print(f"最终 trim 值: {self._trims}")
        return self._trims

    def calibrate_sync(self) -> Dict:
        """同步封装."""
        return asyncio.run(self.calibrate())


async def calibrate_device(device_id: str, api_url: str = None,
                           api_key: str = None) -> Dict:
    """便捷函数: 校准指定设备.

    参数:
        device_id: 设备 ID
        api_url:   云端 API URL (可选, 默认从环境变量读取)
        api_key:   API 密钥 (可选, 默认从环境变量读取)
    """
    bridge = McpCloudBridge(api_url=api_url, device_id=device_id, api_key=api_key)
    try:
        calibrator = ServoCalibrator(bridge)
        return await calibrator.calibrate()
    finally:
        await bridge.close()


if __name__ == "__main__":
    import argparse
    import os
    parser = argparse.ArgumentParser(description="ElectronBot 真机舵机校准")
    parser.add_argument("--device-id", default=os.environ.get("XIAOZHI_DEVICE_ID"),
                        help="设备 ID")
    parser.add_argument("--api-url", default=os.environ.get("XIAOZHI_API_URL"),
                        help="云端 API URL")
    parser.add_argument("--api-key", default=os.environ.get("XIAOZHI_API_KEY"),
                        help="API 密钥")
    args = parser.parse_args()
    if not args.device_id:
        parser.error("请提供 --device-id 或设置 XIAOZHI_DEVICE_ID 环境变量")
    asyncio.run(calibrate_device(args.device_id, args.api_url, args.api_key))
