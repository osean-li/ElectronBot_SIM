"""OnnxPolicyBridge — ONNX 推理 + 降级部署 (路径 B).

对齐 docs/tasks/08-Sim2Real 详细设计说明书 §5.

═══════════════════════════════════════════════════════════════════
  部署路径
═══════════════════════════════════════════════════════════════════
  仿真训练策略 (BC/ACT/PPO) → 导出 ONNX → 本地推理 → 降级为预设动作
  → 通过云端 API (McpCloudBridge) 部署到真机

  延迟: 200-500ms RTT (云端 API 透传, 与路径 A 相同)
  适用: RL/IL 策略部署 (但已降级为预设动作, 失去精细控制)

  ⚠️ 真正实时闭环控制 (<10ms) 需路径 C (WebSocket 直连, 需固件 OTA)
"""
from __future__ import annotations

import logging
from typing import Dict, List, Optional

import numpy as np

from .deploy_cloud import McpCloudBridge
from .capability_downgrade import CapabilityDowngrader, SERVO_CENTER_ONNX

logger = logging.getLogger("electronbot_sim2real.deploy_onnx")


class OnnxPolicyBridge:
    """ONNX 策略推理 + 降级部署 (路径 B).

    流程:
    1. 加载 ONNX 模型
    2. 输入观测 → ONNX 推理 → 6D 舵机角度
    3. 通过 CapabilityDowngrader 降级为预设动作
    4. 通过 McpCloudBridge 调用云端 API 部署到真机

    参数:
        onnx_path:   ONNX 模型文件路径
        cloud_bridge: McpCloudBridge 实例 (已配置 device_id 等)
    """

    def __init__(self, onnx_path: str, cloud_bridge: McpCloudBridge):
        self.onnx_path = onnx_path
        self.cloud_bridge = cloud_bridge
        self.downgrader = CapabilityDowngrader(target="release_v2.2.6")
        self._session = None
        self._input_name = None
        self._output_name = None
        self._load_model()

        logger.info("OnnxPolicyBridge 初始化: model=%s", onnx_path)

    def _load_model(self) -> None:
        """加载 ONNX 模型."""
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise ImportError(
                "未安装 onnxruntime, 请运行: pip install onnxruntime"
            ) from e

        try:
            self._session = ort.InferenceSession(
                self.onnx_path,
                providers=["CPUExecutionProvider"],
            )
            self._input_name = self._session.get_inputs()[0].name
            self._output_name = self._session.get_outputs()[0].name
            logger.info(
                "ONNX 模型加载成功: input=%s, output=%s",
                self._input_name, self._output_name,
            )
        except Exception as e:
            logger.error("ONNX 模型加载失败: %s", e)
            raise

    def predict(self, observation: np.ndarray) -> np.ndarray:
        """ONNX 推理, 返回 6D 舵机角度.

        参数:
            observation: 观测向量, 形状取决于模型 (通常 (1, obs_dim) 或 (obs_dim,))

        返回: (6,) 舵机角度数组
        """
        obs = np.asarray(observation, dtype=np.float32)
        if obs.ndim == 1:
            obs = obs.reshape(1, -1)

        try:
            output = self._session.run(
                [self._output_name], {self._input_name: obs}
            )
            action = output[0]
            if action.ndim > 1:
                action = action.squeeze()
            return np.asarray(action, dtype=np.float32)
        except Exception as e:
            logger.error("ONNX 推理失败: %s, 回退到 stop", e)
            # 推理失败回退到 stop (对齐设计文档)
            return SERVO_CENTER_ONNX.copy()

    async def predict_and_execute_preset(self, observation: np.ndarray) -> Dict:
        """端到端: ONNX 推理 → 降级 → 云端部署.

        参数:
            observation: 观测向量

        返回:
            {"actions": [...], "results": [...], "downgrade_report": ...}
        """
        # 1. ONNX 推理
        action = self.predict(observation)
        logger.debug("ONNX 推理结果 (6D 舵机角度): %s", action)

        # 2. 降级为预设动作
        preset_actions = self.downgrader.downgrade_action_6d(action)

        if not preset_actions:
            # 无可降级动作, 调用 stop
            logger.warning("无可降级动作, 调用 stop")
            try:
                await self.cloud_bridge.call("self.electron.stop", {})
            except Exception as e:
                logger.error("stop 调用失败: %s", e)
            return {
                "actions": [],
                "results": [],
                "downgrade_report": self.downgrader.get_report(),
            }

        # 3. 通过云端 API 执行预设动作
        results: List[Dict] = []
        for act in preset_actions:
            tool = act["tool"]
            args = act["args"]
            try:
                result = await self.cloud_bridge.call(tool, args)
                results.append({"tool": tool, "args": args, "result": result})
            except Exception as e:
                logger.error("云端调用 %s 失败: %s", tool, e)
                # 失败时调用 stop
                try:
                    await self.cloud_bridge.call("self.electron.stop", {})
                except Exception:
                    pass
                results.append({"tool": tool, "args": args, "error": str(e)})
                break

        return {
            "actions": preset_actions,
            "results": results,
            "downgrade_report": self.downgrader.get_report(),
        }

    def close(self) -> None:
        """释放 ONNX 会话."""
        if self._session is not None:
            self._session = None
            logger.info("OnnxPolicyBridge 已关闭")
