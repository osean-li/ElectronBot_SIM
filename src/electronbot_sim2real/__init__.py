"""ElectronBot Sim2Real 真机部署模块.

对齐 docs/tasks/08-Sim2Real 详细设计说明书.
对齐真机: xiaozhi-esp32 release v2.2.6

═══════════════════════════════════════════════════════════════════
  三种部署路径
═══════════════════════════════════════════════════════════════════
  路径 A: 云端 API 透传 (release v2.2.6 当前可用, 200-500ms RTT)
          → deploy_cloud.McpCloudBridge
  路径 B: ONNX 推理 + 降级部署 (基于路径 A)
          → deploy_onnx.OnnxPolicyBridge
  路径 C: WebSocket 直连 (<10ms RTT, 需固件 OTA)
          → deploy_websocket.McpWebSocketBridge (骨架, 标注需 OTA)

  公开接口:
    from electronbot_sim2real import (
        McpCloudBridge, OnnxPolicyBridge,
        CapabilityDowngrader, ServoCalibrator,
    )
"""
from .deploy_cloud import McpCloudBridge
from .capability_downgrade import CapabilityDowngrader
from .deploy_onnx import OnnxPolicyBridge
from .calibrate import ServoCalibrator

__all__ = [
    "McpCloudBridge",
    "OnnxPolicyBridge",
    "CapabilityDowngrader",
    "ServoCalibrator",
]
