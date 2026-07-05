"""
ElectronBot SIM — MuJoCo 仿真核心
Layer 2+3+4+5: 仿真环境、MCP桥接、动作系统、传感器与观测

公开接口:
    from electronbot_sim import ElectronBotEnv, ElectronBotBackend
    env = ElectronBotEnv(render_mode="human")
    obs, info = env.reset()
"""
from __future__ import annotations

__version__ = "0.2.0"

# Gymnasium 环境注册 (对齐 Phase 2 §6.1)
try:
    import gymnasium as gym
    from gymnasium.envs.registration import register, registry

    if "ElectronBot-v0" not in {spec.id for spec in registry.values()} if hasattr(registry, "values") else set():
        register(
            id="ElectronBot-v0",
            entry_point="electronbot_sim.env:ElectronBotEnv",
            max_episode_steps=1000,
        )
    else:
        # 检查是否已注册
        try:
            gym.spec("ElectronBot-v0")
        except Exception:
            register(
                id="ElectronBot-v0",
                entry_point="electronbot_sim.env:ElectronBotEnv",
                max_episode_steps=1000,
            )
except Exception:  # gymnasium 未安装时静默跳过注册
    pass


def __getattr__(name: str):
    """延迟导入, 避免在仅 import 包时触发 mujoco 加载。"""
    if name == "ElectronBotEnv":
        from .env import ElectronBotEnv
        return ElectronBotEnv
    if name == "ElectronBotBackend":
        from .backend import ElectronBotBackend
        return ElectronBotBackend
    if name == "McpSimBridge":
        from .mcp_bridge import McpSimBridge
        return McpSimBridge
    if name == "ElectronBotActions":
        from .actions import ElectronBotActions
        return ElectronBotActions
    raise AttributeError(f"module 'electronbot_sim' has no attribute {name!r}"")


__all__ = ["ElectronBotEnv", "ElectronBotBackend", "McpSimBridge",
           "ElectronBotActions", "__version__"]
