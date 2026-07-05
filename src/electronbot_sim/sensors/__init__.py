"""传感器模块 — Layer 5 感知系统.

对齐 docs/tasks/05-Sensors-Observation 详细设计说明书.

公开接口:
    from electronbot_sim.sensors import CameraSensor, JointSensor, ContactSensor
    from electronbot_sim.observation import build_observation
"""
from .camera import CameraSensor
from .joint import JointSensor
from .contact import ContactSensor

__all__ = ["CameraSensor", "JointSensor", "ContactSensor"]
