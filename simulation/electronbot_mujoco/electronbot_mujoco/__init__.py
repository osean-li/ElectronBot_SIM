"""
ElectronBot MuJoCo 仿真包

模块:
- robot:    ElectronBotRobot (基础) + ElectronBotFirmwareRobot (固件级)
- env:      ElectronBotEnv (position actuator) + ElectronBotFirmwareEnv (DCE PID)
- utils:    角度映射、前向运动学、轨迹生成
- sensors:  仿真相机 (RGB+Depth)
- servo_sim:ServoSimulator (200Hz DCE PID 舵机仿真器)
- impedance_controller: 阻抗/导纳控制
- domain_randomizer: Domain Randomization
- tasks:    5 个 Benchmark 任务 (Reach/Push/Wave/PointAt/Stack)
"""

from .robot import ElectronBotRobot, ElectronBotFirmwareRobot
from .env import ElectronBotEnv, ElectronBotFirmwareEnv
from .utils import (
    JOINT_PARAMS, JOINT_NAMES,
    model_angle_to_mech, mech_angle_to_model,
    normalize_joint_angles, get_joint_limits_rad,
)
__all__ = [
    "ElectronBotRobot",
    "ElectronBotFirmwareRobot",
    "ElectronBotEnv",
    "ElectronBotFirmwareEnv",
]
