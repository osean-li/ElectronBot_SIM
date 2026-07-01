"""
ElectronBot 工具函数
- 机械角度 ↔ 模型角度双向转换
- 前向运动学计算 (基于关节角度)
"""

import math
import numpy as np
from typing import List, Tuple, Dict


# ============================================================
# 关节参数 (来自固件 robot.h)
# ============================================================
JOINT_PARAMS = [
    # name              id  mech_min  mech_max  model_min  model_max  inverted
    ("head",             2,  70,       95,       -15,       15,        True),
    ("left_arm_roll",    4,  -9,       3,        0,         30,        False),
    ("left_arm_pitch",   6,  -16,      117,      -20,       180,       False),
    ("right_arm_roll",   8,  133,      141,      0,         30,        True),
    ("right_arm_pitch",  10, 15,       150,      -20,       180,       True),
    ("body",             12, 0,        180,      -90,       90,        False),
]

JOINT_NAMES = [p[0] for p in JOINT_PARAMS]
JOINT_IDS = [p[1] for p in JOINT_PARAMS]
# 元组索引: p[0]=name, p[1]=id, p[2]=mech_min, p[3]=mech_max, p[4]=model_min, p[5]=model_max, p[6]=inverted
JOINT_MECH_MIN  = np.array([p[2] for p in JOINT_PARAMS], dtype=np.float64)  # deg
JOINT_MECH_MAX  = np.array([p[3] for p in JOINT_PARAMS], dtype=np.float64)  # deg
JOINT_MODEL_MIN = np.array([p[4] for p in JOINT_PARAMS], dtype=np.float64)  # deg
JOINT_MODEL_MAX = np.array([p[5] for p in JOINT_PARAMS], dtype=np.float64)  # deg


def deg_to_rad(deg: float) -> float:
    """度 → 弧度"""
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    """弧度 → 度"""
    return rad * 180.0 / math.pi


def model_angle_to_mech(joint_index: int, model_angle_deg: float) -> float:
    """
    模型角度 → 机械角度 (用于发送到真实舵机)

    来源: robot.cpp:127-152, 241-261 (SetJointInitAngle / UpdateJointAngle)

    正向 (inverted=false):
      sAngle = (angle - modelMin) / (modelMax - modelMin) * (mechMax - mechMin) + mechMin

    反向 (inverted=true):
      sAngle = (angle - modelMin) / (modelMax - modelMin) * (mechMin - mechMax) + mechMax

    """
    p = JOINT_PARAMS[joint_index]
    _, _, mech_min, mech_max, model_min, model_max, inverted = p

    ratio = (model_angle_deg - model_min) / (model_max - model_min)

    if inverted:
        mech_angle = ratio * (mech_min - mech_max) + mech_max
    else:
        mech_angle = ratio * (mech_max - mech_min) + mech_min

    return mech_angle


def mech_angle_to_model(joint_index: int, mech_angle_deg: float) -> float:
    """
    机械角度 → 模型角度 (用于从真实舵机读取)

    来源: robot.cpp:225-261 (UpdateJointAngle)

    正向 (inverted=false):
      jAngle = (angle - mechMin) / (mechMax - mechMin) * (modelMax - modelMin) + modelMin

    反向 (inverted=true):
      jAngle = (mechMax - angle) / (mechMax - mechMin) * (modelMax - modelMin) + modelMin
    """
    p = JOINT_PARAMS[joint_index]
    _, _, mech_min, mech_max, model_min, model_max, inverted = p

    if inverted:
        ratio = (mech_max - mech_angle_deg) / (mech_max - mech_min)
    else:
        ratio = (mech_angle_deg - mech_min) / (mech_max - mech_min)

    model_angle = ratio * (model_max - model_min) + model_min
    return model_angle


def model_angles_to_mech(model_angles_deg: np.ndarray) -> np.ndarray:
    """批量模型角度 → 机械角度"""
    return np.array([
        model_angle_to_mech(i, a) for i, a in enumerate(model_angles_deg)
    ])


def mech_angles_to_model(mech_angles_deg: np.ndarray) -> np.ndarray:
    """批量机械角度 → 模型角度"""
    return np.array([
        mech_angle_to_model(i, a) for i, a in enumerate(mech_angles_deg)
    ])


def get_joint_limits_rad() -> Tuple[np.ndarray, np.ndarray]:
    """返回关节限位 (弧度)"""
    return (
        np.radians(JOINT_MODEL_MIN),
        np.radians(JOINT_MODEL_MAX),
    )


def get_joint_limits_deg() -> Tuple[np.ndarray, np.ndarray]:
    """返回关节限位 (度)"""
    return JOINT_MODEL_MIN.copy(), JOINT_MODEL_MAX.copy()


def normalize_joint_angles(angles: np.ndarray) -> np.ndarray:
    """将关节角度 clip 到合法范围"""
    low, high = get_joint_limits_rad()
    return np.clip(angles, low, high)


# ============================================================
# 前向运动学 (Forward Kinematics)
# ============================================================

def forward_kinematics_arm(
    shoulder_pitch_deg: float,
    arm_roll_deg: float,
    shoulder_offset: np.ndarray,
    arm_length: float,
    sign: float,  # +1 for left, -1 for right
) -> np.ndarray:
    """
    单臂前向运动学 (简化版)

    输入:
      shoulder_pitch_deg: 肩俯仰角 (度)
      arm_roll_deg: 臂 roll 角 (度)
      shoulder_offset: 肩相对于 body 的偏移 [x, y, z]
      arm_length: 臂长
      sign: +1 (左手) / -1 (右手)

    返回:
      末端执行器位置 [x, y, z]
    """
    # 旋转矩阵: 肩俯仰绕 X 轴
    pitch_rad = deg_to_rad(shoulder_pitch_deg)
    R_pitch = np.array([
        [1, 0, 0],
        [0, math.cos(pitch_rad), -math.sin(pitch_rad)],
        [0, math.sin(pitch_rad), math.cos(pitch_rad)],
    ])

    # 旋转矩阵: 臂 roll 绕 Z 轴
    roll_rad = deg_to_rad(arm_roll_deg)
    R_roll = np.array([
        [math.cos(roll_rad), -math.sin(roll_rad), 0],
        [math.sin(roll_rad), math.cos(roll_rad), 0],
        [0, 0, 1],
    ])

    # 臂方向 (沿 -Y 方向延伸)
    arm_dir = np.array([0, -arm_length, 0])
    arm_dir = R_roll @ R_pitch @ arm_dir

    ee_pos = shoulder_offset + arm_dir
    return ee_pos


def compute_end_effector_positions(joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    计算左右手末端执行器位置

    参数:
      joint_angles_deg: [head, left_arm_roll, left_arm_pitch,
                         right_arm_roll, right_arm_pitch, body]
                        (度)

    返回:
      left_ee:  [x, y, z]
      right_ee: [x, y, z]
    """
    # 关节角度提取
    _, la_roll, la_pitch, ra_roll, ra_pitch, body = joint_angles_deg

    # 手臂参数
    arm_length = 0.055  # 从 shoulder 到 ee 的距离 (米)
    shoulder_y_offset = 0.03  # shoulder joint 到 ee joint 的 Y 偏移

    # body 旋转矩阵 (绕 Y 轴)
    body_rad = deg_to_rad(body)
    R_body = np.array([
        [math.cos(body_rad), 0, math.sin(body_rad)],
        [0, 1, 0],
        [-math.sin(body_rad), 0, math.cos(body_rad)],
    ])

    # 左臂
    left_shoulder_offset = np.array([0.025, 0, 0.065])
    left_ee = forward_kinematics_arm(la_pitch, la_roll, left_shoulder_offset, arm_length, +1)
    left_ee = R_body @ left_ee

    # 右臂
    right_shoulder_offset = np.array([-0.025, 0, 0.065])
    right_ee = forward_kinematics_arm(ra_pitch, ra_roll, right_shoulder_offset, arm_length, -1)
    right_ee = R_body @ right_ee

    return left_ee, right_ee


# ============================================================
# 轨迹生成
# ============================================================

def generate_wave_trajectory(
    total_time: float = 2.0,
    dt: float = 0.02,
    amplitude_deg: float = 60.0,
    frequency: float = 1.0,
    arm: str = "right",
) -> np.ndarray:
    """
    生成挥手动作轨迹

    参数:
      total_time: 总时长 (秒)
      dt: 时间步长 (秒)
      amplitude_deg: 振幅 (度)
      frequency: 频率 (Hz)
      arm: "left" / "right"

    返回:
      (n_steps, 6) 关节角度序列 (度)
    """
    n_steps = int(total_time / dt)
    t = np.linspace(0, total_time, n_steps)

    traj = np.zeros((n_steps, 6))

    # 肩俯仰正弦波
    pitch = amplitude_deg * np.sin(2 * math.pi * frequency * t)

    if arm == "right":
        traj[:, 4] = pitch  # right_arm_pitch
    else:
        traj[:, 2] = pitch  # left_arm_pitch

    return traj


def generate_reach_trajectory(
    target_pos: np.ndarray,
    n_steps: int = 50,
    arm: str = "right",
) -> np.ndarray:
    """
    生成到达目标的轨迹 (线性插值)

    参数:
      target_pos: 目标末端位置 [x, y, z]
      n_steps: 步数
      arm: 使用的手臂

    返回:
      (n_steps, 6) 关节角度序列 (度)
      注意: 这里返回的是简单线性插值，实际 IK 需要更复杂的求解
    """
    traj = np.zeros((n_steps, 6))
    # 简化: 线性插值从当前角度到目标
    # 实际使用中应由 IK solver 产生
    return traj
