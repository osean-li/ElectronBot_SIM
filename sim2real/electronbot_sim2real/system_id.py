#!/usr/bin/env python3
"""
系统辨识: 摩擦力模型 + Domain Randomization 参数校准

通过实验数据估计:
- Coulomb 摩擦系数
- Viscous 阻尼系数
- 连杆质量偏差
- 执行器增益偏差

结果用于校准 Domain Randomization 参数范围
"""

import numpy as np
from typing import Dict, Tuple


def estimate_friction(
    velocity: np.ndarray,
    torque: np.ndarray,
) -> Dict[str, float]:
    """
    估计摩擦力参数: torque = sign(v) * f_c + f_v * v

    返回:
      {"coulomb": f_c, "viscous": f_v}
    """
    # 分离正负方向
    pos_mask = velocity > 0.01
    neg_mask = velocity < -0.01

    if sum(pos_mask) < 5 or sum(neg_mask) < 5:
        return {"coulomb": 0.0, "viscous": 0.0}

    # 正方向
    v_pos = velocity[pos_mask]
    t_pos = torque[pos_mask]
    A_pos = np.column_stack([np.ones_like(v_pos), v_pos])
    coeff_pos, _, _, _ = np.linalg.lstsq(A_pos, t_pos, rcond=None)

    # 负方向
    v_neg = velocity[neg_mask]
    t_neg = torque[neg_mask]
    A_neg = np.column_stack([-np.ones_like(v_neg), v_neg])
    coeff_neg, _, _, _ = np.linalg.lstsq(A_neg, t_neg, rcond=None)

    f_c = (coeff_pos[0] + coeff_neg[0]) / 2.0  # Coulomb
    f_v = (coeff_pos[1] + coeff_neg[1]) / 2.0  # Viscous

    return {"coulomb": abs(f_c), "viscous": max(f_v, 0.0)}


def estimate_mass_deviation(
    sim_mass: float,
    real_velocity: np.ndarray,
    sim_velocity: np.ndarray,
    applied_torque: np.ndarray,
    dt: float,
) -> float:
    """
    估计质量偏差: delta_m 使得 real ≈ sim / (1 + delta_m)

    简化方法: 比较相同扭矩下的加速度差异
    """
    real_accel = np.diff(real_velocity) / dt
    sim_accel = np.diff(sim_velocity) / dt

    # 避免除零
    valid = np.abs(sim_accel) > 0.01
    if sum(valid) < 5:
        return 1.0

    ratios = real_accel[valid] / (sim_accel[valid] + 1e-8)
    return float(np.clip(np.median(ratios), 0.7, 1.3))


def calibrate_dr_params(
    friction_params: Dict[str, float],
    mass_deviation: float,
) -> Dict[str, float]:
    """
    根据系统辨识结果校准 Domain Randomization 参数

    返回调整后的 DR 范围:
      {"friction_range": float, "mass_range": float, ...}
    """
    # 基础 DR 范围
    dr = {
        "friction_range": 0.3,
        "damping_range": 0.2,
        "mass_range": 0.15,
        "kp_range": 0.25,
    }

    # 根据实际摩擦调整
    if friction_params["viscous"] > 0.05:
        dr["damping_range"] = 0.3  # 增大阻尼范围

    # 根据质量偏差调整 (偏差越大 → 范围越大)
    mass_bias = abs(1.0 - mass_deviation)
    dr["mass_range"] = max(0.15, mass_bias * 1.5)

    return dr


def system_identify(
    sim_velocity: np.ndarray,
    real_velocity: np.ndarray,
    torque: np.ndarray,
    sim_mass: float,
    dt: float = 0.02,
) -> Dict:
    """
    完整系统辨识流程

    返回:
      {
        "friction": {"coulomb": ..., "viscous": ...},
        "mass_deviation": float,
        "calibrated_dr": {...}
      }
    """
    friction = estimate_friction(sim_velocity, torque)
    mass_dev = estimate_mass_deviation(sim_mass, real_velocity, sim_velocity, torque, dt)
    dr = calibrate_dr_params(friction, mass_dev)

    return {
        "friction": friction,
        "mass_deviation": mass_dev,
        "calibrated_dr": dr,
    }
