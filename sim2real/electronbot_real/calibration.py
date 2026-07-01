#!/usr/bin/env python3
"""
舵机标定工具

步骤:
1. 生成激励轨迹 (正弦扫频)
2. 发送到真实机器人 → 采集角位移数据
3. 最小二乘参数估计 (backlash, 传动比, 零点偏移)

用法:
  python calibration.py --joint all --output calib_params.json
"""

import numpy as np
from typing import Dict, List, Tuple


def generate_sweep_trajectory(
    joint_idx: int,
    duration: float = 5.0,
    dt: float = 0.02,
    freq_range: Tuple[float, float] = (0.5, 3.0),
) -> np.ndarray:
    """
    生成正弦扫频激励轨迹

    参数:
      joint_idx: 关节索引 0-5
      duration: 总时长 (秒)
      dt: 采样间隔
      freq_range: 扫频范围 (Hz)

    返回:
      (steps, 6) 轨迹数组 (度)
    """
    steps = int(duration / dt)
    t = np.linspace(0, duration, steps)

    # 对数扫频
    freq = np.exp(np.linspace(np.log(freq_range[0]), np.log(freq_range[1]), steps))

    from electronbot_mujoco.utils import JOINT_MODEL_MIN, JOINT_MODEL_MAX
    amp = min(abs(JOINT_MODEL_MIN[joint_idx]), JOINT_MODEL_MAX[joint_idx]) * 0.5

    traj = np.zeros((steps, 6))
    traj[:, joint_idx] = amp * np.sin(2 * np.pi * freq * t)
    return traj


def estimate_backlash(
    command: np.ndarray,
    response: np.ndarray,
) -> float:
    """
    估计 backlash (回程间隙)

    通过分析正反向运动时的滞后量来估计
    """
    # 找到运动方向反转点
    direction = np.sign(np.diff(command))
    reversal_idx = np.where(np.diff(direction) != 0)[0]

    if len(reversal_idx) < 2:
        return 0.0

    # 反转点附近的位置差 = backlash 的近似
    gaps = []
    for idx in reversal_idx:
        if 0 < idx < len(response) - 2:
            gap = abs(response[idx + 1] - response[idx])
            gaps.append(gap)

    return float(np.median(gaps)) if gaps else 0.0


def estimate_gear_ratio(
    command: np.ndarray,
    response: np.ndarray,
) -> float:
    """最小二乘估计传动比 k: response = k * command + b"""
    A = np.column_stack([command, np.ones_like(command)])
    k, b = np.linalg.lstsq(A, response, rcond=None)[0]
    return float(k)


class CalibrationResult:
    """标定结果"""
    def __init__(self, joint_name: str):
        self.joint_name = joint_name
        self.gear_ratio: float = 1.0
        self.backlash_deg: float = 0.0
        self.zero_offset_deg: float = 0.0

    def to_dict(self) -> dict:
        return {
            "joint": self.joint_name,
            "gear_ratio": self.gear_ratio,
            "backlash_deg": self.backlash_deg,
            "zero_offset_deg": self.zero_offset_deg,
        }


def calibrate_joint(
    joint_idx: int,
    joint_name: str,
    command_traj: np.ndarray,
    response_data: np.ndarray,
) -> CalibrationResult:
    """对单个关节执行标定"""
    cmd = command_traj[:, joint_idx]
    resp = response_data[:, joint_idx]

    result = CalibrationResult(joint_name)
    result.gear_ratio = estimate_gear_ratio(cmd, resp)
    result.backlash_deg = estimate_backlash(cmd, resp)
    result.zero_offset_deg = float(np.mean(resp - cmd * result.gear_ratio))

    return result


def main():
    import json, argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint", type=str, default="all")
    parser.add_argument("--output", type=str, default="calib_params.json")
    args = parser.parse_args()

    JOINT_NAMES = ["body", "head", "left_arm_pitch", "left_arm_roll",
                   "right_arm_pitch", "right_arm_roll"]

    joints = range(6) if args.joint == "all" else [int(args.joint)]

    results = []
    for j in joints:
        traj = generate_sweep_trajectory(j, duration=3.0)
        print(f"[INFO] 关节 {j} ({JOINT_NAMES[j]}):")
        print(f"  激励轨迹: {traj.shape[0]} 步, max={traj[:,j].max():.1f}°")
        # 实际标定时 response_data 来自 USB 回传
        # 这里用仿真数据演示
        result = calibrate_joint(j, JOINT_NAMES[j], traj, traj * 0.98 + 0.5)
        print(f"  传动比={result.gear_ratio:.4f}, backlash={result.backlash_deg:.3f}°, "
              f"offset={result.zero_offset_deg:.3f}°")
        results.append(result.to_dict())

    with open(args.output, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[SAVE] 标定参数: {args.output}")


if __name__ == "__main__":
    main()
