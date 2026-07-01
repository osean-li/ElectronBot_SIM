"""Benchmark 指标计算 (成功率, 完成时间, Jerk 平滑度, 最终误差)"""
import numpy as np
from typing import List


def compute_success_rate(successes: List[bool]) -> float:
    return float(np.mean(successes)) if successes else 0.0

def compute_completion_time(steps: List[int], max_steps: int) -> float:
    return float(np.mean(steps)) if steps else float(max_steps)

def compute_jerk(actions: np.ndarray) -> float:
    """计算轨迹 Jerk (动作的三阶导数近似)"""
    if len(actions) < 3:
        return 0.0
    jerk = np.diff(actions, n=3, axis=0)
    return float(np.mean(jerk ** 2))

def compute_final_error(ee_pos: np.ndarray, target: np.ndarray) -> float:
    return float(np.linalg.norm(ee_pos - target))

def compute_smoothness(actions: np.ndarray) -> float:
    """动作平滑度 = 相邻动作差的均方根"""
    if len(actions) < 2:
        return 0.0
    diffs = np.diff(actions, axis=0)
    return float(np.sqrt(np.mean(diffs ** 2)))
