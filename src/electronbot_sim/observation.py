"""观测构建器 — 标准化观测空间.

对齐 docs/tasks/05-Sensors-Observation 详细设计说明书 §6.
对齐 docs/概要设计/ElectronBot_SIM-概要设计文档.md §3.1.

═══════════════════════════════════════════════════════════════════
  双观测模式 (核心 Sim2Real 设计)
═══════════════════════════════════════════════════════════════════
  obs_mode="full":
      包含仿真可观测的全部数据, 用于研究/预训练
      joint_pos, joint_vel, ee_pos_left/right, image, depth, segmentation,
      contact_left/right, contact_force_left/right, target_pos (可选)

  obs_mode="realistic":
      仅包含真机可获取的数据, 用于 Sim2Real 训练
      commanded_joint_pos, is_moving, battery_voltage, battery_percent
      ⚠️ 移除 joint_vel / ee_pos / image / depth / segmentation
         (真机 SG90/2g/4.3g 舵机无编码器, ElectronBot 无摄像头)
"""
from __future__ import annotations

import logging
from typing import Any, Dict, Optional

import numpy as np

logger = logging.getLogger("electronbot_sim.observation")


def build_observation(
    bridge: Optional[Any] = None,
    env: Optional[Any] = None,
    camera: Optional[Any] = None,
    joint_sensor: Optional[Any] = None,
    contact_left: Optional[Any] = None,
    contact_right: Optional[Any] = None,
    obs_mode: str = "full",
    target_pos: Optional[np.ndarray] = None,
) -> Dict[str, np.ndarray]:
    """构建标准化观测字典.

    支持 full / realistic 两种模式, 由 obs_mode 参数切换.
    full 模式需要 camera/joint_sensor/contact 等传感器;
    realistic 模式仅需 env (从 env 直接读取 commanded 状态).

    参数:
        bridge:         McpSimBridge 实例 (用于读取 is_moving)
        env:            ElectronBotEnv 实例 (必需)
        camera:         CameraSensor 实例 (full 模式需要)
        joint_sensor:   JointSensor 实例 (full 模式需要)
        contact_left:   ContactSensor 实例 (full 模式, 左手)
        contact_right:  ContactSensor 实例 (full 模式, 右手)
        obs_mode:       "full" 或 "realistic"
        target_pos:     (3,) 目标位置 (可选, 任务相关)

    返回: 观测字典, 不同模式字段不同
    """
    if env is None:
        raise ValueError("env 参数必需")

    if obs_mode == "full":
        return _build_full_observation(
            env, camera, joint_sensor, contact_left, contact_right, target_pos
        )
    elif obs_mode == "realistic":
        return _build_realistic_observation(env, bridge)
    else:
        raise ValueError(f"未知 obs_mode: {obs_mode}, 必须为 'full' 或 'realistic'")


def _build_full_observation(
    env, camera, joint_sensor, contact_left, contact_right, target_pos
) -> Dict[str, np.ndarray]:
    """构建 full 模式观测 (仿真研究, 含全部可观测数据)."""
    obs: Dict[str, np.ndarray] = {}

    # ── 关节状态 (来自 JointSensor, 若无则从 env 读取) ──
    if joint_sensor is not None:
        obs["joint_pos"] = joint_sensor.get_positions(add_noise=False).astype(np.float32)
        obs["joint_vel"] = joint_sensor.get_velocities(add_noise=False).astype(np.float32)
        ee = joint_sensor.get_end_effector_positions()
        obs["ee_pos_left"] = ee["left"].astype(np.float32)
        obs["ee_pos_right"] = ee["right"].astype(np.float32)
    else:
        obs["joint_pos"] = env._get_joint_angles_deg()
        obs["joint_vel"] = env._get_joint_velocities_deg()
        obs["ee_pos_left"] = env._get_ee_position("left_hand")
        obs["ee_pos_right"] = env._get_ee_position("right_hand")

    # ── 视觉 (来自 CameraSensor) ──
    if camera is not None:
        rgb, depth, seg = camera.capture()
        obs["image"] = rgb
        obs["depth"] = depth
        obs["segmentation"] = seg

    # ── 接触力 ──
    if contact_left is not None:
        obs["contact_left"] = np.array(
            [1.0 if contact_left.is_in_contact() else 0.0], dtype=np.float32
        )
        obs["contact_force_left"] = np.array(
            [contact_left.get_total_contact_force()], dtype=np.float32
        )
    if contact_right is not None:
        obs["contact_right"] = np.array(
            [1.0 if contact_right.is_in_contact() else 0.0], dtype=np.float32
        )
        obs["contact_force_right"] = np.array(
            [contact_right.get_total_contact_force()], dtype=np.float32
        )

    # ── 任务相关 (可选) ──
    if target_pos is not None:
        obs["target_pos"] = np.asarray(target_pos, dtype=np.float32).reshape(-1)[:3]

    return obs


def _build_realistic_observation(env, bridge) -> Dict[str, np.ndarray]:
    """构建 realistic 模式观测 (Sim2Real 对齐, 仅真机可获取数据).

    真机可获取:
      - commanded_joint_pos: 最后指令角度 (开环, 舵机无编码器)
      - is_moving: 动作执行中标记
      - battery_voltage: 电池电压 (V)
      - battery_percent: 电池百分比

    真机不可获取 (本函数不返回):
      - joint_vel (角速度, 需编码器)
      - ee_pos (末端位置, 需编码器或正运动学)
      - image/depth/segmentation (无摄像头)
    """
    battery_info = env.get_battery_info()
    is_moving = False
    if bridge is not None:
        is_moving = bridge.is_moving

    return {
        "commanded_joint_pos": env.get_commanded_joint_pos(),
        "is_moving": np.array([1.0 if is_moving else 0.0], dtype=np.float32),
        "battery_voltage": np.array(
            [battery_info["voltage"]], dtype=np.float32
        ),
        "battery_percent": np.array(
            [battery_info["percent"]], dtype=np.float32
        ),
    }


def build_observation_space(obs_mode: str = "full"):
    """构建 Gymnasium 观测空间 (用于 env 注册).

    参数:
        obs_mode: "full" 或 "realistic"

    返回: gymnasium.spaces.Dict
    """
    import gymnasium as gym

    if obs_mode == "full":
        return gym.spaces.Dict({
            "joint_pos": gym.spaces.Box(
                low=-180.0, high=180.0, shape=(6,), dtype=np.float32),
            "joint_vel": gym.spaces.Box(
                low=-360.0, high=360.0, shape=(6,), dtype=np.float32),
            "ee_pos_left": gym.spaces.Box(
                low=-1.0, high=1.0, shape=(3,), dtype=np.float32),
            "ee_pos_right": gym.spaces.Box(
                low=-1.0, high=1.0, shape=(3,), dtype=np.float32),
            "image": gym.spaces.Box(
                low=0, high=255, shape=(240, 240, 3), dtype=np.uint8),
            "depth": gym.spaces.Box(
                low=0.0, high=10.0, shape=(240, 240), dtype=np.float32),
            "segmentation": gym.spaces.Box(
                low=0, high=255, shape=(240, 240, 3), dtype=np.uint8),
            "contact_left": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
            "contact_right": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
            "contact_force_left": gym.spaces.Box(
                low=0.0, high=100.0, shape=(1,), dtype=np.float32),
            "contact_force_right": gym.spaces.Box(
                low=0.0, high=100.0, shape=(1,), dtype=np.float32),
        })
    elif obs_mode == "realistic":
        return gym.spaces.Dict({
            "commanded_joint_pos": gym.spaces.Box(
                low=-180.0, high=180.0, shape=(6,), dtype=np.float32),
            "is_moving": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32),
            "battery_voltage": gym.spaces.Box(
                low=3.0, high=4.2, shape=(1,), dtype=np.float32),
            "battery_percent": gym.spaces.Box(
                low=0, high=100, shape=(1,), dtype=np.float32),
        })
    else:
        raise ValueError(f"未知 obs_mode: {obs_mode}")


__all__ = ["build_observation", "build_observation_space"]
