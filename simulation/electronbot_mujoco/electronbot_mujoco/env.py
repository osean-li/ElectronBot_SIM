"""
ElectronBot Gymnasium 环境 — 双模式

1. ElectronBotEnv (基础模式): 直接 position actuator, 适用于简单 RL 训练
2. ElectronBotFirmwareEnv (固件模式): 1:1 复现固件控制循环, 适用于 Sim2Real

状态空间 (18维):
  - joint_positions (6): 当前关节角度 (rad)
  - joint_velocities (6): 当前关节速度 (rad/s)
  - ee_positions (6): 左右末端执行器 3D 位置 (m)

动作空间 (6维):
  - target_joint_angles (6): 6 个模型角度 (rad)
"""

import numpy as np
from typing import Optional, Tuple, Dict, Any

import gymnasium as gym
from gymnasium import spaces

from .robot import ElectronBotRobot, ElectronBotFirmwareRobot
from .utils import normalize_joint_angles, get_joint_limits_rad


class ElectronBotEnv(gym.Env):
    """ElectronBot 基础 RL 环境 (简单 position actuator)"""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    NUM_JOINTS = 6
    JOINT_LOW, JOINT_HIGH = get_joint_limits_rad()
    OBS_DIM = 18

    def __init__(
        self,
        xml_path: Optional[str] = None,
        render_mode: Optional[str] = None,
        control_freq: int = 50,
        episode_length: int = 500,
    ):
        super().__init__()
        self.control_freq = control_freq
        self.max_episode_steps = episode_length
        self.render_mode = render_mode

        self.robot = ElectronBotRobot(xml_path=xml_path)
        self.model = self.robot.model
        self.data = self.robot.data
        self.model.opt.timestep = 1.0 / (control_freq * 5)

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.NUM_JOINTS,), dtype=np.float64,
        )
        obs_low = np.concatenate([
            self.JOINT_LOW, np.full(6, -10.0), np.full(6, -0.5),
        ]).astype(np.float64)
        obs_high = np.concatenate([
            self.JOINT_HIGH, np.full(6, 10.0), np.full(6, 0.5),
        ]).astype(np.float64)
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float64,
        )

        self._step_count = 0
        self._renderer = None

    def _get_obs(self) -> np.ndarray:
        return self.robot.get_observation()

    def _get_info(self) -> Dict[str, Any]:
        return {
            "joint_positions": self.robot.get_joint_positions(),
            "joint_velocities": self.robot.get_joint_velocities(),
            "step": self._step_count,
        }

    def _compute_reward(self) -> float:
        return 0.0

    def _is_terminated(self) -> bool:
        return False

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._step_count = 0
        if options and "qpos" in options:
            init_qpos = np.array(options["qpos"], dtype=np.float64)
        else:
            init_qpos = self.np_random.uniform(
                low=self.JOINT_LOW * 0.1, high=self.JOINT_HIGH * 0.1, size=(6,),
            )
        self.robot.reset(qpos=init_qpos)
        return self._get_obs(), self._get_info()

    def step(self, action: np.ndarray
             ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self._step_count += 1
        current_q = self.robot.get_joint_positions()
        target_q = current_q + action * 0.1
        target_q = normalize_joint_angles(target_q)
        self.robot.send_position_command(target_q)
        for _ in range(5):
            self.robot.step()
        return (
            self._get_obs(), self._compute_reward(),
            self._is_terminated(),
            self._step_count >= self.max_episode_steps,
            self._get_info(),
        )

    def render(self) -> Optional[np.ndarray]:
        if self.render_mode == "rgb_array":
            return self.robot.get_camera_image()
        return None

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
        super().close()


# ============================================================
# 固件模式环境 — 复现 ElectronBot-fw/UserApp/main.cpp
# ============================================================

class ElectronBotFirmwareEnv(ElectronBotEnv):
    """
    固件级 Gymnasium 环境

    复现 ElectronBot-fw/UserApp/main.cpp:33-94 的完整控制流程:
      每帧 = 4 轮 USB sync + UpdateJointAngle × 6
      舵机 = 200Hz DCE PID 控制 (servo_sim.py)

    与基础环境的区别:
      1. 使用 ElectronBotFirmwareRobot (不是 ElectronBotRobot)
      2. 控制通过 ExtraData 32 字节协议 (与实际 USB 通信一致)
      3. 舵机由 DCE PID 控制 (力矩模式), 而非 MuJoCo position actuator
      4. step() 中执行完整的 physics_step_with_dce()
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 50}

    def __init__(
        self,
        xml_path: Optional[str] = None,
        render_mode: Optional[str] = None,
        control_freq: int = 50,
        episode_length: int = 500,
        apply_tuned_pid: bool = True,
    ):
        # 跳过父类 __init__, 重新初始化
        gym.Env.__init__(self)
        self.control_freq = control_freq
        self.max_episode_steps = episode_length
        self.render_mode = render_mode

        # 使用固件级机器人
        self.robot = ElectronBotFirmwareRobot(
            xml_path=xml_path, apply_tuned_pid=apply_tuned_pid,
        )
        self.model = self.robot.model
        self.data = self.robot.data
        self.model.opt.timestep = 1.0 / self.robot.servo_freq  # 200Hz 子步

        # 动作空间: 6 维模型角度 (rad) — 对应 ExtraData 中的 6 个 float
        self.action_space = spaces.Box(
            low=self.JOINT_LOW, high=self.JOINT_HIGH,
            shape=(6,), dtype=np.float64,
        )

        obs_low = np.concatenate([
            self.JOINT_LOW, np.full(6, -20.0), np.full(6, -0.5),
        ]).astype(np.float64)
        obs_high = np.concatenate([
            self.JOINT_HIGH, np.full(6, 20.0), np.full(6, 0.5),
        ]).astype(np.float64)
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float64,
        )

        self._step_count = 0
        self._renderer = None

    def step(self, action: np.ndarray
             ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        执行一个控制帧

        复现: main.cpp 主循环 + 200Hz DCE 控制

        参数:
          action: 6 维模型角度 (rad) — 等效于 ExtraData 中的 jointSetPoints

        返回:
          obs, reward, terminated, truncated, info
        """
        self._step_count += 1

        # 转换为度 (MuJoCo 顺序: [body,head,l_pitch,l_roll,r_pitch,r_roll])
        angles_mj = np.degrees(normalize_joint_angles(action))

        # 重排为固件 ExtraData 顺序: [head,l_roll,l_pitch,r_roll,r_pitch,body]
        M2F = [1, 3, 2, 5, 4, 0]  # MuJoCo idx → firmware idx
        angles_fw = angles_mj[M2F]

        # 打包 ExtraData (PC → MCU): enable=1 + 6 float setpoints
        from electronbot_real.protocol import encode_extra_data
        extra_data = encode_extra_data(True, angles_fw)

        # 执行完整物理步: 固件控制帧 + 4 × 200Hz DCE 子步 + MuJoCo physics
        tx_data, current_angles_deg = self.robot.physics_step_with_dce(extra_data)

        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = self._is_terminated()
        truncated = self._step_count >= self.max_episode_steps
        info = self._get_info()
        info["dce_outputs"] = self.robot.get_dce_outputs()
        info["joint_angles_deg"] = current_angles_deg

        return obs, reward, terminated, truncated, info

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._step_count = 0

        if options and "qpos" in options:
            init_qpos = np.array(options["qpos"], dtype=np.float64)
        else:
            init_qpos = self.np_random.uniform(
                low=self.JOINT_LOW * 0.1, high=self.JOINT_HIGH * 0.1, size=(6,),
            )

        # 使用固件级 reset（自动同步 JointStatus + 舵机 DCE + position actuator）
        self.robot.reset(qpos=init_qpos)

        return self._get_obs(), self._get_info()

    def enable(self):
        """使能所有舵机"""
        self.robot._set_joint_enable_all(True)

    def disable(self):
        """禁用所有舵机"""
        self.robot._set_joint_enable_all(False)


# 注册
gym.register(id="ElectronBot-v0",
             entry_point="electronbot_mujoco.env:ElectronBotEnv",
             max_episode_steps=500)
gym.register(id="ElectronBot-fw-v0",
             entry_point="electronbot_mujoco.env:ElectronBotFirmwareEnv",
             max_episode_steps=500)
