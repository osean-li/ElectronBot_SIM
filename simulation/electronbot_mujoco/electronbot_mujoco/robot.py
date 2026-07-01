"""
ElectronBot MuJoCo 机器人 — 核心实现 (仿固件逻辑)

主控制流 1:1 复现 ElectronBot-fw/UserApp/main.cpp:33-94:
  每帧 = 4 轮 USB sync:
    1. 打包当前 6 个关节模型角度 → 32B ExtraData
    2. 发送 32B (模拟 SendUsbPacket)
    3. 接收 224B (模拟 ReceiveUsbPacketUntilSizeIs)
    4. 解析 enable 标志 → SetJointEnable × 6
    5. 解析 6 个 float setpoint → jointSetPoints[0..5]
  帧末:
    6. UpdateJointAngle × 6 (model→mech→I2C→servo→readback→model)

舵机层复现 ServoDrive-fw (servo_sim.py):
  - 200Hz DCE PID 控制 (CalcDceOutput)
  - PWM → torque actuator 输出
  - I2C 命令处理 (0x01 set angle, 0x11 get angle, 0xFF enable)
"""

import os
import struct
import numpy as np
from typing import Tuple, Optional, Dict, List, Any
from pathlib import Path
from dataclasses import dataclass

import mujoco
from mujoco import MjModel, MjData

from .utils import (
    JOINT_PARAMS, JOINT_NAMES, JOINT_IDS,
    model_angle_to_mech, mech_angle_to_model,
    normalize_joint_angles, get_joint_limits_rad,
    deg_to_rad, rad_to_deg,
)
from .servo_sim import ServoSimulator, create_servo_pool, TUNED_PID

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
ASSETS_DIR = Path(__file__).parent / "assets"


# ============================================================
# 数据结构: 复现固件 JointStatus_t 和 ExtraData
# ============================================================

@dataclass
class JointStatus:
    """关节状态 (对应 robot.h:143-152 JointStatus_t)"""
    id: int              # I2C 地址
    angle: float = 0.0   # 当前模型角度 (°)
    mech_min: float = 0.0
    mech_max: float = 180.0
    model_min: float = -90.0
    model_max: float = 90.0
    inverted: bool = False


# ============================================================
# ExtraData 协议: 32 字节精确布局
# ============================================================

def _pack_extra_data_tx(joints: List[JointStatus]) -> bytes:
    """
    打包当前关节角度 → 32 字节 ExtraData (MCU → PC)

    复现: ElectronBot-fw/UserApp/main.cpp:39-44

    Byte 0:    unused (enable 标志由 PC 端设置)
    Bytes 1-4: joint[0].angle (float LE)
    Bytes 5-8: joint[1].angle (float LE)
    ...
    Bytes 21-24: joint[5].angle (float LE)
    Bytes 25-31: reserved
    """
    buf = bytearray(32)
    buf[0] = 0x00  # enable 标志初始为 0
    for j in range(6):
        packed = struct.pack('<f', float(joints[j].angle))
        buf[1 + j * 4: 1 + j * 4 + 4] = packed
    return bytes(buf)


def _unpack_extra_data_rx(data: bytes) -> Tuple[bool, np.ndarray]:
    """
    解析接收的 ExtraData → (enable, 6 个模型角度° )

    复现: ElectronBot-fw/UserApp/main.cpp:50-64

    Byte 0:    enable (0/1)
    Bytes 1-4: jointSetPoint[0] (float LE)
    ...
    """
    enable = (data[0] != 0)
    angles = np.array([
        struct.unpack_from('<f', data, 1 + j * 4)[0]
        for j in range(6)
    ], dtype=np.float64)
    return enable, angles


# ============================================================
# 基础模型加载类 (底层的 MuJoCo 操作)
# ============================================================

class ElectronBotRobot:
    """ElectronBot MuJoCo 模型底层加载与操作"""

    JOINT_NAMES = [
        "body_joint", "head_joint",
        "left_shoulder_joint", "left_arm_roll_joint",
        "right_shoulder_joint", "right_arm_roll_joint",
    ]

    ACTUATOR_NAMES = [
        "act_body", "act_head",
        "act_left_shoulder", "act_left_arm_roll",
        "act_right_shoulder", "act_right_arm_roll",
    ]

    MOTOR_NAMES = [
        "motor_body", "motor_head",
        "motor_left_shoulder", "motor_left_arm_roll",
        "motor_right_shoulder", "motor_right_arm_roll",
    ]

    NUM_JOINTS = 6

    def __init__(self, xml_path: Optional[str] = None):
        if xml_path is None:
            xml_path = str(ASSETS_DIR / "scene.xml")
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"MJCF 文件不存在: {xml_path}")

        self.model = MjModel.from_xml_path(xml_path)
        self.data = MjData(self.model)

        # 缓存 ID
        self._joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.JOINT_NAMES
        ]
        self._actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.ACTUATOR_NAMES
        ]
        self._motor_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.MOTOR_NAMES
        ]
        self._left_ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_site"
        )
        self._right_ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_site"
        )
        self._camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "d435_camera"
        )

    # ---- 关节 access ----
    def get_joint_positions(self) -> np.ndarray:
        return np.array([self.data.qpos[jid] for jid in self._joint_ids])

    def get_joint_velocities(self) -> np.ndarray:
        return np.array([self.data.qvel[jid] for jid in self._joint_ids])

    def set_joint_positions(self, angles_rad: np.ndarray):
        for i, a in enumerate(angles_rad):
            self.data.qpos[self._joint_ids[i]] = float(a)

    # ---- 执行器 ----
    def send_position_command(self, target_angles_rad: np.ndarray):
        for i, t in enumerate(target_angles_rad):
            self.data.ctrl[self._actuator_ids[i]] = float(t)

    def send_torque_command(self, torques: np.ndarray):
        for i, t in enumerate(torques):
            self.data.ctrl[self._motor_ids[i]] = float(t)

    def disable_motors(self):
        for mid in self._motor_ids:
            self.data.ctrl[mid] = 0.0

    # ---- 传感器 ----
    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        return (
            self.data.site_xpos[self._left_ee_site_id].copy(),
            self.data.site_xpos[self._right_ee_site_id].copy(),
        )

    def get_camera_image(self, width: int = 240, height: int = 240) -> np.ndarray:
        renderer = mujoco.Renderer(self.model, height, width)
        renderer.update_scene(self.data, camera=self._camera_id)
        pixels = renderer.render()
        renderer.close()
        return pixels

    # ---- 仿真步进 ----
    def step(self):
        mujoco.mj_step(self.model, self.data)

    def reset(self, qpos: Optional[np.ndarray] = None):
        mujoco.mj_resetData(self.model, self.data)
        if qpos is not None:
            self.set_joint_positions(qpos)
        # 清零所有外力 (避免上一步扰动测试的 qfrc_applied 残留)
        if hasattr(self.data, "qfrc_applied"):
            self.data.qfrc_applied[:] = 0
        if hasattr(self.data, "xfrc_applied"):
            self.data.xfrc_applied[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def get_observation(self) -> np.ndarray:
        q = self.get_joint_positions()
        qd = self.get_joint_velocities()
        left_ee, right_ee = self.get_end_effector_positions()
        return np.concatenate([q, qd, left_ee, right_ee])


# ============================================================
# 固件级机器人 = ElectronBot-fw 主循环复现
# ============================================================

class ElectronBotFirmwareRobot(ElectronBotRobot):
    """
    复现 ElectronBot-fw/UserApp/main.cpp 的完整控制流程

    控制循环 = main.cpp:33-93 主循环:

      ┌─ for p in 0..3 ──────────────────────────┐
      │  pack_joint_angles() → 32 B ExtraData     │  4 轮 USB sync
      │  _receive_extra_data() → enable + setpoints│
      │  if enable changed → _set_joint_enable_all()│
      │  parse 6 setpoints → joint_setpoints[0..5] │
      └───────────────────────────────────────────┘
      _update_all_joints(joint_setpoints)          ← 帧末 I2C 通信
      _servo_step_200hz() × (dt * 200)            ← 舵机 DCE 控制
    """

    def __init__(self, xml_path: Optional[str] = None,
                 apply_tuned_pid: bool = True):
        super().__init__(xml_path=xml_path)

        # ── 6 个 JointStatus (对应 robot.h joint[1]~[6]) ──
        self.joints: List[JointStatus] = []
        for i, p in enumerate(JOINT_PARAMS):
            name, i2c_id, mech_min, mech_max, model_min, model_max, inv = p
            self.joints.append(JointStatus(
                id=i2c_id,
                angle=0.0,
                mech_min=float(mech_min),
                mech_max=float(mech_max),
                model_min=float(model_min),
                model_max=float(model_max),
                inverted=inv,
            ))

        # ── 舵机仿真器 (6 路, 对应 I2C ID 2,4,6,8,10,12) ──
        self.servos = create_servo_pool(apply_tuned=apply_tuned_pid)

        # ── 固件索引 → MuJoCo 关节索引映射 ──
        # JOINT_PARAMS(固件顺序):[head,l_roll,l_pitch,r_roll,r_pitch,body]
        # MuJoCo joint order:      [body,head,l_pitch,l_roll,r_pitch,r_roll]
        # 映射: firmware_idx → mujoco_qpos_idx
        self._fw_to_mujoco = np.array([1, 3, 2, 5, 4, 0], dtype=int)

        # ── 状态 ──
        self.is_enabled: bool = False
        self.joint_setpoints: np.ndarray = np.zeros(6)
        self.control_freq: float = 50.0  # 主循环频率
        self.servo_freq: float = 200.0   # 舵机 DCE 频率

        # ── ExtraData 缓冲 ──
        self._extra_data_tx = bytearray(32)
        self._extra_data_rx = bytearray(32)

    # -------------------------------------------------------
    # 复现 UpdateJointAngle (robot.cpp:225-261)
    # -------------------------------------------------------

    def _update_joint_angle(self, joint_idx: int, model_angle_deg: float):
        """
        更新单个关节角度 (模型角 → 机械角 → 舵机)

        复现: robot.cpp:241-261

        1. 模型角 → 机械角 (使用映射公式)
        2. 发送 I2C 0x01 (Set angle) 给舵机
        3. 从舵机读回机械角
        4. 机械角 → 模型角 (反向映射)
        5. 存储到 joint.angle
        """
        j = self.joints[joint_idx]

        # 1. 模型角 → 机械角
        s_angle = model_angle_to_mech(joint_idx, model_angle_deg)

        # Clamp to mechanical range
        s_angle = float(np.clip(s_angle, j.mech_min, j.mech_max))

        # 2-3. 模拟 I2C 通信: 发送 Set angle, 接收当前机械角
        servo = self.servos[j.id]
        rx_data = servo.handle_i2c_command(
            struct.pack('<Bf', 0x01, float(s_angle))
        )
        # 从返回中读取当前机械角度 (bytes 1-5 = float)
        mech_angle = struct.unpack_from('<f', rx_data, 1)[0]

        # 4. 机械角 → 模型角 (反向映射)
        j_angle = mech_angle_to_model(joint_idx, mech_angle)

        # 5. 存储
        j.angle = float(j_angle)

    def _update_all_joints(self, setpoints_deg: np.ndarray):
        """帧末更新全部 6 个关节 (main.cpp:80-85)"""
        for i in range(6):
            self._update_joint_angle(i, float(setpoints_deg[i]))

    # -------------------------------------------------------
    # 复现 SetJointEnable (robot.cpp:70-78)
    # -------------------------------------------------------

    def _set_joint_enable_all(self, enable: bool):
        """启用/禁用所有 6 个舵机 (main.cpp:54-59)"""
        self.is_enabled = enable
        for j in self.joints:
            servo = self.servos[j.id]
            servo.handle_i2c_command(bytes([0xFF, 1 if enable else 0, 0, 0, 0]))

    # -------------------------------------------------------
    # 复现主循环的 4 轮 USB sync (main.cpp:36-73)
    # -------------------------------------------------------

    def _pack_joint_angles(self):
        """
        打包当前关节角度 → ExtraData (main.cpp:39-44)

        for (int j = 0; j < 6; j++)
            pack joint[j].angle as float into extraDataTx[1+j*4 .. 4+j*4]
        """
        for j_idx in range(6):
            packed = struct.pack('<f', float(self.joints[j_idx].angle))
            self._extra_data_tx[1 + j_idx * 4: 1 + j_idx * 4 + 4] = packed

    def _receive_extra_data(self, setpoints_deg: np.ndarray) -> bool:
        """
        接收并解析 ExtraData (main.cpp:50-64)

        1. 读取 enable 标志 → 如果变化则 SetJointEnable
        2. 解析 6 个 float setpoint
        3. 返回 enable 状态
        """
        # 组合: image_data = (240*240*3/4) + 32 = 43232, 取最后 32 字节
        # 仿真中直接从 buffer 读取
        enable, angles = _unpack_extra_data_rx(bytes(self._extra_data_rx))

        # enable 标志变化检测 → SetJointEnable (main.cpp:51-60)
        if enable != self.is_enabled:
            self._set_joint_enable_all(enable)

        # 存储 setpoints (main.cpp:61-64)
        for j in range(6):
            setpoints_deg[j] = angles[j]

        return enable

    # -------------------------------------------------------
    # 主控制步 (复现 main.cpp:33-94)
    # -------------------------------------------------------

    def control_step(
        self,
        extra_data_rx: Optional[bytes] = None,
    ) -> Tuple[bytes, np.ndarray]:
        """
        执行一个完整的控制帧

        参数:
          extra_data_rx: 模拟从 PC 接收的 32 字节 ExtraData
                         (setpoints + enable 标志)

        返回:
          (extra_data_tx, current_angles_deg)
          extra_data_tx:  32 字节，包含当前 6 个关节模型角度
          current_angles: 6 维模型角度 (°)
        """
        # ── 第 1 轮 (p=0) ──
        self._pack_joint_angles()
        # (仿真中跳过 USB 发送/接收，直接使用传入的 extra_data_rx)
        setpoints = np.zeros(6)
        if extra_data_rx is not None:
            self._extra_data_rx = bytearray(extra_data_rx)
            self._receive_extra_data(setpoints)
        self.joint_setpoints = setpoints.copy()

        # ── 第 2-4 轮 (p=1,2,3): 仿真中省略 (同一帧数据不变) ──
        # 实机做 4 轮是为了分片传输 172800 字节的图像
        # 仿真中不需要，因为图像由 MuJoCo renderer 直接产生

        # ── 帧末: UpdateJointAngle × 6 (main.cpp:80-85) ──
        self._update_all_joints(self.joint_setpoints)

        tx_data = _pack_extra_data_tx(self.joints)
        current_angles = np.array([j.angle for j in self.joints])
        return tx_data, current_angles

    # -------------------------------------------------------
    # 200Hz 舵机 DCE 控制步
    # -------------------------------------------------------

    def servo_control_step(self, dt: float) -> np.ndarray:
        """
        执行舵机 DCE 控制 (复现 main.cpp:222-238)
        返回 6 维扭矩 (Nm) — 已翻转符号适配 MuJoCo motor
        """
        torques = np.zeros(6)
        for i, j in enumerate(self.joints):
            servo = self.servos[j.id]
            mujoco_idx = self._fw_to_mujoco[i]
            model_deg = np.degrees(float(self.data.qpos[self._joint_ids[mujoco_idx]]))
            mech_deg = model_angle_to_mech(i, model_deg)
            servo.update_from_mujoco(np.radians(mech_deg), 0.0)
            torque = servo.step_200hz(dt)
            # DCE 输出在机械角空间: error=current_mech-target_mech
            # 对于 non-inverted: model↑=mech↑, MuJoCo motor 方向 = -DCE
            # 对于 inverted:     model↑=mech↓, MuJoCo motor 方向 = +DCE
            sign = 1 if j.inverted else -1
            torques[mujoco_idx] = sign * torque
        return torques

    def commit_servo_configs(self):
        """将所有 pending 的 PID 配置写入生效"""
        for j in self.joints:
            self.servos[j.id].commit_config()

    # -------------------------------------------------------
    # 外部控制接口 (PC 端调用)
    # -------------------------------------------------------

    def set_target_angles(self, model_angles_deg: np.ndarray, enable: bool = True):
        """
        设置 6 个关节目标角度 (PC → MCU)

        复现 SDK 的 SetJointAngles()

        参数:
          model_angles_deg: 6 维模型角度 (°)
          enable: 是否使能舵机
        """
        self._extra_data_rx[0] = 1 if enable else 0
        for j in range(6):
            struct.pack_into('<f', self._extra_data_rx, 1 + j * 4,
                             float(model_angles_deg[j]))

    def get_current_angles(self) -> np.ndarray:
        """获取当前 6 关节模型角度 (°)"""
        return np.array([j.angle for j in self.joints])

    def get_current_angles_rad(self) -> np.ndarray:
        """获取当前 6 关节模型角度 (rad)"""
        return np.radians(self.get_current_angles())

    def get_dce_outputs(self) -> np.ndarray:
        """获取 6 路舵机当前 DCE 输出 (用于调试)"""
        return np.array([self.servos[j.id]._output for j in self.joints])

    # -------------------------------------------------------
    # 仿真集成: step with DCE control
    # -------------------------------------------------------

    def physics_step_with_dce(self, extra_data_rx: Optional[bytes] = None
                              ) -> Tuple[bytes, np.ndarray]:
        """完整物理步: 固件控制帧 + 200Hz DCE + MuJoCo physics"""
        tx_data, angles_deg = self.control_step(extra_data_rx)
        n_servo = int(self.servo_freq / self.control_freq)  # 4
        dt_servo = 1.0 / self.servo_freq                    # 0.005s
        for _ in range(n_servo):
            torques = self.servo_control_step(dt_servo)
            self.send_torque_command(torques)         # DCE → motor actuator
            mujoco.mj_step(self.model, self.data)
        return tx_data, angles_deg

    def reset(self, qpos: Optional[np.ndarray] = None):
        """重置并同步所有状态"""
        super().reset(qpos=qpos)

        # 同步 JointStatus 和舵机 DCE setpoint 到初始位置
        init_q = self.get_joint_positions()  # rad
        init_deg = np.degrees(init_q)

        for i, j in enumerate(self.joints):
            j.angle = float(init_deg[i])
            servo = self.servos[j.id]
            servo.setpoint_pos = float(init_deg[i])
            servo.angle = float(init_deg[i])
            servo._integral_pos = 0.0
            servo._integral_vel = 0.0
            servo._last_error = 0.0

        # 默认使能
        self.is_enabled = True
        for j in self.joints:
            self.servos[j.id].enabled = True

        # ExtraData 缓冲
        self._extra_data_rx = bytearray(32)
        self._extra_data_rx[0] = 1  # enable

    def disable(self):
        """禁用所有舵机 (对应 SetJointEnable all, false)"""
        self._set_joint_enable_all(False)

    def enable(self):
        """使能所有舵机"""
        self._set_joint_enable_all(True)
