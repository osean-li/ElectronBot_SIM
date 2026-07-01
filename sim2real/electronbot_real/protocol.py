"""
ElectronBot 通信协议编解码

1:1 复现 firmware/SDK 的 ExtraData 32 字节协议:

  PC → MCU (SetJointAngles, electron_low_level.cpp:157-176):
    Byte 0:     enable (0/1)
    Bytes 1-24: 6 × float LE (模型角度, 非机械角度!)

  MCU → PC (GetJointAngles, electron_low_level.cpp:179-185):
    同上格式, 包含当前读取的 6 个模型角度

来源:
  - electron_low_level.cpp: SetJointAngles / GetJointAngles
  - robot.h: JointStatus_t / UsbBuffer_t
  - robot.cpp: UpdateJointAngle (模型角 → 机械角的转换在固件内完成)
"""

import struct
import numpy as np
from typing import Tuple

EXTRA_DATA_SIZE = 32


def encode_extra_data(enable: bool, model_angles_deg: np.ndarray) -> bytes:
    """
    编码 ExtraData: PC → MCU

    复现: electron_low_level.cpp:157-176 SetJointAngles()

    参数:
      enable: 使能标志
      model_angles_deg: 6 维模型角度 (°), 对应 joint[0]~[5]
    """
    buf = bytearray(EXTRA_DATA_SIZE)
    buf[0] = 1 if enable else 0
    for j in range(6):
        struct.pack_into('<f', buf, 1 + j * 4, float(model_angles_deg[j]))
    return bytes(buf)


def decode_extra_data(data: bytes) -> Tuple[bool, np.ndarray]:
    """
    解码 ExtraData: MCU → PC

    复现: electron_low_level.cpp:179-185 GetJointAngles()

    返回:
      (enable, model_angles_deg) — 6 维模型角度 (°)
    """
    enable = (data[0] != 0)
    angles = np.array([
        struct.unpack_from('<f', data, 1 + j * 4)[0]
        for j in range(6)
    ], dtype=np.float64)
    return enable, angles


def encode_extra_data_rad(enable: bool, model_angles_rad: np.ndarray) -> bytes:
    """编码 ExtraData (弧度输入)"""
    return encode_extra_data(enable, np.degrees(model_angles_rad))


def decode_extra_data_rad(data: bytes) -> Tuple[bool, np.ndarray]:
    """解码 ExtraData (弧度输出)"""
    enable, angles_deg = decode_extra_data(data)
    return enable, np.radians(angles_deg)


# ============================================================
# I2C 命令帧 (5 字节)
# ============================================================

I2C_COMMANDS = {
    0x01: "SET_ANGLE",
    0x02: "SET_VELOCITY",
    0x03: "SET_TORQUE",
    0x11: "GET_ANGLE",
    0x12: "GET_VELOCITY",
    0x21: "SET_ID",
    0x22: "SET_KP",
    0x23: "SET_KI",
    0x24: "SET_KV",
    0x25: "SET_KD",
    0x26: "SET_TORQUE_LIMIT",
    0x27: "SET_INIT_POS",
    0xFF: "ENABLE",
}


def make_i2c_set_angle(mech_angle_deg: float) -> bytes:
    """构造 I2C 0x01 Set Angle 命令"""
    return struct.pack('<Bf', 0x01, float(mech_angle_deg))


def make_i2c_enable(on: bool) -> bytes:
    """构造 I2C 0xFF Enable/Disable 命令"""
    return bytes([0xFF, 1 if on else 0, 0, 0, 0])


def make_i2c_set_kp(value: float) -> bytes:
    return struct.pack('<Bf', 0x22, float(value))


def make_i2c_set_kd(value: float) -> bytes:
    return struct.pack('<Bf', 0x25, float(value))


def parse_i2c_response(data: bytes) -> Tuple[int, float]:
    """解析 I2C 响应: (command_echo, angle_float)"""
    cmd = data[0]
    angle = struct.unpack_from('<f', data, 1)[0]
    return cmd, angle
