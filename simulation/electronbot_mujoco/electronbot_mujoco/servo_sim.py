"""
ElectronBot 舵机仿真器 (Servo Simulator)

1:1 复现 STM32F042P6 舵机驱动板的 DCE 控制逻辑:
- 200Hz 控制循环 (TIM14 中断)
- 电位器 + 12-bit ADC 角度测量 → 线性映射
- DCE PID (KP/KI/KV/KD) 输出限幅
- FM116B H-bridge PWM (0~1000 占空比)
- I2C 从机命令处理 (0x01 set angle, 0x11 get angle, 0x21-0x27 config, 0xFF enable)

来源文件:
- ServoDrive-fw/Ctrl/motor.cpp  (DCE control)
- ServoDrive-fw/UserApp/main.cpp  (I2C command handler + 200Hz control loop)
- ServoDrive-fw/UserApp/configurations.h  (BoardConfig_t)
"""

import math
import struct
import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass, field


# ============================================================
# 舵机出厂默认配置 = BoardConfig_t
# ============================================================

@dataclass
class ServoFactoryConfig:
    """舵机驱动板出厂默认配置 (对应 configurations.h:20-36)"""
    node_id: int = 12                    # 7-bit I2C 地址 (偶数)
    init_pos: float = 90.0              # 初始机械角度 (°)
    torque_limit: float = 0.5           # 力矩限制 (0~1 的比例)
    velocity_limit: float = 0.0         # 速度限制 (0=不限)
    adc_val_min: int = 250              # 最小角度对应的 ADC 值
    adc_val_max: int = 3000             # 最大角度对应的 ADC 值
    mechanical_angle_min: float = 0.0   # 机械最小角度 (°)
    mechanical_angle_max: float = 180.0 # 机械最大角度 (°)
    dce_kp: float = 10.0                # DCE 比例增益
    dce_kv: float = 0.0                 # DCE 速度积分增益
    dce_ki: float = 0.0                 # DCE 位置积分增益
    dce_kd: float = 50.0                # DCE 微分增益
    enable_on_boot: bool = False        # 上电使能


# ============================================================
# 仿真舵机 = 复现 DCE 控制回路
# ============================================================

class ServoSimulator:
    """
    模拟单个舵机驱动板 (STM32F042P6 + FM116B)

    控制频率: 200 Hz (对应 TIM14 定时器)
    PWM 范围: 0~1000 (对应 TIM3 占空比)
    """

    DCE_INTEGRAL_LIMIT = 500.0           # motor.h:13
    PWM_MAX = 1000                       # motor.cpp:36

    def __init__(self, config: Optional[ServoFactoryConfig] = None):
        cfg = config or ServoFactoryConfig()

        # 配置
        self.node_id = cfg.node_id
        self.adc_min = cfg.adc_val_min
        self.adc_max = cfg.adc_val_max
        self.mech_min = cfg.mechanical_angle_min
        self.mech_max = cfg.mechanical_angle_max

        # DCE 参数 (运行时可通过 I2C 修改)
        self.kp = cfg.dce_kp
        self.ki = cfg.dce_ki
        self.kv = cfg.dce_kv
        self.kd = cfg.dce_kd
        self.torque_limit = cfg.torque_limit * self.PWM_MAX  # 输出限幅

        # 状态
        self.enabled: bool = cfg.enable_on_boot
        self.angle: float = cfg.init_pos      # 当前机械角度
        self.velocity: float = 0.0            # 当前速度
        self.setpoint_pos: float = cfg.init_pos  # 目标机械角度
        self.setpoint_vel: float = 0.0

        # DCE 积分/历史
        self._integral_pos: float = 0.0
        self._integral_vel: float = 0.0
        self._last_error: float = 0.0
        self._output: float = 0.0

        # PID 缓冲 (用于 host 写入, 对应 robot.h 调优值)
        self._pending_kp: Optional[float] = None
        self._pending_kd: Optional[float] = None
        self._pending_ki: Optional[float] = None
        self._pending_kv: Optional[float] = None
        self._pending_torque_limit: Optional[float] = None
        self._pending_init_pos: Optional[float] = None
        self._pending_node_id: Optional[int] = None

    # -----------------------------------------------------------
    # I2C 命令处理 (对应 main.cpp:95-208)
    # -----------------------------------------------------------

    def handle_i2c_command(self, rx_data: bytes) -> bytes:
        """
        处理 5 字节 I2C 命令，返回 5 字节响应

        对应 HAL_I2C_SlaveRxCpltCallback()
        """
        cmd = rx_data[0]
        # 默认返回: command echo + current angle
        tx_data = bytearray(5)
        tx_data[0] = cmd

        if cmd == 0x01:  # Set angle
            val_f = struct.unpack('<f', rx_data[1:5])[0]
            self.setpoint_pos = float(np.clip(val_f, self.mech_min, self.mech_max))
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x02:  # Set velocity
            val_f = struct.unpack('<f', rx_data[1:5])[0]
            self.setpoint_vel = val_f
            tx_data[1:5] = struct.pack('<f', self.velocity)

        elif cmd == 0x03:  # Set torque
            val_f = struct.unpack('<f', rx_data[1:5])[0]
            self.torque_limit = np.clip(val_f, 0, 1) * self.PWM_MAX
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x11:  # Get angle
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x12:  # Get velocity
            tx_data[1:5] = struct.pack('<f', self.velocity)

        elif cmd == 0x21:  # Set ID
            self._pending_node_id = rx_data[1]
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x22:  # Set Kp
            self._pending_kp = struct.unpack('<f', rx_data[1:5])[0]
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x23:  # Set Ki
            self._pending_ki = struct.unpack('<f', rx_data[1:5])[0]
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x24:  # Set Kv
            self._pending_kv = struct.unpack('<f', rx_data[1:5])[0]
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x25:  # Set Kd
            self._pending_kd = struct.unpack('<f', rx_data[1:5])[0]
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x26:  # Set torque limit
            val_f = struct.unpack('<f', rx_data[1:5])[0]
            self._pending_torque_limit = np.clip(val_f, 0, 1)
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0x27:  # Set init pos
            self._pending_init_pos = struct.unpack('<f', rx_data[1:5])[0]
            tx_data[1:5] = struct.pack('<f', self.angle)

        elif cmd == 0xFF:  # Enable/Disable
            self.enabled = (rx_data[1] != 0)
            tx_data[1:5] = struct.pack('<f', self.angle)

        else:
            tx_data[1:5] = struct.pack('<f', self.angle)

        return bytes(tx_data)

    # -----------------------------------------------------------
    # DCE 控制计算 (对应 motor.cpp:5-26 CalcDceOutput)
    # -----------------------------------------------------------

    def calc_dce_output(self, input_pos: float, input_vel: float) -> float:
        """
        DCE PID 输出计算

        output = Kp * errorPos + Ki * integralPos + Kv * integralVel + Kd * deltaPos
        clamped to ±limitTorque
        """
        error_pos = input_pos - self.setpoint_pos
        error_vel = input_vel - self.setpoint_vel
        delta_pos = error_pos - self._last_error

        self._last_error = error_pos

        # Integral with anti-windup
        self._integral_pos += error_pos
        if self._integral_pos > self.DCE_INTEGRAL_LIMIT:
            self._integral_pos = self.DCE_INTEGRAL_LIMIT
        elif self._integral_pos < -self.DCE_INTEGRAL_LIMIT:
            self._integral_pos = -self.DCE_INTEGRAL_LIMIT

        self._integral_vel += error_vel
        if self._integral_vel > self.DCE_INTEGRAL_LIMIT:
            self._integral_vel = self.DCE_INTEGRAL_LIMIT
        elif self._integral_vel < -self.DCE_INTEGRAL_LIMIT:
            self._integral_vel = -self.DCE_INTEGRAL_LIMIT

        # DCE output
        out = (self.kp * error_pos
               + self.ki * self._integral_pos
               + self.kv * self._integral_vel
               + self.kd * delta_pos)

        # Clamp
        torque = np.clip(out, -self.torque_limit, self.torque_limit)
        self._output = torque
        return torque

    # -----------------------------------------------------------
    # PWM 输出 (对应 motor.cpp:29-47 SetPwm)
    # -----------------------------------------------------------

    @staticmethod
    def _pwm_to_torque(pwm: float) -> float:
        """PWM → 扭矩 (Nm), 用于驱动 MuJoCo motor actuator"""
        # FM116B + N20 微型减速电机: 1000 PWM (100%) ≈ 1.5 Nm stall torque
        return pwm / 1000.0 * 1.5

    # -----------------------------------------------------------
    # 每步更新 (200Hz) = HAL_TIM_PeriodElapsedCallback
    # -----------------------------------------------------------

    def step_200hz(self, dt: float = 0.005) -> float:
        """
        执行一个 200Hz 控制步

        对应 main.cpp:222-238

        返回: 等效 MuJoCo 扭矩 (Nm)
        """
        # --- 1. ADC 读取 + 角度计算 ---
        # 在实机中: HAL_ADC_Start_DMA() 从电位器读取 ADC 值
        # 仿真中: 直接使用 self.angle (被 MuJoCo 更新)

        # --- 2. 角度计算 (main.cpp:230-233) ---
        # angle = mech_min + (mech_max - mech_min)
        #        * (adc_val - adc_min) / (adc_max - adc_min)
        # 仿真中: self.angle 已由 MuJoCo physics 更新

        # --- 3. DCE PID 计算 (motor.cpp:5-26) ---
        dce_out = self.calc_dce_output(self.angle, self.velocity)

        # --- 4. PWM 输出 (motor.cpp:29-47) ---
        if self.enabled:
            pwm = np.clip(dce_out, -self.PWM_MAX, self.PWM_MAX)
        else:
            pwm = 0.0

        return self._pwm_to_torque(pwm)

    # -----------------------------------------------------------
    # 外部更新 (由 MuJoCo step 回调)
    # -----------------------------------------------------------

    def update_from_mujoco(self, angle_rad: float, velocity_rad_s: float):
        """MuJoCo 仿真步后，更新舵机的角度/速度"""
        self.angle = math.degrees(angle_rad)
        self.velocity = math.degrees(velocity_rad_s)

    # -----------------------------------------------------------
    # PID 固化 (pending → applied, 对应 CONFIG_COMMIT)
    # -----------------------------------------------------------

    def commit_config(self):
        """将 pending 的 PID 配置写入生效"""
        if self._pending_kp is not None:
            self.kp = self._pending_kp
            self._pending_kp = None
        if self._pending_ki is not None:
            self.ki = self._pending_ki
            self._pending_ki = None
        if self._pending_kv is not None:
            self.kv = self._pending_kv
            self._pending_kv = None
        if self._pending_kd is not None:
            self.kd = self._pending_kd
            self._pending_kd = None
        if self._pending_torque_limit is not None:
            self.torque_limit = self._pending_torque_limit * self.PWM_MAX
            self._pending_torque_limit = None
        if self._pending_init_pos is not None:
            self.setpoint_pos = self._pending_init_pos
            self._pending_init_pos = None
        if self._pending_node_id is not None:
            self.node_id = self._pending_node_id
            self._pending_node_id = None

    @property
    def output_pwm(self) -> float:
        """当前 DCE 输出 (PWM 等效值)"""
        return self._output


# ============================================================
# 6 路舵机组
# ============================================================

# 各关节 Host 调优后的 PID (来自 robot.h 注释)
TUNED_PID = [
    # (Kp, Ki, Kd, TorqueLimit) 对应 joint[1]~[6]
    {"kp": 30,  "ki": 0.4, "kd": 200, "tl": 0.5},   # j1 Head
    {"kp": 50,  "ki": 0.8, "kd": 600, "tl": 1.0},   # j2 L.ArmRoll
    {"kp": 50,  "ki": 0.8, "kd": 300, "tl": 0.5},   # j3 L.ArmPitch
    {"kp": 50,  "ki": 0.8, "kd": 600, "tl": 1.0},   # j4 R.ArmRoll
    {"kp": 50,  "ki": 0.8, "kd": 300, "tl": 0.5},   # j5 R.ArmPitch
    {"kp": 150, "ki": 0.8, "kd": 300, "tl": 0.5},   # j6 Body
]


def create_servo_pool(apply_tuned: bool = True) -> Dict[int, ServoSimulator]:
    """
    创建 6 路舵机仿真器

    参数:
      apply_tuned: 是否应用 Host 调优后的 PID
    """
    pool = {}

    for idx, i2c_addr in enumerate([2, 4, 6, 8, 10, 12]):
        cfg = ServoFactoryConfig(node_id=i2c_addr, init_pos=90.0)

        if apply_tuned:
            pid = TUNED_PID[idx]
            cfg.dce_kp = pid["kp"]
            cfg.dce_ki = pid["ki"]
            cfg.dce_kd = pid["kd"]
            cfg.torque_limit = pid["tl"]

        pool[i2c_addr] = ServoSimulator(config=cfg)

    return pool
