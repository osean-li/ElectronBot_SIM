"""
ElectronBot 阻抗控制器 (Impedance Controller)

基于 MuJoCo torque actuator 实现导纳控制 (Admittance Control)

控制律:
  tau = K_p * (q_d - q) + K_d * (qd_d - qd) + tau_ext_compensation

特点:
- 位置模式: 使用 position actuator (高刚度)
- 阻抗模式: 使用 torque actuator + 导纳控制 (低刚度, 柔顺交互)
- 支持外力扰动下的轨迹跟踪
- 对 Sim2Real 迁移至关重要: 抵抗建模误差和未建模动力学
"""

import numpy as np
from typing import Tuple, Optional, Dict, Any


class ImpedanceController:
    """
    导纳/阻抗控制器

    控制律 (连续时间):
        M * ddx + D * dx + K * x = F_ext
        其中 x = q - q_d, F_ext 为外力

    离散实现:
        tau = K_p * (q_d - q) + K_d * (qd_d - qd)
    """

    def __init__(
        self,
        joint_names: list,
        K_p: np.ndarray = None,
        K_d: np.ndarray = None,
        dt: float = 0.02,
    ):
        """
        参数:
          joint_names: 受控关节名称列表
          K_p: 刚度系数 (6,) 或 (6,6) 矩阵
          K_d: 阻尼系数 (6,) 或 (6,6) 矩阵
          dt: 控制周期 (秒)
        """
        self.joint_names = joint_names
        self.n_joints = len(joint_names)
        self.dt = dt

        # 默认刚度和阻尼参数
        # 高刚度 → 精确位置跟踪; 低刚度 → 柔顺交互
        if K_p is None:
            # 默认: 中等刚度
            self.K_p = np.diag([10.0, 10.0, 5.0, 5.0, 5.0, 5.0, 10.0])
        elif K_p.ndim == 1:
            self.K_p = np.diag(K_p)
        else:
            self.K_p = K_p

        if K_d is None:
            # 临界阻尼: D = 2 * sqrt(K)
            self.K_d = np.diag(2.0 * np.sqrt(np.diag(self.K_p)))
        elif K_d.ndim == 1:
            self.K_d = np.diag(K_d)
        else:
            self.K_d = K_d

        # 外力估计 (用于补偿)
        self._external_torque = np.zeros(self.n_joints)

    def set_stiffness_mode(self, mode: str):
        """
        设置刚度模式

        参数:
          mode: "high" (高刚度, ≈位置控制), "medium" (中等), "low" (低刚度, 柔顺)

        对应场景:
          high   → 精确位置跟踪 (如 Reach 任务)
          medium → 一般操作 (如 Push 任务)
          low    → 柔顺交互 (如人机协作, 接触探测)
        """
        if mode == "high":
            k = [20.0, 20.0, 15.0, 15.0, 15.0, 15.0]
        elif mode == "medium":
            k = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
        elif mode == "low":
            k = [3.0, 3.0, 2.0, 2.0, 2.0, 2.0]
        else:
            raise ValueError(f"未知模式: {mode}")

        self.K_p = np.diag(k)
        self.K_d = np.diag(2.0 * np.sqrt(np.array(k)))

    def compute_torque(
        self,
        q_current: np.ndarray,
        qd_current: np.ndarray,
        q_desired: np.ndarray,
        qd_desired: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        计算控制力矩

        参数:
          q_current: 当前关节位置 (rad, 6维)
          qd_current: 当前关节速度 (rad/s, 6维)
          q_desired: 期望关节位置 (rad, 6维)
          qd_desired: 期望关节速度 (rad/s, 6维, 可选)

        返回:
          tau: 控制力矩 (Nm, 6维)
        """
        if qd_desired is None:
            qd_desired = np.zeros(self.n_joints)

        # 位置误差
        e_pos = q_desired - q_current
        e_vel = qd_desired - qd_current

        # 阻抗控制律
        tau = self.K_p @ e_pos + self.K_d @ e_vel

        return tau

    def send_impedance_command(self, robot, q_desired: np.ndarray, qd_desired: np.ndarray = None):
        """
        通过 torque actuator 发送阻抗控制指令

        参数:
          robot: ElectronBotRobot 实例
          q_desired: 期望关节位置 (rad)
          qd_desired: 期望关节速度 (rad/s)
        """
        q = robot.get_joint_positions()
        qd = robot.get_joint_velocities()

        tau = self.compute_torque(q, qd, q_desired, qd_desired)
        robot.send_torque_command(tau)

    def switch_to_position_mode(self, robot):
        """切换到位置控制模式 (高刚度)"""
        robot.disable_motors()
        q = robot.get_joint_positions()
        robot.send_position_command(q)

    def switch_to_impedance_mode(self, robot, stiffness: str = "medium"):
        """切换到阻抗控制模式"""
        self.set_stiffness_mode(stiffness)
        robot.disable_motors()


def test_disturbance_rejection(robot):
    """
    抗外力扰动测试

    施加外力后，验证阻抗控制器能否保持位置跟踪
    """
    import time

    print("[TEST] 阻抗控制 - 抗外力扰动测试")

    controller = ImpedanceController(
        joint_names=robot.JOINT_NAMES,
        dt=0.02,
    )
    controller.set_stiffness_mode("medium")

    # 设置目标位置
    q_target = np.zeros(6)
    q_target[2] = np.deg2rad(30)  # 左臂抬起 30°

    robot.reset(qpos=np.zeros(6))

    # 切换到阻抗控制
    controller.switch_to_impedance_mode(robot, "medium")

    print(f"  目标位置 (deg): {np.round(np.rad2deg(q_target), 1)}")

    # 运行控制循环
    for step in range(300):
        controller.send_impedance_command(robot, q_target)

        # 在第 150 步模拟外力扰动 (对左肩关节施加 -0.2 Nm)
        if step == 150:
            # 在 MuJoCo 中施加外力
            joint_id = robot._joint_ids[2]  # left_shoulder
            robot.data.qfrc_applied[joint_id] = -0.3
            print(f"\n  [EVENT] 施加外力扰动: -0.3 Nm @ left_shoulder")

        robot.step()

        if step % 50 == 0:
            q = robot.get_joint_positions_deg()
            err = np.max(np.abs(q - np.rad2deg(q_target)))
            print(f"  step {step:3d}: 当前={np.round(q[[2,3]], 1)}°, "
                  f"max_err={err:.2f}°")

    # 恢复位置控制
    controller.switch_to_position_mode(robot)

    # 检查恢复
    final_q = robot.get_joint_positions_deg()
    final_err = np.max(np.abs(final_q - np.rad2deg(q_target)))
    print(f"\n  最终位置误差: {final_err:.2f}°")
    print(f"  [{'OK' if final_err < 5.0 else 'WARN'}] 抗扰动测试完成")

    return final_err < 5.0
