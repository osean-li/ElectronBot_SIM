"""
Sim2Real Bridge: 统一机器人接口抽象层

定义 RobotInterface 基类，仿真模式调用 MuJoCo，实机模式调用 USB CDC，
确保上层代码 (RL/IL/VLA/行为树) 零修改切换。
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple, Optional


class RobotInterface(ABC):
    """
    统一的机器人控制接口

    所有上层模块 (RL, IL, VLA, 行为树) 通过此接口控制机器人，
    无需关心底层是仿真还是真实硬件。
    """

    @abstractmethod
    def connect(self) -> bool:
        """建立连接"""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """断开连接"""
        pass

    @abstractmethod
    def get_joint_positions(self) -> np.ndarray:
        """获取 6 维关节角度 (rad)"""
        pass

    @abstractmethod
    def get_joint_velocities(self) -> np.ndarray:
        """获取 6 维关节速度 (rad/s)"""
        pass

    @abstractmethod
    def send_joint_command(self, angles: np.ndarray) -> None:
        """发送 6 维关节角度指令 (rad)"""
        pass

    @abstractmethod
    def get_camera_image(self) -> np.ndarray:
        """获取相机 RGB 图像 (240x240x3)"""
        pass

    @abstractmethod
    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取左右末端执行器位置"""
        pass

    def is_connected(self) -> bool:
        """检查连接状态"""
        return True  # 默认实现


class SimRobotInterface(RobotInterface):
    """仿真模式: 通过 MuJoCo 控制机器人"""

    def __init__(self, robot):
        """
        参数:
          robot: ElectronBotRobot 实例
        """
        self._robot = robot
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def get_joint_positions(self) -> np.ndarray:
        return self._robot.get_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        return self._robot.get_joint_velocities()

    def send_joint_command(self, angles: np.ndarray) -> None:
        self._robot.send_position_command(angles)

    def get_camera_image(self) -> np.ndarray:
        return self._robot.get_camera_image()

    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._robot.get_end_effector_positions()


class RealRobotInterface(RobotInterface):
    """
    实机模式: 通过 USB CDC 控制真实机器人

    (Phase 9 实现)
    """

    def __init__(self, vid: int = 0x1001, pid: int = 0x8023):
        self._vid = vid
        self._pid = pid
        self._connected = False
        self._driver = None

    def connect(self) -> bool:
        # 待实现: USB CDC 连接
        # self._driver = USBDriver(vid=self._vid, pid=self._pid)
        # self._connected = self._driver.open()
        raise NotImplementedError("RealRobotInterface 待 Phase 9 实现")

    def disconnect(self) -> None:
        raise NotImplementedError("RealRobotInterface 待 Phase 9 实现")

    def get_joint_positions(self) -> np.ndarray:
        raise NotImplementedError()

    def get_joint_velocities(self) -> np.ndarray:
        raise NotImplementedError()

    def send_joint_command(self, angles: np.ndarray) -> None:
        # 模型角度 → 机械角度 → USB ExtraData
        # mech_angles = model_angles_to_mech(np.degrees(angles))
        # self._driver.send_extra_data(mech_angles, enable=True)
        raise NotImplementedError()

    def get_camera_image(self) -> np.ndarray:
        raise NotImplementedError()

    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError()
