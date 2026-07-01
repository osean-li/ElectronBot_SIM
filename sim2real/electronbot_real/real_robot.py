"""真实机器人控制接口 (继承 RobotInterface)"""
import numpy as np
from .usb_driver import USBDriver
from ..electronbot_mujoco_ros2.sim2real_bridge import RobotInterface


class RealRobot(RobotInterface):
    """真实 ElectronBot 控制"""

    def __init__(self):
        self._driver = USBDriver()
        self._connected = False

    def connect(self) -> bool:
        self._connected = self._driver.open()
        return self._connected

    def disconnect(self) -> None:
        self._driver.close()
        self._connected = False

    def get_joint_positions(self) -> np.ndarray:
        mech = self._driver.receive_joint_angles()
        if mech is None:
            return np.zeros(6)
        from electronbot_mujoco.utils import mech_angles_to_model
        return np.radians(mech_angles_to_model(mech))

    def get_joint_velocities(self) -> np.ndarray:
        return np.zeros(6)  # 舵机不直接回报速度

    def send_joint_command(self, angles: np.ndarray) -> None:
        model_deg = np.degrees(angles)
        self._driver.send_extra_data(enable=True, joint_angles=model_deg)

    def get_camera_image(self) -> np.ndarray:
        raise NotImplementedError("真实相机通过 ROS2 topic 获取")

    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        return np.zeros(3), np.zeros(3)
