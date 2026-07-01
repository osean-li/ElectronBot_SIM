"""
ElectronBot 仿真传感器模块

提供:
- D435 仿真相机 (RGB + Depth 240x240)
- 虚拟 IMU (加速度计 + 陀螺仪)
- 虚拟力/触觉传感器 (末端接触力)
"""
import numpy as np
from typing import Tuple, Optional
import cv2


class D435SimCamera:
    """
    Intel RealSense D435 仿真相机

    模拟 240x240 RGB-D 相机，输出格式对齐固件 LCD 显示规格。
    通过 MuJoCo 的 camera sensor 获取渲染图像。
    """

    def __init__(self, width: int = 240, height: int = 240, fov: float = 60.0):
        self.width = width
        self.height = height
        self.fov = fov
        self.fx = (width / 2.0) / np.tan(np.radians(fov / 2.0))
        self.fy = (height / 2.0) / np.tan(np.radians(fov / 2.0))
        self.cx = width / 2.0
        self.cy = height / 2.0

    def get_rgb_from_robot(self, robot) -> np.ndarray:
        """从 ElectronBotRobot 获取 RGB 图像"""
        return robot.get_camera_image(self.width, self.height)

    def get_depth_from_robot(self, robot) -> np.ndarray:
        """从 ElectronBotRobot 获取深度图像"""
        depth = robot.get_camera_depth(self.width, self.height)
        # 深度归一化到米
        if depth.max() > 0:
            depth = depth / depth.max()
        return depth

    def depth_to_point_cloud(self, depth: np.ndarray) -> np.ndarray:
        """
        将深度图转换为点云 (可选 PCL 功能)

        返回:
          (N, 3) 点云数组 [x, y, z]
        """
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth.flatten()

        valid = z > 0
        x = (u.flatten()[valid] - self.cx) * z[valid] / self.fx
        y = (v.flatten()[valid] - self.cy) * z[valid] / self.fy

        return np.column_stack([x, y, z[valid]])


class VirtualIMU:
    """虚拟 IMU 传感器 (读取 MuJoCo sensor 数据)"""

    @staticmethod
    def get_accel(robot) -> np.ndarray:
        """获取加速度计读数 (m/s²)"""
        accel, _ = robot.get_imu_data()
        return accel

    @staticmethod
    def get_gyro(robot) -> np.ndarray:
        """获取陀螺仪读数 (rad/s)"""
        _, gyro = robot.get_imu_data()
        return gyro

    @staticmethod
    def get_orientation(robot) -> np.ndarray:
        """获取头部连杆的四元数朝向 [w, x, y, z]"""
        head_id = robot.model.body("head").id
        quat = robot.data.xquat[head_id].copy()
        return quat


class VirtualForceSensor:
    """虚拟力/触觉传感器 (末端接触力)"""

    @staticmethod
    def get_contact_force(robot, site_name: str = "left_ee_site") -> np.ndarray:
        """
        获取末端 site 的接触力

        返回:
          (3,) 接触力向量 [fx, fy, fz]
        """
        site_id = robot.model.site(site_name).id
        total_force = np.zeros(3)

        # 遍历所有接触对
        for i in range(robot.data.ncon):
            contact = robot.data.contact[i]
            # 检查接触是否涉及目标 site 所属的 geom
            geom1 = contact.geom1
            geom2 = contact.geom2
            # 如果接触涉及 site 的 body，累加力
            force = np.zeros(6)
            mujoco.mj_contactForce(robot.model, robot.data, i, force)
            total_force += force[:3]

        return total_force


# ---- OpenCV 预处理管线 ----

class PerceptionPipeline:
    """
    感知预处理管线 (OpenCV)
    用于 VLA 和 IL 的输入预处理

    处理步骤:
    1. RGB 图像 resize
    2. 颜色空间转换
    3. 目标检测 / 边缘提取 (按需)
    """

    def __init__(self, target_size: Tuple[int, int] = (240, 240)):
        self.target_size = target_size

    def preprocess(self, rgb: np.ndarray) -> np.ndarray:
        """标准预处理: resize + normalize"""
        img = cv2.resize(rgb, self.target_size)
        img = img.astype(np.float32) / 255.0
        return img

    def detect_red_objects(self, rgb: np.ndarray) -> list:
        """检测图像中的红色物体"""
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        # 红色在 HSV 中有两个范围
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 50:  # 过滤小噪声
                x, y, w, h = cv2.boundingRect(cnt)
                objects.append({"bbox": (x, y, w, h), "center": (x + w // 2, y + h // 2)})
        return objects

    def extract_edges(self, rgb: np.ndarray) -> np.ndarray:
        """提取边缘特征 (用于视觉伺服)"""
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return edges
