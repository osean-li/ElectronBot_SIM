#!/usr/bin/env python3
"""
VLA ROS2 节点

闭环交互: 订阅相机图像 → VLM 推理 → 发布关节指令 → 再观察

运行:
  ros2 run electronbot_vla vla_node --ros-args -p vla_mode:=qwen

Topic:
  Sub: /camera/image_raw (sensor_msgs/Image)
  Pub: /joint_trajectory_commands (trajectory_msgs/JointTrajectory)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import numpy as np

from electronbot_vla.vlm_backend import create_vla_backend
from electronbot_vla.action_parser import ActionParser


class VLANode(Node):
    """VLA ROS2 节点"""

    JOINT_NAMES = [
        "body_joint", "head_joint",
        "left_shoulder_joint", "left_arm_roll_joint",
        "right_shoulder_joint", "right_arm_roll_joint",
    ]

    def __init__(self):
        super().__init__("vla_node")

        # 声明参数
        self.declare_parameter("vla_mode", "qwen")  # qwen / openvla / keyword
        self.declare_parameter("task", "wave")       # 默认任务
        self.declare_parameter("control_rate", 10.0) # Hz

        self.vla_mode = self.get_parameter("vla_mode").value
        self.task = self.get_parameter("task").value
        self.control_rate = self.get_parameter("control_rate").value

        # VLA 后端
        if self.vla_mode != "keyword":
            try:
                self.vla_backend = create_vla_backend(mode=self.vla_mode)
                self.get_logger().info(f"VLA 后端: {self.vla_mode}")
            except Exception as e:
                self.get_logger().warn(f"VLA 加载失败: {e}, 使用关键词模式")
                self.vla_backend = None
                self.vla_mode = "keyword"
        else:
            self.vla_backend = None

        # 动作解析器
        self.parser = ActionParser(vla_backend=self.vla_backend)
        self.bridge = CvBridge()

        # 当前任务
        self.current_task = self.task
        self._latest_image = None

        # 订阅相机
        self.camera_sub = self.create_subscription(
            Image, "/camera/image_raw", self._camera_callback, 10
        )

        # 发布关节指令
        self.cmd_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_commands", 10
        )

        # 定时控制循环
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_loop
        )

        self.get_logger().info(
            f"VLA 节点已启动 (mode={self.vla_mode}, task={self.task})"
        )

    def _camera_callback(self, msg: Image):
        """接收相机图像"""
        try:
            self._latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"图像解码失败: {e}")

    def _control_loop(self):
        """控制循环: 图像 → VLM → 关节指令"""
        if self._latest_image is None:
            self.get_logger().debug("等待相机图像...")
            return

        # VLA 推理
        angles_rad = self.parser.parse(
            text=self.current_task,
            image=self._latest_image if self.vla_mode != "keyword" else None,
        )

        # 发布指令
        self._publish_joint_cmd(angles_rad)

        self.get_logger().debug(
            f"指令: {np.round(np.degrees(angles_rad), 1)}°"
        )

    def _publish_joint_cmd(self, angles_rad: np.ndarray):
        """发布关节指令"""
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = angles_rad.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e9 / self.control_rate)
        msg.points = [point]
        self.cmd_pub.publish(msg)

    def set_task(self, task: str):
        """切换任务 (可被 service/action 调用)"""
        self.current_task = task
        self.get_logger().info(f"任务切换: {task}")


def main(args=None):
    rclpy.init(args=args)
    node = VLANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
