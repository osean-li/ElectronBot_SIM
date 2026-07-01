#!/usr/bin/env python3
"""
MuJoCo ↔ ROS2 桥接节点

功能:
- 发布 /joint_states (50Hz)
- 发布 /tf (50Hz)
- 发布 /camera/image_raw (RGB, 30Hz)
- 发布 /camera/depth (Depth, 30Hz)
- 订阅 /joint_trajectory_commands (控制仿真)

运行:
  ros2 run electronbot_mujoco_ros2 mujoco_ros2_bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

import numpy as np
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from electronbot_mujoco.robot import ElectronBotRobot
from electronbot_mujoco.sensors import D435SimCamera, PerceptionPipeline

from cv_bridge import CvBridge


class MuJoCoROS2Bridge(Node):
    """MuJoCo 仿真 → ROS2 消息桥接"""

    JOINT_NAMES = [
        "body_joint", "head_joint",
        "left_shoulder_joint", "left_arm_roll_joint",
        "right_shoulder_joint", "right_arm_roll_joint",
    ]

    def __init__(self):
        super().__init__("mujoco_ros2_bridge")

        # 初始化 MuJoCo
        self.robot = ElectronBotRobot()
        self.camera = D435SimCamera()
        self.pipeline = PerceptionPipeline()
        self.bridge = CvBridge()

        # ---- 发布者 ----
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.rgb_pub = self.create_publisher(
            Image, "/camera/image_raw", 10
        )
        self.depth_pub = self.create_publisher(
            Image, "/camera/depth", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- 订阅者 ----
        self.cmd_sub = self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_commands",
            self._cmd_callback,
            10,
        )

        # ---- 定时器 ----
        self.joint_timer = self.create_timer(
            1.0 / 50.0,  # 50Hz
            self._publish_joint_states,
        )
        self.camera_timer = self.create_timer(
            1.0 / 30.0,  # 30Hz
            self._publish_camera,
        )

        # 仿真步进定时器 (50Hz)
        self.step_timer = self.create_timer(
            1.0 / 50.0,
            self._step_simulation,
        )

        # 目标角度缓存
        self._target_angles = np.zeros(6)

        self.get_logger().info("MuJoCo ROS2 Bridge 已启动")

    def _step_simulation(self):
        """执行一步仿真"""
        if self._target_angles is not None:
            self.robot.send_position_command(self._target_angles)
        self.robot.step()

    def _cmd_callback(self, msg: JointTrajectory):
        """接收 /joint_trajectory_commands"""
        if msg.points:
            point = msg.points[-1]
            if len(point.positions) == 6:
                self._target_angles = np.array(point.positions, dtype=np.float64)
                self.get_logger().debug(
                    f"收到指令: {np.round(np.degrees(self._target_angles), 1)}°"
                )

    def _publish_joint_states(self):
        """发布关节状态和 TF"""
        now = self.get_clock().now().to_msg()

        # Joint states
        js_msg = JointState()
        js_msg.header.stamp = now
        js_msg.name = self.JOINT_NAMES
        js_msg.position = self.robot.get_joint_positions().tolist()
        js_msg.velocity = self.robot.get_joint_velocities().tolist()
        self.joint_state_pub.publish(js_msg)

        # TF (body → base_link)
        # 简化: 只发布 body 相对于 base_link 的变换
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_link"
        t.child_frame_id = "body"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.03
        body_angle = self.robot.get_joint_positions()[0]
        t.transform.rotation.z = float(np.sin(body_angle / 2))
        t.transform.rotation.w = float(np.cos(body_angle / 2))
        # self.tf_broadcaster.sendTransform(t)  # 简化: 单 TF

    def _publish_camera(self):
        """发布相机图像"""
        now = self.get_clock().now().to_msg()

        # RGB
        rgb = self.camera.get_rgb_from_robot(self.robot)
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = "camera_link"
        self.rgb_pub.publish(rgb_msg)

        # Depth (简化为灰度图)
        depth = self.camera.get_depth_from_robot(self.robot)
        depth_uint16 = (depth * 1000).astype(np.uint16)  # mm
        depth_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding="mono16")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "camera_link"
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
