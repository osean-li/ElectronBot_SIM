#!/usr/bin/env python3
"""
ElectronBot MuJoCo 仿真启动文件 (ROS2)

一键启动: MuJoCo 仿真 + ROS2 bridge + RViz2 可视化

使用方法:
  ros2 launch electronbot_mujoco_ros2 sim.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("electronbot_mujoco_ros2")
    desc_dir = get_package_share_directory("electronbot_description")

    urdf_path = os.path.join(desc_dir, "urdf", "electronbot.urdf")

    return LaunchDescription([
        # MuJoCo ROS2 Bridge
        Node(
            package="electronbot_mujoco_ros2",
            executable="mujoco_ros2_bridge",
            name="mujoco_ros2_bridge",
            output="screen",
        ),
        # RViz2 (可选)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(desc_dir, "rviz", "electronbot.rviz")],
            condition=None,  # 默认不启动 RViz，加参数启动
        ),
    ])
