#!/usr/bin/env python3
"""
ElectronBot ROS2 显示启动文件 (RViz2)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("electronbot_description")

    urdf_path = os.path.join(pkg_dir, "urdf", "electronbot.urdf")

    # 读取 URDF
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
        ),
        # Joint State Publisher (GUI)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(pkg_dir, "rviz", "electronbot.rviz")],
        ),
    ])
