# SPDX-FileCopyrightText: Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

"""
Lumi Twist to Joints Controller Launch File

单独启动虚拟差速控制器节点（带坐标变换）。
该节点将 Nav2 的 /cmd_vel 转换为 Lumi 虚拟底盘关节的速度指令。

关键特性：
- 订阅 /odom 获取当前 yaw 角
- 将 base_link 坐标系的速度转换为世界坐标系
- 支持最小转弯半径约束

用法：
  ros2 launch lumi_navigation_test lumi_twist_controller.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== 声明启动参数 ====================
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic to subscribe for velocity commands (base_link frame)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic to subscribe for odometry (to get current yaw)'
    )

    joint_command_topic_arg = DeclareLaunchArgument(
        'joint_command_topic',
        default_value='/lumi/joint_command',
        description='Topic to publish joint velocity commands (world frame)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Rate (Hz) at which to publish joint commands'
    )

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='1.2',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='3.9',
        description='Maximum angular velocity (rad/s)'
    )

    min_turning_radius_arg = DeclareLaunchArgument(
        'min_turning_radius',
        default_value='0.307',
        description='Minimum turning radius (m) for speed constraint'
    )

    # ==================== 创建节点 ====================
    twist_to_joints_node = Node(
        package='lumi_navigation_test',
        executable='lumi_twist_to_joints.py',
        name='lumi_twist_to_joints',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'joint_command_topic': LaunchConfiguration('joint_command_topic'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'min_turning_radius': LaunchConfiguration('min_turning_radius'),
            'joint_x_name': 'dummy_base_prismatic_x_joint',
            'joint_y_name': 'dummy_base_prismatic_y_joint',
            'joint_yaw_name': 'dummy_base_revolute_z_joint',
        }]
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        odom_topic_arg,
        joint_command_topic_arg,
        publish_rate_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        min_turning_radius_arg,
        twist_to_joints_node,
    ])
