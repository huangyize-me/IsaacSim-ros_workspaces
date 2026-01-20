# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Lumi Navigation Launch File
适用于 JAKA-Lumi 移动操作机器人的 Nav2 导航启动文件

包含：
- Nav2 导航栈
- RViz2 可视化
- 点云转激光雷达节点
- 虚拟差速控制器（带坐标变换：base_link -> world）

数据流：
  Nav2 (/cmd_vel) --> lumi_twist_to_joints --> /lumi/joint_command
       [base_link]     (订阅 /odom 获取 yaw)       [world frame]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("lumi_navigation_test"), "maps", "carter_warehouse_navigation.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("lumi_navigation_test"), "params", "lumi_navigation_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("lumi_navigation_test"), "rviz2", "lumi_navigation.rviz")

    return LaunchDescription(
        [
            # ========== 启动参数声明 ==========
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation (Omniverse Isaac Sim) clock if true"
            ),

            # ========== RViz2 可视化 ==========
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),

            # ========== Nav2 导航栈 ==========
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),

            # ========== 点云转激光雷达节点 ==========
            # 如果你的 Lumi 使用 3D LiDAR，需要将点云转为 2D scan
            # 请根据实际话题名称修改 cloud_in 和 target_frame
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/point_cloud']),  # Isaac Sim PhysX Lidar 发布的点云话题
                            ('scan', ['/scan'])],
                parameters=[{
                    'target_frame': 'Radar_Link',  # Lumi 的雷达 link 名称
                    'transform_tolerance': 0.01,
                    'min_height': -0.4,
                    'max_height': 1.5,
                    'angle_min': -3.14159,  # -PI (360度扫描)
                    'angle_max': 3.14159,   # PI
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.1,
                    'range_max': 30.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                }],
                name='pointcloud_to_laserscan'
            ),

            # ========== 虚拟差速控制器（带坐标变换） ==========
            # 将 Nav2 发出的 /cmd_vel (base_link 坐标系) 
            # 转换为世界坐标系下的虚拟关节速度指令
            # 
            # 关键：订阅 /odom 获取当前 yaw 角，进行 Body -> World 坐标变换
            # 参考：lumi/motion_controller.md
            Node(
                package='lumi_navigation_test',
                executable='lumi_twist_to_joints.py',
                name='lumi_twist_to_joints',
                output='screen',
                parameters=[{
                    # 输入话题
                    'cmd_vel_topic': '/cmd_vel',      # Nav2 速度指令 (base_link frame)
                    'odom_topic': '/odom',            # 里程计 (获取当前 yaw)
                    # 输出话题
                    'joint_command_topic': '/lumi/joint_command',  # 关节速度 (world frame)
                    # 控制参数
                    'publish_rate': 50.0,
                    # 物理限制（参考 motion_controller.md）
                    'max_linear_vel': 1.2,            # 最大线速度 m/s
                    'max_angular_vel': 3.9,           # 最大角速度 rad/s
                    'min_turning_radius': 0.307,      # 最小转弯半径 m
                    # 虚拟关节名称
                    'joint_x_name': 'dummy_base_prismatic_x_joint',
                    'joint_y_name': 'dummy_base_prismatic_y_joint',
                    'joint_yaw_name': 'dummy_base_revolute_z_joint',
                }]
            ),
        ]
    )
