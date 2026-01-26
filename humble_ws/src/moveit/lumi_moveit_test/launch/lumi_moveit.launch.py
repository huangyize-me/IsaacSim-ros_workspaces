# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# MoveIt2 launch file for JAKA Lumi robot with Isaac Sim integration

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Get package share directory
    lumi_moveit_test_dir = get_package_share_directory("lumi_moveit_test")

    # Command-line arguments
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # Declare use_sim_time argument
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    # Load robot description from xacro
    from xacro import process_file
    xacro_file = os.path.join(lumi_moveit_test_dir, "config", "jaka_lumi.urdf.xacro")
    robot_description_config = process_file(
        xacro_file,
        mappings={"ros2_control_hardware_type": "isaac"}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Load SRDF
    srdf_file = os.path.join(lumi_moveit_test_dir, "config", "jaka_lumi.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Load kinematics
    kinematics_yaml = load_yaml("lumi_moveit_test", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Load joint limits
    joint_limits_yaml = load_yaml("lumi_moveit_test", "config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # Load OMPL planning configuration
    ompl_planning_yaml = load_yaml("lumi_moveit_test", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": {"planning_plugin": "ompl_interface/OMPLPlanner"}}
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Load trajectory execution / controllers
    moveit_controllers_yaml = load_yaml("lumi_moveit_test", "config/moveit_controllers.yaml")

    # Planning scene monitor options
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = os.path.join(
        lumi_moveit_test_dir,
        "rviz",
        "lumi_moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning_pipeline_config,
            {"use_sim_time": True},
        ],
    )

    # Publish TF (Robot State Publisher)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    # ros2_control node
    ros2_controllers_path = os.path.join(
        lumi_moveit_test_dir,
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Spawners for controllers - with delay to ensure controller_manager is ready
    # Use --controller-manager-timeout to wait longer for controller_manager
    # NOTE: joint_state_broadcaster is NOT needed because Isaac Sim already publishes
    # complete /joint_states including all gripper mimic joints

    jaka_lumi_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jaka_lumi_body_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )

    jaka_lumi_minicobo_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jaka_lumi_minicobo_arm_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )

    # Delay spawners to start after ros2_control_node is ready
    delayed_body_controller = TimerAction(
        period=3.0,
        actions=[jaka_lumi_body_controller_spawner],
    )

    delayed_arm_controller = TimerAction(
        period=4.0,
        actions=[jaka_lumi_minicobo_arm_controller_spawner],
    )

    delayed_gripper_controller = TimerAction(
        period=5.0,
        actions=[gripper_controller_spawner],
    )

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            use_sim_time,
            robot_state_publisher,
            ros2_control_node,
            delayed_body_controller,
            delayed_arm_controller,
            delayed_gripper_controller,
            move_group_node,
            rviz_node,
        ]
    )
