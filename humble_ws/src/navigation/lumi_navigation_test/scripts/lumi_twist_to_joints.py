#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Lumi Twist to Joint Velocity Controller (with Coordinate Transform)

这个节点订阅 Nav2 发出的 /cmd_vel (geometry_msgs/Twist)，
并将其转换为 Lumi 虚拟底盘关节的速度指令。

关键：cmd_vel 是在 base_link 坐标系下的，而虚拟关节是在世界坐标系下运动的。
因此需要根据当前机器人的 yaw 角进行坐标变换：
    world_vx = local_vx * cos(yaw)
    world_vy = local_vx * sin(yaw)
    world_omega = angular_z

参考：lumi/motion_controller.md

发布话题：/lumi/joint_command (sensor_msgs/JointState)
订阅话题：/cmd_vel (geometry_msgs/Twist), /odom (nav_msgs/Odometry)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion


class LumiTwistToJoints(Node):
    """将 Twist 速度指令转换为虚拟关节速度的 ROS 2 节点（带坐标变换）"""

    def __init__(self):
        super().__init__('lumi_twist_to_joints')

        # ==================== 参数声明 ====================
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('joint_command_topic', '/lumi/joint_command')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # 虚拟关节名称（与 Isaac Lab 中的定义保持一致）
        self.declare_parameter('joint_x_name', 'dummy_base_prismatic_x_joint')
        self.declare_parameter('joint_y_name', 'dummy_base_prismatic_y_joint')
        self.declare_parameter('joint_yaw_name', 'dummy_base_revolute_z_joint')
        
        # 物理限制参数（参考 motion_controller.md）
        self.declare_parameter('max_linear_vel', 1.2)    # m/s
        self.declare_parameter('max_angular_vel', 3.9)   # rad/s
        self.declare_parameter('min_turning_radius', 0.307)  # m

        # ==================== 获取参数值 ====================
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        joint_cmd_topic = self.get_parameter('joint_command_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.joint_x_name = self.get_parameter('joint_x_name').value
        self.joint_y_name = self.get_parameter('joint_y_name').value
        self.joint_yaw_name = self.get_parameter('joint_yaw_name').value
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_turning_radius = self.get_parameter('min_turning_radius').value

        # ==================== 状态变量 ====================
        # 本地坐标系下的目标速度（来自 cmd_vel）
        self.local_linear_x = 0.0
        self.local_angular_z = 0.0
        
        # 当前机器人的 yaw 角（来自 odom）
        self.current_yaw = 0.0
        
        # 上次收到指令的时间（用于超时检测）
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # 秒

        # ==================== QoS 配置 ====================
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE
        
        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT

        # ==================== 订阅器 ====================
        # 订阅 /cmd_vel（来自 Nav2）
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            qos_reliable
        )
        
        # 订阅 /odom（来自 Isaac Sim）获取当前 yaw 角
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_best_effort  # Isaac Sim 通常用 best_effort
        )

        # ==================== 发布器 ====================
        # 发布关节速度指令 (JointState 格式)
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            joint_cmd_topic,
            qos_reliable
        )
        
        # 也发布一个简单的 Float64MultiArray 格式（备用）
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/lumi/base_velocity',
            qos_reliable
        )

        # ==================== 定时器 ====================
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_command)

        # ==================== 日志 ====================
        self.get_logger().info('=' * 60)
        self.get_logger().info('Lumi Twist to Joints Controller (with Coord Transform)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Subscribing cmd_vel:  {cmd_vel_topic}')
        self.get_logger().info(f'  Subscribing odom:     {odom_topic}')
        self.get_logger().info(f'  Publishing joints:    {joint_cmd_topic}')
        self.get_logger().info(f'  Max linear vel:       {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Min turning radius:   {self.min_turning_radius} m')
        self.get_logger().info('=' * 60)

    def cmd_vel_callback(self, msg: Twist):
        """处理收到的 Twist 速度指令（base_link 坐标系）"""
        # 存储本地坐标系下的速度
        self.local_linear_x = msg.linear.x
        self.local_angular_z = msg.angular.z
        
        # 更新时间戳
        self.last_cmd_time = self.get_clock().now()
        
        self.get_logger().debug(
            f'Received cmd_vel: vx={msg.linear.x:.3f}, wz={msg.angular.z:.3f}'
        )

    def odom_callback(self, msg: Odometry):
        """处理里程计消息，提取当前 yaw 角"""
        # 从四元数提取 yaw
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        self.current_yaw = yaw

    def compute_world_velocities(self):
        """
        将 base_link 坐标系下的速度转换为世界坐标系下的虚拟关节速度
        
        参考 lumi/motion_controller.md:
            world_vx = V * cos(yaw)
            world_vy = V * sin(yaw)
            world_omega = omega
        """
        # 1. 速度裁剪
        v = np.clip(self.local_linear_x, -self.max_linear_vel, self.max_linear_vel)
        omega = self.local_angular_z
        
        # 2. 转弯半径约束（核心安全保护）
        # 当有线速度时，角速度必须受限以满足最小转弯半径
        if abs(v) > 0.01:
            max_omega = abs(v) / self.min_turning_radius
            omega = np.clip(omega, -max_omega, max_omega)
        else:
            # 原地旋转时不限制角速度（但仍有上限）
            omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        # 3. 坐标变换：Body -> World
        world_vx = v * math.cos(self.current_yaw)
        world_vy = v * math.sin(self.current_yaw)
        world_omega = omega
        
        return world_vx, world_vy, world_omega

    def publish_joint_command(self):
        """定时发布关节速度指令"""
        # 检查是否超时（安全保护）
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_last_cmd > self.cmd_timeout:
            # 超时，停止机器人
            self.local_linear_x = 0.0
            self.local_angular_z = 0.0

        # 计算世界坐标系下的速度
        world_vx, world_vy, world_omega = self.compute_world_velocities()

        # 构建 JointState 消息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'world'  # 世界坐标系
        
        # 关节名称
        joint_state.name = [
            self.joint_x_name,
            self.joint_y_name,
            self.joint_yaw_name
        ]
        
        # 关节速度（已转换到世界坐标系）
        joint_state.velocity = [
            float(world_vx),
            float(world_vy),
            float(world_omega)
        ]
        
        # position 和 effort 留空（我们只做速度控制）
        joint_state.position = []
        joint_state.effort = []

        # 发布
        self.joint_cmd_pub.publish(joint_state)
        
        # 同时发布简化格式
        vel_msg = Float64MultiArray()
        vel_msg.data = [float(world_vx), float(world_vy), float(world_omega)]
        self.velocity_pub.publish(vel_msg)
        
        # 调试日志（可选）
        if abs(world_vx) > 0.001 or abs(world_vy) > 0.001 or abs(world_omega) > 0.001:
            self.get_logger().debug(
                f'Publishing: yaw={math.degrees(self.current_yaw):.1f}° | '
                f'world_vel=[{world_vx:.3f}, {world_vy:.3f}, {world_omega:.3f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = LumiTwistToJoints()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Lumi Twist to Joints Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
