#!/usr/bin/env python3
"""
PLICP激光雷达里程计启动文件 - ROS2版本

此启动文件用于启动PLICP里程计节点，包括：
1. 加载参数配置
2. 启动里程计节点  
3. 可选的可视化配置

使用方法:
    ros2 launch plicp_odometry_ros2 plicp_odometry.launch.py

可选参数:
    - config_file: 自定义配置文件路径
    - use_rviz: 是否启动RViz可视化 (默认: false)
    - log_level: 日志级别 (默认: info)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('plicp_odometry_ros2').find('plicp_odometry_ros2')
    
    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'plicp_odometry.yaml')
    
    # 声明启动参数
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='PLICP里程计配置文件的完整路径'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动RViz进行可视化'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='节点日志级别 (debug, info, warn, error)'
    )
    
    declare_odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='里程计坐标系名称'
    )
    
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame', 
        default_value='base_footprint',
        description='机器人基座坐标系名称'
    )

    # PLICP里程计节点
    plicp_odometry_node = Node(
        package='plicp_odometry_ros2',
        executable='plicp_odometry_node',
        name='plicp_odometry',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # 可以在这里添加话题重映射
            # ('scan', '/your_laser_topic'),
            # ('odom', '/your_odom_topic'),
        ]
    )

    # RViz可视化节点（可选）
    rviz_config_file = os.path.join(pkg_share, 'config', 'plicp_odometry.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # 静态TF发布器（如果需要激光雷达到base_link的变换）
    # 取消注释并根据实际情况调整参数
    static_tf_publisher = Node(
         package='tf2_ros',
         executable='static_transform_publisher',
         name='base_footprint_to_laser_frame_publisher',  # 更清晰的名称
         arguments=['0', '0', '0', '0', '0', '3.14156', 'base_link', 'laser_frame']
     )

    # 添加 base_link 到 base_footprint 的静态变换
    base_link_to_base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 添加启动参数声明
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_odom_frame_cmd)
    ld.add_action(declare_base_frame_cmd)

    # 添加节点
    ld.add_action(static_tf_publisher)  # 先启动 TF 发布器
    ld.add_action(base_link_to_base_footprint_tf)  # 先启动 TF 发布器
    ld.add_action(plicp_odometry_node)
    ld.add_action(rviz_node)

    return ld