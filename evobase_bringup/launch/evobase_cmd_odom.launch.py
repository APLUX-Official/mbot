'''
Author: WALT
Date: 2026-03-02 16:13:10
'''
#!/usr/bin/env python3

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'evobase_bringup'
    
    # 获取包的共享目录路径
    pkg_share = get_package_share_directory(package_name)
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'evobase_config.yaml')
    
    # 声明launch参数
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='是否发布 odom -> base_link TF变换。设为false可与其他定位算法（如AMCL）共存'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom_wheel',
        description='里程计话题名称'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='机器人基座坐标系名称'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS2日志级别 (debug, info, warn, error)'
    )
    
    return LaunchDescription([
        publish_tf_arg,
        odom_topic_arg,
        base_frame_arg,
        log_level_arg,
        
        Node(
            package=package_name,
            executable='evobase_cmd_odom',
            name='evobase_cmd_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                {'publish_tf': LaunchConfiguration('publish_tf')},
                {'odom_topic': LaunchConfiguration('odom_topic')},
                {'base_frame': LaunchConfiguration('base_frame')}
            ],
            # 可以添加参数文件如果需要
            # parameters=[config_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', LaunchConfiguration('odom_topic')),
            ]
        )
    ])
