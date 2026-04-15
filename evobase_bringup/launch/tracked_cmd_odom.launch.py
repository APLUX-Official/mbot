import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 你的功能包名称，请确保与你的 package.xml 中的名称一致
    package_name = 'evobase_bringup'
    
    # 获取参数文件的完整路径
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'tracked_config.yaml' # 确保你的配置文件叫这个名字
    )
    
    # 1. 声明可以在 launch 时从命令行修改的参数
    
    # 是否发布 odom -> base_frame 的TF变换
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true', # 默认值为 true
        description='设置为 true 以发布 TF 变换, false 则不发布'
    )
    
    # 机器人的基座标系名称
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint', # 默认值为 base_footprint
        description='机器人的基座标系 TF ID (例如: base_link, base_footprint)'
    )

    # 2. 定义要启动的节点
    pid_controller_node = Node(
            package=package_name,
            executable='tracked_cmd_odom', # 你的Python脚本文件名
            name='tracked_cmd_node',       # 节点的名称, 必须与yaml文件顶级名称匹配
            output='screen',
            parameters=[
                config_file, # 首先加载yaml文件中的所有参数
                # 然后，用launch文件中的参数覆盖yaml中的同名参数
                {
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'base_frame': LaunchConfiguration('base_frame')
                }
            ]
        )

    return LaunchDescription([
        publish_tf_arg,
        base_frame_arg,
        pid_controller_node
    ])