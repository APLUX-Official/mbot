# component_container.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='pointcloud_to_laserscan_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan_camera1',
                parameters=[{
                    'target_frame': 'base_link',
                    'min_height': 0.05,
                    'max_height': 2.0,
                    'angle_min': -2.0,
                    'angle_max': 2.0,
                    'range_min': 0.01,
                    'range_max': 4.0,
                }],
                remappings=[
                    ('cloud_in', '/gazebo/rgbd1/points'),
                    ('scan', '/scan_camera1')
                ]
            ),
            ComposableNode(
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan_camera2',
                parameters=[{
                    'target_frame': 'base_link',
                    'min_height': 0.05,
                    'max_height': 2.0,
                    'angle_min': -2.0,
                    'angle_max': 2.0,
                    'range_min': 0.01,
                    'range_max': 4.0,
                }],
                remappings=[
                    ('cloud_in', '/gazebo/rgbd2/points'),
                    ('scan', '/scan_camera2')
                ]
            )
        ],
        output='screen',
    )
    
    return LaunchDescription([container])