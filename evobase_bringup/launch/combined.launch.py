import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    evobase_bringup_dir = get_package_share_directory('evobase_bringup')
    bluesea2_dir = get_package_share_directory('bluesea2')
    plicp_odometry_dir = get_package_share_directory('plicp_odometry_ros2')

    # First launch: evobase_bringup evobase_cmd_odom.launch.py
    evobase_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(evobase_bringup_dir, 'launch', 'evobase_cmd_odom.launch.py')
        )
    )

    # Second launch: bluesea2 uart_lidar.launch (lidar)
    bluesea_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(bluesea2_dir, 'launch', 'uart_lidar.launch')
        )
    )

    # Third launch: plicp_odometry_ros2 plicp_odometry.launch.py with 2s delay after lidar
    plicp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plicp_odometry_dir, 'launch', 'plicp_odometry.launch.py')
        )
    )

    # Timer for 2s delay
    delayed_plicp = TimerAction(
        period=2.0,
        actions=[plicp_launch]
    )

    return LaunchDescription([
        evobase_launch,
        bluesea_launch,
        delayed_plicp
    ])