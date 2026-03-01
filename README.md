ros2 launch evobase_bringup combined.launch.py
会启动底盘控制,蓝海雷达(bluesea2),plicp雷达里程计(已融合轮式里程计数据)
ros2 launch slam_toolbox online_async_launch.py
建图
ros2 run nav2_map_server map_saver_cli -f ~/my_map
ros2 launch evobase_navigation2 navigation2.launch.py
nav2导航
