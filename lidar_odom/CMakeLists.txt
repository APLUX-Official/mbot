cmake_minimum_required(VERSION 3.8)
project(plicp_odometry_ros2)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 查找CSM库
find_package(PkgConfig REQUIRED)
pkg_check_modules(CSM REQUIRED csm)

# 包含头文件目录
include_directories(include)

# 设置依赖列表
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 创建可执行文件
add_executable(plicp_odometry_node src/plicp_odometry_node.cpp src/plicp_odometry.cpp)

# 链接依赖库
ament_target_dependencies(plicp_odometry_node ${dependencies})

# 链接CSM库
target_link_libraries(plicp_odometry_node ${CSM_LIBRARIES})
target_include_directories(plicp_odometry_node PRIVATE ${CSM_INCLUDE_DIRS})

# 创建组件库（可选，用于组合节点）
add_library(plicp_odometry_component SHARED src/plicp_odometry.cpp)
ament_target_dependencies(plicp_odometry_component ${dependencies})
target_link_libraries(plicp_odometry_component ${CSM_LIBRARIES})
target_include_directories(plicp_odometry_component PRIVATE ${CSM_INCLUDE_DIRS})

# 注册组件
rclcpp_components_register_node(plicp_odometry_component
  PLUGIN "plicp_odometry_ros2::PlicpOdometry"
  EXECUTABLE plicp_odometry_component_node
)

# 安装可执行文件
install(TARGETS
  plicp_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件
install(TARGETS
  plicp_odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装启动文件和配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(plicp_odometry_component)

ament_package()
