/*
 * PLICP激光雷达里程计节点主函数 - ROS2版本
 * 
 * 这个文件包含ROS2节点的main函数，负责：
 * 1. 初始化ROS2系统
 * 2. 创建PLICP里程计节点
 * 3. 启动节点执行
 * 4. 处理优雅关闭
 * 
 * Copyright 2020 The Project Author: lixiang
 * Licensed under the Apache License, Version 2.0
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "plicp_odometry_ros2/plicp_odometry.hpp"

int main(int argc, char * argv[])
{
    // 初始化ROS2系统
    rclcpp::init(argc, argv);

    // 创建PLICP里程计节点
    auto node = std::make_shared<plicp_odometry_ros2::PlicpOdometry>();

    // 输出启动信息
    RCLCPP_INFO(node->get_logger(), "PLICP里程计节点已启动，等待激光雷达数据...");

    try {
        // 启动节点事件循环
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "节点运行异常: %s", e.what());
        return 1;
    }

    // 清理ROS2系统
    rclcpp::shutdown();
    
    RCLCPP_INFO(node->get_logger(), "PLICP里程计节点已正常关闭");
    return 0;
}
